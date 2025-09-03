import rclpy
import copy
import time
import random

from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
import math
from geometry_msgs.msg import Vector3, TransformStamped, Pose, PoseStamped, Quaternion, Transform

import pm_skills_interfaces.srv as pm_skill_srv
from pm_moveit_interfaces.srv import MoveToPose,  MoveToFrame, MoveRelative
from example_interfaces.srv import SetBool, Trigger
import std_msgs.msg as std_msg
import assembly_manager_interfaces.srv as ami_srv
import assembly_manager_interfaces.msg as ami_msg
import pm_moveit_interfaces.srv as pm_moveit_srv
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, StaticTransformBroadcaster

from pm_msgs.srv import EmptyWithSuccess
from assembly_scene_publisher.py_modules.AssemblyScene import AssemblyManagerScene

from assembly_scene_publisher.py_modules.tf_functions import get_transform_for_frame_in_world

from assembly_scene_publisher.py_modules.scene_functions import is_frame_from_scene

from scipy.spatial.transform import Rotation as R
import numpy as np
from pm_msgs.srv import UVCuringSkill
import pm_msgs.srv as pm_msg_srv

from assembly_scene_publisher.py_modules.scene_functions import (get_parent_of_component,
                                                                 has_component_parent_of_name,
                                                                 find_matches_for_component,
                                                                 get_components_to_assemble,
                                                                 get_component_for_frame_name,
                                                                 check_object_exists,
                                                                 get_assembly_and_target_frames,
                                                                 get_list_of_components,
                                                                 is_component_stationary,
                                                                 is_component_assembled,
                                                                 get_global_statonary_component)

from pm_skills.py_modules.PmRobotUtils import PmRobotUtils, PmRobotError

class PmSkills(Node):
    
    PM_ROBOT_GRIPPER_FRAME = 'PM_Robot_Tool_TCP'
    PM_ROBOT_GONIO_LEFT_FRAME_INDICATOR = 'Gonio_Left_Part'
    PM_ROBOT_GONIO_RIGHT_FRAME_INDICATOR = 'Gonio_Right_Part'
    GRIPPING_FRAME_IDENTIFICATORS = ['Grip', 'grip']
    GRIP_RELATIVE_LIFT_DISTANCE = 0.05
    RELEASE_LIFT_DISTANCE = 0.05
    GRIP_APPROACH_OFFSET = 0.05
    #GRIP_SENSING_START_OFFSET = 0.0005
    GRIP_SENSING_START_OFFSET = 0.0005
    GRIP_SENSING_END_OFFSET = -0.0005
    ASSEMBLY_FRAME_INDICATOR = 'assembly_frame_Description'
    TARGET_FRAME_INDICATOR = 'target_frame_Description'
    GRIPPING_OFFSET = 0.0001
    
    def __init__(self) -> None:
        super().__init__('pm_skills')

        self.callback_group_me = MutuallyExclusiveCallbackGroup()
        self.callback_group_re = ReentrantCallbackGroup()
        self.pm_robot_utils = PmRobotUtils(self)
        self.pm_robot_utils.start_object_scene_subscribtion()

        # services
        self.grip_component_srv = self.create_service(pm_skill_srv.GripComponent, "pm_skills/grip_component", self.grip_component_callback,callback_group=self.callback_group_me)
        self.force_grip_component_srv = self.create_service(pm_skill_srv.GripComponent, "pm_skills/force_grip_component", self.force_grip_component_callback,callback_group=self.callback_group_me)

        self.place_component_srv = self.create_service(pm_skill_srv.PlaceComponent, "pm_skills/place_component", self.place_component_callback,callback_group=self.callback_group_me)
        self.release_component_srv = self.create_service(EmptyWithSuccess, "pm_skills/release_component", self.release_component_callback,callback_group=self.callback_group_me)

        self.assemble_srv  = self.create_service(EmptyWithSuccess, "pm_skills/assemble", self.assemble_callback,callback_group=self.callback_group_me)
        
        self.vacuum_gripper_on_service = self.create_service(EmptyWithSuccess, "pm_skills/vacuum_gripper/vacuum_on", self.vaccum_gripper_on_callback, callback_group=self.callback_group_me)
        self.vacuum_gripper_off_service = self.create_service(EmptyWithSuccess, "pm_skills/vacuum_gripper/vacuum_off", self.vaccum_gripper_off_callback, callback_group=self.callback_group_me)
    
        self.dispenser_service = self.create_service(pm_skill_srv.DispenseAdhesive, "pm_skills/dispense_adhesive", self.dispenser_callback)
        self.confocal_laser_service = self.create_service(pm_skill_srv.ConfocalLaser, "pm_skills/confocal_laser", self.confocal_laser_callback)
        self.vision_service = self.create_service(pm_skill_srv.ExecuteVision, "pm_skills/execute_vision", self.vision_callback)

        self.measue_with_laser_srv = self.create_service(pm_skill_srv.CorrectFrame, "pm_skills/measure_with_laser", self.measure_with_laser_callback, callback_group=self.callback_group_me)
        self.correct_frame_with_laser_srv = self.create_service(pm_skill_srv.CorrectFrame, "pm_skills/correct_frame_with_laser", self.correct_frame_with_laser, callback_group=self.callback_group_me)
        self.force_sensing_move_srv = self.create_service(pm_msg_srv.GripperForceMove, self.get_name()+'/gripper_force_sensing', self.force_sensing_move_callback, callback_group=self.callback_group_me)
        
        self.measure_frame_with_confocal_bottom_srv = self.create_service(pm_skill_srv.CorrectFrame, "pm_skills/measure_frame_with_confocal_bottom", self.measure_frame_with_confocal_bottom, callback_group=self.callback_group_me)
        self.correct_frame_with_confocal_bottom_srv = self.create_service(pm_skill_srv.CorrectFrame, "pm_skills/correct_frame_with_confocal_bottom", self.correct_frame_with_confocal_bottom, callback_group=self.callback_group_me)

        self.measure_frame_with_confocal_top_srv = self.create_service(pm_skill_srv.CorrectFrame, "pm_skills/measure_frame_with_confocal_top", self.measure_frame_with_confocal_top, callback_group=self.callback_group_me)
        self.correct_frame_with_confocal_top_srv = self.create_service(pm_skill_srv.CorrectFrame, "pm_skills/correct_frame_with_confocal_top", self.correct_frame_with_confocal_top, callback_group=self.callback_group_me)

        self.srv_iter_align_gonio_right = self.create_service(pm_skill_srv.IterativeGonioAlign, self.get_name()+'/iterative_align_gonio_right', self.iterative_align_gonio_right, callback_group=self.callback_group_me)
        self.srv_iter_align_gonio_left = self.create_service(pm_skill_srv.IterativeGonioAlign, self.get_name()+'/iterative_align_gonio_left', self.iterative_align_gonio_left, callback_group=self.callback_group_me)

        # clienets
        self.attach_component = self.create_client(ami_srv.ChangeParentFrame, '/assembly_manager/change_obj_parent_frame')
        self.move_robot_tool_client = self.create_client(MoveToFrame, '/pm_moveit_server/move_tool_to_frame')
        self.move_robot_tool_relative = self.create_client(MoveRelative, '/pm_moveit_server/move_tool_relative')
        self.attach_component = self.create_client(ami_srv.ChangeParentFrame, '/assembly_manager/change_obj_parent_frame')
        self.align_gonio_right_client = self.create_client(pm_moveit_srv.AlignGonio, '/pm_moveit_server/align_gonio_right')
        self.align_gonio_left_client = self.create_client(pm_moveit_srv.AlignGonio, '/pm_moveit_server/align_gonio_left')
        
        self.vacuum_gripper_on_client = self.create_client(EmptyWithSuccess, '/pm_nozzle_controller/Head_Nozzle/Vacuum')
        self.vacuum_gripper_off_client = self.create_client(EmptyWithSuccess, '/pm_nozzle_controller/Head_Nozzle/TurnOff')
        self.vacuum_gripper_pressure_client = self.create_client(EmptyWithSuccess, '/pm_nozzle_controller/Head_Nozzle/Pressure')
        self.uv_cure_clinet = self.create_client(UVCuringSkill, '/pm_robot_primitive_skills/uv_curing')
        
        self.simtime_subscriber = self.create_subscription(std_msg.Bool, '/sim_time', self.simtime_callback, 10, callback_group=self.callback_group_re)

        self.adapt_frame_client = self.create_client(ami_srv.ModifyPoseAbsolut, '/assembly_manager/modify_frame_absolut')

        self.logger = self.get_logger()
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
    
    
    def force_sensing_move_callback(self, request:pm_msg_srv.GripperForceMove.Request, response:pm_msg_srv.GripperForceMove.Response):

        self.get_logger().info('Received ForceSensingMove request.')

        self.pm_robot_utils.set_force_sensor_bias()
        time.sleep(2)
        # Validate the request parameters. If any max force is > than 10, set threshold_exceeded to True and return failure.
        threshold_value = 10.0  # N
        max_step_size = 100  # micrometers
        max_steps = 1000  # maximum number of steps


        if abs(request.max_f_xyz[0]) > threshold_value or abs(request.max_f_xyz[1]) > threshold_value or abs(request.max_f_xyz[2]) > threshold_value:
            self.get_logger().error('Max force exceeded the threshold of 10N.')
            response.success = False
            response.error = 'Max force exceeded the threshold of 10N.'
            return response
        
        if request.step_size > max_step_size:
            self.get_logger().error(f'Step size {request.step_size} micrometers exceeds the maximum allowed step size of {max_step_size} micrometers.')
            response.success = False
            response.error = f'Step size exceeds the maximum allowed step size of {max_step_size} micrometers.'
            return response

        step_size = request.step_size*1e-6  # Convert step size from micrometers to meters
        # start_x = request.initial_joints_xyzt[0]
        # start_y = request.initial_joints_xyzt[1]
        # start_z = request.initial_joints_xyzt[2]

        start_x = self.pm_robot_utils.get_current_joint_state(self.pm_robot_utils.X_Axis_JOINT_NAME)
        start_y = self.pm_robot_utils.get_current_joint_state(self.pm_robot_utils.Y_Axis_JOINT_NAME)
        start_z = self.pm_robot_utils.get_current_joint_state(self.pm_robot_utils.Z_Axis_JOINT_NAME)

        target_x = request.target_joints_xyz[0]
        target_y = request.target_joints_xyz[1]
        target_z = request.target_joints_xyz[2]
        current_position = [start_x, start_y, start_z]
        target_position = [target_x, target_y, target_z]

        length_x = target_x - start_x
        length_y = target_y - start_y
        length_z = target_z- start_z
        # Calculate the length of the vector from start to target position
        length = math.sqrt(length_x**2 + length_y**2 + length_z**2)

        if (length/ step_size) > max_steps:
            self.get_logger().error(f'The distance to the target position is too large. The maximum number of steps is {max_steps}.')
            response.success = False
            response.error = f'The distance to the target position is too large. The maximum number of steps is {max_steps}.'
            return response

        self.get_logger().info(f'Current force sensor data: {self.pm_robot_utils._current_force_sensor_data.data}')

        step_size_x = step_size * length_x / length
        step_size_y = step_size * length_y / length
        step_size_z = step_size * length_z / length

        while current_position != target_position:
            # Check if the force sensor data exceeds the thresholds
            for i, (force, max_force, axis) in enumerate(zip(self.pm_robot_utils._current_force_sensor_data.data, [abs(request.max_f_xyz[0]), abs(request.max_f_xyz[1]), abs(request.max_f_xyz[2])], ['X', 'Y', 'Z'])):
                if abs(force) > max_force:
                    self.get_logger().warn(f'Force in {axis} direction exceeded threshold: {force} > {max_force}')
                    response.success = True
                    response.completed = True
                    return response

            step_target = [
                current_position[0] + step_size_x,
                current_position[1] + step_size_y,
                current_position[2] + step_size_z,
            ]

            self.get_logger().info(f'Moving to position: {step_target}')

            # Move to the next step position
            success = self.pm_robot_utils.send_xyz_trajectory_goal_absolut(
                step_target[0],
                step_target[1],
                step_target[2],
                time=1.0
            )

            if not success:
                self.get_logger().error('Failed to move to the next position.')
                response.success = False
                response.error = 'Failed to move to the next position.'
                return response

            # Update the current position
            current_position = step_target

            # calculate the distance to the target position
            distance_to_target = math.sqrt(
                (target_position[0] - current_position[0])**2 +
                (target_position[1] - current_position[1])**2 +
                (target_position[2] - current_position[2])**2
            )
            if distance_to_target < step_size:
                self.get_logger().info('Reached the target position.')
                break

        self.get_logger().info('Target position reached. Nothing found.')
        response.success = True
        response.completed = False
        response.error = 'Target position reached. Nothing found.'
        return response


    def iterative_align_gonio_right(self, request: pm_skill_srv.IterativeGonioAlign.Request, response:pm_skill_srv.IterativeGonioAlign.Response):
        try:

            # the request never changes
            align_request = pm_moveit_srv.AlignGonio.Request()
            align_request.execute_movement = True
            align_request.endeffector_frame_override = request.gonio_endeffector_frame
            align_request.target_frame = request.target_alignment_frame

            iterations = request.num_iterations

            for iter in range(iterations):
                self.logger.info(f"STARTING RUN '{iter}/{iterations}")
            
                for frame in request.frames_to_measure:
                    self.logger.info(f"Measuring frame '{frame}'")
                    measure_request = pm_skill_srv.CorrectFrame.Request()
                    measure_response = pm_skill_srv.CorrectFrame.Response()
                    measure_request.frame_name = frame
                    measure_request.use_iterative_sensing = True

                    measure_response = self.correct_frame_with_laser(measure_request, measure_response)

                    if not measure_response.success:
                        raise PmRobotError(f"Correcting frame '{frame} failed!")

                # calculating the angle difference
                try:
                    transform: Transform = self.pm_robot_utils.get_transform_for_frame(request.gonio_endeffector_frame, 
                                                                            request.target_alignment_frame)
                    
                    current_gonio_angles: Transform = self.pm_robot_utils.get_transform_for_frame(request.gonio_endeffector_frame, 
                                                                                                    'world')
                except ValueError as e:
                    self.logger.error(f"Error: {e}")
                    self.logger.error(f"Frame '{frame}' does not seem to exist.")

                angles = R.from_quat([transform.rotation.x,
                                    transform.rotation.y,
                                    transform.rotation.z,
                                    transform.rotation.w]
                                    ).as_euler('xyz', degrees=True)

                angles_current = R.from_quat([current_gonio_angles.rotation.x,
                                                current_gonio_angles.rotation.y,
                                                current_gonio_angles.rotation.z,
                                                current_gonio_angles.rotation.w]
                                            ).as_euler('xyz', degrees=True)

                self.logger.info(f"Initial deviation at {iter} - x: {angles[0]}, y: {angles[1]}, z: {angles[2]}")
                self.logger.info(f"Current angles of the goniometer at {iter} - x: {angles_current[0]}, y: {angles_current[1]}, z: {angles_current[2]}")
                self.logger.info(f"Aligning goniometer!")

                #get joints
                joint_1_pre = self.pm_robot_utils.get_current_joint_state(self.pm_robot_utils.GONIO_RIGHT_STAGE_1)
                joint_2_pre = self.pm_robot_utils.get_current_joint_state(self.pm_robot_utils.GONIO_RIGHT_STAGE_2)

                if not self.pm_robot_utils.client_align_gonio_right.wait_for_service(1):
                    raise PmRobotError(f"Client '{self.pm_robot_utils.client_align_gonio_right.srv_name} not available!")
                
                align_response:pm_moveit_srv.AlignGonio.Response = self.pm_robot_utils.client_align_gonio_right.call(align_request)

                if not align_response.success:
                    raise PmRobotError(f"Aligning goniometer failed!")

                time.sleep(1)  # wait for the robot to settle

                joint_1_post = self.pm_robot_utils.get_current_joint_state(self.pm_robot_utils.GONIO_RIGHT_STAGE_1)
                joint_2_post = self.pm_robot_utils.get_current_joint_state(self.pm_robot_utils.GONIO_RIGHT_STAGE_2)

                difference_joint_1 = (joint_1_post - joint_1_pre)*180/np.pi
                difference_joint_2 = (joint_2_post - joint_2_pre)*180/np.pi

                self.logger.warn(f"Gonio joints moved by {round(difference_joint_1, 5)} and {round(difference_joint_2, 5)} (deg)")

            self.logger.info(f"Success")
            move_relative_request = MoveRelative.Request()
            move_relative_request.execute_movement = True
            move_relative_request.translation.z = 0.03


            move_relative_response: MoveRelative.Response = self.pm_robot_utils.client_move_laser_relative.call(move_relative_request)

            if not move_relative_response.success:
                raise PmRobotError(f"Endmove relative failed!")

            response.success = True

        except PmRobotError as e:
            self.logger.error(f"Error occurred: {e}")
            response.success = False

        return response

    def iterative_align_gonio_left(self, request: pm_skill_srv.IterativeGonioAlign.Request, response:pm_skill_srv.IterativeGonioAlign.Response):
        
        if not self.pm_robot_utils.client_align_gonio_left.wait_for_service(1):
            self.logger.error(f"Client '{self.pm_robot_utils.client_align_gonio_left.srv_name} not available!")
            response.success = False
            return response

        # the request never changes
        align_request = pm_moveit_srv.AlignGonio.Request()
        align_request.execute_movement = True
        align_request.endeffector_frame_override = request.gonio_endeffector_frame
        align_request.target_frame = request.target_alignment_frame

        iterations = request.num_iterations

        for iter in range(iterations):
            self.logger.info(f"STARTING RUN '{iter}/{iterations}")

            for frame in request.frames_to_measure:
                self.logger.info(f"Measuring frame '{frame}'")
                measure_request = pm_skill_srv.CorrectFrame.Request()
                measure_response = pm_skill_srv.CorrectFrame.Response()
                measure_request.frame_name = frame
                measure_request.use_iterative_sensing = True

                measure_response = self.correct_frame_with_laser(measure_request, measure_response)

                if not measure_response.success:
                    self.logger.error(f"Correcting frame '{frame} failed!")
                    response.success = False
                    return response
                
            # calculating the angle difference 
            try:
                transform: Transform = self.pm_robot_utils.get_transform_for_frame(request.gonio_endeffector_frame, 
                                                                        request.target_alignment_frame)
            except ValueError as e:
                self.logger.error(f"Error: {e}")
                self.logger.error(f"Frame '{frame}' does not seem to exist.")

            
            angles = R.from_quat([transform.rotation.x,
                                  transform.rotation.y,
                                  transform.rotation.z,
                                  transform.rotation.w]
                                 ).as_euler('xyz', degrees=True)

            self.logger.info(f"Initial deviation at {iter} - x: {angles[0]}, y: {angles[1]}, z: {angles[2]}")
            self.logger.info(f"Aligning goniometer!")

            joint_1_pre = self.pm_robot_utils.get_current_joint_state(self.pm_robot_utils.GONIO_LEFT_STAGE_1)
            joint_2_pre = self.pm_robot_utils.get_current_joint_state(self.pm_robot_utils.GONIO_LEFT_STAGE_2)

            align_response:pm_moveit_srv.AlignGonio.Response = self.pm_robot_utils.client_align_gonio_left.call(align_request)

            if not align_response.success:
                response.success = False
                return response
            
            time.sleep(1)  # wait for the robot to settle

            joint_1_post = self.pm_robot_utils.get_current_joint_state(self.pm_robot_utils.GONIO_LEFT_STAGE_1)
            joint_2_post = self.pm_robot_utils.get_current_joint_state(self.pm_robot_utils.GONIO_LEFT_STAGE_2)

            difference_joint_1 = (joint_1_post - joint_1_pre)*180/np.pi
            difference_joint_2 = (joint_2_post - joint_2_pre)*180/np.pi

            self.logger.warn(f"Gonio joints moved by {round(difference_joint_1,5)} and {round(difference_joint_2,5)} (deg)")

        self.logger.info(f"Success")
        move_relative_request = MoveRelative.Request()
        move_relative_request.execute_movement = True
        move_relative_request.translation.z = 0.03

        move_relative_response: MoveRelative.Response = self.pm_robot_utils.client_move_laser_relative.call(move_relative_request)

        if not move_relative_response.success:
            self.logger.error(f"Endmove relative failed!")
            response.success = False
            return response
        
        response.success = True

        return response

    def force_grip_component_callback(self, request:pm_skill_srv.GripComponent.Request, response:pm_skill_srv.GripComponent.Response):
        """TO DO: Add docstring"""
        self.pm_robot_utils.wait_for_initial_scene_update()
        try:
            if not check_object_exists(self.pm_robot_utils.object_scene, request.component_name):
                raise PmRobotError(f"Object '{request.component_name}' does not exist!")
            
            if not self.is_gripper_empthy():
                raise PmRobotError("Gripper is not empty! Can not grip new component!")

            gripping_frame = self.get_gripping_frame(request.component_name)

            if gripping_frame is None:
                raise PmRobotError(f"No gripping frame found for object '{request.component_name}'")
            
            self.logger.info(f"Gripping component '{request.component_name}' at frame '{gripping_frame}'")

            if self.is_object_on_gonio_left(request.component_name):
                algin_success = self.align_gonio_left(gripping_frame)

            elif self.is_object_on_gonio_right(request.component_name):
                algin_success = self.align_gonio_right(gripping_frame)
            
            else:
                raise PmRobotError("Object not on gonio!")

            if not algin_success:
                raise PmRobotError(f"Failed to align gonio for component '{request.component_name}'")

            #Offset
            move_tool_to_part_offset_success = self.move_gripper_to_frame(gripping_frame, z_offset=self.GRIP_APPROACH_OFFSET)

            if not move_tool_to_part_offset_success:
                raise PmRobotError(f"Failed to move gripper to part '{request.component_name}' at offset distance!")

            self.logger.info(f"Moving to grip sensing start position!")

            move_tool_to_part_success = self.move_gripper_to_frame(gripping_frame, z_offset=self.GRIP_SENSING_START_OFFSET)

            if not move_tool_to_part_success:
                raise PmRobotError(f"Failed to move gripper to part '{request.component_name}'!")

            #tip_name = self.pm_robot_utils.pm_robot_config.tool.get_tool().get_current_tool_attachment()

            tip_name = 'PM_Robot_Vacuum_Tool_Tip'

            col_success = self.pm_robot_utils.set_collision(request.component_name, 
                                                            tip_name,
                                                            False)
            
            if not col_success:
                raise PmRobotError(f"Could not disable collision between '{tip_name}' and '{request.component_name}'!")

            # calculating the lower position
            planning_req = pm_moveit_srv.MoveToFrame.Request()
            planning_req.target_frame = gripping_frame
            planning_req.execute_movement = False
            planning_req.translation.z = self.GRIP_SENSING_END_OFFSET

            planning_res:pm_moveit_srv.MoveToFrame.Response = self.move_robot_tool_client.call(planning_req)
            
            if planning_res.success is False:
                raise PmRobotError(f"Planning of END_OFFSET position failed!")

            self.logger.warn(f"joints {str(planning_res.joint_names)}")
            self.logger.warn(f"joints {str(planning_res.joint_values)}")

            # Iterative approach  
            force_sensing_request = pm_msg_srv.GripperForceMove.Request()
            force_sensing_response = pm_msg_srv.GripperForceMove.Response()

            force_sensing_request.step_size = float(50) # in um
            force_sensing_request.target_joints_xyz = [planning_res.joint_values[0], planning_res.joint_values[1], planning_res.joint_values[2]]
            force_sensing_request.max_f_xyz = [1.0, 1.0, 1.0]

            force_sensing_response = self.force_sensing_move_callback(force_sensing_request, force_sensing_response)
            
            if not force_sensing_response.success:
                raise PmRobotError(f"Force sensing failed!")
            
            # # enable vacuum
            enable_success = self.pm_robot_utils.set_tool_vaccum(True)

            if not enable_success:
                raise PmRobotError(f"Failed to enable vacuum for component '{request.component_name}'!")

            disable_success = True

            # turn off the vacuum
            if self.is_object_on_gonio_left(request.component_name, max_depth=1):
                self.logger.info(f"Disabling vacuum on gonio left for component '{request.component_name}'")
                disable_success = self.pm_robot_utils.set_gonio_left_vacuum(False)

            elif self.is_object_on_gonio_right(request.component_name, max_depth=1):
                self.logger.info(f"Disabling vacuum on gonio right for component '{request.component_name}'")
                disable_success = self.pm_robot_utils.set_gonio_right_vacuum(False)
            else:
                self.logger.info(f"Component '{request.component_name}' is not on a gonio!")

            if not disable_success:
                raise PmRobotError(f"Failed to disable vacuum for component '{request.component_name}'!")

            # attach the component to the gripper
            attach_component_success = self.attach_component_to_gripper(request.component_name)
            
            if not attach_component_success:
                raise PmRobotError(f"Failed to attach component '{request.component_name}' to gripper")
            
            response.success = True

            response.message = f"Component '{request.component_name}' gripped successfully!"
            self.logger.info(response.message)

        except PmRobotError as e:
            response.success = False
            response.message = str(e)
            self.logger.error(response.message)

        finally:
            move_relatively_success = self.lift_gripper_relative(self.GRIP_RELATIVE_LIFT_DISTANCE)
            if not move_relatively_success:
                response.success = False
                response.message = f"Failed to lift gripper after attaching component '{request.component_name}'"
                self.logger.error(response.message)

        return response
    
    def grip_component_callback(self, request:pm_skill_srv.GripComponent.Request, response:pm_skill_srv.GripComponent.Response):
        """TO DO: Add docstring"""
       
        try:
            if not check_object_exists(request.component_name):
                response.success = False
                response.message = f"Object '{request.component_name}' does not exist!"
                self.logger.error(response.message)
                return response
            
            if not self.is_gripper_empthy():
                response.success = False
                response.message = "Gripper is not empty! Can not grip new component!"
                self.logger.error(response.message)
                return response
            
        except ValueError as e:
            response.success = False
            response.message = str(e)
            self.logger.error(response.message)
            return response
        
        gripping_frame = self.get_gripping_frame(request.component_name)

        if gripping_frame is None:
            response.success = False
            response.message = f"No gripping frame found for object '{request.component_name}'"
            self.logger.error(response.message)
            return response
        
        self.logger.info(f"Gripping component '{request.component_name}' at frame '{gripping_frame}'")
        
        if self.is_object_on_gonio_left(request.component_name):
            algin_success = self.align_gonio_left(gripping_frame)

        elif self.is_object_on_gonio_right(request.component_name):
            algin_success = self.align_gonio_right(gripping_frame)
        
        else:
            response.success = False
            response.message = "Object not on gonio!"
            self.logger.error("Object not on gonio!")
            return response
        
        if not algin_success:
            response.success = False
            response.message = f"Failed to align gonio for component '{request.component_name}'"
            self.logger.error(response.message)
            return response
        
        #Offset
        move_tool_to_part_offset_success = self.move_gripper_to_frame(gripping_frame, z_offset=self.RELEASE_LIFT_DISTANCE)

        if not move_tool_to_part_offset_success:
            response.success = False
            response.message = f"Failed to move gripper to part '{request.component_name}' at offset distance!"
            self.logger.error(response.message)
            return response
        
        move_tool_to_part_success = self.move_gripper_to_frame(gripping_frame,z_offset=self.GRIPPING_OFFSET)

        if not move_tool_to_part_success:
            response.success = False
            response.message = f"Failed to move gripper to part '{request.component_name}'!"
            self.logger.error(response.message)
            return response
        
        attach_component_success = self.attach_component_to_gripper(request.component_name)

        if not attach_component_success:
            move_relatively_success = self.lift_gripper_relative(self.GRIP_RELATIVE_LIFT_DISTANCE)
            response.success = False
            response.message = f"Failed to attach component '{request.component_name}' to gripper"
            self.logger.error(response.message)
            return response
        
        move_relatively_success = self.lift_gripper_relative(self.GRIP_RELATIVE_LIFT_DISTANCE)

        if not move_relatively_success:
            response.success = False
            response.message = f"Failed to lift gripper after attaching component '{request.component_name}'"
            self.logger.error(response.message)
            return response
        
        self.logger.info(f"Move tool relative success: {move_relatively_success}")

        response.success = True

        response.message = f"Component '{request.component_name}' gripped successfully!"
        self.logger.info(response.message)

        return response

    def place_component_callback(self, request:pm_skill_srv.PlaceComponent.Request, response:pm_skill_srv.PlaceComponent.Response):
        
        try:
            should_move_up_at_error = False

            if self.is_gripper_empthy():
                raise PmRobotError(f"Gripper is empty! Can not place component!")
            
            gripped_component = self.get_gripped_component()
            
            if gripped_component is None:
                raise PmRobotError(f"No gripped component found!")
            
            gripped_component_frames = self.get_assembly_target_frame_gripped_component()

            self.logger.warn(f"Component frames '{str(gripped_component_frames)}'!")

            all_involved_frames_tuples = self.get_assembly_and_target_frames()

            target_frame = None

            for component_frame in gripped_component_frames:
                if self.ASSEMBLY_FRAME_INDICATOR in component_frame:
                    component_assembly_frame = component_frame
                    #strip indicator from frame name
                    assembly_description = component_frame.replace(self.ASSEMBLY_FRAME_INDICATOR, '')
                    self.logger.info(f"Placing component '{gripped_component}'")

                    for frame_tuple in all_involved_frames_tuples:
                        if assembly_description in frame_tuple[1] and frame_tuple[0] != gripped_component:
                            target_frame = frame_tuple[1]
                            break
                    
                    self.logger.info(f"Placing component '{gripped_component}' at frame '{target_frame}'")

            if target_frame is None:
                raise PmRobotError(f"No target frame found for component '{gripped_component}'")
            
            target_component = self.get_component_for_frame_name(target_frame)

            if target_component is None:
                raise PmRobotError(f"No component found for target frame '{target_frame}'")
            
            # RECALCULATE THE ASSEMBLY TRANSFORMATIONplace_component_callback
            # INSERT HERE

            # Align gonio to the left or right depending on the target frame
            if self.is_object_on_gonio_left(target_component):
                algin_success = self.align_gonio_left(endeffector_override=target_frame,
                                                        alignment_frame=component_assembly_frame)

            elif self.is_object_on_gonio_right(target_component):
                algin_success = self.align_gonio_right(endeffector_override=target_frame,
                                                        alignment_frame=component_assembly_frame)

            else:
                raise PmRobotError("Target object not on gonio!")

            if not algin_success:
                raise PmRobotError(f"Failed to align gonio for target component '{target_component}'")
            
            self.logger.warn("Endeffector override: " + component_assembly_frame)
            # Move component to the target frame with a z offset
            move_component_to_part_offset_success = self.move_gripper_to_frame(target_frame, 
                                                                               endeffector_override=component_assembly_frame, 
                                                                               z_offset=self.RELEASE_LIFT_DISTANCE)

            if not move_component_to_part_offset_success:
                raise PmRobotError(f"Failed to move gripper to target part '{target_component}'")

            should_move_up_at_error = True

            # Move component to the target frame
            move_component_to_part_success = self.move_gripper_to_frame(target_frame, 
                                                                        endeffector_override=component_assembly_frame,
                                                                        z_offset=0.0)
            
            if not move_component_to_part_success:
                raise PmRobotError(f"Failed to move gripper to target part '{target_component}'")
            
            response.success = True
            response.message = f"Component '{gripped_component}' placed successfully!"

        except PmRobotError as e:
            if should_move_up_at_error:
                move_relatively_success = self.lift_gripper_relative(self.RELEASE_LIFT_DISTANCE)
                if not move_relatively_success:
                    response.success = False
                    response.message = f"Failed to lift gripper after placing component '{gripped_component}'"
                    self.logger.error(response.message)
            response.success = False
            response.message = str(e)
            self.logger.error(response.message)

        finally:
            pass

        return response

    def release_component_callback(self, request:EmptyWithSuccess.Request, response:EmptyWithSuccess.Response):
        
        try:
            if self.is_gripper_empthy():
                raise PmRobotError(f"Gripper is empty! Can not place component!")
            
            gripped_component = self.get_gripped_component()
            
            if gripped_component is None:
                raise PmRobotError(f"No gripped component found!")
            
            gripped_component_frames = self.get_assembly_target_frame_gripped_component()

            self.logger.warn(f"Component frames '{str(gripped_component_frames)}'!")

            all_involved_frames_tuples = self.get_assembly_and_target_frames()

            target_frame = None

            for component_frame in gripped_component_frames:
                if self.ASSEMBLY_FRAME_INDICATOR in component_frame:
                    component_assembly_frame = component_frame
                    #strip indicator from frame name
                    assembly_description = component_frame.replace(self.ASSEMBLY_FRAME_INDICATOR, '')
                    self.logger.info(f"Placing component '{gripped_component}'")

                    for frame_tuple in all_involved_frames_tuples:
                        if assembly_description in frame_tuple[1] and frame_tuple[0] != gripped_component:
                            target_frame = frame_tuple[1]
                            break
                    
                    self.logger.info(f"Placing component '{gripped_component}' at frame '{target_frame}'")

            if target_frame is None:
                raise PmRobotError(f"No target frame found for component '{gripped_component}'")
            
            target_component = self.get_component_for_frame_name(target_frame)

            if target_component is None:
                raise PmRobotError(f"No component found for target frame '{target_frame}'")

            # release component
            attach_component_success = self.attach_component_to_component(gripped_component, target_component)

            if not attach_component_success:
                raise PmRobotError(f"Failed to attach component '{gripped_component}' to component '{target_component}'")
            
            vaccum_off_success = self.pm_robot_utils.set_tool_vaccum(False)

            if not vaccum_off_success:
                raise PmRobotError(f"Failed to deactivate vacuum for component '{gripped_component}'")

            #####
            # !!!! ACTIVATE COLLISION BETWEEN PART AND GRIPPER !!!!!
            #####

            move_relatively_success = self.lift_gripper_relative(self.RELEASE_LIFT_DISTANCE)
            if not move_relatively_success:
                raise PmRobotError(f"Failed to lift gripper after releasing component '{gripped_component}'")

            response.success = True
            response.message = f"Component '{gripped_component}' released successfully!"
                    
        except PmRobotError as e:
            response.success = False
            response.message = str(e)
            self.logger.error(response.message)

        return response

    def assemble_callback(self, request:EmptyWithSuccess.Request, response:EmptyWithSuccess.Response):
        
        global_stationary_component = self.get_global_statonary_component()
        assemble_list = self.get_components_to_assemble()
        first_component = self.find_matches_for_component(global_stationary_component, only_unassembled=False)

        self.assembly_loop(first_component)
        
        response.success = True

        return response

    def vaccum_gripper_on_callback(self, request:EmptyWithSuccess.Request, response:EmptyWithSuccess.Response):
        """Mimics the vacuum gripper funtionality by activating the vacuum and changing the parent frame"""

        sim_time = self.pm_robot_utils.is_gazebo_running()
        self.logger.info(f"Gazebo Simulation: {sim_time}")

        assembly_and_target_frames = self.get_assembly_and_target_frames()      

        try:
            if not self.is_gripper_empthy():
                response.success = False
                response.message = "Gripper is not empty! Can not grip new component!"
                self.logger.error(response.message)
                return response
            
            # get gripping component
            for component_frame in assembly_and_target_frames:
                if self.ASSEMBLY_FRAME_INDICATOR in component_frame[1]:
                    target_component = component_frame[0]
                    self.logger.info(f"moving component: '{target_component}'")
                    break
            
            # activate the vacuum at head_nozzle
            if not sim_time:
                response_vacuum:EmptyWithSuccess.Response = self.vacuum_gripper_on_client.call(EmptyWithSuccess.Request())
                if not response_vacuum.success:
                    response.success = False
                    response.message = "Failed to activate vacuum!"
                    self.logger.error(response.message)
                    return response

            response_attachment = self.attach_component_to_gripper(target_component)
            if not response_attachment:
                response.success = False
                response.message = "Failed change parent frame!"
                self.logger.error(response.message)
                return response

            response.success = True
            response.message = "Component gripped and attached to the gripper!" 
                
        except ValueError as e:
            response.success = False
            response.message = str(e)
            self.logger.error(response.message)
            return response
                
        return response

    def vaccum_gripper_off_callback(self, request:EmptyWithSuccess.Request, response:EmptyWithSuccess.Response):
        """Mimics the vacuum gripper funtionality by deactivating the vacuum and changing the parent frame"""

        sim_time = self.pm_robot_utils.is_gazebo_running()
        self.logger.info(f"Gazebo Simulation: {sim_time}")

        assembly_and_target_frames = self.get_assembly_and_target_frames()    

        try:
            if not sim_time:
                response_vacuum:EmptyWithSuccess.Response = self.vacuum_gripper_off_client.call(EmptyWithSuccess.Request())
                if not response_vacuum.success:
                    response.success = False
                    response.message = "Failed to deactivate vacuum!"
                    self.logger.error(response.message)
                    return response
            
            # get target component
            for component_frame in assembly_and_target_frames:
                if self.TARGET_FRAME_INDICATOR in component_frame[1]:
                    target_component = component_frame[0]
                    self.logger.info(f"target component: '{target_component}'")
                    break

            response_attachment = self.attach_component_to_component(self.get_gripped_component(), target_component)
            if not response_attachment:
                response.success = False
                response.message = "Failed change parent frame!"
                self.logger.error(response.message)
                return response
            
            response.success = True
            response.message = "Component released and attached to the target component!"   
                
        except ValueError as e:
            response.success = False
            response.message = str(e)
            self.logger.error(response.message)
            return response
                
        return response

    def dispenser_callback(self, request, response):
        """TO DO: Add docstring"""

        self.logger.info(f"Starting {request.dispenser}")

        # Simulating dispension time by adding delay
        wait_duration = request.dispense_time
        if wait_duration > 0.0:
            time.sleep(wait_duration)
            response.success = True
        else:
            self.logger.warn("Invaild dispense Duration!")
            response.success = False
            
        self.logger.info(f"Done {request.dispenser}")
        return response

    def confocal_laser_callback(self, request, response):
        """TO DO: Add docstring"""

        self.logger.info(f"Starting laser: {request.laser}")

        # Generating random number for meassurement    
        result = ((random.randrange(0, 6)) / 10.0) - 0.3

        response.result = str(f"{result} mm.")
        response.success = True

        return response
    
    def vision_callback(self, request, response):
        self.logger.info(f'Loading {request.process_filename} as process file.')
        self.logger.info(f'Starting process {request.process_uid}')
        self.logger.info(f"Loading {request.camera_config_filename} as camera configuration file.")

        if request.image_display_time > 0.0:
            self.logger.info('Image aquisiton is running')
            time.sleep(request.image_display_time)
            response.success = True
        else: 
            self.logger.warn('Invalid image display time!')
            response.success = False

        if request.run_cross_validation:
            self.logger.info("Running cross validtation.")

        response.results_dict = "Result dict"
        self.logger.info(f'Saving results ...')
        response.results_path = "Results path."
        response.points = [random.random()*100 for _ in range(4)]
        response.process_uid = str(random.randint(1000,50000))

        return response
            
    def assembly_loop(self, list_of_components:list[str]):
        
        for component in list_of_components:
            self.logger.error(f"ComponentTTTTT: {component}")
            if not self.is_component_assembled(component):

                response = self.grip_component_callback(pm_skill_srv.GripComponent.Request(component_name=component), pm_skill_srv.GripComponent.Response())
                if not response.success:
                    self.logger.error(f"Failed to grip component '{component}'")
                    return False
                
                response = self.place_component_callback(pm_skill_srv.PlaceComponent.Request(), pm_skill_srv.PlaceComponent.Response())
                if not response.success:
                    self.logger.error(f"Failed to place component '{component}'")
                    return False
                
            list_of_component= self.find_matches_for_component(component,only_unassembled=False)
            self.logger.warn(f"List of components: {list_of_component}")
            self.assembly_loop(list_of_component)

        return True
    
    def simtime_callback(self, msg:std_msg.Bool):
        if msg.data:
            self.logger.info("Simulation time is active!")
            self.sim_time = True
        else:
            self.logger.info("Simulation time is inactive!")
            self.sim_time = False

    def lift_gripper_relative(self, distance:float)-> bool:
        call_async = False

        if not self.move_robot_tool_relative.wait_for_service(timeout_sec=1.0):
            self.logger.error("Service '/pm_moveit_server/move_tool_relative' not available")
            return False
        
        req = pm_moveit_srv.MoveRelative.Request()
        req.translation.z = distance
        req.execute_movement = True

        if call_async:
            future = self.move_robot_tool_relative.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is None:
                self.logger.error('Service call failed %r' % (future.exception(),))
                return False
            return future.result().success
        else:
            response:pm_moveit_srv.MoveRelative.Response = self.move_robot_tool_relative.call(req)
            return response.success
        
    def move_gripper_to_frame(self, frame_name:str, endeffector_override = None, z_offset=None)-> bool:
        call_async = False

        if not self.move_robot_tool_client.wait_for_service(timeout_sec=1.0):
            self.logger.error("Service '/pm_moveit_server/move_tool_to_frame' not available")
            return False
        
        req = pm_moveit_srv.MoveToFrame.Request()
        req.target_frame = frame_name
        req.execute_movement = True
        req.translation.z = 0.0000001

        if endeffector_override is not None:
            req.endeffector_frame_override = endeffector_override

        if z_offset is not None:
            req.translation.z = z_offset

        if call_async:
            future = self.move_robot_tool_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is None:
                self.logger.error('Service call failed %r' % (future.exception(),))
                return False
            return future.result().success
        else:
            response:pm_moveit_srv.MoveToFrame.Response = self.move_robot_tool_client.call(req)
            return response.success

    def move_laser_to_frame(self, frame_name:str, z_offset=None)-> bool:
        """
        z_offset in m
        """
        call_async = False

        if not self.pm_robot_utils.client_move_robot_laser_to_frame.wait_for_service(timeout_sec=1.0):
            self.logger.error("Service '/pm_moveit_server/move_laser_to_frame' not available")
            return False
        
        req = pm_moveit_srv.MoveToFrame.Request()
        req.target_frame = frame_name
        req.execute_movement = True

        if z_offset is not None:
            req.translation.z = z_offset

        if call_async:
            future = self.pm_robot_utils.client_move_robot_laser_to_frame.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is None:
                self.logger.error('Service call failed %r' % (future.exception(),))
                return False
            return future.result().success
        else:
            response:pm_moveit_srv.MoveToFrame.Response = self.pm_robot_utils.client_move_robot_laser_to_frame.call(req)
            return response.success
    
      
    def measure_with_laser_callback(self, 
                                    request:pm_skill_srv.CorrectFrame.Request, 
                                    response:pm_skill_srv.CorrectFrame.Response):
        try:
            move_laser_to_frame_success = self.move_laser_to_frame(request.frame_name)
            
            offset = 0.0
            
            if not move_laser_to_frame_success:
                response.success = False
                response.message = f"Failed to move laser to frame '{request.frame_name}'"
                self.logger.error(response.message)
                return response
            
            if not self.pm_robot_utils._check_for_valid_laser_measurement():

                if request.use_iterative_sensing:

                    time.sleep(1)

                    initial_z = self.pm_robot_utils.get_current_joint_state(PmRobotUtils.Z_Axis_JOINT_NAME)

                    time.sleep(1)

                    # move up
                    self._logger.warn(f"MOVING UP")
                    move_success = self.pm_robot_utils.send_xyz_trajectory_goal_relative(0, 0, -3.0*1e-3,time=1)
                                                    
                    if not move_success:
                            response.success = False
                            return response
                    
                    step_inc = 0.4 # in mm
                    self._logger.warn(f"Laser measurement not valid! Trying to iteratively find a valid value!")                

                    x, y, final_z = self.pm_robot_utils.interative_sensing(measurement_method=self.pm_robot_utils.get_laser_measurement,
                                                    measurement_valid_function = self.pm_robot_utils._check_for_valid_laser_measurement,
                                                    length = (0.0, 0.0, 4.0),
                                                    step_inc = step_inc,
                                                    total_time = 8.0)
                    
                    if x is None:
                        response.success = False
                        self._logger.warn(f"Laser measurement not valid! OUT OF RANGE")
                        return response
                    
                    offset = initial_z - final_z
                    self._logger.info(f"Found valid value at: {offset} m")

                else:
                    response.success = False
                    self._logger.warn(f"Laser measurement not valid! OUT OF RANGE")
                    return response

                self._logger.info(f"Valid value found!")      
            
            laser_measurement = self.pm_robot_utils.get_laser_measurement(unit="m") + float(offset)
            
            self._logger.info(f"Laser measurement: {laser_measurement} m ")

            response.correction_values.z = laser_measurement
            response.success = True
            response.message = f"Measurement: {laser_measurement}"

        except PmRobotError as e:
            response.success = False
            response.message = str(e)
            self._logger.error(response.message)

        return response

    def correct_frame_with_laser(self, request:pm_skill_srv.CorrectFrame.Request, response:pm_skill_srv.CorrectFrame.Response):
        
        self.pm_robot_utils.wait_for_initial_scene_update()
        
        _obj_name, _frame_name =  is_frame_from_scene(self.pm_robot_utils.object_scene, request.frame_name)

        #self._logger.warn(f"Correcting frame: {request.frame_name}...")
        #self._logger.warn(f"Object name: {_obj_name}")
        #self._logger.warn(f"Frame name: {_frame_name}")

        if _frame_name is not None:
            frame_from_scene = True
        else:
            frame_from_scene = False

        measure_frame_request = pm_skill_srv.CorrectFrame.Request()
        measure_frame_response = pm_skill_srv.CorrectFrame.Response()
        
        measure_frame_request.frame_name = request.frame_name
        measure_frame_request.remeasure_after_correction = request.remeasure_after_correction
        measure_frame_request.use_iterative_sensing = request.use_iterative_sensing
        
        response_mes:pm_skill_srv.CorrectFrame.Response = self.measure_with_laser_callback(measure_frame_request, measure_frame_response)
        
        #self._logger.warn(f"REs {str(response_mes)}")

        if not response_mes.success:
            response.success = False
            return response
                
        world_pose:TransformStamped = get_transform_for_frame_in_world(request.frame_name, self.tf_buffer, self._logger)

        world_pose.transform.translation.z += response_mes.correction_values.z

        adapt_frame_request = ami_srv.ModifyPoseAbsolut.Request()
        adapt_frame_request.frame_name = request.frame_name
        adapt_frame_request.pose.position.x = world_pose.transform.translation.x
        adapt_frame_request.pose.position.y = world_pose.transform.translation.y
        adapt_frame_request.pose.position.z = world_pose.transform.translation.z
        adapt_frame_request.pose.orientation = world_pose.transform.rotation

        #self._logger.warn(f"Check3")
        
        if frame_from_scene:
            if not self.adapt_frame_client.wait_for_service(timeout_sec=1.0):
                self._logger.error(f"Service '{self.adapt_frame_client.srv_name}' not available. Assembly manager started?...")
                response.success= False
                return response
            
            result_adapt:ami_srv.ModifyPoseAbsolut.Response = self.adapt_frame_client.call(adapt_frame_request)
        else:
            result_adapt = ami_srv.ModifyPoseAbsolut.Response()

        #self._logger.warn(f"Check4")
        
        response.success = result_adapt.success
        
        return response
    
    def measure_frame_with_confocal_bottom(self, request:pm_skill_srv.CorrectFrame.Request, response:pm_skill_srv.CorrectFrame.Response):
        
        tcp_name = self.pm_robot_utils.TCP_CONFOCAL_BOTTOM  # we need to move the frame attached to the robot to the tcp. We use the move camera method for that

        move_frame_to_confocal_bottom_success = self.pm_robot_utils.move_camera_top_to_frame(   frame_name = tcp_name,
                                                                                                endeffector_override=request.frame_name)
        
        if not move_frame_to_confocal_bottom_success:
            response.success = False
            response.message = f"Failed to move frame '{request.frame_name}' to confocal bottom!" 
            self.logger.error(response.message)
            return response
        
        time.sleep(1)

        if not self.pm_robot_utils.check_confocal_bottom_measurement_in_range():
            response.success = False
            self._logger.warn(f"Confocal bottom measurement not valid! OUT OF RANGE")
            return response
        
        confocal_measurement = self.pm_robot_utils.get_confocal_bottom_measurement(unit="m")
        
        self._logger.info(f"Measurement: {confocal_measurement*1e6} um ")

        response.correction_values.z = confocal_measurement
        response.success = True
        response.message = f"Measurement: {confocal_measurement*1e6} um"

        return response

    def correct_frame_with_confocal_bottom(self, request:pm_skill_srv.CorrectFrame.Request, response:pm_skill_srv.CorrectFrame.Response):
        
        self.pm_robot_utils.wait_for_initial_scene_update()
        
        _obj_name, _frame_name =  is_frame_from_scene(self.pm_robot_utils.object_scene, request.frame_name)

        #self._logger.warn(f"Correcting frame: {request.frame_name}...")
        #self._logger.warn(f"Object name: {_obj_name}")
        #self._logger.warn(f"Frame name: {_frame_name}")

        if _frame_name is not None:
            frame_from_scene = True
        else:
            frame_from_scene = False

        measure_frame_request = pm_skill_srv.CorrectFrame.Request()
        measure_frame_response = pm_skill_srv.CorrectFrame.Response()
        
        measure_frame_request.frame_name = request.frame_name
        measure_frame_request.remeasure_after_correction = request.remeasure_after_correction
        measure_frame_request.use_iterative_sensing = request.use_iterative_sensing
        
        response_mes:pm_skill_srv.CorrectFrame.Response = self.measure_frame_with_confocal_bottom(measure_frame_request, measure_frame_response)
        
        #self._logger.warn(f"REs {str(response_mes)}")

        if not response_mes.success:
            response.success = False
            return response
                
        world_pose:TransformStamped = get_transform_for_frame_in_world(request.frame_name, self.tf_buffer, self._logger)

        world_pose.transform.translation.z += response_mes.correction_values.z

        adapt_frame_request = ami_srv.ModifyPoseAbsolut.Request()
        adapt_frame_request.frame_name = request.frame_name
        adapt_frame_request.pose.position.x = world_pose.transform.translation.x
        adapt_frame_request.pose.position.y = world_pose.transform.translation.y
        adapt_frame_request.pose.position.z = world_pose.transform.translation.z
        adapt_frame_request.pose.orientation = world_pose.transform.rotation
        
        if frame_from_scene:
            if not self.adapt_frame_client.wait_for_service(timeout_sec=1.0):
                self._logger.error(f"Service '{self.adapt_frame_client.srv_name}' not available. Assembly manager started?...")
                response.success= False
                return response
            
            result_adapt:ami_srv.ModifyPoseAbsolut.Response = self.adapt_frame_client.call(adapt_frame_request)
        else:
            result_adapt = ami_srv.ModifyPoseAbsolut.Response()

        response.success = result_adapt.success
        response.correction_values.z = response_mes.correction_values.z
        
        return response

    def measure_frame_with_confocal_top(self, request:pm_skill_srv.CorrectFrame.Request, response:pm_skill_srv.CorrectFrame.Response):

        move_confocal_top_to_frame_success = self.pm_robot_utils.move_confocal_top_to_frame(request.frame_name)
        
        offset = 0.0
        
        if not move_confocal_top_to_frame_success:
            response.success = False
            response.message = f"Failed to move laser to frame '{request.frame_name}'"
            self.logger.error(response.message)
            return response
        
        if not self.pm_robot_utils.check_confocal_top_measurement_in_range():
            response.success = False
            self._logger.warn(f"Confocal top measurement not valid! OUT OF RANGE")
            return response
        
        confocal_measurement = self.pm_robot_utils.get_confocal_top_measurement(unit="m")
        
        self._logger.info(f"Laser measurement: {confocal_measurement} m ")

        response.correction_values.z = confocal_measurement
        response.success = True
        response.message = f"Measurement: {confocal_measurement}"
        return response

    def correct_frame_with_confocal_top(self, request:pm_skill_srv.CorrectFrame.Request, response:pm_skill_srv.CorrectFrame.Response):
        self.pm_robot_utils.wait_for_initial_scene_update()
        
        _obj_name, _frame_name =  is_frame_from_scene(self.pm_robot_utils.object_scene, request.frame_name)

        self._logger.warn(f"Correcting frame: {request.frame_name}...")
        self._logger.warn(f"Object name: {_obj_name}")
        self._logger.warn(f"Frame name: {_frame_name}")

        if _frame_name is not None:
            frame_from_scene = True
        else:
            frame_from_scene = False

        measure_frame_request = pm_skill_srv.CorrectFrame.Request()
        measure_frame_response = pm_skill_srv.CorrectFrame.Response()
        
        measure_frame_request.frame_name = request.frame_name
        measure_frame_request.remeasure_after_correction = request.remeasure_after_correction
        measure_frame_request.use_iterative_sensing = request.use_iterative_sensing
        
        response_mes:pm_skill_srv.CorrectFrame.Response = self.measure_frame_with_confocal_top(measure_frame_request, measure_frame_response)
        
        self._logger.warn(f"REs {str(response_mes)}")

        if not response_mes.success:
            response.success = False
            return response
                
        world_pose:TransformStamped = get_transform_for_frame_in_world(request.frame_name, self.tf_buffer, self._logger)

        world_pose.transform.translation.z += response_mes.correction_values.z

        adapt_frame_request = ami_srv.ModifyPoseAbsolut.Request()
        adapt_frame_request.frame_name = request.frame_name
        adapt_frame_request.pose.position.x = world_pose.transform.translation.x
        adapt_frame_request.pose.position.y = world_pose.transform.translation.y
        adapt_frame_request.pose.position.z = world_pose.transform.translation.z
        adapt_frame_request.pose.orientation = world_pose.transform.rotation
        
        if frame_from_scene:
            if not self.adapt_frame_client.wait_for_service(timeout_sec=1.0):
                self._logger.error(f"Service '{self.adapt_frame_client.srv_name}' not available. Assembly manager started?...")
                response.success= False
                return response
            
            result_adapt:ami_srv.ModifyPoseAbsolut.Response = self.adapt_frame_client.call(adapt_frame_request)
        else:
            result_adapt = ami_srv.ModifyPoseAbsolut.Response()
            self._logger.error(f"The frame '{request.frame_name}' could not be corrected, as it is not from the assembly scene.")
        
        response.success = result_adapt.success
        response.correction_values.z = response_mes.correction_values.z
        
        return response
        
    def attach_component_to_gripper(self, component_name:str)-> bool:
        call_async = False

        if not self.attach_component.wait_for_service(timeout_sec=1.0):
            self.logger.error("Service '/assembly_manager/change_obj_parent_frame' not available")
            return False
        
        req = ami_srv.ChangeParentFrame.Request()
        req.obj_name = component_name
        req.new_parent_frame = self.PM_ROBOT_GRIPPER_FRAME

        if call_async:
            future = self.attach_component.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is None:
                self.logger.error('Service call failed %r' % (future.exception(),))
                return False
            return future.result().success
        else:
            response:ami_srv.ChangeParentFrame.Response = self.attach_component.call(req)
            return response.success
        
    def attach_component_to_component(self, component_name:str, parent_component_name:str)-> bool:
        call_async = False

        if not self.attach_component.wait_for_service(timeout_sec=1.0):
            self.logger.error("Service '/assembly_manager/change_obj_parent_frame' not available")
            return False
        
        req = ami_srv.ChangeParentFrame.Request()
        req.obj_name = component_name
        req.new_parent_frame = parent_component_name

        if call_async:
            future = self.attach_component.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is None:
                self.logger.error('Service call failed %r' % (future.exception(),))
                return False
            return future.result().success
        else:
            response:ami_srv.ChangeParentFrame.Response = self.attach_component.call(req)
            return response.success

    def align_gonio_right(self, endeffector_override:str, alignment_frame:str = PM_ROBOT_GRIPPER_FRAME)-> bool:
        call_async = False

        if not self.align_gonio_right_client.wait_for_service(timeout_sec=1.0):
            self.logger.error("Service '/pm_moveit_server/align_gonio_right' not available")
            return False
        
        req = pm_moveit_srv.AlignGonio.Request()
        req.target_frame = alignment_frame
        req.execute_movement = True
        req.endeffector_frame_override = endeffector_override

        if call_async:
            future = self.align_gonio_right_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is None:
                self.logger.error('Service call failed %r' % (future.exception(),))
                return False
            return future.result().success
        else:
            response:pm_moveit_srv.AlignGonio.Response = self.align_gonio_right_client.call(req)
            return response.success
        
    def align_gonio_left(self, endeffector_override:str, alignment_frame:str = PM_ROBOT_GRIPPER_FRAME)-> bool:
        call_async = False

        if not self.align_gonio_left_client.wait_for_service(timeout_sec=1.0):
            self.logger.error("Service '/pm_moveit_server/align_gonio_left' not available")
            return False
        
        req = pm_moveit_srv.AlignGonio.Request()
        req.target_frame = alignment_frame
        req.execute_movement = True
        req.endeffector_frame_override = endeffector_override

        if call_async:
            future = self.align_gonio_left_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is None:
                self.logger.error('Service call failed %r' % (future.exception(),))
                return False
            return future.result().success
        else:
            response:pm_moveit_srv.AlignGonio.Response = self.align_gonio_left_client.call(req)
            return response.success
            
    def get_gripping_frame(self, object_name:str)-> str:
        grip_frames = []
        for obj in self.pm_robot_utils.object_scene.objects_in_scene:
            obj:ami_msg.Object
            if obj.obj_name == object_name:
                for frame in obj.ref_frames:
                    frame:ami_msg.RefFrame
                    for identificador in self.GRIPPING_FRAME_IDENTIFICATORS:
                        # check if string is part of string
                        if identificador in frame.frame_name:
                            grip_frames.append(frame.frame_name)

        if len(grip_frames) == 0:
            self.logger.error(f"No gripping frame found for object '{object_name}'")
            return None
        
        if len(grip_frames) > 1:
            self.logger.error(f"Multiple potential gripping frames found for object '{object_name}'. Identificators: {self.GRIPPING_FRAME_IDENTIFICATORS}")
            return None
        else:
            return grip_frames[0]
    
    # delete
    def get_parent_of_object(self, object_name:str):
        self.pm_robot_utils.wait_for_initial_scene_update()
        for obj in self.pm_robot_utils.object_scene.objects_in_scene:
            obj:ami_msg.Object
            if obj.obj_name == object_name:
                return obj.parent_frame
        return None
    
    def is_object_on_gonio_left(self, object_name: str, max_depth: int = 3) -> bool:
        """
        Check if the object is on the gonio left frame, up to a specified depth.

        :param object_name: Name of the object to check.
        :param max_depth: How many levels up the hierarchy to check.
        :return: True if the object is on or connected to the gonio left frame.
        """
        current_object = object_name

        for _ in range(max_depth):
            parent_frame = self.get_parent_of_object(current_object)
            if parent_frame is None:
                return False
            if self.PM_ROBOT_GONIO_LEFT_FRAME_INDICATOR in parent_frame:
                return True
            current_object = parent_frame

        return False

    def is_object_on_gonio_right(self, object_name: str, max_depth: int = 3) -> bool:
        """
        Check if the object is on the gonio right frame, up to a specified depth.
        
        :param object_name: Name of the object to check.
        :param max_depth: How many levels up the hierarchy to check.
        :return: True if the object is on or connected to the gonio right frame.
        """
        current_object = object_name

        for _ in range(max_depth):
            parent_frame = self.get_parent_of_object(current_object)
            if parent_frame is None:
                return False
            if self.PM_ROBOT_GONIO_RIGHT_FRAME_INDICATOR in parent_frame:
                return True
            current_object = parent_frame

        return False

    def is_gripper_empthy(self)-> bool:
        self.pm_robot_utils.wait_for_initial_scene_update()
        

        for obj in self.pm_robot_utils.object_scene.objects_in_scene:
            obj:ami_msg.Object
            if obj.parent_frame == self.PM_ROBOT_GRIPPER_FRAME:
                return False
        return True

    def get_gripped_component(self)-> str:
        self.pm_robot_utils.wait_for_initial_scene_update()
        for obj in self.pm_robot_utils.object_scene.objects_in_scene:
            obj:ami_msg.Object
            if obj.parent_frame == self.PM_ROBOT_GRIPPER_FRAME:
                return obj.obj_name
        return None

    # delete
    def get_assembly_and_target_frames(self)-> list[tuple[str,str]]:
        frames = []
        self.pm_robot_utils.wait_for_initial_scene_update()
        for obj in self.pm_robot_utils.object_scene.objects_in_scene:
            obj:ami_msg.Object
            for frame in obj.ref_frames:
                frame:ami_msg.RefFrame
                if self.ASSEMBLY_FRAME_INDICATOR in frame.frame_name:
                    frames.append((obj.obj_name,frame.frame_name))
                if self.TARGET_FRAME_INDICATOR in frame.frame_name:
                    frames.append((obj.obj_name,frame.frame_name))
        return frames

    def get_assembly_target_frame_gripped_component(self)->list[str]:
        """
        Get the assembly and target frames for the gripped component. 

        Returns:
            list[str]: List of assembly and target frames for the gripped component.
        """
        gripped_component = self.get_gripped_component()
        
        frames = []

        if gripped_component is None:
            return None
        
        frames_tuple = self.get_assembly_and_target_frames()
        for frame_tuple in frames_tuple:
            if frame_tuple[0] == gripped_component:
                frames.append(frame_tuple[1])
        
        return frames

    # delete
    def get_component_for_frame_name(self, frame_name:str)-> str:
        self.pm_robot_utils.wait_for_initial_scene_update()
        for obj in self.pm_robot_utils.object_scene.objects_in_scene:
            obj:ami_msg.Object
            for frame in obj.ref_frames:
                frame:ami_msg.RefFrame
                if frame.frame_name == frame_name:
                    return obj.obj_name
        return None

    # delete
    def get_list_of_components(self)-> list[str]:
        self.pm_robot_utils.wait_for_initial_scene_update()
        components = []
        for obj in self.pm_robot_utils.object_scene.objects_in_scene:
            obj:ami_msg.Object
            components.append(obj.obj_name)
        return components

    # delete
    def get_global_statonary_component(self)-> str:
        """
        Returns the name of the component that is not moved among all of the components.
        """

        self.pm_robot_utils.wait_for_initial_scene_update()

        components = self.get_list_of_components()
        
        for component in components:
            if self.is_component_stationary(component):
                return component

    # delete
    def is_component_stationary(self, component_name:str)-> bool:

        self.pm_robot_utils.wait_for_initial_scene_update()

        is_stationary = True
        for instruction in self.pm_robot_utils.object_scene.assembly_instructions:
            instruction:ami_msg.AssemblyInstruction
            if component_name == instruction.component_1 and instruction.component_1_is_moving_part:
                is_stationary = False

            if component_name == instruction.component_2 and not instruction.component_1_is_moving_part:
                is_stationary = False

        return is_stationary

    # delete
    def is_component_assembled(self, component_name:str)-> bool:
        self.pm_robot_utils.wait_for_initial_scene_update()

        is_assembled = False

        for instruction in self.pm_robot_utils.object_scene.assembly_instructions:
            instruction:ami_msg.AssemblyInstruction
            if component_name == instruction.component_1 and instruction.component_2 == self.get_parent_of_object(component_name):
                return True

            if component_name == instruction.component_2 and instruction.component_1 == self.get_parent_of_object(component_name):
                return True
        
        return False

    # delete
    def get_components_to_assemble(self)-> list[str]:
        components_to_assembly = []

        self.pm_robot_utils.wait_for_initial_scene_update()

        for instruction in self.pm_robot_utils.object_scene.assembly_instructions:
            instruction:ami_msg.AssemblyInstruction
            if instruction.component_1_is_moving_part:
                moving_object = instruction.component_1
                stationary_object = instruction.component_2
            else:
                moving_object = instruction.component_2
                stationary_object = instruction.component_1
            
            if stationary_object != self.get_parent_of_object(moving_object):
                components_to_assembly.append(moving_object)
        
        return components_to_assembly

    # delete
    def find_matches_for_component(self, component_name:str, only_unassembled = True)-> list[str]:
        """
        The component to find the matches for should be the stationary component.
        """
        self.pm_robot_utils.wait_for_initial_scene_update()

        matches = []
        for instruction in self.pm_robot_utils.object_scene.assembly_instructions:
            instruction:ami_msg.AssemblyInstruction
            self.logger.warn(f"Instruction: {instruction.component_1} -> {instruction.component_2}")

            test= self.is_component_assembled(instruction.component_2)
            self.logger.warn(f"Is component {instruction.component_2} assembled: {test}")

            if component_name == instruction.component_1:
                if ((not only_unassembled or self.is_component_assembled(instruction.component_2)) and 
                    not instruction.component_1_is_moving_part):
                    #not instruction.component_2 == self.get_global_statonary_component()):

                    matches.append(instruction.component_2)

            if component_name == instruction.component_2:
                if ((not only_unassembled or self.is_component_assembled(instruction.component_1)) and 
                    instruction.component_1_is_moving_part):
                    #not instruction.component_1 == self.get_global_statonary_component()):

                    matches.append(instruction.component_1)
        return matches


def main(args=None):
    rclpy.init(args=args)

    node = PmSkills()

    executor = MultiThreadedExecutor(num_threads=6) 
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()
