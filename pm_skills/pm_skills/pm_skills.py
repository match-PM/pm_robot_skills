import rclpy
import copy
import time
import random

from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor

from geometry_msgs.msg import Vector3, TransformStamped, Pose, PoseStamped, Quaternion

import pm_skills_interfaces.srv as pm_skill_srv
from pm_moveit_interfaces.srv import MoveToPose,  MoveToFrame, MoveRelative
from example_interfaces.srv import SetBool, Trigger
import std_msgs.msg as std_msg
import assembly_manager_interfaces.srv as ami_srv
import assembly_manager_interfaces.msg as ami_msg
import pm_moveit_interfaces.srv as pm_moveit_srv
from pm_msgs.srv import LaserGetMeasurement
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, StaticTransformBroadcaster

from pm_msgs.srv import EmptyWithSuccess
from assembly_scene_publisher.py_modules.AssemblyScene import AssemblyManagerScene

from assembly_scene_publisher.py_modules.tf_functions import get_transform_for_frame_in_world

from assembly_scene_publisher.py_modules.scene_functions import is_frame_from_scene

from pm_msgs.srv import UVCuringSkill

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


class PmSkills(Node):
    
    PM_ROBOT_GRIPPER_FRAME = 'PM_Robot_Tool_TCP'
    PM_ROBOT_GONIO_LEFT_FRAME_INDICATOR = 'Gonio_Left_Part'
    PM_ROBOT_GONIO_RIGHT_FRAME_INDICATOR = 'Gonio_Right_Part'
    GRIPPING_FRAME_IDENTIFICATORS = ['Grip', 'grip']
    GRIP_RELATIVE_LIFT_DISTANCE = 0.02
    RELEASE_LIFT_DISTNACE = 0.01
    ASSEMBLY_FRAME_INDICATOR = 'assembly_frame_Description'
    TARGET_FRAME_INDICATOR = 'target_frame_Description'
    GRIPPING_OFFSET = 0.0001
    
    def __init__(self) -> None:
        super().__init__('pm_skills')

        self.callback_group_me = MutuallyExclusiveCallbackGroup()
        self.callback_group_re = ReentrantCallbackGroup()

        self.grip_component_srv = self.create_service(pm_skill_srv.GripComponent, "pm_skills/grip_component", self.grip_component_callback,callback_group=self.callback_group_me)
        self.place_component_srv = self.create_service(pm_skill_srv.PlaceComponent, "pm_skills/place_component", self.place_component_callback,callback_group=self.callback_group_me)
        self.assemble_srv  = self.create_service(EmptyWithSuccess, "pm_skills/assemble", self.assemble_callback,callback_group=self.callback_group_me)
        #self.vision_service = self.create_service(ExecuteVision, "pm_skills/execute_vision", self.vision_callback)
        
        self.vacuum_gripper_service = self.create_service(pm_skill_srv.VacuumGripper, "pm_skills/vacuum_gripper", self.vaccum_gripper_callback, callback_group=self.callback_group_me)
    
        self.dispenser_service = self.create_service(pm_skill_srv.DispenseAdhesive, "pm_skills/dispense_adhesive", self.dispenser_callback)
        self.confocal_laser_service = self.create_service(pm_skill_srv.ConfocalLaser, "pm_skills/confocal_laser", self.confocal_laser_callback)
        self.vision_service = self.create_service(pm_skill_srv.ExecuteVision, "pm_skills/execute_vision", self.vision_callback)

        self.measue_with_laser_srv = self.create_service(pm_skill_srv.CorrectFrame, "pm_skills/measure_with_laser", self.measure_with_laser_callback, callback_group=self.callback_group_me)
        self.correct_frame_with_laser_srv = self.create_service(pm_skill_srv.CorrectFrame, "pm_skills/correct_frame_with_laser", self.correct_frame_with_laser, callback_group=self.callback_group_me)
        self.dummy_uv_cure_service = self.create_service(EmptyWithSuccess, "pm_skills/uv_cure_dummy", self.activate_uv_dummy, callback_group=self.callback_group_me)

        
        self.attach_component = self.create_client(ami_srv.ChangeParentFrame, '/assembly_manager/change_obj_parent_frame')
        self.move_robot_tool_client = self.create_client(MoveToFrame, '/pm_moveit_server/move_tool_to_frame')
        self.move_robot_tool_relative = self.create_client(MoveRelative, '/pm_moveit_server/move_tool_relative')
        self.attach_component = self.create_client(ami_srv.ChangeParentFrame, '/assembly_manager/change_obj_parent_frame')
        self.align_gonio_right_client = self.create_client(pm_moveit_srv.AlignGonio, '/pm_moveit_server/align_gonio_right')
        self.align_gonio_left_client = self.create_client(pm_moveit_srv.AlignGonio, '/pm_moveit_server/align_gonio_left')

        self.move_robot_laser_client = self.create_client(MoveToFrame, '/pm_moveit_server/move_laser_to_frame')
        
        self.vacuum_gripper_on_client = self.create_client(EmptyWithSuccess, '/pm_nozzle_controller/Head_Nozzle/Vacuum')
        self.vacuum_gripper_off_client = self.create_client(EmptyWithSuccess, '/pm_nozzle_controller/Head_Nozzle/TurnOff')
        self.vacuum_gripper_pressure_client = self.create_client(EmptyWithSuccess, '/pm_nozzle_controller/Head_Nozzle/Pressure')
        self.get_laser_mes_client = self.create_client(LaserGetMeasurement, '/pm_sensor_controller/Laser/GetMeasurement')
        self.uv_cure_clinet = self.create_client(UVCuringSkill, '/pm_robot_primitive_skills/uv_curing')
        self.objcet_scene_subscriber = self.create_subscription(ami_msg.ObjectScene, '/assembly_manager/scene', self.object_scene_callback, 10, callback_group=self.callback_group_re)
        self.simtime_subscriber = self.create_subscription(std_msg.Bool, '/sim_time', self.simtime_callback, 10, callback_group=self.callback_group_re)

        self.adapt_frame_client = self.create_client(ami_srv.ModifyPoseAbsolut, '/assembly_manager/modify_frame_absolut')

        self.object_scene:ami_msg.ObjectScene = None

        self.logger = self.get_logger()
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.logger.info(f"PM Skills node started! Gazabo running: {self.is_gazebo_running()}")
    
    def grip_component_callback(self, request:pm_skill_srv.GripComponent.Request, response:pm_skill_srv.GripComponent.Response):
        """TO DO: Add docstring"""
       
        try:
            if not self.check_object_exists(request.component_name):
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
        move_tool_to_part_offset_success = self.move_gripper_to_frame(gripping_frame, z_offset=self.RELEASE_LIFT_DISTNACE)

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
            if self.is_gripper_empthy():
                response.success = False
                response.message = "Gripper is empty! Can not place component!"
                self.logger.error(response.message)
                return response
        except ValueError as e:
            response.success = False
            response.message = str(e)
            self.logger.error(response.message)
            return response

        
        gripped_component = self.get_gripped_component()
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
            response.success = False
            response.message = "No assembly or target frame found for gripped component!"
            self.logger.error(response.message)
            return response

        if target_frame is None:
            response.success = False
            response.message = f"No target frame found for component '{gripped_component}'"
            self.logger.error(response.message)
            return response 
        
        target_component = self.get_component_for_frame_name(target_frame)

        if target_component is None:
            response.success = False
            response.message = f"No component found for target frame '{target_frame}'"
            self.logger.error(response.message)
            return response
        
        # RECALCULATE THE ASSEMBLY TRANSFORMATION
        # INSERT HERE

        # Align gonio to the left or right depending on the target frame
        if self.is_object_on_gonio_left(target_component):
            algin_success = self.align_gonio_left(endeffector_override=target_frame,
                                                    alignment_frame=component_assembly_frame)

        elif self.is_object_on_gonio_right(target_component):
            algin_success = self.align_gonio_right(endeffector_override=target_frame,
                                                    alignment_frame=component_assembly_frame)

        else:
            response.success = False
            response.message = "Target object not on gonio!"
            self.logger.error("Target object not on gonio!")
            return response
        
        if not algin_success:
            response.success = False
            response.message = f"Failed to align gonio for target component '{target_component}'"
            self.logger.error(response.message)
            return response
        
        self.logger.warn("Endeffector override: " + component_assembly_frame)
        # Move component to the target frame with a z offset
        move_component_to_part_offset_success = self.move_gripper_to_frame(target_frame, endeffector_override=component_assembly_frame, z_offset=self.RELEASE_LIFT_DISTNACE)

        if not move_component_to_part_offset_success:
            response.success = False
            response.message = f"Failed to move gripper to target part '{target_component}'"
            self.logger.error(response.message)
            return response

        # Move component to the target frame
        move_component_to_part_success = self.move_gripper_to_frame(target_frame, endeffector_override=component_assembly_frame)

        if not move_component_to_part_success:
            response.success = False
            response.message = f"Failed to move gripper to target part '{target_component}'"
            self.logger.error(response.message)
            return response
        
        # release component
        attach_component_success = self.attach_component_to_component(gripped_component, target_component)

        if not attach_component_success:
            move_relatively_success = self.lift_gripper_relative(self.RELEASE_LIFT_DISTNACE)
            response.success = False
            response.message = f"Failed to attach component '{gripped_component}' to component '{target_component}'"
            self.logger.error(response.message)
            return response

        move_relatively_success = self.lift_gripper_relative(self.GRIP_RELATIVE_LIFT_DISTANCE)

        if not move_relatively_success:
            response.success = False
            response.message = f"Failed to lift gripper after placing component '{gripped_component}'"
            self.logger.error(response.message)
            return response
        
        response.success = True
        response.message = f"Component '{gripped_component}' placed successfully!"
        return response
        
    def assemble_callback(self, request:EmptyWithSuccess.Request, response:EmptyWithSuccess.Response):
        
        global_stationary_component = self.get_global_statonary_component()
        assemble_list = self.get_components_to_assemble()
        first_component = self.find_matches_for_component(global_stationary_component, only_unassembled=False)

        self.assembly_loop(first_component)
        
        response.success = True

        return response

    def vaccum_gripper_callback(self, request:pm_skill_srv.VacuumGripper.Request, response:pm_skill_srv.VacuumGripper.Response):
        """Mimics the vacuum gripper funtionality by activating/deactivating the vacuum and changing the parent frame"""

        sim_time = self.is_gazebo_running()
        self.logger.info(f"Gazebo Simulation: {sim_time}")

        assembly_and_target_frames = self.get_assembly_and_target_frames()      

        try:
            if request.activate_vacuum:
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

            else:
                # if self.is_gripper_empthy():
                    # response.success = False
                    # response.message = "Gripper is empty! Can not release component!"
                    # self.logger.error(response.message)
                    # return response

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
        call_async = False

        if not self.move_robot_laser_client.wait_for_service(timeout_sec=1.0):
            self.logger.error("Service '/pm_moveit_server/move_laser_to_frame' not available")
            return False
        
        req = pm_moveit_srv.MoveToFrame.Request()
        req.target_frame = frame_name
        req.execute_movement = True

        if z_offset is not None:
            req.translation.z = z_offset

        if call_async:
            future = self.move_robot_laser_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is None:
                self.logger.error('Service call failed %r' % (future.exception(),))
                return False
            return future.result().success
        else:
            response:pm_moveit_srv.MoveToFrame.Response = self.move_robot_laser_client.call(req)
            return response.success
    
    def get_laser_measurement(self, unit:str = "m")-> float:
        """
        Method to get the laser measurement from the laser sensor
        

        Returns:
            float: _description_
        """
        call_async = False

        if not self.get_laser_mes_client.wait_for_service(timeout_sec=1.0):
            self.logger.error("Service '/pm_sensor_controller/Laser/GetMeasurement' not available")
            return None
        
        req = LaserGetMeasurement.Request()

        if call_async:
            future = self.get_laser_mes_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is None:
                self.logger.error('Service call failed %r' % (future.exception(),))
                return None
            response= future.result()
            
        else:
            response:LaserGetMeasurement.Response = self.get_laser_mes_client.call(req)
        
        multiplier = 1.0
        
        if unit == "mm":
            multiplier = 1e-3
        elif unit == "cm":
            multiplier = 1e-2
        elif unit == "m":
            multiplier = 1e-6
        elif unit == "um":
            multiplier = 1.0
            
            
        return response.measurement * multiplier
      
    def measure_with_laser_callback(self, request:pm_skill_srv.CorrectFrame.Request, response:pm_skill_srv.CorrectFrame.Response):
        
        move_laser_to_frame_success = self.move_laser_to_frame(request.frame_name)
        
        if not move_laser_to_frame_success:
            response.success = False
            response.message = f"Failed to move laser to frame '{request.frame_name}'"
            self.logger.error(response.message)
            return response
        
        max_iterations = 5
        correction_distance = 0.0003
        z_offset = 0.0
        for iteration in range(max_iterations):
            laser_measurement = self.get_laser_measurement(unit="m")
            sign = self.check_laser_measurement_out_of_range(laser_measurement)

            # if valid laser measurement
            if sign == 0:
                break   # exit loop
            
            # if this is executed the laser has not yet a valid measurement
            z_offset = iteration * sign * correction_distance
            move_laser_to_frame_success = self.move_laser_to_frame(request.frame_name, z_offset = z_offset)

            if not move_laser_to_frame_success:
                response.success = False
                response.message = f"Failed to move laser to frame '{request.frame_name}'"
                self.logger.error(response.message)
                return response
            
        response.correction_values.z = laser_measurement - z_offset
        response.success = True
        response.message = f"Measurement: {laser_measurement}"
        return response
    
    def check_laser_measurement_out_of_range(self, value:float):
        if value * 1e6 > 300.0:
            return -1
        elif value * 1e6 < 0.0:
            return 1
        return 0
    
    def correct_frame_with_laser(self, request:pm_skill_srv.CorrectFrame.Request, response:pm_skill_srv.CorrectFrame.Response):
        
        self.wait_for_initial_scene_update()
        _obj_name, _frame_name =  is_frame_from_scene(self.object_scene, request.frame_name)

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
        
        response:pm_skill_srv.CorrectFrame.Response = self.measure_with_laser_callback(measure_frame_request, measure_frame_response)
                
        if not response.success:
            response.success = False
            return response
                
        world_pose:TransformStamped = get_transform_for_frame_in_world(request.frame_name, self.tf_buffer, self._logger)

        world_pose.transform.translation.z -= response.correction_values.z

        adapt_frame_request = ami_srv.ModifyPoseAbsolut.Request()
        adapt_frame_request.frame_name = request.frame_name
        adapt_frame_request.pose.position.x = world_pose.transform.translation.x
        adapt_frame_request.pose.position.y = world_pose.transform.translation.y
        adapt_frame_request.pose.position.z = world_pose.transform.translation.z
        adapt_frame_request.pose.orientation = world_pose.transform.rotation

        #self._logger.warn(f"Check3")
        
        if frame_from_scene:
            while not self.adapt_frame_client.wait_for_service(timeout_sec=1.0):
                self._logger.error("Service 'ModifyPoseAbsolut' not available. Assembly manager started?...")
                response.success= False
                return response
            
            result_adapt:ami_srv.ModifyPoseAbsolut.Response = self.adapt_frame_client.call(adapt_frame_request)
        else:
            result_adapt = ami_srv.ModifyPoseAbsolut.Response()

        #self._logger.warn(f"Check4")
        
        response.success = result_adapt.success
        
        return response
    
      
    def move_gripper_to_relative(self, frame_name:str)-> bool:
        call_async = False

        if not self.move_robot_tool_client.wait_for_service(timeout_sec=1.0):
            self.logger.error("Service '/pm_moveit_server/move_tool_to_frame' not available")
            return False
        
        req = pm_moveit_srv.MoveToFrame.Request()
        req.target_frame = frame_name
        req.execute_movement = True
        req.translation.z = 0.0000001

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
    
    def activate_uv(self,request: UVCuringSkill.Request)->bool:
        call_async = False

        if not self.uv_cure_clinet.wait_for_service(timeout_sec=1.0):
            self.logger.error("Service '/pm_moveit_server/align_gonio_left' not available")
            return False
    
        if call_async:
            future = self.uv_cure_clinet.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is None:
                self.logger.error('Service call failed %r' % (future.exception(),))
                return False
            return future.result().success
        else:
            response:pm_moveit_srv.AlignGonio.Response = self.uv_cure_clinet.call(request)
            return response.success
    
    def activate_uv_dummy(self,request:EmptyWithSuccess.Request,response: EmptyWithSuccess.Response):

        req = UVCuringSkill.Request()
        req.duration = [2.0, 2.0, 2.0, 2.0]
        req.intensity_percent = [50, 50, 50, 50]

        activate_success = self.activate_uv(request=req)

        response.success = activate_success

        return response 
    

    def object_scene_callback(self, msg:ami_msg.ObjectScene)-> str:
        self.object_scene = msg
    
    def get_gripping_frame(self, object_name:str)-> str:
        grip_frames = []
        for obj in self.object_scene.objects_in_scene:
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
        self.wait_for_initial_scene_update()
        for obj in self.object_scene.objects_in_scene:
            obj:ami_msg.Object
            if obj.obj_name == object_name:
                return obj.parent_frame
        return None

    def is_object_on_gonio_left(self, object_name:str)-> bool:
        parent_frame = self.get_parent_of_object(object_name)

        if self.PM_ROBOT_GONIO_LEFT_FRAME_INDICATOR in parent_frame:
            return True
        
        parent_object_1 = self.get_parent_of_object(parent_frame)

        if parent_object_1 is None:
            return False
        
        if self.PM_ROBOT_GONIO_LEFT_FRAME_INDICATOR in parent_object_1:
            return True
        
        parent_object_2 = self.get_parent_of_object(parent_object_1)

        if parent_object_2 is None:
            return False
        
        if self.PM_ROBOT_GONIO_LEFT_FRAME_INDICATOR in parent_object_2:
            return True
        
        return False

    def is_object_on_gonio_right(self, object_name:str)-> bool:
        parent_frame = self.get_parent_of_object(object_name)
        if self.PM_ROBOT_GONIO_RIGHT_FRAME_INDICATOR in parent_frame:
            return True
        
        parent_object_1 = self.get_parent_of_object(parent_frame)

        if parent_object_1 is None:
            return False
        
        if self.PM_ROBOT_GONIO_RIGHT_FRAME_INDICATOR in parent_object_1:
            return True
        
        parent_object_2 = self.get_parent_of_object(parent_object_1)

        if parent_object_2 is None:
            return False
        
        if self.PM_ROBOT_GONIO_RIGHT_FRAME_INDICATOR in parent_object_2:
            return True
        
        return False

    def is_gripper_empthy(self)-> bool:
        self.wait_for_initial_scene_update()

        if self.object_scene is None:
            raise ValueError("Object scene not available!")
        

        for obj in self.object_scene.objects_in_scene:
            obj:ami_msg.Object
            if obj.parent_frame == self.PM_ROBOT_GRIPPER_FRAME:
                return False
        return True


    # delete
    def check_object_exists(self, object_name:str)-> bool:
        self.wait_for_initial_scene_update()
        if self.object_scene is None:
            return False
        
        for obj in self.object_scene.objects_in_scene:
            obj:ami_msg.Object
            if obj.obj_name == object_name:
                return True
        return False

    def get_gripped_component(self)-> str:
        self.wait_for_initial_scene_update()
        for obj in self.object_scene.objects_in_scene:
            obj:ami_msg.Object
            if obj.parent_frame == self.PM_ROBOT_GRIPPER_FRAME:
                return obj.obj_name
        return None

    # delete
    def get_assembly_and_target_frames(self)-> list[tuple[str,str]]:
        frames = []
        self.wait_for_initial_scene_update()
        for obj in self.object_scene.objects_in_scene:
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
        self.wait_for_initial_scene_update()
        for obj in self.object_scene.objects_in_scene:
            obj:ami_msg.Object
            for frame in obj.ref_frames:
                frame:ami_msg.RefFrame
                if frame.frame_name == frame_name:
                    return obj.obj_name
        return None

    # delete
    def get_list_of_components(self)-> list[str]:
        self.wait_for_initial_scene_update()
        components = []
        for obj in self.object_scene.objects_in_scene:
            obj:ami_msg.Object
            components.append(obj.obj_name)
        return components

    # delete
    def get_global_statonary_component(self)-> str:
        """
        Returns the name of the component that is not moved among all of the components.
        """

        self.wait_for_initial_scene_update()

        components = self.get_list_of_components()
        
        for component in components:
            if self.is_component_stationary(component):
                return component

    # delete
    def is_component_stationary(self, component_name:str)-> bool:

        self.wait_for_initial_scene_update()

        is_stationary = True
        for instruction in self.object_scene.assembly_instructions:
            instruction:ami_msg.AssemblyInstruction
            if component_name == instruction.component_1 and instruction.component_1_is_moving_part:
                is_stationary = False

            if component_name == instruction.component_2 and not instruction.component_1_is_moving_part:
                is_stationary = False

        return is_stationary

    # delete
    def is_component_assembled(self, component_name:str)-> bool:
        self.wait_for_initial_scene_update()

        is_assembled = False

        for instruction in self.object_scene.assembly_instructions:
            instruction:ami_msg.AssemblyInstruction
            if component_name == instruction.component_1 and instruction.component_2 == self.get_parent_of_object(component_name):
                return True

            if component_name == instruction.component_2 and instruction.component_1 == self.get_parent_of_object(component_name):
                return True
        
        return False

    # delete
    def get_components_to_assemble(self)-> list[str]:
        components_to_assembly = []

        self.wait_for_initial_scene_update()

        for instruction in self.object_scene.assembly_instructions:
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
        self.wait_for_initial_scene_update()

        matches = []
        for instruction in self.object_scene.assembly_instructions:
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

    def wait_for_initial_scene_update(self):
        while self.object_scene is None:
            self.logger.warn("Waiting for object scene to be updated...")
            time.sleep(0.1)

    def is_gazebo_running(self):
        """Check if the Gazebo node is active."""
        node_names = self.get_node_names()
        if 'gazebo' in node_names:
            return True
        return False

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
