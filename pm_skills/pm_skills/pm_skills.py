from urllib import request, response
import rclpy
import copy
import time
import random

from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
import math
from geometry_msgs.msg import Vector3, TransformStamped, Pose, PoseStamped, Quaternion, Transform, Point
import pm_skills_interfaces.srv as pm_skill_srv
from pm_moveit_interfaces.srv import MoveToPose,  MoveToFrame, MoveRelative
from example_interfaces.srv import SetBool, Trigger
import std_msgs.msg as std_msg
import assembly_manager_interfaces.srv as ami_srv
import assembly_manager_interfaces.msg as ami_msg
import pm_moveit_interfaces.srv as pm_moveit_srv
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, StaticTransformBroadcaster
from assembly_scene_publisher.py_modules.scene_errors import *
from pm_msgs.srv import EmptyWithSuccess
from assembly_scene_publisher.py_modules.AssemblyScene import AssemblyManagerScene
from pm_robot_modules.submodules.pm_dispense_path_generator import DispenseSequenceGenerator
import re
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from typing import List, Tuple, Optional
from assembly_scene_publisher.py_modules.tf_functions import get_transform_for_frame_in_world
from scipy.spatial.transform import Rotation as R
import numpy as np
from pm_msgs.srv import UVCuringSkill
import pm_msgs.srv as pm_msg_srv
import pm_msgs.msg as pm_msg
from assembly_scene_publisher.py_modules.geometry_functions import quaternion_multiply
from assembly_scene_publisher.py_modules.scene_errors import (RefAxisNotFoundError, 
                                                              RefFrameNotFoundError, 
                                                              RefPlaneNotFoundError, 
                                                              ComponentNotFoundError,
                                                              TargetFrameNotFoundError,
                                                              AssemblyFrameNotFoundError)
from ament_index_python.packages import get_package_share_directory
from pm_skills.py_modules.PmRobotUtils import PmRobotUtils
from pm_robot_primitive_skills.py_modules.PmRobotError import PmRobotError

from assembly_scene_publisher.py_modules.geometry_functions import multiply_ros_transforms, inverse_ros_transform
from datetime import datetime


class PmSkills(Node):
    
    PM_ROBOT_GRIPPER_FRAME = 'PM_Robot_Tool_TCP'
    PM_ROBOT_GONIO_LEFT_FRAME_INDICATOR = 'Gonio_Left_Part'
    PM_ROBOT_GONIO_RIGHT_FRAME_INDICATOR = 'Gonio_Right_Part'
    GRIP_RELATIVE_LIFT_DISTANCE = 0.05
    RELEASE_LIFT_DISTANCE = 0.05
    GRIP_APPROACH_OFFSET = 0.05
    #GRIP_SENSING_START_OFFSET = 0.0005
    GRIP_SENSING_START_OFFSET = 0.000500  # 500 um
    GRIP_SENSING_END_OFFSET = -0.0005   # -500 um
    GRIP_SENSING_END_ROUGH = 0.00020    # 80 um

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
        #self.grip_component_srv = self.create_service(pm_skill_srv.GripComponent, "pm_skills/grip_component", self.grip_component_callback,callback_group=self.callback_group_me)
        self.force_grip_component_srv = self.create_service(pm_skill_srv.GripComponent, "pm_skills/force_grip_component", self.force_grip_component_callback,callback_group=self.callback_group_me)

        self.place_component_srv = self.create_service(pm_skill_srv.PlaceComponent, "pm_skills/place_component", self.place_component_callback,callback_group=self.callback_group_me)
        self.release_component_srv = self.create_service(EmptyWithSuccess, "pm_skills/release_component", self.release_component_callback,callback_group=self.callback_group_me)

        #self.assemble_srv  = self.create_service(EmptyWithSuccess, "pm_skills/assemble", self.assemble_callback,callback_group=self.callback_group_me)
        
        self.vacuum_gripper_on_service = self.create_service(EmptyWithSuccess, "pm_skills/vacuum_gripper/vacuum_on", self.vaccum_gripper_on_callback, callback_group=self.callback_group_me)
        self.vacuum_gripper_off_service = self.create_service(EmptyWithSuccess, "pm_skills/vacuum_gripper/vacuum_off", self.vaccum_gripper_off_callback, callback_group=self.callback_group_me)
            
        # dummy
        #self.dispenser_service = self.create_service(pm_skill_srv.DispenseAdhesive, "pm_skills/dispense_adhesive", self.dispenser_callback)
        #self.confocal_laser_service = self.create_service(pm_skill_srv.ConfocalLaser, "pm_skills/confocal_laser", self.confocal_laser_callback)
        #self.vision_service = self.create_service(pm_skill_srv.ExecuteVision, "pm_skills/execute_vision", self.vision_callback)

        self.move_uv_in_curing_position_service = self.create_service(pm_msg_srv.EmptyWithSuccess, "/pm_skills"+"/move_uv_in_curing_position", self.move_uv_in_curing_position_service_callback,callback_group=self.callback_group_me)
        self.move_uv_out_of_curing_position_service = self.create_service(pm_msg_srv.EmptyWithSuccess, "/pm_skills"+"/move_uv_out_of_curing_position", self.move_uv_out_of_curing_position_service_callback,callback_group=self.callback_group_me)        

        self.measue_with_laser_srv = self.create_service(pm_skill_srv.CorrectFrameLaser, "pm_skills/measure_with_laser", self.measure_with_laser_callback, callback_group=self.callback_group_me)
        self.correct_frame_with_laser_srv = self.create_service(pm_skill_srv.CorrectFrameLaser, "pm_skills/correct_frame_with_laser", self.correct_frame_with_laser, callback_group=self.callback_group_me)
        self.force_sensing_move_srv = self.create_service(pm_msg_srv.GripperForceMove, self.get_name()+'/gripper_force_sensing', self.force_sensing_move_callback, callback_group=self.callback_group_me)
        self.force_scan_srv = self.create_service(pm_skill_srv.ForceScan, self.get_name() + "/force_scan", self.force_scan_callback, callback_group=self.callback_group_me)
        
        self.measure_frame_with_confocal_bottom_srv = self.create_service(pm_skill_srv.CorrectFrameLaser, "pm_skills/measure_frame_with_confocal_bottom", self.measure_frame_with_confocal_bottom, callback_group=self.callback_group_me)
        self.correct_frame_with_confocal_bottom_srv = self.create_service(pm_skill_srv.CorrectFrameLaser, "pm_skills/correct_frame_with_confocal_bottom", self.correct_frame_with_confocal_bottom, callback_group=self.callback_group_me)

        self.disp_at_path_srv = self.create_service(pm_msg_srv.DispenseAtPath, self.get_name()+'/dispense_2k_at_path', self.dispense_2k_at_path, callback_group=self.callback_group_me)

        self.measure_frame_with_confocal_top_srv = self.create_service(pm_skill_srv.CorrectFrameLaser, "pm_skills/measure_frame_with_confocal_top", self.measure_frame_with_confocal_top, callback_group=self.callback_group_me)
        self.correct_frame_with_confocal_top_srv = self.create_service(pm_skill_srv.CorrectFrameLaser, "pm_skills/correct_frame_with_confocal_top", self.correct_frame_with_confocal_top, callback_group=self.callback_group_me)

        self.srv_iter_align_gonio_right = self.create_service(pm_skill_srv.IterativeGonioAlign, self.get_name()+'/iterative_align_gonio_right', self.iterative_align_gonio_right, callback_group=self.callback_group_me)
        self.srv_iter_align_gonio_left = self.create_service(pm_skill_srv.IterativeGonioAlign, self.get_name()+'/iterative_align_gonio_left', self.iterative_align_gonio_left, callback_group=self.callback_group_me)

        self.srv_check_frame_measureble_confocal_top = self.create_service(pm_skill_srv.CheckFrameMeasurable, self.get_name()+'/check_frame_measureble_confocal_top', self.check_frame_mes_confocal_top, callback_group=self.callback_group_me)
        self.srv_check_frame_measureble_laser_top = self.create_service(pm_skill_srv.CheckFrameMeasurable, self.get_name()+'/check_frame_measureble_laser_top', self.check_frame_mes_laser_top, callback_group=self.callback_group_me)   
        self.srv_check_frame_measureble_confocal_bottom = self.create_service(pm_skill_srv.CheckFrameMeasurable, self.get_name()+'/check_frame_measureble_confocal_bottom', self.check_frame_mes_confocal_bottom, callback_group=self.callback_group_me)   

        self.srv_dispense_at_points = self.create_service(pm_msg_srv.DispenseAtPoints, self.get_name()+'/dispense_at_frames', self.dispense_at_frames_callback, callback_group=self.callback_group_me)
        
        self.srv_dispense_at_points_adv = self.create_service(pm_msg_srv.DispenseAtPoints, self.get_name()+'/dispense_at_frames_adv', self.dispense_at_frames_adv_callback, callback_group=self.callback_group_me)
        self.srv_uv_cure_adv = self.create_service(pm_msg_srv.UVCuringSkill, self.get_name()+'/uv_cure', self.uv_cure_adv_callback, callback_group=self.callback_group_me)

        # clients
        self.attach_component = self.create_client(ami_srv.ChangeParentFrame, '/assembly_manager/change_obj_parent_frame')
        self.smart_gripper_force_client = self.create_client(pm_msg_srv.GripperGetForces, '/SmarAct_Gripper/GetForces')
        
        self.dispense_2k_unity_client = self.create_client(EmptyWithSuccess, '/unity_skills/dispense_2k_unity')

        self.simtime_subscriber = self.create_subscription(std_msg.Bool, '/sim_time', self.simtime_callback, 10, callback_group=self.callback_group_re)

        self.adapt_frame_client = self.create_client(ami_srv.ModifyPoseAbsolut, '/assembly_manager/modify_frame_absolut')

        self.client_move_robot_tool_to_frame = self.create_client(pm_moveit_srv.MoveToFrame, "/pm_moveit_server/move_tool_to_frame")

        self.logger = self.get_logger()
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
    

    def move_uv_in_curing_position_service_callback(self, request:pm_msg_srv.EmptyWithSuccess.Request, response:pm_msg_srv.EmptyWithSuccess.Response):
        """Moves both UV LEDs in curing position"""
        try:
            srv_request = SetBool.Request()
            srv_request.data = True
            response.success = self.pm_robot_utils.move_uv_in_curing_position(srv_request)
             
        except PmRobotError as e:
            self.get_logger().error(f"Error moving UV LEDs in curing position: {e.message}")
            response.success = False
            response.message = e.message
        return response
    
    def move_uv_out_of_curing_position_service_callback(self, request:pm_msg_srv.EmptyWithSuccess.Request, response:pm_msg_srv.EmptyWithSuccess.Response):
        """Moves both UV LEDs out of curing position"""
        try:
            srv_request = SetBool.Request()
            srv_request.data = False
            response.success = self.pm_robot_utils.move_uv_in_curing_position(srv_request)

        except PmRobotError as e:
            self.get_logger().error(f"Error moving UV LEDs out of curing position: {e.message}")
            response.success = False
            response.message = e.message
        return response

    def force_sensing_move_callback(self, request:pm_msg_srv.GripperForceMove.Request, response:pm_msg_srv.GripperForceMove.Response):

        self.get_logger().info('Received ForceSensingMove request.')

        self.pm_robot_utils.set_force_sensor_bias()
        time.sleep(2.0)
        
        # Check force sensor values after bias reset
        current_force_values = self.pm_robot_utils._current_force_sensor_data.data
        self.get_logger().info(f'Force sensor values after bias reset: {current_force_values}')
        
        force_thrshold = [abs(request.max_f_xyz[0]), abs(request.max_f_xyz[1]), abs(request.max_f_xyz[2])]

        if not self.pm_robot_utils.is_unity_running() and (abs(current_force_values[0]) > force_thrshold[0] or abs(current_force_values[1]) > force_thrshold[1] or abs(current_force_values[2]) > force_thrshold[2]):
            response.error = f'Initial force values exceed threshold of {force_thrshold} N: X={current_force_values[0]}, Y={current_force_values[1]}, Z={current_force_values[2]}'
            raise PmRobotError(f"{response.error}")
        
        # Validate the request parameters. If any max force is > than 10, set threshold_exceeded to True and return failure.
        threshold_value = 10.0  # N
        max_step_size = 100  # micrometers
        max_steps = 1000  # maximum number of steps


        if not self.pm_robot_utils.is_unity_running() and (abs(request.max_f_xyz[0]) > threshold_value or abs(request.max_f_xyz[1]) > threshold_value or abs(request.max_f_xyz[2]) > threshold_value):
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
        length_z = target_z - start_z
        # Calculate the length of the vector from start to target position
        length = math.sqrt(length_x**2 + length_y**2 + length_z**2)

        if length == 0.0:
            self.get_logger().info('Already at target position.')
            response.success = True
            response.completed = False
            return response

        if (length/ step_size) > max_steps:
            self.get_logger().error(f'The distance to the target position is too large. The maximum number of steps is {max_steps}.')
            response.success = False
            response.error = f'The distance to the target position is too large. The maximum number of steps is {max_steps}.'
            return response

        self.get_logger().info(f'Current force sensor data: {self.pm_robot_utils._current_force_sensor_data.data}')

        step_size_x = step_size * length_x / length
        step_size_y = step_size * length_y / length
        step_size_z = step_size * length_z / length
        counter = 0
        while current_position != target_position:
            # Check if the force sensor data exceeds the thresholds
            for i, (force, max_force, axis) in enumerate(zip(self.pm_robot_utils._current_force_sensor_data.data, [abs(request.max_f_xyz[0]), abs(request.max_f_xyz[1]), abs(request.max_f_xyz[2])], ['X', 'Y', 'Z'])):
                if abs(force) > max_force:
                    self.get_logger().warn(f'Force in {axis} direction exceeded threshold: {force:.3f} > {max_force:.3f}')
                    response.success = True
                    response.completed = True
                    return response

            step_target = [
                current_position[0] + step_size_x,
                current_position[1] + step_size_y,
                current_position[2] + step_size_z,
            ]

            self.get_logger().info(f'Moving to position: {step_target}. Step {counter}. With step size: {[round(step_size_x*1e6, 5), round(step_size_y*1e6, 5), round(step_size_z*1e6, 5)]} um')

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
            counter += 1

        self.get_logger().info('Target position reached. Nothing found.')
        response.success = True
        response.completed = False
        response.error = 'Target position reached. Nothing found.'
        return response
    
    
    def force_scan_callback(self, request: pm_skill_srv.ForceScan.Request, response: pm_skill_srv.ForceScan.Response):
        """
        Request:
            - target_frame (string):    Frame des Bauteils das gemessen werden soll
            - direction (Vector3):      Richtung der Bewegung in Weltkoordinaten (wird normalisiert)
            - max_force (Vector3):      Kraftschwellwerte in N fuer X, Y, Z
            - step_size (float32):      Schrittgroesse in Mikrometern

        Response:
            - success (bool):           True wenn Kraft erkannt wurde
            - message (string):         Statusmeldung
            - detected_position (Pose): Weltkoordinaten des TCPs beim Kraftkontakt
        """




        try:

            # for i in range (30):
                # scan_number = i+1
                scan_number = 1
                self.get_logger().info(f"Starting force scan {scan_number} of 25.")

                timestamp = datetime.now().isoformat()

                piezo_force_request = pm_msg_srv.GripperGetForces.Request()
                frame_position_gripper = get_transform_for_frame_in_world(
                    self.PM_ROBOT_GRIPPER_FRAME,
                    self.tf_buffer,
                    self.get_logger()
                )

                PiezoGripperForce: pm_msg_srv.GripperGetForces.Response = self.smart_gripper_force_client.call(piezo_force_request)

                self.logger.info(f"Current gripper forces: X={PiezoGripperForce.fx}, Y={PiezoGripperForce.fy}, Z={PiezoGripperForce.fz}")

                # raise NotImplementedError("Force scan skill is still in development. The current implementation is a placeholder and may not work as expected.")


                # Schritt 1: Richtungsvektor pruefen und normalisieren
                direction_length = math.sqrt(
                    request.direction.x**2 +
                    request.direction.y**2 +
                    request.direction.z**2
                )
                if direction_length == 0.0:
                    response.success = False
                    response.message = 'Direction vector is zero!'
                    self.get_logger().error(response.message)
                    return response

                direction_normalized = [
                    request.direction.x / direction_length,
                    request.direction.y / direction_length,
                    request.direction.z / direction_length
                ]

                # Schritt 2: Schrittgroesse und Kraftgrenzwert pruefen
                max_step_size_um = 100
                if request.step_size > max_step_size_um:
                    response.success = False
                    response.message = (
                        f'Step size {request.step_size} um exceeds '
                        f'the maximum of {max_step_size_um} um.'
                    )
                    self.get_logger().error(response.message)
                    return response

                max_force_limit = 400.0
                if not self.pm_robot_utils.is_unity_running():
                    if (abs(request.max_force.x) > max_force_limit or
                        abs(request.max_force.y) > max_force_limit or
                        abs(request.max_force.z) > max_force_limit):
                        response.success = False
                        response.message = f'Force threshold {max_force_limit}, pick lower force values'
                        self.get_logger().error(response.message)
                        return response

                # Schritt 3: Zur Anfahrposition fahren
                # (Offset von 20 mm entgegen der Scanrichtung je Achse, Vorzeichen
                # ergibt sich aus der Richtung des normierten Vektors)
                # offset = [
                #     -0.002 if axis > 0
                #     else 0.002 if axis < 0
                #     else 0.0
                #     for axis in direction_normalized
                # ]

                self.get_logger().info('target frame: ' + request.target_frame)
                frame_target = get_transform_for_frame_in_world(
                    request.target_frame,
                    self.tf_buffer,
                    self.get_logger()
                )
                frame_position_target = [
                    frame_target.transform.translation.x,
                    frame_target.transform.translation.y,
                    frame_target.transform.translation.z
                ]

                success, message = self.move_gripper_to_frame(request.target_frame, x_offset=-0.0021, y_offset=0.0, z_offset=-0.0015) 
                if not success:
                    response.success = False
                    response.message = f'could not move to target frame: {message}'
                    self.get_logger().error(response.message)
                    return response

                # Schritt 4: Tatsaechliche Startposition ermitteln
                start_position = [
                    self.pm_robot_utils.get_current_joint_state(self.pm_robot_utils.X_Axis_JOINT_NAME),
                    self.pm_robot_utils.get_current_joint_state(self.pm_robot_utils.Y_Axis_JOINT_NAME),
                    self.pm_robot_utils.get_current_joint_state(self.pm_robot_utils.Z_Axis_JOINT_NAME)
                ]
                current_position = list(start_position)


                self.get_logger().info(
                    f'Start position: '
                    f'X={round(start_position[0]*1e3, 3)} mm, '
                    f'Y={round(start_position[1]*1e3, 3)} mm, '
                    f'Z={round(start_position[2]*1e3, 3)} mm'
                )
                
                # Schritt 5: Kraftsensor nullen (Vermeidung eines Messbias)
                self.pm_robot_utils.set_force_sensor_bias()
                time.sleep(2.0)

                # Schritt 6: Anfangskraefte pruefen
                if not self.pm_robot_utils.is_unity_running():
                    force_response = self.smart_gripper_force_client.call(piezo_force_request)

                    current_force_values = [
                        force_response.fx,
                        force_response.fy,
                        force_response.fz
                    ]
                else:
                    current_force_values = self.pm_robot_utils._current_force_sensor_data.data[:3]
                self.get_logger().info(f'Force sensor initial values: {current_force_values}')

                force_threshold = [
                    abs(request.max_force.x),
                    abs(request.max_force.y),
                    abs(request.max_force.z)
                ]

                if not self.pm_robot_utils.is_unity_running():
                    if (abs(current_force_values[0]) > force_threshold[0] or
                        abs(current_force_values[1]) > force_threshold[1] or
                        abs(current_force_values[2]) > force_threshold[2]):
                        response.success = False
                        response.message = (
                            f'current force values exceed threshold {force_threshold} before starting the scan: '
                            f'X={current_force_values[0]:.3f}, '
                            f'Y={current_force_values[1]:.3f}, '
                            f'Z={current_force_values[2]:.3f}'
                        )
                        self.get_logger().error(response.message)
                        return response

                # Schritt 7: Schrittgroessen in Meter umrechnen
                step_size_m_min = request.step_size * 1e-6

                # Grobe Suchschrittgroesse: 0,5 mm pro Schritt, unabhaengig von der
                # angeforderten Ziel-Schrittgroesse. Die Verfeinerung in Richtung
                # der Ziel-Schrittgroesse erfolgt erst im Zustand REFINE.
                step_size_m_search = 0.0002

                # Schritt 8: Scan-Schleife (Zustandsmaschine)

                max_scan_distance_m = 0.025
                travelled_distance_m = 0.0

            
                last_valid_position = current_position.copy()

                contact_position = None        

                detected_position = None

                contact_detected = False

                scan_start_time = time.time()
                search_iterations = 0
                refine_iterations = 0
                contact_force = [math.nan, math.nan, math.nan]
                frame_position_contact = [math.nan, math.nan, math.nan]
                
                state = "SEARCH"

                while travelled_distance_m < max_scan_distance_m:

                    if state == "SEARCH":            
                        search_iterations += 1        
                        # Schrittweise Annaeherung in Richtung des normierten Vektors

                        current_position[0] += step_size_m_search * direction_normalized[0]
                        current_position[1] += step_size_m_search * direction_normalized[1]
                        current_position[2] += step_size_m_search * direction_normalized[2]

                        self.pm_robot_utils.send_xyz_trajectory_goal_absolut(
                            *current_position,
                            time=0.05
                        )

                        time.sleep(0.5)

                        current_position = [
                            self.pm_robot_utils.get_current_joint_state(self.pm_robot_utils.X_Axis_JOINT_NAME),
                            self.pm_robot_utils.get_current_joint_state(self.pm_robot_utils.Y_Axis_JOINT_NAME),
                            self.pm_robot_utils.get_current_joint_state(self.pm_robot_utils.Z_Axis_JOINT_NAME)
                        ]

                        if not self.pm_robot_utils.is_unity_running():
                            force_response = self.smart_gripper_force_client.call(piezo_force_request)
                            force_values = [
                                force_response.fx,
                                force_response.fy,
                                force_response.fz
                                ]
                        else:
                            force_values = self.pm_robot_utils._current_force_sensor_data.data[:3]

                        self.csv_force_scan_step(
                            scan_number,
                            request,
                            search_iterations + refine_iterations, 
                            "SEARCH",
                            step_size_m_search * 1e6,
                            force_values,
                            current_position,
                            datetime.now().isoformat()
                        )

                        contact_detected = any(
                            abs(force_values[i]) > force_threshold[i]
                            for i in range(3)
                        )

                        if contact_detected:

                            contact_position = [
                                self.pm_robot_utils.get_current_joint_state(self.pm_robot_utils.X_Axis_JOINT_NAME),
                                self.pm_robot_utils.get_current_joint_state(self.pm_robot_utils.Y_Axis_JOINT_NAME),
                                self.pm_robot_utils.get_current_joint_state(self.pm_robot_utils.Z_Axis_JOINT_NAME)
                            ]

                            frame_position_contact = [
                                frame_position_gripper.transform.translation.x,
                                frame_position_gripper.transform.translation.y,
                                frame_position_gripper.transform.translation.z
                            ]
                            
                            if not self.pm_robot_utils.is_unity_running():
                                force_response = self.smart_gripper_force_client.call(piezo_force_request)

                                contact_force = [
                                    force_response.fx,
                                    force_response.fy,
                                    force_response.fz
                                ]
                            else:
                                contact_force = self.pm_robot_utils._current_force_sensor_data.data[:3]

                            self.csv_force_scan_step(
                                scan_number,
                                request,
                                search_iterations + refine_iterations, 
                                "CONTACT",
                                step_size_m_search * 1e6,
                                contact_force,
                                contact_position,
                                datetime.now().isoformat()
                            )

                            self.get_logger().info(f'force at current contact: {contact_force}')

                            state = "CONTACT"
                            continue

                        else:
                            # Kein Kontakt -> Position als gueltig speichern und weiterfahren
                            last_valid_position = current_position.copy()
                            travelled_distance_m += step_size_m_search
                            continue

                    elif state == "CONTACT":                    
                        # Zurueck zur letzten kontaktfreien Position fahren
                        self.pm_robot_utils.send_xyz_trajectory_goal_absolut(*last_valid_position,
                            time=0.05
                        )

                        travelled_distance_m -= step_size_m_search

                        time.sleep(0.5)

                        current_position = [
                            self.pm_robot_utils.get_current_joint_state(self.pm_robot_utils.X_Axis_JOINT_NAME),
                            self.pm_robot_utils.get_current_joint_state(self.pm_robot_utils.Y_Axis_JOINT_NAME),
                            self.pm_robot_utils.get_current_joint_state(self.pm_robot_utils.Z_Axis_JOINT_NAME)
                        ]

                        if not self.pm_robot_utils.is_unity_running():
                            force_response = self.smart_gripper_force_client.call(piezo_force_request)

                            current_force = [
                                force_response.fx,
                                force_response.fy,
                                force_response.fz
                            ]
                        else:
                            current_force = self.pm_robot_utils._current_force_sensor_data.data[:3]

                        self.csv_force_scan_step(
                            scan_number,
                            request,
                            search_iterations + refine_iterations, 
                            "moved back",
                            step_size_m_search * 1e6,
                            current_force,
                            current_position,
                            datetime.now().isoformat()
                        )

                        if step_size_m_search <= step_size_m_min:
                            # Ziel-Schrittgroesse bereits erreicht -> Scan beenden
                            state = "STOP"
                            continue

                        else:
                            # Noch nicht fein genug -> weiter verfeinern
                            state = "REFINE"
                            continue

                    elif state == "REFINE":
                        refine_iterations += 1
                        # Suchschrittgroesse halbieren, jedoch nicht unter die
                        # angeforderte Ziel-Schrittgroesse
                        step_size_m_search = max(step_size_m_search / 2, step_size_m_min)

                        state = "SEARCH"
                        continue

                    elif state == "STOP":
                        detected_position = list(contact_position)
                        break


                # Frame-Korrektur berechnen und anwenden
                # HINWEIS: Stiftradius wird hier noch nicht beruecksichtigt, da der
                # Wert noch nicht eingemessen wurde. Muss vor den eigentlichen
                # Versuchen ergaenzt werden, sonst verbleibt ein systematischer
                # Versatz in der Korrektur.

                # scene = self.pm_robot_utils.assembly_scene_analyzer
                # target_pose = scene.get_pose_from_frame(request.target_frame)

                # if detected_position is not None:
                #     correction_transform = self.compute_frame_correction(
                #         detected_position,
                #         target_pose
                #     )

                #     self.pm_robot_utils.assembly_scene_analyzer.modify_frame_with_transform(
                #         request.target_frame,
                #         correction_transform
                #     )

                scan_time = time.time() - scan_start_time
                contact_distance = np.dot(
                    np.array(detected_position) - np.array(start_position),
                    np.array(direction_normalized)
                ) if detected_position is not None else math.nan

                # Schritt 9: Antwort setzen
                if detected_position is None:
                    response.success = False
                    response.message = (
                    f'No contact detected after {contact_distance * 1e3:.3f} mm'                
                    )
                else:
                    response.success = True
                    response.message = (
                        f'Contact detected after {contact_distance * 1e3:.3f} mm'
                    )

                    response.detected_position.position.x = detected_position[0]
                    response.detected_position.position.y = detected_position[1]
                    response.detected_position.position.z = detected_position[2]
                    response.detected_position.orientation.w = 1.0

                    self.get_logger().info(
                        f'Contact position (World): '
                        f'X={round(detected_position[0]*1e3, 3)} mm, '
                        f'Y={round(detected_position[1]*1e3, 3)} mm, '
                        f'Z={round(detected_position[2]*1e3, 3)} mm'
                    )


                detected_position_log = [detected_position[i] for i in range(3)] if detected_position is not None else [math.nan, math.nan, math.nan]
                self.get_logger().info(
                    f"detected_position: "
                    f"{detected_position_log}"
                )

                # final_search_step_um = step_size_m_search * 1e6

                # success_flag = detected_position is not None                
                # self.csv_force_scan(
                #     scan_number,
                #     request,
                #     final_search_step_um,
                #     direction_normalized,
                #     detected_position_log,
                #     frame_position_contact,
                #     frame_position_target,
                #     start_position,
                #     contact_distance,
                #     contact_force,
                #     search_iterations,
                #     refine_iterations,
                #     scan_time,
                #     timestamp,
                #     success_flag
                # )
                # self.get_logger().info("Data saved to CSV.")

                # Schritt 10: Zurück zur Startposition (Anfahrposition)
                self.get_logger().info('Returning to start position...')

                returned = self._return_to_start(start_position)

                if not returned:
                    self.get_logger().error("Stopping repeatability test: return failed")
                    # break

                self.get_logger().info(response.message)
                time.sleep(0.5)

        except Exception as e:
            response.success = False
            response.message = f'Error: {str(e)}'
            self.get_logger().error(response.message)

        return response
    



    def _return_to_start(self, start_position: list) -> bool:
        """Hilfsfunktion: Faehrt zur angegebenen Startposition zurueck."""
        return_ok = self.pm_robot_utils.send_xyz_trajectory_goal_absolut(
            start_position[0],
            start_position[1],
            start_position[2],
            time=1.0
        )
        if not return_ok:
            self.get_logger().error('could not return to start position!')
        return return_ok

    def csv_force_scan_step(
            self,
            scan_number,
            request,
            step,
            state,
            step_size_um,
            current_force,
            current_position,
            timestamp
        ):

        import os
        import csv

        base_folder = (
            "/home/pmlab/pm_Server/01_PM_Zelle/03_PM_DataBase/pm_assembly_database/RSAP_Processes/Bente/documentation_and_plots/messungen_neu"
        )

        folder_name = (
            f"S{request.step_size}"
            f"_fx{request.max_force.x:.1f}"
            f"_fy{request.max_force.y:.1f}"
            f"_fz{request.max_force.z:.1f}" 
        )

        folder = os.path.join(base_folder, folder_name)

        data_folder = os.path.join(folder, "data")

        os.makedirs(data_folder, exist_ok=True)

        file_name = "search_refine_log.csv"
        file_path = os.path.join(data_folder, file_name)

        file_exists = os.path.isfile(file_path)

        fieldnames = [
            "ScanNumber",
            "Timestamp",
            "Step",
            "State",
            "StepSize_um",
            "Force_x",
            "Force_y",
            "Force_z",
            "Position_x",
            "Position_y",
            "Position_z",
        ]

        row = {
            "ScanNumber": scan_number,
            "Timestamp": timestamp,
            "Step": step,
            "State": state,
            "StepSize_um": step_size_um,
            "Force_x": current_force[0],
            "Force_y": current_force[1],
            "Force_z": current_force[2],
            "Position_x": current_position[0],
            "Position_y": current_position[1],
            "Position_z": current_position[2],
        }

        with open(file_path, "a", newline="") as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            if not file_exists:
                writer.writeheader()
            writer.writerow(row)


    def csv_force_scan(
        self,
        scan_number,
        request,
        final_search_step_um,
        direction_normalized,
        detected_position,
        frame_position_contact,
        frame_position_target,
        start_position,
        contact_distance,
        contact_force,
        search_iterations,
        refine_iterations,
        scan_time,
        timestamp,
        success_flag
    ):

        import os
        import csv
        import math
    
        base_folder = (
            "/home/pmlab/pm_Server/01_PM_Zelle/03_PM_DataBase/pm_assembly_database/RSAP_Processes/Bente/documentation_and_plots/messungen_neu"
        )

        folder_name = (
            f"S{request.step_size}"
            f"_fx{request.max_force.x:.1f}"
            f"_fy{request.max_force.y:.1f}"
            f"_fz{request.max_force.z:.1f}" 
        )

        folder = os.path.join(base_folder, folder_name)

        data_folder = os.path.join(folder, "data")

        os.makedirs(data_folder, exist_ok=True)


        file_name = "data.csv"
        file_path = os.path.join(data_folder, file_name)

        file_exists = os.path.isfile(file_path)


        fieldnames = [
            "ScanNumber",
            "Timestamp",
            "StepSize_um",
            "final_search_step_um",
            "Threshold_x",
            "Threshold_y",
            "Threshold_z",
            "Direction_x",
            "Direction_y",
            "Direction_z",
            "Start_x",
            "Start_y",
            "Start_z",
            "Contact_x",
            "Contact_y",
            "Contact_z",
            "Frame_x_contact",
            "Frame_y_contact",
            "Frame_z_contact",
            "Frame_x_target",
            "Frame_y_target",
            "Frame_z_target",
            "Contact_distance_um",
            "Force_x_contact",
            "Force_y_contact",
            "Force_z_contact",
            "Search_iterations",
            "Refine_iterations",
            "Scan_time_s",
            "Success"
        ]

        def safe_vec3(v):
            if v is None or len(v) != 3:
                return (math.nan, math.nan, math.nan)
            return v

        fx, fy, fz = safe_vec3(frame_position_contact)
        cx, cy, cz = safe_vec3(detected_position) if success_flag else (math.nan, math.nan, math.nan)

        with open(file_path, mode='a', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames, delimiter=';')

            if not file_exists:
                writer.writeheader()

            writer.writerow({
                "ScanNumber": scan_number,
                "Timestamp": timestamp,
                "StepSize_um": request.step_size,
                "final_search_step_um": final_search_step_um,
                "Threshold_x": request.max_force.x,
                "Threshold_y": request.max_force.y,
                "Threshold_z": request.max_force.z,
                "Direction_x": direction_normalized[0],
                "Direction_y": direction_normalized[1],
                "Direction_z": direction_normalized[2],
                "Start_x": start_position[0],
                "Start_y": start_position[1],
                "Start_z": start_position[2],
                "Contact_x": cx,
                "Contact_y": cy,
                "Contact_z": cz,
                "Frame_x_contact": fx,
                "Frame_y_contact": fy,
                "Frame_z_contact": fz,
                "Frame_x_target": frame_position_target[0],
                "Frame_y_target": frame_position_target[1],
                "Frame_z_target": frame_position_target[2],
                "Contact_distance_um": contact_distance * 1e6,
                "Force_x_contact": contact_force[0],
                "Force_y_contact": contact_force[1],
                "Force_z_contact": contact_force[2],
                "Search_iterations": search_iterations,
                "Refine_iterations": refine_iterations,
                "Scan_time_s": scan_time,
                "Success": int(success_flag)
            })


    def iterative_align_gonio_right(self, request: pm_skill_srv.IterativeGonioAlign.Request, response:pm_skill_srv.IterativeGonioAlign.Response):
        
        self.pm_robot_utils.assembly_scene_analyzer.wait_for_initial_scene_update()

        try:

            # the request never changes
            align_request = pm_moveit_srv.AlignGonio.Request()
            align_request.execute_movement = True
            align_request.endeffector_frame_override = request.component_alignment_frame
            align_request.target_frame = request.target_alignment_frame

            log_message = {}
            iterations = request.num_iterations

            initial_approach = True
            for iter in range(iterations):
                run_number = iter + 1
                self.logger.info(f"STARTING RUN '{run_number}/{iterations}")
            
                for frame in request.frames_to_measure:
                    self.logger.info(f"Measuring frame '{frame}'")
                    measure_request = pm_skill_srv.CorrectFrameLaser.Request()
                    measure_response = pm_skill_srv.CorrectFrameLaser.Response()
                    measure_request.frame_name = frame
                    measure_request.use_iterative_sensing = True

                    if initial_approach:
                        move_success, move_msg = self.move_laser_to_frame(frame, z_offset=0.02)
                        if not move_success:
                            self.logger.error(f"Moving laser to frame '{frame}' for initial approach failed!")
                            response.success = False
                            response.message = f"Moving laser to frame '{frame}' for initial approach failed: {move_msg}"
                            return response
                        initial_approach = False

                    if request.confocal_laser:
                        measure_response = self.correct_frame_with_confocal_top(measure_request, measure_response)
                    else:
                        measure_response = self.correct_frame_with_laser(measure_request, measure_response)

                    if not measure_response.success:
                        raise PmRobotError(f"Correcting frame '{frame}' failed! {measure_response.message}")

                # calculating the angle difference
                try:
                    transform: Transform = self.pm_robot_utils.get_transform_for_frame(request.component_alignment_frame, 
                                                                            request.target_alignment_frame)
                    
                    current_gonio_angles: Transform = self.pm_robot_utils.get_transform_for_frame(request.component_alignment_frame, 
                                                                                                    'world')
                except ValueError as e:
                    self.logger.error(f"Error: {e}")
                    self.logger.error(f"Frame '{frame}' does not seem to exist.")
                    raise PmRobotError(f"Frame '{frame}' does not seem to exist: {e}")

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
                    raise PmRobotError(f"Aligning goniometer failed! {align_response.message}")
                
                time.sleep(1)  # wait for the robot to settle

                joint_1_post = self.pm_robot_utils.get_current_joint_state(self.pm_robot_utils.GONIO_RIGHT_STAGE_1)
                joint_2_post = self.pm_robot_utils.get_current_joint_state(self.pm_robot_utils.GONIO_RIGHT_STAGE_2)

                difference_joint_1 = (joint_1_post - joint_1_pre)*180/np.pi
                difference_joint_2 = (joint_2_post - joint_2_pre)*180/np.pi

                log_message[iter] = f"Iteration {run_number}: Gonio joints moved by {round(difference_joint_1, 5)} and {round(difference_joint_2, 5)} (deg)"
                self.logger.warn(log_message[iter])

            self.logger.info(f"Success")
            move_relative_request = MoveRelative.Request()
            move_relative_request.execute_movement = True
            move_relative_request.translation.z = 0.03


            move_relative_response: MoveRelative.Response = self.pm_robot_utils.client_move_laser_relative.call(move_relative_request)

            if not move_relative_response.success:
                raise PmRobotError(f"Endmove relative failed! {move_relative_response.message}")

            response.success = True
            response.message = "Iterative gonio right alignment completed successfully! \n Frames measured: " + ", ".join(request.frames_to_measure) + "\n"+ "\n".join(log_message.values())

        except PmRobotError as e:
            self.logger.error(f"Error occurred: {e}")
            response.success = False
            response.message = str(e)

        return response

    def iterative_align_gonio_left(self, request: pm_skill_srv.IterativeGonioAlign.Request, response:pm_skill_srv.IterativeGonioAlign.Response):
        
        self.pm_robot_utils.assembly_scene_analyzer.wait_for_initial_scene_update()

        if not self.pm_robot_utils.client_align_gonio_left.wait_for_service(1):
            self.logger.error(f"Client '{self.pm_robot_utils.client_align_gonio_left.srv_name}' not available!")
            response.success = False
            response.message = f"Client '{self.pm_robot_utils.client_align_gonio_left.srv_name}' not available!"
            return response

        # the request never changes
        align_request = pm_moveit_srv.AlignGonio.Request()
        align_request.execute_movement = True
        align_request.endeffector_frame_override = request.component_alignment_frame
        align_request.target_frame = request.target_alignment_frame

        iterations = request.num_iterations

        initial_approach = True

        log_message = {}

        for iter in range(iterations):
            run_number = iter + 1
            self.logger.info(f"STARTING RUN '{run_number}/{iterations}")

            for frame in request.frames_to_measure:
                self.logger.info(f"Measuring frame '{frame}'")
                measure_request = pm_skill_srv.CorrectFrameLaser.Request()
                measure_response = pm_skill_srv.CorrectFrameLaser.Response()
                measure_request.frame_name = frame
                measure_request.use_iterative_sensing = True

                if initial_approach:
                    move_success, move_msg = self.move_laser_to_frame(frame, z_offset=0.02)
                    if not move_success:
                        response.success = False
                        response.message = f"Moving laser to frame '{frame}' for initial approach failed: {move_msg}"
                        self.logger.error(response.message)
                        return response
                    initial_approach = False
                
                if request.confocal_laser:
                    measure_response = self.correct_frame_with_confocal_top(measure_request, measure_response)
                else:
                    measure_response = self.correct_frame_with_laser(measure_request, measure_response)

                if not measure_response.success:
                    response.success = False
                    response.message = f"Correcting frame '{frame}' failed! {measure_response.message}"
                    self.logger.error(response.message)
                    return response
                
            # calculating the angle difference 
            try:
                transform: Transform = self.pm_robot_utils.get_transform_for_frame(request.component_alignment_frame, 
                                                                        request.target_alignment_frame)
            except ValueError as e:
                self.logger.error(f"Error: {e}")
                self.logger.error(f"Frame '{frame}' does not seem to exist.")
                response.success = False
                response.message = f"Frame '{frame}' does not seem to exist: {e}"
                return response

            
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
                response.message = "Aligning goniometer left failed! " + align_response.message
                self.logger.error(response.message)
                return response
            
            time.sleep(1)  # wait for the robot to settle

            joint_1_post = self.pm_robot_utils.get_current_joint_state(self.pm_robot_utils.GONIO_LEFT_STAGE_1)
            joint_2_post = self.pm_robot_utils.get_current_joint_state(self.pm_robot_utils.GONIO_LEFT_STAGE_2)

            difference_joint_1 = (joint_1_post - joint_1_pre)*180/np.pi
            difference_joint_2 = (joint_2_post - joint_2_pre)*180/np.pi

            log_message[iter] = f"Iteration {run_number}: Gonio joints moved by {round(difference_joint_1,5)} and {round(difference_joint_2,5)} (deg)"
            self.logger.warn(log_message[iter])

        self.logger.info(f"Success")
        move_relative_request = MoveRelative.Request()
        move_relative_request.execute_movement = True
        move_relative_request.translation.z = 0.03

        move_relative_response: MoveRelative.Response = self.pm_robot_utils.client_move_laser_relative.call(move_relative_request)

        if not move_relative_response.success:
            response.success = False
            response.message = "Endmove relative failed! " + move_relative_response.message
            self.logger.error(response.message)
            return response
        
        response.success = True
        response.message = "Iterative gonio right alignment completed successfully! \n Frames measured: " + ", ".join(request.frames_to_measure) + "\n"+ "\n".join(log_message.values())

        return response

    def force_grip_component_callback(self, request:pm_skill_srv.GripComponent.Request, response:pm_skill_srv.GripComponent.Response):
        """TO DO: Add docstring"""

        self.pm_robot_utils.wait_for_initial_scene_update()

        try:
            should_move_up_at_error = False
            self.logger.info(f"Force gripping component '{request.component_name}'")

            if not self.pm_robot_utils.assembly_scene_analyzer.check_object_exists(request.component_name):
                raise PmRobotError(f"Object '{request.component_name}' does not exist!")

            self.logger.info(f"Force gripping component '{request.component_name}'")

            if not self.pm_robot_utils.assembly_scene_analyzer.is_gripper_empty():
                raise PmRobotError("Gripper is not empty! Can not grip new component!")

            if self.pm_robot_utils.assembly_scene_analyzer.check_component_assembled(request.component_name):
                raise PmRobotError(f"Component '{request.component_name}' is already assembled! Can not grip it!")
            
            gripping_frame = self.pm_robot_utils.assembly_scene_analyzer.get_gripping_frame_of_component(request.component_name)
            
            self.logger.info(f"Gripping component '{request.component_name}' at frame '{gripping_frame}'")
            
            algin_success = True
            align_msg = ""

            if self.pm_robot_utils.assembly_scene_analyzer.is_object_on_gonio_left(request.component_name) and request.align_orientation:
                algin_success, align_msg = self.align_gonio_left(gripping_frame)

            elif self.pm_robot_utils.assembly_scene_analyzer.is_object_on_gonio_right(request.component_name) and request.align_orientation:
                algin_success, align_msg = self.align_gonio_right(gripping_frame)
            
            else:
                pass

            if not algin_success:
                raise PmRobotError(f"Failed to align gonio for component '{request.component_name}': {align_msg}")

            #Offset
            move_tool_to_part_offset_success, move_offset_msg = self.move_gripper_to_frame(gripping_frame, z_offset=self.GRIP_APPROACH_OFFSET)

            if not move_tool_to_part_offset_success:
                raise PmRobotError(f"Failed to move gripper to part '{request.component_name}' at offset distance: {move_offset_msg}")
            

            self.logger.info(f"Moving to grip sensing start position!")

            move_tool_to_part_success, move_part_msg = self.move_gripper_to_frame(gripping_frame, z_offset=self.GRIP_SENSING_START_OFFSET)

            if not move_tool_to_part_success:
                raise PmRobotError(f"Failed to move gripper to part '{request.component_name}': {move_part_msg}")

            should_move_up_at_error = True

            self.pm_robot_utils.set_gripper_component_collision(component_name=request.component_name, 
                                                                state=False)
            
            # This is a position that is above the gripping point for a rough approach
            planning_req_rough = pm_moveit_srv.MoveToFrame.Request()
            planning_req_rough.target_frame = gripping_frame
            planning_req_rough.execute_movement = False
            planning_req_rough.translation.z = self.GRIP_SENSING_END_ROUGH

            planning_res_rough:pm_moveit_srv.MoveToFrame.Response = self.client_move_robot_tool_to_frame.call(planning_req_rough)

            if planning_res_rough.success is False:
                raise PmRobotError(f"Planning of END_ROUGH_OFFSET position failed!")
            
            # calculating the lower position
            planning_req = pm_moveit_srv.MoveToFrame.Request()
            planning_req.target_frame = gripping_frame
            planning_req.execute_movement = False
            planning_req.translation.z = self.GRIP_SENSING_END_OFFSET

            planning_res:pm_moveit_srv.MoveToFrame.Response = self.client_move_robot_tool_to_frame.call(planning_req)
            
            if planning_res.success is False:
                raise PmRobotError(f"Planning of END_OFFSET position failed!")
            
            if not self.pm_robot_utils.is_unity_running():
            # Iterative approach  
                force_sensing_rough_request = pm_msg_srv.GripperForceMove.Request()
                force_sensing_rough_response = pm_msg_srv.GripperForceMove.Response()

                force_sensing_rough_request.step_size = float(100) # in um
                force_sensing_rough_request.target_joints_xyz = [planning_res_rough.joint_values[0], planning_res_rough.joint_values[1], planning_res_rough.joint_values[2]]
                force_sensing_rough_request.max_f_xyz = [1.0, 1.0, 1.0] # in N

                self.logger.info(f"Starting rough approach with force sensing (step size: {force_sensing_rough_request.step_size} um. Approaching gripping point at {self.GRIP_SENSING_END_ROUGH*1e6} um above it)")
                
                force_sensing_rough_response = self.force_sensing_move_callback(force_sensing_rough_request, force_sensing_rough_response)
                
                if not force_sensing_rough_response.success:
                    raise PmRobotError(f"Rough approach force sensing failed!")
                
                if force_sensing_rough_response.completed:
                    raise PmRobotError(f"Rough approach already exceeded the force limits! Make sure the gripping frame has been measured correctly!")
                else:
                    self.logger.info(f"Rough approach completed without exceeding force limits, proceeding to fine approach.")

                self.logger.warn(f"joints {str(planning_res.joint_names)}")
                self.logger.warn(f"joints {str(planning_res.joint_values)}")


            # Iterative approach  
            force_sensing_request = pm_msg_srv.GripperForceMove.Request()
            force_sensing_response = pm_msg_srv.GripperForceMove.Response()

            force_sensing_request.step_size = float(10) # in um
            force_sensing_request.target_joints_xyz = [planning_res.joint_values[0], planning_res.joint_values[1], planning_res.joint_values[2]]
            force_sensing_request.max_f_xyz = [1.0, 1.0, 1.0] # in N

            self.logger.info(f"Starting fine approach with force sensing (step size: {force_sensing_request.step_size} um. ")

            force_sensing_response = self.force_sensing_move_callback(force_sensing_request, force_sensing_response)
            
            if not force_sensing_response.success:
                raise PmRobotError(f"Force sensing failed!")
            
            if not force_sensing_response.completed:
                raise PmRobotError(f"Fine approach did not detect contact! Make sure the gripping frame has been measured correctly and that the force thresholds are set appropriately!")
            
            # enable vacuum
            enable_success = self.pm_robot_utils.set_tool_vaccum(True)

            if not enable_success:
                raise PmRobotError(f"Failed to enable vacuum for component '{request.component_name}'!")

            disable_success = True

            # turn off the vacuum
            if self.pm_robot_utils.assembly_scene_analyzer.is_object_on_gonio_left(request.component_name, max_depth=1):
                self.logger.info(f"Disabling vacuum on gonio left for component '{request.component_name}'")
                disable_success = self.pm_robot_utils.set_gonio_left_vacuum(False)

            elif self.pm_robot_utils.assembly_scene_analyzer.is_object_on_gonio_right(request.component_name, max_depth=1):
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
            
            properties = ami_msg.ComponentProperties()
            properties.is_gripped = True

            set_properties_response: ami_srv.SetComponentProperties.Response = self.pm_robot_utils.set_component_properties(request.component_name, properties)

            if not set_properties_response.success:
                raise PmRobotError(f"Failed to set component properties for component '{request.component_name}' after gripping!")  
            
            response.success = True

            response.message = f"Component '{request.component_name}' gripped successfully!"
            self.logger.info(response.message)
            time.sleep(0.5)  # wait for properties to update in the assembly manager

        except (PmRobotError, ComponentNotFoundError, GrippingFrameNotFoundError) as e:
            response.success = False
            response.message = str(e)
            self.logger.error(response.message)

        finally:
            if should_move_up_at_error:
                move_relatively_success, lift_msg = self.lift_gripper_relative(self.GRIP_RELATIVE_LIFT_DISTANCE)
                if not move_relatively_success:
                    response.success = False
                    response.message = f"Failed to lift gripper after attaching component '{request.component_name}': {lift_msg}"
                    self.logger.error(response.message)

        return response
    
    def grip_component_callback(self, request:pm_skill_srv.GripComponent.Request, response:pm_skill_srv.GripComponent.Response):
        """TO DO: Add docstring"""

        self.pm_robot_utils.assembly_scene_analyzer.wait_for_initial_scene_update()
        
        raise PmRobotError(f"This skill should not be used!")
    
        try:
            if not self.pm_robot_utils.assembly_scene_analyzer.check_object_exists(request.component_name):
                response.success = False
                response.message = f"Object '{request.component_name}' does not exist!"
                self.logger.error(response.message)
                return response

            if not self.pm_robot_utils.assembly_scene_analyzer.is_gripper_empty():
                response.success = False
                response.message = "Gripper is not empty! Can not grip new component!"
                self.logger.error(response.message)
                return response
            
        except ValueError as e:
            response.success = False
            response.message = str(e)
            self.logger.error(response.message)
            return response
        
        gripping_frame = self.pm_robot_utils.assembly_scene_analyzer.get_gripping_frame(request.component_name)

        if gripping_frame is None:
            response.success = False
            response.message = f"No gripping frame found for object '{request.component_name}'"
            self.logger.error(response.message)
            return response
        
        self.logger.info(f"Gripping component '{request.component_name}' at frame '{gripping_frame}'")

        if self.pm_robot_utils.assembly_scene_analyzer.is_object_on_gonio_left(request.component_name):
            algin_success, align_msg = self.align_gonio_left(gripping_frame)

        elif self.pm_robot_utils.assembly_scene_analyzer.is_object_on_gonio_right(request.component_name):
            algin_success, align_msg = self.align_gonio_right(gripping_frame)
        
        else:
            response.success = False
            response.message = "Object not on gonio!"
            self.logger.error("Object not on gonio!")
            return response
        
        if not algin_success:
            response.success = False
            response.message = f"Failed to align gonio for component '{request.component_name}': {align_msg}"
            self.logger.error(response.message)
            return response
        
        #Offset
        move_tool_to_part_offset_success, move_offset_msg = self.move_gripper_to_frame(gripping_frame, z_offset=self.RELEASE_LIFT_DISTANCE)

        if not move_tool_to_part_offset_success:
            response.success = False
            response.message = f"Failed to move gripper to part '{request.component_name}' at offset distance: {move_offset_msg}"
            self.logger.error(response.message)
            return response
        
        move_tool_to_part_success, move_part_msg = self.move_gripper_to_frame(gripping_frame,z_offset=self.GRIPPING_OFFSET)

        if not move_tool_to_part_success:
            response.success = False
            response.message = f"Failed to move gripper to part '{request.component_name}': {move_part_msg}"
            self.logger.error(response.message)
            return response
        
        attach_component_success = self.attach_component_to_gripper(request.component_name)

        if not attach_component_success:
            move_relatively_success, lift_msg = self.lift_gripper_relative(self.GRIP_RELATIVE_LIFT_DISTANCE)
            response.success = False
            response.message = f"Failed to attach component '{request.component_name}' to gripper"
            self.logger.error(response.message)
            return response
        
        move_relatively_success, lift_msg = self.lift_gripper_relative(self.GRIP_RELATIVE_LIFT_DISTANCE)

        if not move_relatively_success:
            response.success = False
            response.message = f"Failed to lift gripper after attaching component '{request.component_name}': {lift_msg}"
            self.logger.error(response.message)
            return response
        
        self.logger.info(f"Move tool relative success: {move_relatively_success}")

        response.success = True

        response.message = f"Component '{request.component_name}' gripped successfully!"
        self.logger.info(response.message)

        return response

    def place_component_callback(self, request:pm_skill_srv.PlaceComponent.Request, response:pm_skill_srv.PlaceComponent.Response):
        self.pm_robot_utils.assembly_scene_analyzer.wait_for_initial_scene_update()

        try:
            should_move_up_at_error = False

            if self.pm_robot_utils.assembly_scene_analyzer.is_gripper_empty():
                raise PmRobotError(f"Gripper is empty! Can not place component!")
            
            gripped_component = self.pm_robot_utils.assembly_scene_analyzer.get_gripped_component()
            
            assembly_frame = self.pm_robot_utils.assembly_scene_analyzer.get_assembly_frame_for_component(gripped_component)

            target_frame = self.pm_robot_utils.assembly_scene_analyzer.get_target_frame_for_component(gripped_component)

            self.logger.warn(f"Gripped Component: '{str(gripped_component)}'!")
            self.logger.warn(f"Assembly Frame: '{str(assembly_frame)}'!")
            self.logger.warn(f"Target Frame: '{str(target_frame)}'!")

            target_component = self.pm_robot_utils.assembly_scene_analyzer.get_component_for_frame_name(target_frame)
            
            instruction = self.pm_robot_utils.assembly_scene_analyzer.get_assembly_instruction(assembly_component=gripped_component,
                                                                                                  target_component=target_component)

            # recalculate the instruction before creating the place frame
            self.pm_robot_utils.recalculate_assembly_instruction(instruction_id=instruction.id)

            placing_frame = self._create_place_offset_frame(target_frame, request)

            placing_frame = target_frame

            # Align gonio to the left or right depending on the target frame

            algin_success = True
            align_msg = ""

            if self.pm_robot_utils.assembly_scene_analyzer.is_object_on_gonio_left(target_component) and request.align_orientation:
                algin_success, align_msg = self.align_gonio_left(endeffector_override=placing_frame,
                                                        alignment_frame=assembly_frame)

            elif self.pm_robot_utils.assembly_scene_analyzer.is_object_on_gonio_right(target_component) and request.align_orientation:
                algin_success, align_msg = self.align_gonio_right(endeffector_override=placing_frame,
                                                        alignment_frame=assembly_frame)

            else:
                pass

            if not algin_success:
                raise PmRobotError(f"Failed to align gonio for target component '{target_component}': {align_msg}")
            
            self.logger.warn("Endeffector override: " + assembly_frame)
            # Move component to the target frame with a z offset
            move_component_to_part_offset_success, move_offset_msg = self.move_gripper_to_frame(placing_frame, 
                                                                               endeffector_override=assembly_frame, 
                                                                               z_offset=self.RELEASE_LIFT_DISTANCE)

            if not move_component_to_part_offset_success:
                raise PmRobotError(f"Failed to move gripper to target part '{target_component}': {move_offset_msg}")

            should_move_up_at_error = True

            # Move component to the target frame
            move_component_to_part_success, move_part_msg = self.move_gripper_to_frame(placing_frame, 
                                                                        endeffector_override=assembly_frame,
                                                                        z_offset=0.0)
            
            if not move_component_to_part_success:
                raise PmRobotError(f"Failed to move gripper to target part '{target_component}': {move_part_msg}")
            
                      
            properties = ami_msg.ComponentProperties()
            properties.is_gripped = True
            properties.is_placed = True

            set_properties_response: ami_srv.SetComponentProperties.Response = self.pm_robot_utils.set_component_properties(gripped_component, properties)

            if not set_properties_response.success:
                raise PmRobotError(f"Failed to set component properties for component '{gripped_component}' after placing!")  
            
            response.success = True
            response.message = f"Component '{gripped_component}' placed successfully!"

        except (PmRobotError, 
                ComponentNotFoundError, 
                RefFrameNotFoundError, 
                AssemblyFrameNotFoundError,
                AssemblyInstructionNotFoundError,
                TargetFrameNotFoundError) as e:
            if should_move_up_at_error:
                move_relatively_success, lift_msg = self.lift_gripper_relative(self.RELEASE_LIFT_DISTANCE)
                if not move_relatively_success:
                    response.success = False
                    response.message = f"Failed to lift gripper after placing component '{gripped_component}': {lift_msg}"
                    self.logger.error(response.message)
            response.success = False
            response.message = str(e)
            self.logger.error(response.message)
            
        finally:
            pass

        return response

    def _create_place_offset_frame(self, 
                                  target_frame, 
                                  request:pm_skill_srv.PlaceComponent.Request)->str:
        """
        Create a new frame for placing the component with an offset
        Args:
            target_frame (str): The name of the target frame to place the component
            request (pm_skill_srv.PlaceComponent.Request): The request containing the offset values
        Returns:
            str: The name of the created offset frame
        Raises:
            PmRobotError: If the frame creation fails
            RefFrameNotFoundError: If the target frame does not exist
            ComponentNotFoundError: If the target component does not exist
        """

        target_frame_obj = self.pm_robot_utils.assembly_scene_analyzer.get_ref_frame_by_name(target_frame)
        target_component = self.pm_robot_utils.assembly_scene_analyzer.get_component_for_frame_name(target_frame)

        #generate random number
        random_number = random.randint(1000, 9999)
        spawn_request = ami_srv.CreateRefFrame.Request()
        spawn_request.ref_frame.frame_name = f"{target_component}_PLACEMENT_offset_{random_number}"
        spawn_request.ref_frame.parent_frame = target_component
        spawn_request.ref_frame.pose.position.x = target_frame_obj.pose.position.x + request.x_offset_um * 1e-6
        spawn_request.ref_frame.pose.position.y = target_frame_obj.pose.position.y + request.y_offset_um * 1e-6
        spawn_request.ref_frame.pose.position.z = target_frame_obj.pose.position.z + request.z_offset_um * 1e-6

        # convert euler angles to quaternion
        if request.rx_offset_deg != 0.0 or request.ry_offset_deg != 0.0 or request.rz_offset_deg != 0.0:
            q = R.from_euler('xyz', [request.rx_offset_deg, request.ry_offset_deg, request.rz_offset_deg], degrees=True).as_quat()
            quat = Quaternion()
            quat.x = q[0]
            quat.y = q[1]
            quat.z = q[2]
            quat.w = q[3]
            # multiply quaternions
            # current orientation
            result_quat = quaternion_multiply(target_frame_obj.pose.orientation, quat)
            spawn_request.ref_frame.pose.orientation = result_quat
        else:
            spawn_request.ref_frame.pose.orientation = target_frame_obj.pose.orientation
        
        spawn_response:ami_srv.CreateRefFrame.Response = self.pm_robot_utils.create_ref_frame(spawn_request)

        return spawn_request.ref_frame.frame_name

    def release_component_callback(self, request:EmptyWithSuccess.Request, response:EmptyWithSuccess.Response):
        self.pm_robot_utils.assembly_scene_analyzer.wait_for_initial_scene_update()

        try:
            if self.pm_robot_utils.assembly_scene_analyzer.is_gripper_empty():
                raise PmRobotError(f"Gripper is empty! Can not release component!")

            gripped_component = self.pm_robot_utils.assembly_scene_analyzer.get_gripped_component()
            
            if gripped_component is None:
                raise PmRobotError(f"No gripped component found!")
            
            gripped_component = self.pm_robot_utils.assembly_scene_analyzer.get_gripped_component()
            
            assembly_frame = self.pm_robot_utils.assembly_scene_analyzer.get_assembly_frame_for_component(gripped_component)

            target_frame = self.pm_robot_utils.assembly_scene_analyzer.get_target_frame_for_component(gripped_component)

            target_component = self.pm_robot_utils.assembly_scene_analyzer.get_component_for_frame_name(target_frame)

            self.logger.warn(f"Gripped Component: '{str(gripped_component)}'!")
            self.logger.warn(f"Assembly Frame: '{str(assembly_frame)}'!")
            self.logger.warn(f"Target Frame: '{str(target_frame)}'!")

            # release component
            attach_component_success = self.attach_component_to_component(gripped_component, target_component)

            if not attach_component_success:
                raise PmRobotError(f"Failed to attach component '{gripped_component}' to component '{target_component}'")
            
            vaccum_off_success = self.pm_robot_utils.set_tool_vaccum(False)

            if not vaccum_off_success:
                raise PmRobotError(f"Failed to deactivate vacuum for component '{gripped_component}'")

            move_relatively_success, lift_msg = self.lift_gripper_relative(self.RELEASE_LIFT_DISTANCE)
            if not move_relatively_success:
                raise PmRobotError(f"Failed to lift gripper after releasing component '{gripped_component}': {lift_msg}")

            self.pm_robot_utils.set_gripper_component_collision(component_name=gripped_component, 
                                                    state=True)
            
            properties = ami_msg.ComponentProperties()
            properties.is_gripped = False
            properties.is_placed = True

            set_properties_response: ami_srv.SetComponentProperties.Response = self.pm_robot_utils.set_component_properties(gripped_component, properties)

            if not set_properties_response.success:
                raise PmRobotError(f"Failed to set component properties for component '{gripped_component}' after releasing!")  
            
            response.success = True
            response.message = f"Component '{gripped_component}' released successfully. New parent: '{target_component}'!"
            
        except (PmRobotError, 
                ComponentNotFoundError, 
                RefFrameNotFoundError, 
                AssemblyFrameNotFoundError,
                TargetFrameNotFoundError) as e:

                response.message = str(e)
                self.logger.error(response.message)
                response.success = False

        return response


    # def assemble_callback(self, request:EmptyWithSuccess.Request, response:EmptyWithSuccess.Response):
    #     self.pm_robot_utils.assembly_scene_analyzer.wait_for_initial_scene_update()

    #     global_stationary_component = self.pm_robot_utils.assembly_scene_analyzer.get_global_stationary_component()
    #     assemble_list = self.pm_robot_utils.assembly_scene_analyzer.get_components_to_assemble()
    #     first_component = self.pm_robot_utils.assembly_scene_analyzer.find_matches_for_component(global_stationary_component, only_unassembled=False)

    #     self.assembly_loop(first_component)
        
    #     response.success = True

    #     return response

    def vaccum_gripper_on_callback(self, request:EmptyWithSuccess.Request, response:EmptyWithSuccess.Response):
        """Mimics the vacuum gripper funtionality by activating the vacuum and changing the parent frame"""

        self.pm_robot_utils.assembly_scene_analyzer.wait_for_initial_scene_update()

        sim_time = self.pm_robot_utils.is_gazebo_running() or self.pm_robot_utils.is_unity_running()
        self.logger.info(f"Gazebo Simulation: {sim_time}")

        assembly_and_target_frames = self.pm_robot_utils.assembly_scene_analyzer.get_all_assembly_and_target_frames()      

        try:
            if not self.pm_robot_utils.assembly_scene_analyzer.is_gripper_empty():
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
                response_vacuum:EmptyWithSuccess.Response = self.pm_robot_utils.client_turn_on_vacuum_tool_head.call(EmptyWithSuccess.Request())
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

        sim_time = self.pm_robot_utils.is_gazebo_running() or self.pm_robot_utils.is_unity_running()
        self.logger.info(f"Gazebo Simulation: {sim_time}")

        assembly_and_target_frames = self.pm_robot_utils.assembly_scene_analyzer.get_all_assembly_and_target_frames()

        try:
            if not sim_time:
                response_vacuum:EmptyWithSuccess.Response = self.pm_robot_utils.client_turn_off_vacuum_tool_head.call(EmptyWithSuccess.Request())
                if not response_vacuum.success:
                    response.success = False
                    response.message = "Failed to deactivate vacuum!"
                    self.logger.error(response.message)
                    return response
            
            # get target component
            target_component = None
            for component_frame in assembly_and_target_frames:
                if self.TARGET_FRAME_INDICATOR in component_frame[1]:
                    target_component = component_frame[0]
                    self.logger.info(f"target component: '{target_component}'")
                    break

            if target_component is None:
                raise ValueError(f"No target component found in assembly_and_target_frames (indicator: '{self.TARGET_FRAME_INDICATOR}')")

            response_attachment = self.attach_component_to_component(self.pm_robot_utils.assembly_scene_analyzer.get_gripped_component(), target_component)
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

            
    # def assembly_loop(self, list_of_components:list[str]):
    #     self.pm_robot_utils.assembly_scene_analyzer.wait_for_initial_scene_update()
    #     for component in list_of_components:
    #         self.logger.error(f"ComponentTTTTT: {component}")
    #         if not self.pm_robot_utils.assembly_scene_analyzer.check_component_assembled(component):

    #             response = self.grip_component_callback(pm_skill_srv.GripComponent.Request(component_name=component), pm_skill_srv.GripComponent.Response())
    #             if not response.success:
    #                 self.logger.error(f"Failed to grip component '{component}'")
    #                 return False
                
    #             response = self.place_component_callback(pm_skill_srv.PlaceComponent.Request(), pm_skill_srv.PlaceComponent.Response())
    #             if not response.success:
    #                 self.logger.error(f"Failed to place component '{component}'")
    #                 return False
                
    #         list_of_component= self.pm_robot_utils.assembly_scene_analyzer.find_matches_for_component(component,only_unassembled=False)
    #         self.logger.warn(f"List of components: {list_of_component}")
    #         self.assembly_loop(list_of_component)

    #     return True
    
    def simtime_callback(self, msg:std_msg.Bool):
        if msg.data:
            self.logger.info("Simulation time is active!")
            self.sim_time = True
        else:
            self.logger.info("Simulation time is inactive!")
            self.sim_time = False

    def lift_gripper_relative(self, distance:float)-> tuple[bool, str]:
        call_async = False

        if not self.pm_robot_utils.client_move_robot_tool_relative.wait_for_service(timeout_sec=1.0):
            self.logger.error("Service '/pm_moveit_server/move_tool_relative' not available")
            return False, "Service '/pm_moveit_server/move_tool_relative' not available"
        
        req = pm_moveit_srv.MoveRelative.Request()
        req.translation.z = distance
        req.execute_movement = True

        if call_async:
            future = self.pm_robot_utils.client_move_robot_tool_relative.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is None:
                self.logger.error('Service call failed %r' % (future.exception(),))
                return False, f"Service call failed: {future.exception()}"
            return future.result().success, future.result().message
        else:
            response:pm_moveit_srv.MoveRelative.Response = self.pm_robot_utils.client_move_robot_tool_relative.call(req)
            return response.success, response.message
        
    def move_gripper_to_frame(self, frame_name:str, endeffector_override = None, x_offset=None, y_offset=None, z_offset=None)-> tuple[bool, str]:
        call_async = False

        if not self.client_move_robot_tool_to_frame.wait_for_service(timeout_sec=1.0):
            self.logger.error("Service '/pm_moveit_server/move_tool_to_frame' not available")
            return False, "Service '/pm_moveit_server/move_tool_to_frame' not available"
        
        req = pm_moveit_srv.MoveToFrame.Request()
        req.target_frame = frame_name
        req.execute_movement = True
        req.translation.x = 0.0000001
        req.translation.y = 0.0000001
        req.translation.z = 0.0000001

        if endeffector_override is not None:
            req.endeffector_frame_override = endeffector_override

        if x_offset is not None:
            req.translation.x = x_offset

        if y_offset is not None:
            req.translation.y = y_offset

        if z_offset is not None:
            req.translation.z = z_offset

        if call_async:
            future = self.client_move_robot_tool_to_frame.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is None:
                self.logger.error('Service call failed %r' % (future.exception(),))
                return False, f"Service call failed: {future.exception()}"
            return future.result().success, future.result().message
        else:
            response:pm_moveit_srv.MoveToFrame.Response = self.client_move_robot_tool_to_frame.call(req)
            return response.success, response.message

    def move_laser_to_frame(self, frame_name:str, z_offset: float=None)-> tuple[bool, str]:
        """
        z_offset in m
        """
        call_async = False

        if not self.pm_robot_utils.client_move_robot_laser_to_frame.wait_for_service(timeout_sec=1.0):
            self.logger.error("Service '/pm_moveit_server/move_laser_to_frame' not available")
            return False, "Service '/pm_moveit_server/move_laser_to_frame' not available"
        
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
                return False, f"Service call failed: {future.exception()}"
            return future.result().success, future.result().message
        else:
            response:pm_moveit_srv.MoveToFrame.Response = self.pm_robot_utils.client_move_robot_laser_to_frame.call(req)
            return response.success, response.message
    
      
    def measure_with_laser_callback(self, 
                                    request:pm_skill_srv.CorrectFrameLaser.Request, 
                                    response:pm_skill_srv.CorrectFrameLaser.Response):
        try:
            move_laser_to_frame_success, move_msg = self.move_laser_to_frame(request.frame_name)
            
            # get compenent id if possible
            comp_id = "None"
            component_name = "Not a component frame"
            try:
                component_name = self.pm_robot_utils.assembly_scene_analyzer.get_component_for_frame_name(request.frame_name)
                component = self.pm_robot_utils.assembly_scene_analyzer.get_component_by_name(component_name)
                comp_id = component.uuid

            except (ComponentNotFoundError) as e:
                component_name = "None"
                comp_id = "None"

            except (RefFrameNotFoundError) as e:
                message = str(e)
                raise PmRobotError(message)

            response.component_name = component_name
            response.component_uuid = comp_id

            offset = 0.0
            
            if not move_laser_to_frame_success:
                raise PmRobotError(f"Failed to move laser to frame '{request.frame_name}': {move_msg}")


            if not self.pm_robot_utils._check_for_valid_laser_measurement():

                if request.use_iterative_sensing:

                    # time.sleep(1)

                    initial_z = self.pm_robot_utils.get_current_joint_state(PmRobotUtils.Z_Axis_JOINT_NAME)

                    # time.sleep(1)

                    # move up
                    self._logger.warn(f"MOVING UP")
                    move_success = self.pm_robot_utils.send_xyz_trajectory_goal_relative(0, 0, -3.0*1e-3,time=1)
                                                    
                    if not move_success:
                        raise PmRobotError(f"Failed to move up for iterative laser sensing on frame '{request.frame_name}'")
                    
                    step_inc = 0.4 # in mm
                    self._logger.warn(f"Laser measurement not valid! Trying to iteratively find a valid value!")                

                    x, y, final_z = self.pm_robot_utils.interative_sensing(measurement_method=self.pm_robot_utils.get_laser_measurement,
                                                    measurement_valid_function = self.pm_robot_utils._check_for_valid_laser_measurement,
                                                    length = (0.0, 0.0, 4.0),
                                                    step_inc = step_inc,
                                                    total_time = 8.0)
                    
                    if x is None:
                        raise PmRobotError(f"Iterative sensing failed to find valid laser measurement for frame '{request.frame_name}'")
                    
                    offset = initial_z - final_z
                    self._logger.info(f"Found valid value at: {offset} m")

                else:
                    raise PmRobotError(f"Laser measurement not valid for frame '{request.frame_name}'! OUT OF RANGE")


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

    def correct_frame_with_laser(self, request:pm_skill_srv.CorrectFrameLaser.Request, response:pm_skill_srv.CorrectFrameLaser.Response):
        try:
            self.pm_robot_utils.assembly_scene_analyzer.wait_for_initial_scene_update()

            frame_from_scene = self.pm_robot_utils.assembly_scene_analyzer.is_frame_from_scene(request.frame_name)

            if not frame_from_scene:
                raise RefFrameNotFoundError(f"Frame '{request.frame_name}' is not from assembly scene!")
            
            measure_frame_request = pm_skill_srv.CorrectFrameLaser.Request()

            measure_frame_request.frame_name = request.frame_name
            measure_frame_request.remeasure_after_correction = request.remeasure_after_correction
            measure_frame_request.use_iterative_sensing = request.use_iterative_sensing
            
            iterations = 1

            if request.remeasure_after_correction:
                iterations = 2
            
            for i in range(iterations):

                if i == 1:
                    self._logger.info(f"Remeasuring after correction...")
                    
                measure_frame_response = pm_skill_srv.CorrectFrameLaser.Response()

                response_mes:pm_skill_srv.CorrectFrameLaser.Response = self.measure_with_laser_callback(measure_frame_request, measure_frame_response)
                
                response.component_name = response_mes.component_name
                response.component_uuid = response_mes.component_uuid

                if not response_mes.success:
                    raise PmRobotError(f"Measuring frame '{request.frame_name}' with laser failed! Reason: {response_mes.message}")
                
                world_pose:TransformStamped = get_transform_for_frame_in_world(request.frame_name, self.tf_buffer, self._logger)

                world_pose.transform.translation.z += response_mes.correction_values.z
                response.correction_values.z = response_mes.correction_values.z
                
                adapt_frame_request = ami_srv.ModifyPoseAbsolut.Request()
                adapt_frame_request.frame_name = request.frame_name
                adapt_frame_request.pose.position.x = world_pose.transform.translation.x
                adapt_frame_request.pose.position.y = world_pose.transform.translation.y
                adapt_frame_request.pose.position.z = world_pose.transform.translation.z
                adapt_frame_request.pose.orientation = world_pose.transform.rotation
                adapt_frame_request.set_laser_measured = True
                
                if not self.adapt_frame_client.wait_for_service(timeout_sec=1.0):
                    self._logger.error(f"Service '{self.adapt_frame_client.srv_name}' not available. Assembly manager started?...")
                    response.success= False
                    response.message = f"Service '{self.adapt_frame_client.srv_name}' not available. Assembly manager started?..."
                    return response
                
                result_adapt:ami_srv.ModifyPoseAbsolut.Response = self.adapt_frame_client.call(adapt_frame_request)

                response.success = result_adapt.success

        except (PmRobotError,RefFrameNotFoundError) as e:
            response.success = False
            response.message = str(e)
            self._logger.error(response.message)

        return response
    
    def measure_frame_with_confocal_bottom(self, request:pm_skill_srv.CorrectFrameLaser.Request, response:pm_skill_srv.CorrectFrameLaser.Response):
        
        tcp_name = self.pm_robot_utils.TCP_CONFOCAL_BOTTOM  # we need to move the frame attached to the robot to the tcp. We use the move camera method for that
        try:
            # get compenent id if possible
            comp_id = "None"
            component_name = "Not a component frame"
            try:
                component_name = self.pm_robot_utils.assembly_scene_analyzer.get_component_for_frame_name(request.frame_name)
                component = self.pm_robot_utils.assembly_scene_analyzer.get_component_by_name(component_name)
                comp_id = component.uuid

            except (ComponentNotFoundError) as e:
                component_name = "None"
                comp_id = "None"

            except (RefFrameNotFoundError) as e:
                message = str(e)
                raise PmRobotError(message)

            response.component_name = component_name
            response.component_uuid = comp_id

            move_frame_to_confocal_bottom_success, move_msg = self.pm_robot_utils.move_camera_top_to_frame(   frame_name = tcp_name,
                                                                                                    endeffector_override=request.frame_name)
            
            if not move_frame_to_confocal_bottom_success:
                raise PmRobotError(f"Failed to move frame '{request.frame_name}' to confocal bottom: {move_msg}")

            time.sleep(1)

            if not self.pm_robot_utils.check_confocal_bottom_measurement_in_range():
                raise PmRobotError(f"Confocal bottom measurement not valid for frame '{request.frame_name}'! OUT OF RANGE")
            
            confocal_measurement = self.pm_robot_utils.get_confocal_bottom_measurement(unit="m")
            
            self._logger.info(f"Measurement: {confocal_measurement*1e6} um ")

            response.correction_values.z = confocal_measurement
            response.success = True
            response.message = f"Measurement: {confocal_measurement*1e6} um"

        except PmRobotError as e:
            response.success = False
            response.message = str(e)
            self._logger.error(response.message)

        return response

    def correct_frame_with_confocal_bottom(self, request:pm_skill_srv.CorrectFrameLaser.Request, response:pm_skill_srv.CorrectFrameLaser.Response):
        
        try:
            self.pm_robot_utils.assembly_scene_analyzer.wait_for_initial_scene_update()

            if not(self.pm_robot_utils.assembly_scene_analyzer.is_frame_from_scene(request.frame_name)):
                raise RefFrameNotFoundError(f"Frame '{request.frame_name}' is not from assembly scene!")

            measure_frame_request = pm_skill_srv.CorrectFrameLaser.Request()
            measure_frame_response = pm_skill_srv.CorrectFrameLaser.Response()
            
            measure_frame_request.frame_name = request.frame_name
            measure_frame_request.remeasure_after_correction = request.remeasure_after_correction
            measure_frame_request.use_iterative_sensing = request.use_iterative_sensing
            
            response_mes:pm_skill_srv.CorrectFrameLaser.Response = self.measure_frame_with_confocal_bottom(measure_frame_request, measure_frame_response)
            
            response.component_name = response_mes.component_name
            response.component_uuid = response_mes.component_uuid

            #self._logger.warn(f"REs {str(response_mes)}")

            if not response_mes.success:
                raise PmRobotError(f"Measuring frame '{request.frame_name}' with confocal bottom failed!")
                    
            world_pose:TransformStamped = get_transform_for_frame_in_world(request.frame_name, self.tf_buffer, self._logger)

            world_pose.transform.translation.z += response_mes.correction_values.z

            adapt_frame_request = ami_srv.ModifyPoseAbsolut.Request()
            adapt_frame_request.frame_name = request.frame_name
            adapt_frame_request.pose.position.x = world_pose.transform.translation.x
            adapt_frame_request.pose.position.y = world_pose.transform.translation.y
            adapt_frame_request.pose.position.z = world_pose.transform.translation.z
            adapt_frame_request.pose.orientation = world_pose.transform.rotation
            adapt_frame_request.set_laser_measured = True

            if not self.adapt_frame_client.wait_for_service(timeout_sec=1.0):
                raise PmRobotError(f"Service '{self.adapt_frame_client.srv_name}' not available. Assembly manager started?...")

            result_adapt:ami_srv.ModifyPoseAbsolut.Response = self.adapt_frame_client.call(adapt_frame_request)

            # result_adapt = ami_srv.ModifyPoseAbsolut.Response()

            response.success = result_adapt.success
            response.correction_values.z = response_mes.correction_values.z

        except (PmRobotError,RefFrameNotFoundError) as e:
            response.success = False
            response.message = str(e)
            self._logger.error(response.message)

        return response

    def measure_frame_with_confocal_top(self, request:pm_skill_srv.CorrectFrameLaser.Request, response:pm_skill_srv.CorrectFrameLaser.Response):

        move_confocal_top_to_frame_success, move_msg = self.pm_robot_utils.move_confocal_top_to_frame(request.frame_name)

        time.sleep(1)

        try:
            comp_id = "None"
            component_name = "Not a component frame"
            try:
                component_name = self.pm_robot_utils.assembly_scene_analyzer.get_component_for_frame_name(request.frame_name)
                component = self.pm_robot_utils.assembly_scene_analyzer.get_component_by_name(component_name)
                comp_id = component.uuid

            except (ComponentNotFoundError) as e:
                component_name = "None"
                comp_id = "None"

            except (RefFrameNotFoundError) as e:
                message = str(e)
                raise PmRobotError(message)

            response.component_name = component_name
            response.component_uuid = comp_id

            offset = 0.0
            
            if not move_confocal_top_to_frame_success:
                raise PmRobotError(f"Failed to move confocal top to frame '{request.frame_name}': {move_msg}")

            if not self.pm_robot_utils.check_confocal_top_measurement_in_range():

                if request.use_iterative_sensing:

                    # time.sleep(1)

                    initial_z = self.pm_robot_utils.get_current_joint_state(PmRobotUtils.Z_Axis_JOINT_NAME)

                    # time.sleep(1)

                    # move up
                    self._logger.warn(f"MOVING UP")
                    move_success = self.pm_robot_utils.send_xyz_trajectory_goal_relative(0, 0, -3.0*1e-3,time=1)
                                                    
                    if not move_success:
                        raise PmRobotError(f"Failed to move up for iterative confocal top sensing on frame '{request.frame_name}'")
                    
                    step_inc = 0.4 # in mm
                    self._logger.warn(f"Confocal top measurement not valid! Trying to iteratively find a valid value!")                

                    x, y, final_z = self.pm_robot_utils.interative_sensing(measurement_method=self.pm_robot_utils.get_confocal_top_measurement,
                                                    measurement_valid_function = self.pm_robot_utils.check_confocal_top_measurement_in_range,
                                                    length = (0.0, 0.0, 4.0),
                                                    step_inc = step_inc,
                                                    total_time = 8.0)
                    
                    if x is None:
                        raise PmRobotError(f"Iterative sensing failed to find valid confocal top measurement for frame '{request.frame_name}'")
                    
                    offset = initial_z - final_z
                    self._logger.info(f"Found valid value at: {offset} m")

                else:
                    raise PmRobotError(f"Confocal top measurement not valid for frame '{request.frame_name}'! OUT OF RANGE")


            confocal_measurement = self.pm_robot_utils.get_confocal_top_measurement(unit="m") + float(offset)

            self._logger.warn(f"Confocal measurement: {confocal_measurement*1e6} um ")

            response.correction_values.z = confocal_measurement
            response.success = True
            response.message = f"Measurement: {confocal_measurement*1e6:.2f} um"

        except PmRobotError as e:
            response.success = False
            response.message = str(e)
            self._logger.error(response.message)

        return response
    

    def correct_frame_with_confocal_top(self, request:pm_skill_srv.CorrectFrameLaser.Request, response:pm_skill_srv.CorrectFrameLaser.Response):

        try:
            self.pm_robot_utils.assembly_scene_analyzer.wait_for_initial_scene_update()

            if not self.pm_robot_utils.assembly_scene_analyzer.is_frame_from_scene(request.frame_name):
                raise RefFrameNotFoundError(f"Frame '{request.frame_name}' is not from assembly scene!")

            measure_frame_request = pm_skill_srv.CorrectFrameLaser.Request()
            
            measure_frame_request.frame_name = request.frame_name
            measure_frame_request.remeasure_after_correction = request.remeasure_after_correction
            measure_frame_request.use_iterative_sensing = request.use_iterative_sensing
            

            iterations = 1

            if request.remeasure_after_correction:
                iterations = 2
            
            for i in range(iterations):
                if i == 1:
                    self._logger.warn(f"Re-measuring after correction...")

                measure_frame_response = pm_skill_srv.CorrectFrameLaser.Response()

                response_mes:pm_skill_srv.CorrectFrameLaser.Response = self.measure_frame_with_confocal_top(measure_frame_request, measure_frame_response)
                
                response.component_name = response_mes.component_name
                response.component_uuid = response_mes.component_uuid
                
                #self._logger.warn(f"REs {str(response_mes)}")

                if not response_mes.success:
                    raise PmRobotError(f"Measuring frame '{request.frame_name}' with confocal top failed!")
                        
                world_pose:TransformStamped = get_transform_for_frame_in_world(request.frame_name, self.tf_buffer, self._logger)

                world_pose.transform.translation.z += response_mes.correction_values.z

                response.correction_values.z = response_mes.correction_values.z
                
                adapt_frame_request = ami_srv.ModifyPoseAbsolut.Request()
                adapt_frame_request.frame_name = request.frame_name
                adapt_frame_request.pose.position.x = world_pose.transform.translation.x
                adapt_frame_request.pose.position.y = world_pose.transform.translation.y
                adapt_frame_request.pose.position.z = world_pose.transform.translation.z
                adapt_frame_request.pose.orientation = world_pose.transform.rotation
                adapt_frame_request.set_laser_measured = True
                
                if not self.adapt_frame_client.wait_for_service(timeout_sec=1.0):
                    self._logger.error(f"Service '{self.adapt_frame_client.srv_name}' not available. Assembly manager started?...")
                    response.success= False
                    response.message = f"Service '{self.adapt_frame_client.srv_name}' not available. Assembly manager started?..."
                    return response
                
                result_adapt:ami_srv.ModifyPoseAbsolut.Response = self.adapt_frame_client.call(adapt_frame_request)

                response.success = result_adapt.success


        except (PmRobotError,RefFrameNotFoundError) as e:
            response.success = False
            response.message = str(e)
            self._logger.error(response.message)    
        
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

    def align_gonio_right(self, endeffector_override:str, alignment_frame:str = PM_ROBOT_GRIPPER_FRAME)-> tuple[bool, str]:
        call_async = False

        if not self.pm_robot_utils.client_align_gonio_right.wait_for_service(timeout_sec=1.0):
            self.logger.error("Service '/pm_moveit_server/align_gonio_right' not available")
            return False, "Service '/pm_moveit_server/align_gonio_right' not available"
        
        req = pm_moveit_srv.AlignGonio.Request()
        req.target_frame = alignment_frame
        req.execute_movement = True
        req.endeffector_frame_override = endeffector_override

        if call_async:
            future = self.pm_robot_utils.client_align_gonio_right.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is None:
                self.logger.error('Service call failed %r' % (future.exception(),))
                return False, f"Service call failed: {future.exception()}"
            return future.result().success, future.result().message
        else:
            response:pm_moveit_srv.AlignGonio.Response = self.pm_robot_utils.client_align_gonio_right.call(req)
            return response.success, response.message
        
    def align_gonio_left(self, endeffector_override:str, alignment_frame:str = PM_ROBOT_GRIPPER_FRAME)-> tuple[bool, str]:
        call_async = False

        if not self.pm_robot_utils.client_align_gonio_left.wait_for_service(timeout_sec=1.0):
            self.logger.error("Service '/pm_moveit_server/align_gonio_left' not available")
            return False, "Service '/pm_moveit_server/align_gonio_left' not available"
        
        req = pm_moveit_srv.AlignGonio.Request()
        req.target_frame = alignment_frame
        req.execute_movement = True
        req.endeffector_frame_override = endeffector_override

        if call_async:
            future = self.pm_robot_utils.client_align_gonio_left.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is None:
                self.logger.error('Service call failed %r' % (future.exception(),))
                return False, f"Service call failed: {future.exception()}"
            return future.result().success, future.result().message
        else:
            response:pm_moveit_srv.AlignGonio.Response = self.pm_robot_utils.client_align_gonio_left.call(req)
            return response.success, response.message
    
    def check_frame_mes_confocal_top(self, request:pm_skill_srv.CheckFrameMeasurable.Request, response:pm_skill_srv.CheckFrameMeasurable.Response):

        CONFOCAL_LASER_OFFSET = 8*1e-3 # in m, distance from confocal top to laser point

        res = self.pm_robot_utils.check_frame_mes(request = request,
                                                         offset_m=CONFOCAL_LASER_OFFSET,
                                                         move_client= self.pm_robot_utils.client_move_robot_laser_to_frame)
        response = res
        
        return response


    def check_frame_mes_laser_top(self, request:pm_skill_srv.CheckFrameMeasurable.Request, response:pm_skill_srv.CheckFrameMeasurable.Response):
        CONFOCAL_LASER_OFFSET = 3*1e-3 # in m, distance from confocal top to laser point

        res = self.pm_robot_utils.check_frame_mes(request = request,
                                                         offset_m=CONFOCAL_LASER_OFFSET,
                                                         move_client = self.pm_robot_utils.client_move_robot_laser_to_frame)
        response = res

        return response

    def check_frame_mes_confocal_bottom(self, request:pm_skill_srv.CheckFrameMeasurable.Request, response:pm_skill_srv.CheckFrameMeasurable.Response):
        CONFOCAL_LASER_OFFSET =40*1e-3 # in m, distance from confocal top to laser point

        res = self.pm_robot_utils.check_frame_mes_bot(request = request,
                                                        target_tcp_frame = self.pm_robot_utils.TCP_CONFOCAL_BOTTOM,
                                                        offset_m=CONFOCAL_LASER_OFFSET)
        response = res

        return response

    def dispense_2k_at_path(self, request: pm_msg_srv.DispenseAtPath.Request, response:pm_msg_srv.DispenseAtPath.Response):

        try:
            if not request.sequence_file_path:
                raise PmRobotError("sequence_file_path is empty!")
            disp_gen = DispenseSequenceGenerator()
            disp_gen.load_from_file(request.sequence_file_path)

            self.logger.info(f"Loaded dispense sequence from file '{request.sequence_file_path}'!")

            if not (self.pm_robot_utils.client_move_robot_cam1_to_frame.wait_for_service(timeout_sec=1.0)):
                raise PmRobotError(f"Service '{self.pm_robot_utils.client_move_robot_cam1_to_frame.srv_name}' not available!")
            
            move_request = pm_moveit_srv.MoveToFrame.Request()
            move_request.execute_movement = False
            move_request.target_frame = request.target_frame_disp

            response_move: pm_moveit_srv.MoveToFrame.Response = self.pm_robot_utils.client_move_robot_cam1_to_frame.call(move_request)

            start_joints = Point() 
            start_joints.x = response_move.joint_values[0]
            start_joints.y = response_move.joint_values[1]
            start_joints.z = response_move.joint_values[2]

            transform = self.tf_buffer.lookup_transform("world", request.target_frame_disp, rclpy.time.Time())
            
            start_pose = Pose()
            start_pose.position.x = transform.transform.translation.x
            start_pose.position.y = transform.transform.translation.y
            start_pose.position.z = transform.transform.translation.z
            start_pose.orientation = transform.transform.rotation

            g_code = disp_gen.generate_g_code(start_pose=start_pose,
                                            start_joint_values=start_joints)
            
            disp_gen.save_g_code_to_file(start_pose=start_pose,
                                            start_joint_values=start_joints,
                                            file_path=get_package_share_directory("pm_skills") + "/example_g_code")

            if self.pm_robot_utils.get_mode() == self.pm_robot_utils.REAL_MODE:

                self.logger.warn(f"Switching off controller!")

                self.pm_robot_utils.set_controller_activation("pm_robot_xyz_axis_controller", activate = False) 

                time.sleep(10.0) # wait for controller switch

                self.pm_robot_utils.set_controller_activation("pm_robot_xyz_axis_controller", True) 

                self.logger.warn(f"Switching on controller!")

            elif self.pm_robot_utils.get_mode() == self.pm_robot_utils.UNITY_MODE:
                self.logger.warn(f"Unity mode detected!")

                self.pm_robot_utils.extend_2k_dispenser()

                self.logger.warn(f"Switching off controller!")

                self.pm_robot_utils.set_controller_activation("pm_robot_xyz_axis_controller", activate = False) 

                time.sleep(3.0) # wait for controller switch

                # Call the Unity-specific dispensing service
                unity_request = EmptyWithSuccess.Request()
                unity_response = self.dispense_2k_unity_client.call(unity_request)
                time.sleep(10.0)
                self.pm_robot_utils.set_controller_activation("pm_robot_xyz_axis_controller", True) 
                time.sleep(5.0) # wait for controller switch

                if not unity_response.success:
                    raise PmRobotError(f"Unity dispensing service failed: {unity_response.message}")
            else:
                # only for gazebo
                self._test_gcode(g_code, start_frame=request.target_frame_disp)

            self.logger.info(f"Generated G-code:\n{g_code}")
            response.success = True     

        except (PmRobotError,
            LookupException,
            ConnectivityException,
            ExtrapolationException,
            FileNotFoundError,
            Exception) as e:
            response.success = False
            response.message = f"Error during dispensing at path: {e}"
            self.logger.error(response.message)

        finally:
            
            self.pm_robot_utils.retract_2k_dispenser()

            success = self.pm_robot_utils.send_xyz_trajectory_goal_relative(0,0,-0.05,time=0.5) # move up after dispensing to be safe
            if not success:
                response.message = response.message + f"Failed to move up after dispensing! Please check the robot state!"
                self.logger.error(response.message)
                response.success = False

        return response

    def _test_gcode(self, g_code: str, start_frame:str):

        def extract_xyzf_from_gcode_meters(gcode: str) -> List[Tuple[float, float, float, float]]:
            """
            Extract X, Y, Z coordinates + speed (F) from G-code and convert to meters.

            Special handling:
                - G30: dip → go to position, then return to previous position

            Returns:
                List of tuples (x, y, z, speed)
                - position in meters
                - speed unchanged
            """

            # Capture G-code + X Y Z + optional F
            pattern = r"(G\d+).*?X([-+]?\d*\.?\d+)\s+Y([-+]?\d*\.?\d+)\s+Z([-+]?\d*\.?\d+)(?:\s+F([-+]?\d*\.?\d+))?"
            
            matches = re.findall(pattern, gcode)

            result = []
            last_point: Optional[Tuple[float, float, float]] = None
            last_speed: float = 0.0

            for cmd, x, y, z, f in matches:
                point = (
                    float(x) * 0.001,
                    float(y) * 0.001,
                    float(z) * 0.001
                )

                speed = float(f) if f else last_speed

                if cmd == "G30":
                    # Move to dip position
                    result.append((point[0], point[1], point[2], speed))

                    # Return to previous position
                    if last_point is not None:
                        result.append((last_point[0], last_point[1], last_point[2], last_speed))

                else:
                    result.append((point[0], point[1], point[2], speed))
                    last_point = point
                    last_speed = speed

            return result
    
        move_off_success = self.pm_robot_utils.move_camera_top_to_frame(frame_name=start_frame,
                                                                        z_offset=0.01)
        
        if not move_off_success:
            raise PmRobotError(f"Failed to move off surface for g-code testing on frame '{start_frame}'")
        
        move_succes = self.pm_robot_utils.move_camera_top_to_frame(frame_name=start_frame)

        if not move_succes:
            raise PmRobotError(f"Failed to move to start frame '{start_frame}' for g-code testing")

        coords = extract_xyzf_from_gcode_meters(g_code)

        for x, y, z, f in coords:
            if f <= 0:
                self.logger.warn(f"Non-positive speed {f} in G-code, using default speed 0.01 m/s")
                f = 10

            # convert mm/s to time
            speed_mm_s = 10 
            time = speed_mm_s/f
            self.logger.info(f"Moving to X:{x}, Y:{y}, Z:{z} from frame '{start_frame}'")
            move_success = self.pm_robot_utils.send_xyz_trajectory_goal_absolut(x, y, z, time=time)
            if not move_success:
                raise PmRobotError(f"Failed to move to X:{x}, Y:{y}, Z:{z} relative to frame '{start_frame}' during g-code testing")

    def dispense_at_frames_callback(self, request: pm_msg_srv.DispenseAtPoints.Request, response:pm_msg_srv.DispenseAtPoints.Response):
        self.pm_robot_utils.assembly_scene_analyzer.wait_for_initial_scene_update()

        try:
            if len(request.dispense_points) == 0:
                raise PmRobotError("Dispense points list is empty!")
            
            dispenser_prepared = False

            for dispense_point in request.dispense_points:
            
                prop = ami_msg.RefFrameProperties()
                prop.glue_pt_frame_properties.dispense_offset_mm = dispense_point.dispense_z_offset_mm
                prop.glue_pt_frame_properties.is_glue_point = True
                prop.glue_pt_frame_properties.time_ms = dispense_point.time_ms
                prop.glue_pt_frame_properties.has_been_placed = True

                dispense_point: pm_msg.DispensePoint
                move_to_frame_request = pm_moveit_srv.MoveToFrame.Request()
                
                move_to_frame_request.target_frame = dispense_point.frame_name
                move_to_frame_request.execute_movement = True
                
                
                if not dispenser_prepared:
                    self.pm_robot_utils.prepare_dispenser(move_to_frame_request)
                    dispenser_prepared = True

                self.pm_robot_utils.dispense_at_frame(move_to_frame_request, 
                                                dispense_point.frame_name,
                                                time=dispense_point.time_ms,
                                                dispense_z_offset_mm=dispense_point.dispense_z_offset_mm)

                self.pm_robot_utils.set_frame_properties(dispense_point.frame_name, prop)
                        
            self.pm_robot_utils.retract_dispenser()
            time.sleep(0.5)
            self.pm_robot_utils.close_protection()
            
            response.success = True

        except PmRobotError as e:
            self.get_logger().error(f"Error during dispensing at points: {e.message}")
            response.success = False
            response.message = e.message
        return response
    

    
    def dispense_at_frames_adv_callback(self, request: pm_skill_srv.DispenseAtPointsAdv.Request, response:pm_skill_srv.DispenseAtPointsAdv.Response):
        dispense_request = pm_msg_srv.DispenseAtPoints.Request()
        dispense_response = pm_msg_srv.DispenseAtPoints.Response()

        self.pm_robot_utils.assembly_scene_analyzer.wait_for_initial_scene_update()
        try:
            # populate the request
            for dispense_point in request.dispense_points:
                dispense_point_msg = pm_msg.DispensePoint()
                dispense_point_msg.frame_name = dispense_point

                properties = self.pm_robot_utils.assembly_scene_analyzer.get_frame_properties(dispense_point).glue_pt_frame_properties

                if not properties.is_glue_point:
                    raise PmRobotError(f"Frame '{dispense_point}' is not a glue point frame according to the assembly scene analyzer!")

                dispense_point_msg.time_ms = properties.time_ms
                dispense_point_msg.dispense_z_offset_mm = properties.dispense_offset_mm

                if dispense_point_msg.time_ms <= 0:
                    raise PmRobotError(f"Frame '{dispense_point}' has invalid dispense time '{dispense_point_msg.time_ms}' ms. Time must be positive and non-zero!")
            
                if dispense_point_msg.dispense_z_offset_mm <= 0:
                    raise PmRobotError(f"Frame '{dispense_point}' has invalid dispense offset '{dispense_point_msg.dispense_z_offset_mm}' mm. Offset must be positive and non-zero!")

                # THIS IS OPTIONAL DECIDE FOR BEHAVIOUR
                # if properties.has_been_placed:
                #     self._logger.warn(f"Frame '{dispense_point}' has already been marked as placed. Skipping dispensing at this frame to avoid double dispensing!")
                #     continue

                dispense_request.dispense_points.append(dispense_point_msg)

            response_disp = self.dispense_at_frames_callback(dispense_request, dispense_response)

            response.success = response_disp.success
            response.message = response_disp.message

        except (PmRobotError, RefFrameNotFoundError) as e:
            self.get_logger().error(f"Error during advanced dispensing at points: {e.message}")
            response.success = False
            response.message = str(e)
        return response


    def uv_cure_adv_callback(self, request: pm_msg_srv.UVCuringSkill.Request, response:pm_msg_srv.UVCuringSkill.Response):
        """
        Advanced UV curing skill that not only performs the UV curing action but also updates the assembly scene analyzer's frame properties for glue points. 
        This ensures that after curing, the system is aware of which glue points have been cured, allowing for better decision-making in subsequent steps of the assembly process.
        """
        
        self.pm_robot_utils.assembly_scene_analyzer.wait_for_initial_scene_update()
        try:

            if not self.pm_robot_utils.client_uv_cure.wait_for_service(timeout_sec=1.0):
                raise PmRobotError(f"Service '{self.pm_robot_utils.client_uv_cure.srv_name}' not available!")

            res: pm_msg_srv.UVCuringSkill.Response = self. pm_robot_utils.client_uv_cure.call(request)
            
            if not res.success:
                response.message = res.message
                raise PmRobotError(f"UV curing service call failed: {res.message}")
            
            all_frame_names = self.pm_robot_utils.assembly_scene_analyzer.get_all_component_frames()

            for frame_name in all_frame_names:
                frame = self.pm_robot_utils.assembly_scene_analyzer.get_ref_frame_by_name(frame_name)
                g_properties = frame.properties.glue_pt_frame_properties
                    
                if g_properties.is_glue_point and g_properties.has_been_placed:
                    g_properties.has_been_cured = True
                    self.pm_robot_utils.set_frame_properties(frame.frame_name, frame.properties)

            placed_components = self.pm_robot_utils.assembly_scene_analyzer.get_placed_components()
            
            if not placed_components:
                raise PmRobotError(f"No placed components found!")
            
            properties = ami_msg.ComponentProperties()
            properties.is_assembled = True

            # do not change the is_gripped property during curing
            if self.pm_robot_utils.assembly_scene_analyzer.is_gripper_empty():
                properties.is_gripped = False
            else:
                properties.is_gripped = True

            for placed_component in placed_components:
                set_properties_response: ami_srv.SetComponentProperties.Response = self.pm_robot_utils.set_component_properties(placed_component, properties)
            
            if not set_properties_response.success:
                raise PmRobotError(f"Failed to set component properties for component '{placed_component}' after curing!") 
            
            response.success = True
            response.message = "UV curing completed successfully and glue point frame properties updated."

        except (PmRobotError, RefFrameNotFoundError) as e:
            self.get_logger().error(f"Error during advanced UV curing: {e.message}")
            response.success = False
            response.message = str(e)
        return response
    
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
