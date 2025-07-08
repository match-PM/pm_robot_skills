import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor
import yaml
import sys
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from pm_skills_interfaces.srv import MeasureFrame, CorrectFrame
from pm_moveit_interfaces.srv import MoveToPose,  MoveToFrame
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, StaticTransformBroadcaster
from pm_vision_interfaces.srv import ExecuteVision, CalibrateAngle, CalibratePixelPerUm
import pm_vision_interfaces.msg as vision_msg
from geometry_msgs.msg import Vector3, TransformStamped, Pose, PoseStamped, Quaternion, Transform

from scipy.spatial.transform import Rotation as R

import assembly_manager_interfaces.srv as ami_srv
import assembly_manager_interfaces.msg as ami_msg

from assembly_scene_publisher.py_modules.AssemblyScene import AssemblyManagerScene
from assembly_manager_interfaces.srv import SpawnFramesFromDescription
from pm_msgs.srv import EmptyWithSuccess, ForceSensorGetMeasurement
import numpy as np
from circle_fit import circle_fit
import matplotlib.pyplot as plt

# import get_package_share_directory
from ament_index_python.packages import get_package_share_directory

from ros_sequential_action_programmer.submodules.pm_robot_modules.widget_pm_robot_config import VacuumGripperConfig, ParallelGripperConfig
from pm_skills.py_modules.PmRobotUtils import PmRobotUtils
import time
import os
import datetime
import copy
import math

from assembly_manager_interfaces.srv import ModifyPoseFromFrame

from assembly_scene_publisher.py_modules.scene_functions import (get_rel_transform_for_frames, 
                                                                 is_frame_from_scene, 
                                                                 get_ref_frame_by_name)

from assembly_scene_publisher.py_modules.geometry_type_functions import (get_relative_transform_for_transforms)

TOOL_VACUUM_IDENT = 'pm_robot_vacuum_tools'
TOOL_GRIPPER_1_JAW_IDENT = 'pm_robot_tool_parallel_gripper_1_jaw'
TOOL_GRIPPER_2_JAW_IDENT = 'pm_robot_tool_parallel_gripper_2_jaws'

class PmRobotCalibrationNode(Node):
    INFO_TEXT = """
    PM Robot Calibration Node
    This node is responsible for calibrating the PM robot.
    It provides services to calibrate the gripper, dispenser, cameras, and laser.
    Note Calibration_Order:
    --> 1. /pm_robot_calibration/calibrate_cameras
    --> 2. /pm_robot_calibration/calibrate_calibration_cube_to_cam_top
    --> 3. /pm_robot_calibration/calibrate_laser_on_calibration_cube
    --> 4. /pm_robot_calibration/calibrate_confocal_top
    --> 5. /pm_robot_calibration/calibrate_calibration_target_to_cam_bottom
    --> 5. /pm_robot_calibration/calibrate_confocal_bottom
    --> 6. /pm_robot_calibration/calibrate_gonio_left_chuck
    --> 7. /pm_robot_calibration/calibrate_gonio_right_chuck
    --> 8. /pm_robot_calibration/calibrate_gripper
    --> 9. /pm_robot_calibration/calibrate_1K_dispenser
    """

    def __init__(self):
        
        super().__init__('pm_robot_calibration')
        self._logger = self.get_logger()

        self._logger.info(" Pm Robot Calibration Node started...")
        
        self.callback_group = MutuallyExclusiveCallbackGroup()
        self.pm_robot_utils = PmRobotUtils(self)
        self.pm_robot_utils.start_object_scene_subscribtion()
        
        self.pm_robot_config = {}
        self.gripper_frames_spawned = False
        # clients
        self.client_spawn_frames = self.create_client(SpawnFramesFromDescription, '/assembly_manager/spawn_frames_from_description')
        self.client_measure_frame_cam = self.create_client(MeasureFrame, '/pm_skills/vision_measure_frame')
        self.client_correct_frame = self.create_client(CorrectFrame, '/pm_skills/vision_correct_frame')
        self.client_modify_pose_from_frame = self.create_client(ami_srv.ModifyPoseFromFrame, '/assembly_manager/modify_frame_from_frame')
        self.client_calibrate_camera_pixel = self.create_client(CalibratePixelPerUm, '/pm_vision_manager/SetCameraPixelPerUm')
        self.client_calibrate_camera_angle = self.create_client(CalibrateAngle, '/pm_vision_manager/SetCameraAngle')

        self.client_move_calibration_target_forward = self.create_client(EmptyWithSuccess, '/pm_pneumatic_controller/Camera_Calibration_Platelet_Joint/MoveForward')
        self.client_move_calibration_target_backward = self.create_client(EmptyWithSuccess, '/pm_pneumatic_controller/Camera_Calibration_Platelet_Joint/MoveBackward')
        
        # services
        self.calibrate_cameras_srv = self.create_service(EmptyWithSuccess, '/pm_robot_calibration/calibrate_cameras', self.calibrate_cameras_callback, callback_group=self.callback_group)
        self.calibrate_calibration_cube_to_cam_top_srv = self.create_service(EmptyWithSuccess, '/pm_robot_calibration/calibrate_calibration_cube_to_cam_top', self.calibrate_calibration_cube_to_cam_top_callback, callback_group=self.callback_group)
        self.calibrate_laser_on_calibration_cube_srv = self.create_service(EmptyWithSuccess, '/pm_robot_calibration/calibrate_laser_on_calibration_cube', self.calibrate_laser_on_calibration_cube_callback, callback_group=self.callback_group)
        self.calbirate_confocal_top_srv = self.create_service(EmptyWithSuccess, '/pm_robot_calibration/calibrate_confocal_top', self.calibrate_confocal_top_callback, callback_group=self.callback_group)
        self.calibrate_siemens_gripper_on_cal_cube = self.create_service(EmptyWithSuccess, '/pm_robot_calibration/calibrate_siemens_gripper_on_cal_cube', self.calibrate_sim_gripper_on_cube, callback_group=self.callback_group)
        self.calbirate_calibration_target_to_cam_bottom_srv = self.create_service(EmptyWithSuccess, '/pm_robot_calibration/calibrate_calibration_target_to_cam_bottom', self.calibrate_calibration_target_to_cam_bottom_callback, callback_group=self.callback_group)
        self.calbirate_confocal_bottom_srv = self.create_service(EmptyWithSuccess, '/pm_robot_calibration/calibrate_confocal_bottom', self.calibrate_confocal_bottom_callback, callback_group=self.callback_group)
        self.calbibrate_gonio_left_chuck_srv = self.create_service(EmptyWithSuccess, '/pm_robot_calibration/calibrate_gonio_left_chuck', self.calibrate_gonio_left_chuck_callback, callback_group=self.callback_group)
        self.calbibrate_gonio_right_chuck_srv = self.create_service(EmptyWithSuccess, '/pm_robot_calibration/calibrate_gonio_right_chuck', self.calibrate_gonio_right_chuck_callback, callback_group=self.callback_group)
        self.calibrate_gripper_srv = self.create_service(EmptyWithSuccess, '/pm_robot_calibration/calibrate_gripper', self.calibrate_gripper_callback, callback_group=self.callback_group)
        self.calibrate_dispenser_srv = self.create_service(EmptyWithSuccess, '/pm_robot_calibration/calibrate_1K_dispenser', self.calibrate_1K_dispenser_callback, callback_group=self.callback_group)
        
        # paths
        self.bringup_share_path = get_package_share_directory('pm_robot_bringup')
        self.calibration_frame_dict_path = get_package_share_directory('pm_robot_description') + '/urdf/urdf_configs/calibration_frame_dictionaries'
        self.pm_robot_config_path = self.bringup_share_path + '/config/pm_robot_bringup_config.yaml'

        #self.update_pm_robot_config()
        self.get_logger().warn(f"Check!")

        self.get_logger().info(self.INFO_TEXT)
        self.last_calibrations_data = {}
        
    def update_pm_robot_config(self):
        self.pm_robot_utils.pm_robot_config.reload_config()
        # with open(self.pm_robot_config_path, 'r') as file:
        #     self.pm_robot_config = yaml.load(file, Loader=yaml.FullLoader)
        # self.vacuum_gripper_config = VacuumGripperConfig(TOOL_VACUUM_IDENT, self.pm_robot_config['pm_robot_tools'][TOOL_VACUUM_IDENT])
        # self.parallel_gripper_1_jaw_config = ParallelGripperConfig(TOOL_GRIPPER_1_JAW_IDENT, self.pm_robot_config['pm_robot_tools'][TOOL_GRIPPER_1_JAW_IDENT])
        # self.parallel_gripper_2_jaw_config = ParallelGripperConfig(TOOL_GRIPPER_2_JAW_IDENT, self.pm_robot_config['pm_robot_tools'][TOOL_GRIPPER_2_JAW_IDENT])
        
    def get_gripper_calibration_frame_dictionary(self)->dict:
        calibration_frame_dict = {}
        file_path = None
        
        file_name = self.pm_robot_utils.pm_robot_config.tool.get_tool().get_calibration_frame_dict_file_name()
        file_path = self.calibration_frame_dict_path + '/' + file_name 

        # if self.vacuum_gripper_config.get_activate_status():
        #     print("Vacuum gripper calibration frame dictionary")
        #     file_name = self.vacuum_gripper_config.get_calibration_frame_dict_file_name()
        
        # elif self.parallel_gripper_1_jaw_config.get_activate_status():
        #     print("Parallel gripper 1 jaw calibration frame dictionary")
        #     file_name = self.parallel_gripper_1_jaw_config.get_calibration_frame_dict_file_name()
        #     file_path = self.calibration_frame_dict_path + '/' + file_name
            
        # elif self.parallel_gripper_2_jaw_config.get_activate_status():
        #     print("Parallel gripper 2 jaws calibration frame dictionary")
        #     file_name = self.parallel_gripper_2_jaw_config.get_calibration_frame_dict_file_name()
        #     file_path = self.calibration_frame_dict_path + '/' + file_name
            
        # open the file and load the calibration frame dictionary
        try:
            with open(file_path, 'r') as file:
                calibration_frame_dict = yaml.load(file, Loader=yaml.FullLoader)
        
        except Exception as e:
            self._logger.error("Error: " + str(e))
            return {}
        
        return calibration_frame_dict, file_path
    
    def spawn_frames_for_current_gripper(self)->tuple[bool, str]:
        
        self.update_pm_robot_config()
        
        calibration_frame_dict, file_path = self.get_gripper_calibration_frame_dictionary()
        frames_list = []
        unique_identifier = calibration_frame_dict.get('unique_identifier', '')
        
        if calibration_frame_dict is None or file_path is None:
            self._logger.error("No calibration frame dictionary found...")
            return False, None
        
        for frame in calibration_frame_dict['frames']:
            frames_list.append(f"{unique_identifier}{frame['name']}")
            
        #if self.gripper_frames_spawned:
        #    return True, frames_list
            
        request = SpawnFramesFromDescription.Request()
        request.dict_or_path = file_path
        
        if not self.client_spawn_frames.wait_for_service(timeout_sec=1.0):
            self._logger.error('Assembly manager not available...')
        
        self._logger.info('Calibration: Spawning frames for gripper...')
        
        response:SpawnFramesFromDescription.Response = self.client_spawn_frames.call(request=request)
        self.gripper_frames_spawned = True
        return response.success, frames_list, unique_identifier

    
    def spawn_calibration_frames(self, calibration_file_name:str):
        # spawn the calibration frames
        request = SpawnFramesFromDescription.Request()
        request.dict_or_path = self.calibration_frame_dict_path + '/' + calibration_file_name

        if not self.client_spawn_frames.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Assembly manager not available...')
            return False
        
        spawn_response:SpawnFramesFromDescription.Response = self.client_spawn_frames.call(request)        
        if not spawn_response.success:
            self.get_logger().error("Failed to spawn frames for calibration cube to cam top")
            return False
        
        return True

    def get_unique_identifier(self, calibration_file_name:str)->str:
        # get the unique identifier from the calibration file
        calibration_frame_dict = {}
        file_path = self.calibration_frame_dict_path + '/' + calibration_file_name
        
        try:
            with open(file_path, 'r') as file:
                calibration_frame_dict = yaml.load(file, Loader=yaml.FullLoader)
        
        except Exception as e:
            self._logger.error("Error: " + str(e))
            return ''
        
        return calibration_frame_dict.get('unique_identifier', '')
    
    
    ###########################################
    ### Calibrate cameras #####################
    ###########################################
    
    def calibrate_cameras_callback(self, request:EmptyWithSuccess.Request, response:EmptyWithSuccess.Response):
        self._logger.warn("Starting Camera calibration!")
        self.load_last_calibrations_data()

        forward_request = EmptyWithSuccess.Request()
        backward_request = EmptyWithSuccess.Request()
        forward_response:EmptyWithSuccess.Response = self.client_move_calibration_target_forward.call(forward_request)

        if not forward_response.success:
            self._logger.error("Failed to move calibration target forward")
            response.success = False
            return response
        
        request_move_to_frame = MoveToFrame.Request()
        request_move_to_frame.target_frame = 'Camera_Station_TCP'
        request_move_to_frame.execute_movement = True
        request_move_to_frame.translation.z = 0.003
        request_move_to_frame.translation.x = 0.00

        if not self.pm_robot_utils.client_move_robot_cam1_to_frame.wait_for_service(timeout_sec=1.0):
            self._logger.error('Camera move service not available...')
            response.success = False
            backward_response:EmptyWithSuccess.Response = self.client_move_calibration_target_backward.call(backward_request)
            return response
        
        response_move_to_frame:MoveToFrame.Response = self.pm_robot_utils.client_move_robot_cam1_to_frame.call(request_move_to_frame)
        
        if not response_move_to_frame.success:
            self._logger.error("Failed to move camera to frame")
            response.success = False
            backward_response:EmptyWithSuccess.Response = self.client_move_calibration_target_backward.call(backward_request)
            return response

        move_success = self.pm_robot_utils.send_xyz_trajectory_goal_absolut(   x_joint= -0.266460,
                                                                y_joint= -0.045949,
                                                                z_joint= 0.002535,
                                                                time=1)
        
        if not move_success:
            response.success = False
            return response

        # this loop is run twice.
        # the first loop calibrates the pixel size and the angle
        # the second loop the displacement of the cooridnate systems
        for iter in range(2):
            # measure frame with bottom cam
            request_execute_vision_bottom = ExecuteVision.Request()
            request_execute_vision_bottom.camera_config_filename = self.pm_robot_utils.get_cam_file_name_bottom()
            request_execute_vision_bottom.image_display_time = -1
            request_execute_vision_bottom.process_filename = "PM_Robot_Calibration/Camera_Calibration_Bottom_Process.json"
            request_execute_vision_bottom.process_uid = f"Cam_Cal_B_{iter}"

            if not self.pm_robot_utils.client_execute_vision.wait_for_service(timeout_sec=1.0):
                self._logger.error('Vision service not available...')
                response.success = False
                backward_response:EmptyWithSuccess.Response = self.client_move_calibration_target_backward.call(backward_request)
                return response
            
            response_execute_vision_bottom:ExecuteVision.Response = self.pm_robot_utils.client_execute_vision.call(request_execute_vision_bottom)
            
            if not response_execute_vision_bottom.success:
                self._logger.error("Failed to execute vision for bottom camera")
                response.success = False
                backward_response:EmptyWithSuccess.Response = self.client_move_calibration_target_backward.call(backward_request)
                return response
            
            # measure frame with top cam
            request_execute_vision_top = ExecuteVision.Request()
            request_execute_vision_top.camera_config_filename = self.pm_robot_utils.get_cam_file_name_top()
            request_execute_vision_top.image_display_time = -1
            request_execute_vision_top.process_filename = "PM_Robot_Calibration/Camera_Calibration_Top_Process.json"
            request_execute_vision_top.process_uid = f"Cam_Cal_T_{iter}"


            if not self.pm_robot_utils.client_execute_vision.wait_for_service(timeout_sec=1.0):
                self._logger.error('Vision service not available...')
                response.success = False
                backward_response:EmptyWithSuccess.Response = self.client_move_calibration_target_backward.call(backward_request)
                return response
            
            response_execute_vision_top:ExecuteVision.Response = self.pm_robot_utils.client_execute_vision.call(request_execute_vision_top)

            if not response_execute_vision_top.success:
                self._logger.error("Failed to execute vision for top camera")
                response.success = False
                backward_response:EmptyWithSuccess.Response = self.client_move_calibration_target_backward.call(backward_request)
                return response 
            
            # Only for the first iteration
            if iter == 0:
                process_success = self._process_calibrate_cameras(vision_result_top=response_execute_vision_top.vision_response.results,
                                        vision_result_bottom=response_execute_vision_bottom.vision_response.results)
                
                if not process_success:
                    response.success = False
                    return response
            
        calibrate_cs_success = self._process_calibrate_coordinate_systems(vision_result_top=response_execute_vision_top.vision_response.results,
                                                                          vision_result_bottom=response_execute_vision_bottom.vision_response.results)
        
        if not calibrate_cs_success:
            response.success = False
            return response


        backward_response:EmptyWithSuccess.Response = self.client_move_calibration_target_backward.call(backward_request)

        if not backward_response.success:
            self._logger.error("Failed to move calibration target backward")
            response.success = False
            return response
        
        response.success = True

        self.last_calibrations_data['cameras'] = f'{datetime.datetime.now()}'
        self.save_last_calibrations_data()
        
        response.success = process_success

        
        return response

    def _process_calibrate_coordinate_systems(self, vision_result_top: vision_msg.VisionResults,
                                vision_result_bottom: vision_msg.VisionResults)->bool:
        
        circle_0_top:vision_msg.VisionCircle = vision_result_top.circles[0]
        circle_1_top:vision_msg.VisionCircle = vision_result_top.circles[1]
        circle_2_top:vision_msg.VisionCircle = vision_result_top.circles[2]
        circle_3_top:vision_msg.VisionCircle = vision_result_top.circles[3]

        circle_0_bottom:vision_msg.VisionCircle = vision_result_bottom.circles[0]
        circle_1_bottom:vision_msg.VisionCircle = vision_result_bottom.circles[1]
        circle_2_bottom:vision_msg.VisionCircle = vision_result_bottom.circles[2]
        circle_3_bottom:vision_msg.VisionCircle = vision_result_bottom.circles[3]

        d_x_0 = circle_0_top.center_point.axis_value_1 - circle_0_bottom.center_point.axis_value_1
        d_y_0 = circle_0_top.center_point.axis_value_2 - circle_0_bottom.center_point.axis_value_2

        d_x_1 = circle_1_top.center_point.axis_value_1 - circle_1_bottom.center_point.axis_value_1
        d_y_1 = circle_1_top.center_point.axis_value_2 - circle_1_bottom.center_point.axis_value_2

        d_x_2 = circle_2_top.center_point.axis_value_1 - circle_2_bottom.center_point.axis_value_1
        d_y_2 = circle_2_top.center_point.axis_value_2 - circle_2_bottom.center_point.axis_value_2

        d_x_3 = circle_3_top.center_point.axis_value_1 - circle_3_bottom.center_point.axis_value_1
        d_y_3 = circle_3_top.center_point.axis_value_2 - circle_3_bottom.center_point.axis_value_2

        d_x = (d_x_0 + d_x_1 + d_x_2 + d_x_3)/4
        d_y = (d_y_0 + d_y_1 + d_y_2 + d_y_3)/4


        self._logger.error(f"Results x: {d_x_0}, {d_x_1}, {d_x_2}, {d_x_3}")
        self._logger.error(f"Results y: {d_y_0}, {d_y_1}, {d_y_2}, {d_y_3}")

        self._logger.error(f"Result x: {d_x}")
        self._logger.error(f"Result y: {d_y}")

        rel_trans = Transform()

        rel_trans.translation.x = d_x * 1e-6
        rel_trans.translation.y = d_y * 1e-6

        trans = self.pm_robot_utils.get_transform_for_frame(frame_name='Cam1_Toolhead_TCP',
                                                            parent_frame='Camera_Station_TCP')
        
        rel_trans.translation.x += trans.translation.x
        rel_trans.translation.y += trans.translation.y

        self._logger.error(f"Result trans x: {rel_trans.translation.x}")
        self._logger.error(f"Result trans y: {rel_trans.translation.y}")


        save_success = self.save_joint_config ( joint_name='Cam1_Toolhead_TCP_Joint',
                                                rel_transformation=rel_trans,
                                                overwrite=True)


        save_success = True

        return save_success
        

    def _process_calibrate_cameras(self, vision_result_top: vision_msg.VisionResults,
                                   vision_result_bottom: vision_msg.VisionResults)->bool:

        ######## At this point, we can start with the calculations for the calibraiton
        # Top image
        self._logger.error(f"Processing Top Vision Circles")
        top_angle, top_dist = self._process_four_circles(vision_result_top.circles)

        self._logger.error(f"Processing Bottom Vision Circles")
        bottom_angle, bottom_dist = self._process_four_circles(vision_result_bottom.circles)

        if (not self.client_calibrate_camera_angle.wait_for_service(1)):
            self._logger.error(f"Service not available {self.client_calibrate_camera_angle.srv_name}")
            return False
        
        if not self.client_calibrate_camera_pixel.wait_for_service(1):
            self._logger.error(f"Service not available {self.client_calibrate_camera_pixel.srv_name}")
            return False
        
        FIDUCIAL_DISTANCE = 1000
        # Set the results for the bottom cam
        angle = 0
        pixel_multiplicator = FIDUCIAL_DISTANCE/bottom_dist

        request_pixel = CalibratePixelPerUm.Request()
        request_pixel.multiplicator = float(pixel_multiplicator)
        request_pixel.camera_config_file_name = self.pm_robot_utils.get_cam_file_name_bottom()

        request_angle = CalibrateAngle.Request()
        request_angle.angle_diff = float(angle)
        request_angle.camera_config_file_name = self.pm_robot_utils.get_cam_file_name_bottom()

        response_pixel: CalibratePixelPerUm.Response = self.client_calibrate_camera_pixel.call(request_pixel)
        response_angle: CalibrateAngle.Response = self.client_calibrate_camera_angle.call(request_angle)

        if (not response_angle.success and response_pixel.success):
            return False

        # Set the results for the top cam
        pixel_multiplicator = FIDUCIAL_DISTANCE/top_dist

        request_pixel = CalibratePixelPerUm.Request()
        request_pixel.multiplicator = float(pixel_multiplicator)
        request_pixel.camera_config_file_name = self.pm_robot_utils.get_cam_file_name_top()

        request_angle = CalibrateAngle.Request()
        request_angle.angle_diff = float(top_angle)
        #request_angle.angle_diff = 0.0

        request_angle.camera_config_file_name = self.pm_robot_utils.get_cam_file_name_top()

        response_pixel: CalibratePixelPerUm.Response = self.client_calibrate_camera_pixel.call(request_pixel)
        response_angle: CalibrateAngle.Response = self.client_calibrate_camera_angle.call(request_angle)

        return (response_pixel.success and response_angle.success)

    def _process_four_circles(self, circle_list: list[vision_msg.VisionCircle])-> tuple[float, float]:
        circle_ind_0 = 0
        circle_ind_1 = 1
        circle_ind_2 = 2
        circle_ind_3 = 3

        circle_0 = circle_list[0]
        circle_1 = circle_list[1]
        circle_2 = circle_list[2]
        circle_3 = circle_list[3]

        average_radius = (circle_0.radius + circle_1.radius + circle_2.radius + circle_3.radius)/len(circle_list) 

        max_radius = max([circle_0.radius, circle_1.radius, circle_2.radius, circle_3.radius])
        min_radius = min([circle_0.radius, circle_1.radius, circle_2.radius, circle_3.radius])

        def distance_between_centers(circle_a:vision_msg.VisionCircle, circle_b:vision_msg.VisionCircle):
            dx = circle_a.center_point.axis_value_1 - circle_b.center_point.axis_value_1
            dy = circle_a.center_point.axis_value_2 - circle_b.center_point.axis_value_2
            return np.sqrt(dx**2 + dy**2)

        length_01 = distance_between_centers(circle_0, circle_1)
        length_13 = distance_between_centers(circle_1, circle_3)
        length_32 = distance_between_centers(circle_3, circle_2)
        length_20 = distance_between_centers(circle_2, circle_0)

        def angle_between(a:vision_msg.VisionCircle, b:vision_msg.VisionCircle):
            dx = b.center_point.axis_value_1 - a.center_point.axis_value_1
            dy = b.center_point.axis_value_2 - a.center_point.axis_value_2
            return np.degrees(np.arctan2(dy, dx))

        angle_01 = angle_between(circle_0, circle_1)-180
        angle_13 = angle_between(circle_1, circle_3)-90
        angle_32 = angle_between(circle_3, circle_2)-0
        angle_20 = angle_between(circle_2, circle_0)+90

        if abs(angle_01)>90:
            angle_01 = -(angle_01+360) 

        self.get_logger().warn(f"Average radius: {average_radius} um")
        self.get_logger().warn(f"max radius: {max_radius} um")
        self.get_logger().warn(f"min radius: {min_radius} um")
        self.get_logger().warn(f"Radius: {average_radius} um")
        self.get_logger().warn(f"Radius: [{circle_0.radius}, {circle_1.radius}, {circle_2.radius}, {circle_3.radius}] um")
        self.get_logger().warn(f"Distances: [{length_01}, {length_13}, {length_32}, {length_20}] um")
        self.get_logger().warn(f"Angles: [{angle_01}, {angle_13}, {angle_32}, {angle_20}] um")

        average_angle = (angle_01 + angle_13 + angle_20 + angle_32)/4

        average_length = (length_01 + length_13 + length_32 + length_20)/4

        self.get_logger().warn(f"Avg angle: {average_angle} um")
        self.get_logger().warn(f"Avg length: {average_length} um")

        return (average_angle, average_length)
        
    ###########################################
    ### Calibrate 1K_dispenser ################
    ###########################################
    
    def calibrate_1K_dispenser_callback(self, request:EmptyWithSuccess.Request, response:EmptyWithSuccess.Response):
        #self.load_last_calibrations_data()
        #frame = '1K_Dispenser_TCP'
        
        data1 = self.pm_robot_utils.get_confocal_bottom_measurement()
        data2 = self.pm_robot_utils.get_confocal_top_measurement()
        self.get_logger().warn(f"Data {str(data1)}, {str(data2)}")

        #measure_success, result_vector = self.measure_frame(frame)
        
        #response.success = measure_success
        
        #self.last_calibrations_data['1K_dispenser'] = f'{datetime.datetime.now()}'
        #self.save_last_calibrations_data()
        return response
     
    ###############################################
    ### Calibrate gripper #########################
    ###############################################
    
    def calibrate_gripper_callback(self, request:EmptyWithSuccess.Request, response:EmptyWithSuccess.Response):
        self.load_last_calibrations_data()
        
        spawn_success, frames, unique_identifier = self.spawn_frames_for_current_gripper()
        
        if not spawn_success:
            self.get_logger().error("Failed to spawn gripper frames...")
            response.success = False
            return response
                
        #rotations = [0.0, 20, 30, 40, 50, 60, 80]
        #rotations = [90, 65, 55, 45, 35, 25, 0]
        rotations = [90, 60, 30, 0]

        
        #rotations = [60, 40, 20, 0]

        # convert to rad
        rotations = [r * np.pi / 180.0 for r in rotations]
        
        distance_list: list[float] = []  
        frame_poses_list:list[Pose] = []      
        for index, rotation in enumerate(rotations):
            move_success = self.pm_robot_utils.send_t_trajectory_goal_absolut(rotation, 2.0)
            self.get_logger().error("Gripper rotation: " + str(rotation))
            if not move_success:
                self.get_logger().error("Failed to move gripper to rotation: " + str(rotation))
                response.success = False
                return response
                        
            for frame in frames:
                                
                if 'Vision' in frame or 'vision' in frame:
                    correct_frame_success = self.correct_frame(frame)
                    #correct_frame_success = self.measure_frame(frame)
                    
                    if not correct_frame_success:
                        self.get_logger().error("Failed to correct frame: " + frame)
                        response.success = False
                        return response
              
            ref_frame = get_ref_frame_by_name(self.pm_robot_utils.object_scene, f'{unique_identifier}CALIBRATION_PM_Robot_Tool_TCP')
            
            if ref_frame is None:
                self.get_logger().error("Failed to get reference frame...")
                response.success = False
                return response
            
            if index !=0:
                pose_1 = frame_poses_list[index-1]
                pose_2 = ref_frame.pose
                distance = np.sqrt((pose_1.position.x - pose_2.position.x)**2 + (pose_1.position.y - pose_2.position.y)**2)
                distance_list.append(distance)


            frame_poses_list.append(copy.deepcopy(ref_frame.pose))
        
        relative_transform:Transform = get_rel_transform_for_frames(scene=self.pm_robot_utils.object_scene,
                                    from_frame=f'{unique_identifier}CALIBRATION_PM_Robot_Tool_TCP',
                                    to_frame=f'{unique_identifier}CALIBRATION_PM_Robot_Tool_TCP_initial',
                                    tf_buffer=self.pm_robot_utils.tf_buffer,
                                    logger=self._logger)
        
        min_distance = min(distance_list)
        # fit a circle to the points to find the center
                
        #2.3656241026821336e-07
        self._logger.info("Min distance: " + str(min_distance * 1e6) + " um")
        self._logger.info(f" length of frame_poses_list: {len(frame_poses_list)}")
        
        # check if the fit is good
        if distance > 20 * 1e-6:
            x,y,r,s = self.find_circle_coordinates(copy.copy(frame_poses_list))
            self._logger.info("Circle center: " + str(x) + ", " + str(y) + ", radius (um): " + str(r* 1e6) + ", s: " + str(s))
            
            rel_t_joint = Transform()
            rel_t_joint.translation.x = -1*(relative_transform.translation.x - x)
            rel_t_joint.translation.y = -1*(relative_transform.translation.y - y)
            
            relative_transform.translation.x = x
            relative_transform.translation.y = y
            
            self._logger.warn(f"T-Axis has offset: {x* 1e6}, {y* 1e6} um")
            
            self._logger.error(f"Translation of the rotation point")
            self._logger.error(f"x offset: {rel_t_joint.translation.x * 1e6} um")
            self._logger.error(f"y offset: {rel_t_joint.translation.y * 1e6} um")
            
            self.save_joint_config('T_Axis_Joint', rel_t_joint)
            
            self.plot_gripper_calibration_poses(copy.copy(frame_poses_list),
                                                radius=r,
                                                circle_x=x,
                                                circle_y=y)
            
        # assuming the gripper is at the center of the circle
        else:
            self._logger.warn("T-Axis has no offset...")

        # log relavite pose
        #self._logger.error("Relative pose: " + str(relative_transform))
        self._logger.error("Translation of the gripper tip")
        self._logger.error(f"x offset: {relative_transform.translation.x * 1e6} um")
        self._logger.error(f"y offset: {relative_transform.translation.y * 1e6} um")
        
        self.save_joint_config('PM_Robot_Tool_TCP_Joint', relative_transform)
        
        self.last_calibrations_data['gripper'] = f'{datetime.datetime.now()}'
        self.save_last_calibrations_data()
        return response        
    
    def find_circle_coordinates(self, poses:list[Pose]):
        points = []
        for pose in poses:
            points.append([pose.position.x*1e6, pose.position.y*1e6])    
        # Fit a circle to the points
        x, y, r, s = circle_fit.hyperLSQ(points)
        x = x * 1e-6
        y = y * 1e-6
        r = r * 1e-6
        return x,y,r,s
    
    def plot_gripper_calibration_poses(self, frame_poses_list: list[Pose], 
                                    radius: float, 
                                    circle_x: float, 
                                    circle_y: float):
        
        fig, ax = plt.subplots()
        ax.set_title('Gripper Calibration Poses')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')

        for pose in frame_poses_list:
            x = pose.position.x * 1e6
            y = pose.position.y * 1e6
            ax.scatter(x, y, c='r', marker='o')

        # Create and add the circle to the axes
        circle = plt.Circle((circle_x * 1e6, circle_y * 1e6), radius * 1e6, color='b', fill=False)
        ax.add_patch(circle)  # ← This is what was missing

        ax.grid(True)
        ax.set_aspect('equal')  # Ensures circle is not distorted
        #plt.show()
        #save the figure
        fig.savefig(f'{self.calibration_frame_dict_path}/gripper_calibration_poses.png')    


    ###########################################
    ### calibration_cube_to_cam_top ###########
    ###########################################

    
    def calibrate_calibration_cube_to_cam_top_callback(self, request:EmptyWithSuccess.Request, response:EmptyWithSuccess.Response):
        self.load_last_calibrations_data()
        
        # Spawn the frames
        spawn_success = self.spawn_calibration_frames('CF_Calibration_Qube_Cam_Top.json')
        
        if not spawn_success:
            self.get_logger().error(f"Spawning of frames failed!")

            response.success = False
            return response
        unique_identifier = self.get_unique_identifier('CF_Calibration_Qube_Cam_Top.json')

        # To be implemented...
        self.get_logger().error("Calibration cube to cam top not fully implemented yet...")

        vision_request = ExecuteVision.Request()
        vision_request.process_filename = f"Assembly_Manager/Frames/{unique_identifier}Vision_Dynamic.json"
        vision_request.process_uid = "Cal_Calibrate_Cube"
        vision_request.image_display_time = -1
        vision_request.camera_config_filename = self.pm_robot_utils.get_cam_file_name_top()


        if not self.pm_robot_utils.client_execute_vision.wait_for_service(timeout_sec=1.0):
            self._logger.error("Service 'ExecuteVision' not available...")
            response.success= False
            return response
        
        # try to move the frame to the bottom camera
        move_request = MoveToFrame.Request()
        move_request.execute_movement = True
        move_request.target_frame = f'{unique_identifier}Vision_Dynamic'


        #if not self.pm_robot_utils.client_move_robot_tool_to_frame.wait_for_service(timeout_sec=1.0):
        if not self.pm_robot_utils.client_move_robot_cam1_to_frame.wait_for_service(timeout_sec=1.0):
            self._logger.error("Service 'MoveCamToFrame' not available...")
            response.success= False
            return response
        
        #result_tool_to_bottom_cam:MoveToFrame.Response = self.pm_robot_utils.client_move_robot_tool_to_frame.call(move_request)
        result_tool_to_frame:MoveToFrame.Response = self.pm_robot_utils.client_move_robot_cam1_to_frame.call(move_request)

        if not result_tool_to_frame.success:
            self._logger.error(f"Could not move the camera to frame {move_request.target_frame}...")
            response.success= False
            return response

        # Measure the frame


        result:ExecuteVision.Response = self.pm_robot_utils.client_execute_vision.call(vision_request)

        if not result.success:
            self._logger.error("Vision process failed...")
            response.success= False
            return response
            
        detected_point:vision_msg.VisionPoint = result.vision_response.results.points[0]

        self.pm_robot_utils.send_xyz_trajectory_goal_relative(x_joint_rel=detected_point.axis_value_1*1e-6,
                                                              y_joint_rel=detected_point.axis_value_2*1e-6,
                                                              z_joint_rel=0,
                                                              time=0.5)
        
        rel_transform = Transform()
        rel_transform.translation.x = detected_point.axis_value_1
        rel_transform.translation.y = detected_point.axis_value_2

        self._logger.warn(f"Result x {rel_transform.translation.x}...")
        self._logger.warn(f"Result y {rel_transform.translation.y}...")


        success = self.save_joint_config('Calibration_Qube_Joint', rel_transform, unit = 'um')
        
        if not success:
            self.get_logger().error("Failed to save joint config...")
            response.success = False
        
        response.success = True

        self.last_calibrations_data['calibration_cube_to_cam_top'] = f'{datetime.datetime.now()}'
        self.save_last_calibrations_data()
        
        return response
    
    ###########################################
    ### laser_on_calibration_cube ############
    ###########################################
    
    def calibrate_laser_on_calibration_cube_callback(self, request:EmptyWithSuccess.Request, response:EmptyWithSuccess.Response):
        # To be implemented...
        self.get_logger().warn("Laser on calibration cube not fully implemented yet...")
        
        if not is_frame_from_scene(self.pm_robot_utils.object_scene, 
                                   'CAL_Calibration_Qube_Cam_Top_Vision_Dynamic'):
            response.success = False
            self.get_logger().error("Missing calibration frame. Exeucte 'calibrate_calibration_cube_to_cam_top' first...")
            return response
        
        self.load_last_calibrations_data()

        spawn_success = self.spawn_calibration_frames('CF_Laser_to_Calibration_Qube.json')
        
        unique_identifier = self.get_unique_identifier('CF_Laser_to_Calibration_Qube.json')

        if not spawn_success:
            self.get_logger().error("Failed to spawn calibration frames...")
            response.success = False
            return response
        
        # Move laser to calibration cube
        move_request = MoveToFrame.Request()
        move_request.target_frame = 'CAL_Calibration_Qube_Cam_Top_Vision_Dynamic'
        move_request.execute_movement = True
        move_request.translation.x = 1*1e-3
        move_request.translation.y = 1*1e-3
        move_request.translation.z = 0.0

        if not self.pm_robot_utils.client_move_robot_laser_to_frame.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Laser move service not available...')
            response.success = False
            return response
        
        # move to intial position
        response_move:MoveToFrame.Response = self.pm_robot_utils.client_move_robot_laser_to_frame.call(move_request)
        
        if not response_move.success:
            self.get_logger().error("Failed to move laser to calibration cube")
            response.success = False
            return response
                
        # set the laser to the zero height
        initial_z_height = self.pm_robot_utils.get_laser_measurement(unit='m')
        
        self.get_logger().error(f'Heigt {initial_z_height}')

        self.pm_robot_utils.send_xyz_trajectory_goal_relative(0.0, 0.0, -initial_z_height, 0.5)
        
        step_inc = 0.1
        self.get_logger().error("STARTING Y DIRECTION ROUGTH")

        # sense the y direction
        x, y_joint_result, z = self.interative_sensing(measurement_method=self.pm_robot_utils.get_laser_measurement,
                                                 measurement_valid_function = self.pm_robot_utils._check_for_valid_laser_measurement,
                                                 length = (0.0, -3.0, 0.0),
                                                 step_inc = step_inc,
                                                 total_time = 8.0)

        
        # move back to initial state
        response_move:MoveToFrame.Response = self.pm_robot_utils.client_move_robot_laser_to_frame.call(move_request)

        if not response_move.success:
            self.get_logger().error("Failed to move laser to calibration cube")
            response.success = False
            return response     

        self.get_logger().error("STARTING X DIRECTION ROUGTH")

        x_joint_result, y, z = self.interative_sensing(measurement_method=self.pm_robot_utils.get_laser_measurement,
                                                        measurement_valid_function = self.pm_robot_utils._check_for_valid_laser_measurement,
                                                        length = (-3.0, 0.0, 0.0),
                                                        step_inc = step_inc,
                                                        total_time = 8.0)
        
        if x_joint_result is None or y_joint_result is None:
            self.get_logger().error("Failed to get laser measurement...")
            response.success = False
            return response
        
        self.get_logger().error("Current X joint: " + str(x_joint_result))
        self.get_logger().error("Current Y joint: " + str(y_joint_result))

    
        #move to result position      
        self.pm_robot_utils.send_xyz_trajectory_goal_absolut(x_joint_result,
                                                            y_joint_result,
                                                            z,
                                                            time=1.0)
        
        time.sleep(1.0)
        
        # second iteration
        self.pm_robot_utils.send_xyz_trajectory_goal_relative(x_joint_rel=0.0002, 
                                                              y_joint_rel=0.0002, 
                                                              z_joint_rel=0, 
                                                              time=0.5)
        
        time.sleep(1.0)
        cal_height = self.pm_robot_utils.get_laser_measurement(unit='m')

        self.pm_robot_utils.send_xyz_trajectory_goal_relative(0.0, 0.0, -cal_height, 0.5)

        time.sleep(1.0)
        
        step_inc = 0.01
        self.get_logger().error("STARTING Y DIRECTION FINE")

        x, y_joint_result_2, z = self.interative_sensing(measurement_method=self.pm_robot_utils.get_laser_measurement,
                                                    measurement_valid_function = self.pm_robot_utils._check_for_valid_laser_measurement,
                                                 length = (0.0, -1.0, 0.0),
                                                 step_inc = step_inc,
                                                 total_time = 8.0)
        
        self.pm_robot_utils.send_xyz_trajectory_goal_relative(  x_joint_rel=0, 
                                                                y_joint_rel=0.0002, 
                                                                z_joint_rel=0, 
                                                                time=0.5)
        
        self.get_logger().error("STARTING X DIRECTION FINE")

        time.sleep(1)

        x_joint_result_2, y, z = self.interative_sensing(measurement_method=self.pm_robot_utils.get_laser_measurement,
                                                        measurement_valid_function = self.pm_robot_utils._check_for_valid_laser_measurement,
                                                        length = (-1.0, 0.0, 0.0),
                                                        step_inc = step_inc,
                                                        total_time = 8.0)

        
        if x_joint_result_2 is None or y_joint_result_2 is None:
            self.get_logger().error("Failed to get laser measurement...")
            response.success = False
            return response
        
        self.get_logger().error("Current X joint: " + str(x_joint_result_2))
        self.get_logger().error("Current Y joint: " + str(y_joint_result_2))

        # move to result position
        self.pm_robot_utils.send_xyz_trajectory_goal_absolut(x_joint_result_2,
                                                            y_joint_result_2,
                                                            z_joint=z,
                                                            time=1.0)
        
        time.sleep(1.0)
        
        relative_transform = get_rel_transform_for_frames(scene=self.pm_robot_utils.object_scene,
                                                from_frame='CAL_Calibration_Qube_Cam_Top_Vision_Dynamic',
                                                to_frame=f'CAL_Laser_Toolhead_TCP_to_Calibration_Qube',
                                                tf_buffer=self.pm_robot_utils.tf_buffer,
                                                logger=self._logger)
        
        self._logger.error("Relative pose: " + str(relative_transform))

        if relative_transform is None:
            self.get_logger().error("Failed to get relative pose...")
            response.success = False
            return response
        
        success = self.save_joint_config('Laser_Toolhead_TCP_Joint', 
                                         relative_transform,
                                         overwrite=False)
        
        if not success:
            self.get_logger().error("Failed to save joint config...")
            response.success = False
            return response    
                
        response.success = True
        self.last_calibrations_data['laser_on_calibration_cube'] = f'{datetime.datetime.now()}'
        self.save_last_calibrations_data()

        return response
    
    
    ###########################################
    ### Confocal_top ##########################
    ###########################################
        
    def calibrate_confocal_top_callback(self, request:EmptyWithSuccess.Request, response:EmptyWithSuccess.Response):
        self.load_last_calibrations_data()

        if not is_frame_from_scene(self.pm_robot_utils.object_scene, 
                                   'CAL_Calibration_Qube_Cam_Top_Vision_Dynamic'):
            
            self.get_logger().error("Missing calibration frame. Exeucte 'calibrate_calibration_cube_to_cam_top' first...")
            response.success =False
            return response
        
        self.load_last_calibrations_data()

        spawn_success = self.spawn_calibration_frames('CF_Confocal_to_Calibration_Qube.json')
        
        unique_identifier = self.get_unique_identifier('CF_Confocal_to_Calibration_Qube.json')

        if not spawn_success:
            self.get_logger().error("Failed to spawn calibration frames...")
            response.success = False
            return response
        
        # Move laser to calibration cube
        move_request = MoveToFrame.Request()
        move_request.target_frame = 'CAL_Calibration_Qube_Cam_Top_Vision_Dynamic'
        move_request.execute_movement = True
        move_request.translation.x = 0.5*1e-3
        move_request.translation.y = 0.5*1e-3

        if not self.pm_robot_utils.client_move_robot_confocal_top_to_frame.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Laser move service not available...')
            response.success = False
            return response
        
        # move to intial position
        response_move:MoveToFrame.Response = self.pm_robot_utils.client_move_robot_confocal_top_to_frame.call(move_request)
        
        if not response_move.success:
            self.get_logger().error("Failed to move laser to calibration cube")
            response.success = False
            return response
                
        # set the laser to the zero height
        initial_z_height = self.pm_robot_utils.get_confocal_top_measurement(unit='m')
        
        self.get_logger().error(f'Heigt {initial_z_height}')

        self.pm_robot_utils.send_xyz_trajectory_goal_relative(0.0, 0.0, -initial_z_height, 0.5)

        time.sleep(2)

        step_inc = 0.1 # mm
        self.get_logger().error("STARTING Y DIRECTION ROUGTH")

        # sense the y direction
        x, y_joint_result, z = self.interative_sensing(measurement_method=self.pm_robot_utils.get_confocal_top_measurement,
                                                        measurement_valid_function = self.pm_robot_utils.check_confocal_top_measurement_in_range,
                                                 length = (0.0, -3.0, 0.0),
                                                 step_inc = step_inc,
                                                 total_time = 8.0)

        
        # move back to initial state
        response_move:MoveToFrame.Response = self.pm_robot_utils.client_move_robot_confocal_top_to_frame.call(move_request)

        if not response_move.success:
            self.get_logger().error("Failed to move laser to calibration cube")
            response.success = False
            return response     

        self.get_logger().error("STARTING X DIRECTION ROUGTH")

        x_joint_result, y, z = self.interative_sensing(measurement_method=self.pm_robot_utils.get_confocal_top_measurement,
                                                        measurement_valid_function = self.pm_robot_utils.check_confocal_top_measurement_in_range,
                                                        length = (-3.0, 0.0, 0.0),
                                                        step_inc = step_inc,
                                                        total_time = 8.0)
        
        if x_joint_result is None or y_joint_result is None:
            self.get_logger().error("Failed to get laser measurement...")
            response.success = False
            return response
        
        self.get_logger().error("Current X joint: " + str(x_joint_result))
        self.get_logger().error("Current Y joint: " + str(y_joint_result))

    
        #move to result position      
        self.pm_robot_utils.send_xyz_trajectory_goal_absolut(x_joint_result,
                                                            y_joint_result,
                                                            z,
                                                            time=1.0)
        
        time.sleep(1.0)

        
        
        # second iteration
        self.pm_robot_utils.send_xyz_trajectory_goal_relative(x_joint_rel=0.0002, 
                                                              y_joint_rel=0.0002, 
                                                              z_joint_rel=0, 
                                                              time=0.5)
        
        time.sleep(1.0)

        cal_height = self.pm_robot_utils.get_confocal_top_measurement(unit='m')

        self.pm_robot_utils.send_xyz_trajectory_goal_relative(0.0, 0.0, -cal_height, 0.5)
        
        time.sleep(1.0)

        step_inc = 0.01 # mm
        self.get_logger().error("STARTING Y DIRECTION FINE")

        x, y_joint_result_2, z = self.interative_sensing(measurement_method=self.pm_robot_utils.get_confocal_top_measurement,
                                                        measurement_valid_function = self.pm_robot_utils.check_confocal_top_measurement_in_range,
                                                        length = (0.0, -1.0, 0.0),
                                                        step_inc = step_inc,
                                                        total_time = 8.0)
        
        time.sleep(1.0)
        
        self.pm_robot_utils.send_xyz_trajectory_goal_relative(  x_joint_rel=0, 
                                                                y_joint_rel=0.0002, 
                                                                z_joint_rel=0, 
                                                                time=0.5)
        
        time.sleep(1.0)
        
        self.get_logger().error("STARTING X DIRECTION FINE")

        x_joint_result_2, y, z = self.interative_sensing(measurement_method=self.pm_robot_utils.get_confocal_top_measurement,
                                                        measurement_valid_function = self.pm_robot_utils.check_confocal_top_measurement_in_range,
                                                        length = (-1.0, 0.0, 0.0),
                                                        step_inc = step_inc,
                                                        total_time = 8.0)
        
        time.sleep(1.0)

        if x_joint_result_2 is None or y_joint_result_2 is None:
            self.get_logger().error("Failed to get laser measurement...")
            response.success = False
            return response
        
        self.get_logger().error("Current X joint: " + str(x_joint_result_2))
        self.get_logger().error("Current Y joint: " + str(y_joint_result_2))
        
        # move to result position
        self.pm_robot_utils.send_xyz_trajectory_goal_absolut(   x_joint_result_2,
                                                                y_joint_result_2,
                                                                z_joint=z,
                                                                time=1.0)
            
        time.sleep(1.0)
        
        relative_transform = get_rel_transform_for_frames(scene=self.pm_robot_utils.object_scene,
                                                from_frame='CAL_Calibration_Qube_Cam_Top_Vision_Dynamic',
                                                to_frame=f'CAL_Confocal_TCP_to_Calibration_Qube',
                                                tf_buffer=self.pm_robot_utils.tf_buffer,
                                                logger=self._logger)
        
        relative_transform_2 = get_rel_transform_for_frames(scene=self.pm_robot_utils.object_scene,
                                        from_frame='CAL_Calibration_Qube_Cam_Top_Vision_Dynamic',
                                        to_frame=f'CAL_Confocal_TCP_2_to_Calibration_Qube',
                                        tf_buffer=self.pm_robot_utils.tf_buffer,
                                        logger=self._logger)
        
        if relative_transform is None or relative_transform_2 is None:
            self.get_logger().error("Failed to get relative pose...")
            response.success = False
            return response
        
        if abs(relative_transform.translation.x) > abs(relative_transform_2.translation.x):
            transfrom = relative_transform_2
            self._logger.info(f"Confocal TCP is at position 2.")


        else:
            transfrom = relative_transform
            self._logger.info(f"Confocal TCP is at position 1.")


        # This affects both TCPs at it uses the same joint calibration. Is is no issue as only one tcp is uesed at a time
        success = self.save_joint_config('Confocal_Sensor_Top_TCP_Joint', 
                                transfrom,
                                overwrite=False)

        if not success:
            self.get_logger().error("Failed to save joint config...")
            response.success = False
            return response    
        
        response.success = True

        self.get_logger().error(f"Results x: {transfrom.translation.x*1e6} um, y: {transfrom.translation.y*1e6} um, z: {transfrom.translation.z*1e6} um")

        self.last_calibrations_data['confocal_top'] = f'{datetime.datetime.now()}'
        self.save_last_calibrations_data()

        time.sleep(4.0)

        # Move confocal away
        move_request = MoveToFrame.Request()
        move_request.target_frame = 'CAL_Calibration_Qube_Cam_Top_Vision_Dynamic'
        move_request.execute_movement = True
        move_request.translation.z = 0.03

        if not self.pm_robot_utils.client_move_robot_confocal_top_to_frame.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Laser move service not available...')
            response.success = False
            return response
        
        # move to intial position
        response_move:MoveToFrame.Response = self.pm_robot_utils.client_move_robot_confocal_top_to_frame.call(move_request)
        
        if not response_move.success:
            self.get_logger().error("Failed to move laser to calibration cube")
            response.success = False
            return response
        
        return response
    
    ############################################
    ### Calibration Siemens Gripper to cube ####
    ############################################

    def calibrate_sim_gripper_on_cube(self, request:EmptyWithSuccess.Request, response:EmptyWithSuccess.Response):
        # We currently need this calbiration function. However we would like to get rid of this calibration as it is only possible to do with the siemens gripper

        self.pm_robot_utils.pm_robot_config.tool.reload_config()
        current_tool = self.pm_robot_utils.pm_robot_config.tool.get_tool().get_current_tool()

        if current_tool != 'Siemens_Vacuum_Array_short':
            response.success = False
            self.get_logger().error(f'Current tool is not "Siemens_Vacuum_Array_short"! Executing this funciton is not possible!')
            return response

        set_success = self.pm_robot_utils.set_force_sensor_bias()
        
        if not set_success:
            response.success = False
            return response
        
        # Move gripper away
        move_request = MoveToFrame.Request()
        move_request.target_frame = 'Calibration_Qube'
        move_request.execute_movement = True
        move_request.translation.z = 0.001 # 1 mm above the cube

        if not self.pm_robot_utils.client_move_robot_tool_to_frame.wait_for_service(timeout_sec=1.0):
            self.get_logger().error(f'Client {self.pm_robot_utils.client_move_robot_tool_to_frame.srv_name} not available!')
            response.success = False
            return response
        
        # move to intial position
        response_move:MoveToFrame.Response = self.pm_robot_utils.client_move_robot_tool_to_frame.call(move_request)
        
        if not response_move.success:
            self.get_logger().error("Failed to move laser to calibration cube")
            response.success = False
            return response
        
        # check client avaliable
        if not self.pm_robot_utils.client_get_force_measurement.wait_for_service(1):
            self.get_logger().error(f'Client {self.pm_robot_utils.client_get_force_measurement.srv_name} not available!')
            response.success = False
            return response
        # get force measurement
        request_force = ForceSensorGetMeasurement.Request()

        # Rougth sensing
        step_size = 0.0001 # m
        distance = 0.0013 # m
        max_steps = distance //step_size 

        for iterator in range(int(max_steps)):
            
            response_measurement: ForceSensorGetMeasurement.Response = self.pm_robot_utils.client_get_force_measurement.call(request_force)
            z_force = response_measurement.data[2]
            self.get_logger().error(f"Z Force is: {z_force}")

            if abs(z_force) > 0.1:
                self.get_logger().error(f"Force is exeeded!")
                break
            
            else:
                self.pm_robot_utils.send_xyz_trajectory_goal_relative(  x_joint_rel=0, 
                                                                y_joint_rel=0.0, 
                                                                z_joint_rel=step_size, 
                                                                time=0.5)
                time.sleep(1.0)

            self.get_logger().error(f"Running iteration: {iterator}")

        if (iterator >= max_steps):
            self.get_logger().error(f"Sensing failed. Max steps reached!")
            response.success = False
            return response
        
        time.sleep(1.0)

        self.pm_robot_utils.send_xyz_trajectory_goal_relative(  x_joint_rel=0, 
                                                                y_joint_rel=0.0, 
                                                                z_joint_rel=-(0.00015), 
                                                                time=0.5)
        
        time.sleep(1.0)
        self.get_logger().error(f"Starting fine search!")
        # Fine Sensing
        step_size = 0.00001 # m
        distance = 0.00025 # m
        max_steps = distance //step_size 

        for iterator in range(int(max_steps)):
            
            response_measurement: ForceSensorGetMeasurement.Response = self.pm_robot_utils.client_get_force_measurement.call(request_force)
            z_force = response_measurement.data[2]
            self.get_logger().error(f"Z Force is: {z_force}")

            if abs(z_force) > 0.1:
                self.get_logger().error(f"Force is exeeded!")
                break
            
            else:
                self.pm_robot_utils.send_xyz_trajectory_goal_relative(  x_joint_rel=0, 
                                                                y_joint_rel=0.0, 
                                                                z_joint_rel=step_size, 
                                                                time=0.5)
                time.sleep(1.0)

            self.get_logger().error(f"Running iteration: {iterator}")

        if (iterator >= max_steps):
            self.get_logger().error(f"Sensing failed. Max steps reached!")
            response.success = False
            return response
        
        time.sleep(1.0)
        transform = self.pm_robot_utils.get_transform_for_frame('PM_Robot_Tool_TCP', 'Calibration_Qube')
        self.get_logger().error(f"Transform: {str(transform)}")

        time.sleep(5.0)

        # Move gripper away
        move_request = MoveToFrame.Request()
        move_request.target_frame = 'Calibration_Qube'
        move_request.execute_movement = True
        move_request.translation.z = 0.03 # 30 mm above the cube

        if not self.pm_robot_utils.client_move_robot_tool_to_frame.wait_for_service(timeout_sec=1.0):
            self.get_logger().error(f'Client {self.pm_robot_utils.client_move_robot_tool_to_frame.srv_name} not available!')
            response.success = False
            return response
        
        # move to intial position
        response_move:MoveToFrame.Response = self.pm_robot_utils.client_move_robot_tool_to_frame.call(move_request)

        response.success = True

        return response
    ###########################################
    ### Calibration target to cam bottom ######
    ###########################################
    
    def calibrate_calibration_target_to_cam_bottom_callback(self, request:EmptyWithSuccess.Request, response:EmptyWithSuccess.Response):
        # To be implemented...
        self.load_last_calibrations_data()
        self.get_logger().error("Calibration target to cam bottom not implemented yet...")
        self.last_calibrations_data['calibration_target_to_cam_bottom'] = f'{datetime.datetime.now()}'
        self.save_last_calibrations_data()
        return response

    ###########################################
    ### Confocal_bottom #######################
    ###########################################
    
    def calibrate_confocal_bottom_callback(self, request:EmptyWithSuccess.Request, response:EmptyWithSuccess.Response):
        self.load_last_calibrations_data()
        # To be implemented...
        self.get_logger().error("Confocal bottom not implemented yet...")
        self.last_calibrations_data['confocal_bottom'] = f'{datetime.datetime.now()}'
        self.save_last_calibrations_data()
        return response
    
    ###########################################
    ### Gonio left chuck ######################
    ###########################################
    
    def calibrate_gonio_left_chuck_callback(self, request:EmptyWithSuccess.Request, response:EmptyWithSuccess.Response):
        self.load_last_calibrations_data()
        # To be implemented...
        self.get_logger().error("Gonio left chuck not implemented yet...")
        self.last_calibrations_data['gonio_left_chuck'] = f'{datetime.datetime.now()}'
        self.save_last_calibrations_data()
        return response
    
    ###########################################
    ### Gonio right chuck #####################
    ###########################################
    
    def calibrate_gonio_right_chuck_callback(self, request:EmptyWithSuccess.Request, response:EmptyWithSuccess.Response):
        self.load_last_calibrations_data()
        # To be implemented...
        self.get_logger().error("Gonio right chuck not implemented yet...")
        self.last_calibrations_data['gonio_right_chuck'] = f'{datetime.datetime.now()}'
        self.save_last_calibrations_data()
        return response
    
    def measure_frame(self, frame_id:str)->tuple[bool, Vector3]:
        
        if not self.client_measure_frame_cam.wait_for_service(timeout_sec=1.0):
            self._logger.error('Vision measure frame service not available...')
            return False, None
        
        request = MeasureFrame.Request()
        request.frame_name = frame_id
        response:MeasureFrame.Response = self.client_measure_frame_cam.call(request)
        
        return response.success, response.result_vector
    
    def correct_frame(self, frame_id:str)->bool:
        
        if not self.client_correct_frame.wait_for_service(timeout_sec=1.0):
            self._logger.error('Vision correct frame service not available...')
            return False
        
        request = CorrectFrame.Request()
        request.frame_name = frame_id
        response:CorrectFrame.Response = self.client_correct_frame.call(request)
        
        return response.success
    
    def modify_pose_from_frame(self, modify_pose_request:ami_srv.ModifyPoseFromFrame.Request)->bool:
        if not self.client_modify_pose_from_frame.wait_for_service(timeout_sec=1.0):
            self._logger.error('Modify pose from frame service not available...')
            return False
        
        request = ami_srv.ModifyPoseFromFrame.Request()
        
        response:ami_srv.ModifyPoseFromFrame.Response = self.client_modify_pose_from_frame.call(request)
        
        return response.success
    
    def save_joint_config(self, joint_name:str, 
                          rel_transformation:Transform,
                          unit = 'm',
                          overwrite = False)->bool:
        
        if unit == 'm':
            pass
        elif unit == 'mm':
            rel_transformation.translation.x = rel_transformation.translation.x * 1e-3
            rel_transformation.translation.y = rel_transformation.translation.y * 1e-3
            rel_transformation.translation.z = rel_transformation.translation.z * 1e-3
        elif unit == 'um':
            rel_transformation.translation.x = rel_transformation.translation.x * 1e-6
            rel_transformation.translation.y = rel_transformation.translation.y * 1e-6
            rel_transformation.translation.z = rel_transformation.translation.z * 1e-6
        else:
            self._logger.fatal('Fatal error in calibration program. Invalid unit!')
            return False
        
        package_name = 'pm_robot_description'
        file_name = 'calibration_config/pm_robot_joint_calibration.yaml'
        file_path = get_package_share_directory(package_name) + '/' + file_name

        calibration_config = {}
        # check if the file exists
        try:
            with open(file_path, 'r') as file:
                calibration_config = yaml.load(file, Loader=yaml.FullLoader)
            
            joint_config = calibration_config.get(joint_name, None)
            current_transformation = Transform()
            
            if joint_config is not None:
                current_transformation.translation.x = joint_config['x_offset'] * 1e-6 
                current_transformation.translation.y = joint_config['y_offset'] * 1e-6 
                current_transformation.translation.z = joint_config['z_offset'] * 1e-6 
                rx_offset = joint_config['rx_offset']
                ry_offset = joint_config['ry_offset']
                rz_offset = joint_config['rz_offset']
                
                quat = R.from_euler('xyz', [rx_offset, ry_offset, rz_offset], degrees=True).as_quat()
                current_transformation.rotation.x = quat[0]
                current_transformation.rotation.y = quat[1]
                current_transformation.rotation.z = quat[2]
            #new_transform = get_relative_transform_for_transforms(current_transformation, rel_transformation)
            if not overwrite:
                new_transform = get_relative_transform_for_transforms(rel_transformation, current_transformation)
            else:
                new_transform = rel_transformation
                self._logger.warn(f"Overwriting existing calibration data.")

            calibration_config[joint_name]['x_offset'] = new_transform.translation.x * 1e6
            calibration_config[joint_name]['y_offset'] = new_transform.translation.y * 1e6
            calibration_config[joint_name]['z_offset'] = new_transform.translation.z * 1e6

            q = [new_transform.rotation.x,
                 new_transform.rotation.y,
                 new_transform.rotation.z,
                 new_transform.rotation.w]

            # Convert to Euler angles (roll, pitch, yaw) in radians
            r = R.from_quat(q)
            roll, pitch, yaw = r.as_euler('xyz', degrees=True)
            
            calibration_config[joint_name]['rx_offset'] = float(roll)
            calibration_config[joint_name]['ry_offset'] = float(pitch)
            calibration_config[joint_name]['rz_offset'] = float(yaw)
            
        except FileNotFoundError:
            calibration_config = {}
        except Exception as e:
            self._logger.error("Error: " + str(e))
            return False
        # write the file
        try:
            with open(file_path, 'w') as file:
                yaml.dump(calibration_config, file)
                pass
            #print("Calibration config saved to: " + file_path)
            return True
        except Exception as e:
            self._logger.error("Error: " + str(e))
            return False
    
    def interative_sensing(self,
                           measurement_method:any,
                           measurement_valid_function:any,
                           length: tuple[float, float, float],
                           step_inc: float,
                           total_time: float):
        """_summary_

        Args:
            measurement_method (any): _description_
            measurement_bounds (tuple[float, float]): _description_
            length (tuple[float, float, float]): in mm
            step_inc (float): in mm
            total_time (float): _description_

        Returns:
            _type_: _description_
        """
        abs_length = [abs(length[0]), abs(length[1]), abs(length[2])]
        
        max_length = max(abs_length)
        total_steps = int(max_length / step_inc)
        step_time = total_time / total_steps
        
        x_step = length[0] / total_steps
        y_step = length[1] / total_steps
        z_step = length[2] / total_steps

        # log all the values
        self.get_logger().warn("Total steps: " + str(total_steps))
        self.get_logger().warn("Step time: " + str(step_time))
        self.get_logger().warn("X step: " + str(x_step))
        self.get_logger().warn("Y step: " + str(y_step))
        self.get_logger().warn("Z step: " + str(z_step))
        self.get_logger().warn("Length: " + str(length))
        
        for i in range(total_steps):

            move_success = self.pm_robot_utils.send_xyz_trajectory_goal_relative(x_joint_rel = x_step*1e-3,
                                                                                 y_joint_rel = y_step*1e-3, 
                                                                                 z_joint_rel = z_step*1e-3, 
                                                                                 time = step_time)
            
            if not move_success:
                self.get_logger().error("Failed to move laser ROUTINE 2")
                return False
            
            time.sleep(0.1)

            #laser_measurement = measurement_method(unit='um')
            mesurement = measurement_method(unit='um')
            self.get_logger().info(f"Measurement is {mesurement} um.")

            if not measurement_valid_function():
                current_x_joint_result = self.pm_robot_utils.get_current_joint_state(PmRobotUtils.X_Axis_JOINT_NAME)
                current_y_joint_result = self.pm_robot_utils.get_current_joint_state(PmRobotUtils.Y_Axis_JOINT_NAME)
                current_z_joint_result = self.pm_robot_utils.get_current_joint_state(PmRobotUtils.Z_Axis_JOINT_NAME)
                return (current_x_joint_result, current_y_joint_result, current_z_joint_result)

        return None, None, None
    
    def load_last_calibrations_data(self):
        path = get_package_share_directory('pm_robot_calibration')
        file_name = 'last_calibrations.yaml'

        if os.path.exists(path + '/' + file_name):
            with open(path + '/' + file_name, 'r') as file:
                self.last_calibrations_data = yaml.load(file, Loader=yaml.FullLoader)
        else:
            pass

    def save_last_calibrations_data(self):
        path = get_package_share_directory('pm_robot_calibration')
        file_name = 'last_calibrations.yaml'

        with open(path + '/' + file_name, 'w') as file:
            yaml.dump(self.last_calibrations_data, file)
            pass
    
def main(args=None):
    rclpy.init(args=args)

    node = PmRobotCalibrationNode()

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
