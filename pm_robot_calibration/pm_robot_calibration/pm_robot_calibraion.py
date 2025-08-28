from urllib import response
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
from assembly_manager_interfaces.srv import SpawnFramesFromDescription, ModifyPoseFromFrame
import assembly_manager_interfaces.msg as amimsg
from pm_msgs.srv import EmptyWithSuccess, ForceSensorGetMeasurement
import numpy as np
from circle_fit import circle_fit
import matplotlib.pyplot as plt

# import get_package_share_directory
from ament_index_python.packages import get_package_share_directory

from ros_sequential_action_programmer.submodules.pm_robot_modules.widget_pm_robot_config import VacuumGripperConfig, ParallelGripperConfig
from pm_skills.py_modules.PmRobotUtils import PmRobotUtils, PmRobotError
import time
import os
import datetime
import copy
import math
import json

from assembly_scene_publisher.py_modules.scene_functions import (get_rel_transform_for_frames, 
                                                                 is_frame_from_scene, 
                                                                 get_ref_frame_by_name)

from assembly_scene_publisher.py_modules.geometry_type_functions import (get_relative_transform_for_transforms, get_relative_transform_for_transforms_calibration)


TOOL_VACUUM_IDENT = 'pm_robot_vacuum_tools'
TOOL_GRIPPER_1_JAW_IDENT = 'pm_robot_tool_parallel_gripper_1_jaw'
TOOL_GRIPPER_2_JAW_IDENT = 'pm_robot_tool_parallel_gripper_2_jaws'
DISPENSER_TRAVEL_DISTANCE = 0.04

LASER_CALIBRATION_TARGET_THICKNESS =  3.87 # mm

CAMERA_CALIBRATION_JOINT_VALUES_X = -0.266460 # in m
CAMERA_CALIBRATION_JOINT_VALUES_Y = -0.045949 # in m
CAMERA_CALIBRATION_JOINT_VALUES_Z = 0.002535 # in m
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
        self.client_correct_frame_vision = self.create_client(CorrectFrame, '/pm_skills/vision_correct_frame')
        self.client_modify_pose_from_frame = self.create_client(ami_srv.ModifyPoseFromFrame, '/assembly_manager/modify_frame_from_frame')
        self.client_calibrate_camera_pixel = self.create_client(CalibratePixelPerUm, '/pm_vision_manager/SetCameraPixelPerUm')
        self.client_calibrate_camera_angle = self.create_client(CalibrateAngle, '/pm_vision_manager/SetCameraAngle')
        self.client_correct_frame_confocal_bottom = self.create_client(CorrectFrame, '/pm_skills/correct_frame_with_confocal_bottom')
        #self.client_correct_frame_confocal_bottom = self.create_client(CorrectFrame, '/pm_skills/measure_frame_with_confocal_bottom')

        self.client_move_calibration_target_forward = self.create_client(EmptyWithSuccess, '/pm_pneumatic_controller/Camera_Calibration_Platelet_Joint/MoveForward')
        self.client_move_calibration_target_backward = self.create_client(EmptyWithSuccess, '/pm_pneumatic_controller/Camera_Calibration_Platelet_Joint/MoveBackward')
    
        self.client_create_ref_frame = self.create_client(ami_srv.CreateRefFrame, '/assembly_manager/create_ref_frame')

        # services
        self.calibrate_cameras_srv = self.create_service(EmptyWithSuccess, '/pm_robot_calibration/calibrate_cameras', self.calibrate_cameras_callback, callback_group=self.callback_group)
        self.calibrate_calibration_cube_xy_on_cam_top_srv = self.create_service(EmptyWithSuccess, '/pm_robot_calibration/calibrate_calibration_cube_xy_on_camera_top', self.calibrate_calibration_cube_to_cam_top_callback, callback_group=self.callback_group)
        self.calibrate_calibration_cube_z_laser_srv = self.create_service(EmptyWithSuccess, '/pm_robot_calibration/calibrate_calibration_cube_z_on_laser', self.calibrate_calibration_cube_z_on_laser, callback_group=self.callback_group)

        #self.calibrate_laser_on_calibration_cube_srv = self.create_service(EmptyWithSuccess, '/pm_robot_calibration/calibrate_laser_on_calibration_cube', self.calibrate_laser_on_calibration_cube_callback, callback_group=self.callback_group)
        #self.calbirate_confocal_top_srv = self.create_service(EmptyWithSuccess, '/pm_robot_calibration/calibrate_confocal_top', self.calibrate_confocal_top_callback, callback_group=self.callback_group)
        self.calibrate_laser_head_on_camera_srv = self.create_service(EmptyWithSuccess, '/pm_robot_calibration/calibrate_laser_xy_on_camera_bottom', self.calibrate_laser_xy_on_camera_bottom, callback_group=self.callback_group)
        self.calibrate_confocal_head_on_camera_srv = self.create_service(EmptyWithSuccess, '/pm_robot_calibration/calibrate_confocal_top_xy_on_camera_bottom', self.calibrate_confocal_top_xy_on_camera_bottom, callback_group=self.callback_group)
        self.calibrate_confocal_bottom_z_srv = self.create_service(EmptyWithSuccess, '/pm_robot_calibration/calibrate_confocal_bottom_z_on_laser', self.calibrate_confocal_bottom_z_on_laser, callback_group=self.callback_group)
        self.calibrate_confocal_top_z_srv = self.create_service(EmptyWithSuccess, '/pm_robot_calibration/calibrate_confocal_top_z_on_laser', self.calibrate_confocal_top_z_on_laser, callback_group=self.callback_group)
        self.calibrate_confocal_bottom_xy_srv = self.create_service(EmptyWithSuccess, '/pm_robot_calibration/calibrate_confocal_bottom_xy_on_cam_top', self.calibrate_confocal_bottom_xy_on_cam_top, callback_group=self.callback_group)

        #self.calibrate_siemens_gripper_on_cal_cube = self.create_service(EmptyWithSuccess, '/pm_robot_calibration/calibrate_siemens_gripper_on_cal_cube', self.calibrate_sim_gripper_on_cube, callback_group=self.callback_group)
        self.calbibrate_gonio_left_chuck_srv = self.create_service(EmptyWithSuccess, '/pm_robot_calibration/calibrate_gonio_left_chuck', self.calibrate_gonio_left_chuck_callback, callback_group=self.callback_group)
        self.calbibrate_gonio_right_chuck_srv = self.create_service(EmptyWithSuccess, '/pm_robot_calibration/calibrate_gonio_right_chuck', self.calibrate_gonio_right_chuck_callback, callback_group=self.callback_group)
        self.calibrate_gripper_srv = self.create_service(EmptyWithSuccess, '/pm_robot_calibration/calibrate_gripper', self.calibrate_gripper_callback, callback_group=self.callback_group)
        self.calibrate_gripper_plane_srv = self.create_service(EmptyWithSuccess, '/pm_robot_calibration/calibrate_gripper_plane', self.calibrate_gripper_plane, callback_group=self.callback_group)
        
        self.calibrate_1K_dispenser_xy_on_cam_bottom_srv = self.create_service(EmptyWithSuccess, '/pm_robot_calibration/calibrate_1K_dispenser_xy_on_cam_bottom', self.calibrate_1K_dispenser_xy_on_cam_bottom, callback_group=self.callback_group)

        # paths
        self.bringup_share_path = get_package_share_directory('pm_robot_bringup')
        self.calibration_frame_dict_path = get_package_share_directory('pm_robot_description') + '/urdf/urdf_configs/calibration_frame_dictionaries'
        self.pm_robot_config_path = self.bringup_share_path + '/config/pm_robot_bringup_config.yaml'
        self.calibration_log_dir = get_package_share_directory('pm_robot_calibration') + '/calibration_logs/'

        #self.update_pm_robot_config()
        self.get_logger().info(self.INFO_TEXT)
        self._last_calibrations_data = {}
        
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
            return False, None
        
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

        move_success = self.pm_robot_utils.send_xyz_trajectory_goal_absolut(    x_joint= self.CAMERA_CALIBRATION_JOINT_VALUES_X,
                                                                                y_joint= self.CAMERA_CALIBRATION_JOINT_VALUES_Y,
                                                                                z_joint= self.CAMERA_CALIBRATION_JOINT_VALUES_Z,
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

        time.sleep(1.0)

        trans = self.pm_robot_utils.get_transform_for_frame(frame_name='Cam1_Toolhead_TCP',
                                                            parent_frame='Camera_Station_TCP')
        
        rel_trans.translation.x += trans.translation.x
        rel_trans.translation.y += trans.translation.y

        self._logger.error(f"Result trans x: {rel_trans.translation.x}")
        self._logger.error(f"Result trans y: {rel_trans.translation.y}")


        save_success = self.save_joint_config ( joint_name='Cam1_Toolhead_TCP_Joint',
                                                rel_transformation=rel_trans,
                                                overwrite=True)

        self.log_calibration(file_name='Cam1_Toolhead_TCP_Joint', calibration_dict=self._transform_to_dict(rel_trans))


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

        result_dict = {}
        result_dict["bottom_camera"] = {
            "pixel": request_pixel.multiplicator,
            "angle": request_angle.angle_diff}

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

        result_dict["top_camera"] = {
            "pixel": request_pixel.multiplicator,
            "angle": request_angle.angle_diff
        }

        self.log_calibration(file_name='Camera_Calibration_Angle', calibration_dict=result_dict)

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
    
    def calibrate_1K_dispenser_xy_on_cam_bottom(self, request:EmptyWithSuccess.Request, response:EmptyWithSuccess.Response):
        
        #frame = '1K_Dispenser_TCP'
        try:
            move_success = self.pm_robot_utils.move_1k_dispenser_to_frame(frame_name=self.pm_robot_utils.TCP_CAMERA_BOTTOM,
                                                        z_offset=DISPENSER_TRAVEL_DISTANCE + 0.01)

            if not move_success:
                raise PmRobotError("Failed to move 1K dispenser to camera bottom")

            self.pm_robot_utils.open_protection()

            time.sleep(1.0)

            self.pm_robot_utils.extend_dispenser()

            time.sleep(1.0)

            move_success = self.pm_robot_utils.move_1k_dispenser_to_frame(frame_name=self.pm_robot_utils.TCP_CAMERA_BOTTOM)

            if not move_success:
                raise PmRobotError("Failed to move 1K dispenser to camera bottom")

            frame_name = self.spawn_1k_dispenser_cal_frame()

            success_correct_frame = self.correct_frame_vison(frame_name)

            if not success_correct_frame:
                seconds = 100
                self.get_logger().error(f"Failed to correct 1K dispenser vision frame. You have {seconds} seconds to fix this issue.")
                time.sleep(seconds)
                raise PmRobotError("Failed to correct 1K dispenser vision frame")
            
            # relative_transform:Transform = get_rel_transform_for_frames(scene=self.pm_robot_utils.object_scene,
            #                                 from_frame=self.pm_robot_utils.TCP_1K_DISPENSER,
            #                                 to_frame=frame_name,
            #                                 tf_buffer=self.pm_robot_utils.tf_buffer,
            #                                 logger=self._logger)
            
            relative_transform:Transform = self.pm_robot_utils.get_transform_for_frame(frame_name=frame_name,
                                                                                        parent_frame=self.pm_robot_utils.TCP_1K_DISPENSER)
            

            self.save_joint_config('1K_Dispenser_TCP_Joint', relative_transform)

            cal_dict = self._transform_to_dict(relative_transform)

            self.log_calibration(file_name='calibrate_1K_dispenser_xy_on_cam_bottom', calibration_dict=cal_dict)
                
            response.success = True
        
        except PmRobotError as e:
            self.get_logger().error(f"Error occurred while calibrating 1K dispenser: {e.message}")
            response.success = False

        finally:
            #Always try to reset the dispenser
            try:
                self.pm_robot_utils.retract_dispenser()
                time.sleep(1.0)
                self.pm_robot_utils.close_protection()
                time.sleep(1.0)
            except PmRobotError as e2:
                self.get_logger().error(f"Error occurred while resetting 1K dispenser: {e2.message}")
                response.success = False
            pass

        return response

    def spawn_1k_dispenser_cal_frame(self)->str:

        spawn_request = ami_srv.CreateRefFrame.Request()

        # get current dispenser tip
        dispenser_tip = self.pm_robot_utils.pm_robot_config.dispenser_1k.get_current_dispenser_tip()

        frame_name = f"CAL_{dispenser_tip}_Vision_Frame"

        spawn_request.ref_frame.frame_name = frame_name
        spawn_request.ref_frame.parent_frame = self.pm_robot_utils.TCP_1K_DISPENSER


        if not self.client_create_ref_frame.wait_for_service(timeout_sec=1.0):
            raise PmRobotError(f"Client '{self.client_create_ref_frame.srv_name}' not available")

        response:ami_srv.CreateRefFrame.Response = self.client_create_ref_frame.call(spawn_request)

        if not response.success:
            raise PmRobotError(f"Failed to create reference frame: {spawn_request.ref_frame.frame_name}")

        return spawn_request.ref_frame.frame_name

    ###############################################
    ### Calibrate gripper #########################
    ###############################################
    
    def calibrate_gripper_callback(self, request:EmptyWithSuccess.Request, response:EmptyWithSuccess.Response):
        
        
        spawn_success, frames, unique_identifier = self.spawn_frames_for_current_gripper()
        
        if not spawn_success:
            self.get_logger().error("Failed to spawn gripper frames...")
            response.success = False
            return response
        
        move_success = self.pm_robot_utils.send_t_trajectory_goal_absolut(0.0, 2.0)

        if not move_success:
            self.get_logger().error("Failed to move gripper to rotation 0.0")
            response.success = False
            return response

        # maybe make this a service input?? so that the user can dynamically decide on the number of rotations?

        #rotations = [0.0, 20, 30, 40, 50, 60, 80]
        #rotations = [90, 65, 55, 45, 35, 25, 0]
        rotations = [90, 60, 30, 0]

        #rotations = [60, 40, 20, 0]

        # move gripper close to camera to calibration start position
        move_to_start_success = self.pm_robot_utils.move_camera_top_to_frame(frame_name=self.pm_robot_utils.TCP_CAMERA_BOTTOM,
                                                                             #endeffector_override=self.pm_robot_utils.TCP_TOOL,
                                                                             z_offset=0.01)
        
        if not move_to_start_success:
            self.get_logger().error("Failed to move to start position...")
            response.success = False
            return response

        # convert to rad
        rotations = [r * np.pi / 180.0 for r in rotations]
        
        if len(rotations) == 0:
            self.get_logger().error("No angles for the gripper calibration specified")
            response.success = False
            return response
        
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
                    correct_frame_success = self.correct_frame_vison(frame)
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
        max_distance = max(distance_list)
        # fit a circle to the points to find the center
                
        #2.3656241026821336e-07
        self._logger.info("Min distance: " + str(min_distance * 1e6) + " um")
        self._logger.info(f" length of frame_poses_list: {len(frame_poses_list)}")
        
        # if the rotational axis is not ideal all the frames in the 'frame_poses_list' form a circle. We saved all the distances between the points in the circle
        # if we are already calibrated, the points should be very close together and not form a circle.
        # we check the distances between the points, if the max_distance exeedes a threshold we calibrate the axis offset

        #if distance > 20 * 1e-6:
        # If the rotation axis need to be corrected
        if max_distance > 20 * 1e-6:
            # plot a circle through all the poses
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

            t_axis_dict = self._transform_to_dict(rel_t_joint)

            self.log_calibration(file_name='calibrate_gripper_T_axis', calibration_dict=t_axis_dict)

            self.plot_gripper_calibration_poses(copy.copy(frame_poses_list),
                                                radius=r,
                                                circle_x=x,
                                                circle_y=y)
            
        # assuming the gripper is at the center of the circle
        else:
            self._logger.warn("T-Axis has no offset...")

        # log relative pose
        #self._logger.error("Relative pose: " + str(relative_transform))
        self._logger.error("Translation of the gripper tip")
        self._logger.error(f"x offset: {relative_transform.translation.x * 1e6} um")
        self._logger.error(f"y offset: {relative_transform.translation.y * 1e6} um")
        
        self.save_joint_config('PM_Robot_Tool_TCP_Joint', relative_transform)
        self.log_calibration(file_name='PM_Robot_Tool_TCP_Joint', calibration_dict=self._transform_to_dict(relative_transform))

        # move out of danger zone
        self.pm_robot_utils.send_xyz_trajectory_goal_relative(0.0, 0.0, -0.05, 1.0)

        return response

    def _transform_to_dict(self, transform:Transform)->dict:

        translation_dict = {
            "x": round(transform.translation.x*1e6, 1),
            "y": round(transform.translation.y*1e6, 1),
            "z": round(transform.translation.z*1e6, 1)
        }
        rx, ry, rz = R.from_quat([transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w]).as_euler('xyz', degrees=True)

        final_dict =     {
            "translation": translation_dict,
            "rotation": {
                "rx": round(rx, 5),
                "ry": round(ry, 5),
                "rz": round(rz, 5),
            }
        }
        return final_dict

    def calibrate_gripper_plane(self, request:EmptyWithSuccess.Request, response:EmptyWithSuccess.Response):
        
        spawn_success, frames, unique_identifier = self.spawn_frames_for_current_gripper()
        
        result_dict = {}
        if not spawn_success:
            self.get_logger().error("Failed to spawn gripper frames...")
            response.success = False
            return response
        
        move_success = self.pm_robot_utils.send_t_trajectory_goal_absolut(0.0, 0.5)

        if not move_success:
            self.get_logger().error("Failed to move gripper to rotation 0.0")
            response.success = False
            return response

        result_list = []
        # we now measure the rotational deviations of the gripper
        for frame in frames:                      
            if 'Laser' in frame or 'laser' in frame:
                correct_frame_success = self.correct_frame_confocal_bottom(frame)
                
                if not correct_frame_success:
                    self.get_logger().error("Failed to correct frame: " + frame)
                    response.success = False
                    return response

                ref_frame: amimsg.RefFrame = get_ref_frame_by_name(self.pm_robot_utils.object_scene, frame)

                x_value = ref_frame.pose.position.x
                y_value = ref_frame.pose.position.y
                z_value = ref_frame.pose.position.z

                z_joint = self.pm_robot_utils.get_current_joint_state(self.pm_robot_utils.Z_Axis_JOINT_NAME)
                self._logger.error(f"z joint: {z_joint*1e3} mm")
                result_list.append({#"x_value": int(x_value*1e6), 
                                    #"y_value": int(y_value*1e6), 
                                    "frame_name": frame,
                                    "z_value": round(z_value*1e6,1), 
                                    "z_joint": round(z_joint*1e6,1)})

        default_transformation = Transform()

        relative_transform_angle:Transform =    get_rel_transform_for_frames(scene=self.pm_robot_utils.object_scene,
                                                from_frame=f'{unique_identifier}CALIBRATION_PM_Robot_Tool_TCP',
                                                to_frame=f'{unique_identifier}CALIBRATION_PM_Robot_Tool_TCP_initial',
                                                tf_buffer=self.pm_robot_utils.tf_buffer,
                                                logger=self._logger)
        
        roll, pitch, yaw = R.from_quat([relative_transform_angle.rotation.x, 
                                        relative_transform_angle.rotation.y,
                                        relative_transform_angle.rotation.z,
                                        relative_transform_angle.rotation.w,]).as_euler('xyz', degrees=True)
        
        self._logger.warn(f"Results - Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}")

        result_dict['results'] = result_list
        result_dict["rx"] = round(roll, 5)
        result_dict["ry"] = round(pitch, 5)
        result_dict["rz"] = round(yaw, 5)

        default_transformation.rotation = relative_transform_angle.rotation
        default_transformation.translation.z = relative_transform_angle.translation.z
        
        self.save_joint_config('PM_Robot_Tool_TCP_Joint', 
                               default_transformation, 
                               overwrite=False)

        # move out of danger zone
        self.pm_robot_utils.send_xyz_trajectory_goal_relative(0.0, 0.0, -0.04, 0.5)

        self.log_calibration(file_name='calibrate_gripper_plane',
                             calibration_dict=result_dict)
        
        response.success = True
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
        path = self.calibration_log_dir+ 'gripper_plots/'

        # check path
        if not os.path.exists(path):
            os.makedirs(path)

        file_dir = path + f'gripper_calibration_poses_{datetime.datetime.now()}.png'

        #save the figure
        fig.savefig(file_dir)   
        self.get_logger().info(f"Calibration image has been saved to '{file_dir}'.")


    ###########################################
    ### calibration_cube_to_cam_top ###########
    ###########################################

    
    def calibrate_calibration_cube_to_cam_top_callback(self, request:EmptyWithSuccess.Request, response:EmptyWithSuccess.Response):
        
        
        # Spawn the frames
        spawn_success = self.spawn_calibration_frames('CF_Calibration_Qube_Cam_Top.json')
        
        if not spawn_success:
            self.get_logger().error(f"Spawning of frames failed!")

            response.success = False
            return response
        unique_identifier = self.get_unique_identifier('CF_Calibration_Qube_Cam_Top.json')

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
        
        return response
    
    ###########################################
    ### laser_on_calibration_cube ############
    ###########################################
    
    # def calibrate_laser_on_calibration_cube_callback(self, request:EmptyWithSuccess.Request, response:EmptyWithSuccess.Response):
    #     # To be implemented...
    #     self.get_logger().warn("Laser on calibration cube not fully implemented yet...")
        
    #     if not is_frame_from_scene(self.pm_robot_utils.object_scene, 
    #                                'CAL_Calibration_Qube_Cam_Top_Vision_Dynamic'):
    #         response.success = False
    #         self.get_logger().error("Missing calibration frame. Exeucte 'calibrate_calibration_cube_to_cam_top' first...")
    #         return response
        
    #     

    #     spawn_success = self.spawn_calibration_frames('CF_Laser_to_Calibration_Qube.json')
        
    #     unique_identifier = self.get_unique_identifier('CF_Laser_to_Calibration_Qube.json')

    #     if not spawn_success:
    #         self.get_logger().error("Failed to spawn calibration frames...")
    #         response.success = False
    #         return response
        
    #     # Move laser to calibration cube
    #     move_request = MoveToFrame.Request()
    #     move_request.target_frame = 'CAL_Calibration_Qube_Cam_Top_Vision_Dynamic'
    #     move_request.execute_movement = True
    #     move_request.translation.x = 1*1e-3
    #     move_request.translation.y = 1*1e-3
    #     move_request.translation.z = 0.0

    #     if not self.pm_robot_utils.client_move_robot_laser_to_frame.wait_for_service(timeout_sec=1.0):
    #         self.get_logger().error('Laser move service not available...')
    #         response.success = False
    #         return response
        
    #     # move to intial position
    #     response_move:MoveToFrame.Response = self.pm_robot_utils.client_move_robot_laser_to_frame.call(move_request)
        
    #     if not response_move.success:
    #         self.get_logger().error("Failed to move laser to calibration cube")
    #         response.success = False
    #         return response
                
    #     # set the laser to the zero height
    #     initial_z_height = self.pm_robot_utils.get_laser_measurement(unit='m')
        
    #     self.get_logger().error(f'Heigt {initial_z_height}')

    #     self.pm_robot_utils.send_xyz_trajectory_goal_relative(0.0, 0.0, -initial_z_height, 0.5)
        
    #     step_inc = 0.1
    #     self.get_logger().error("STARTING Y DIRECTION ROUGTH")

    #     # sense the y direction
    #     x, y_joint_result, z = self.pm_robot_utils.interative_sensing(measurement_method=self.pm_robot_utils.get_laser_measurement,
    #                                              measurement_valid_function = self.pm_robot_utils._check_for_valid_laser_measurement,
    #                                              length = (0.0, -3.0, 0.0),
    #                                              step_inc = step_inc,
    #                                              total_time = 8.0)

        
    #     # move back to initial state
    #     response_move:MoveToFrame.Response = self.pm_robot_utils.client_move_robot_laser_to_frame.call(move_request)

    #     if not response_move.success:
    #         self.get_logger().error("Failed to move laser to calibration cube")
    #         response.success = False
    #         return response     

    #     self.get_logger().error("STARTING X DIRECTION ROUGTH")

    #     x_joint_result, y, z = self.pm_robot_utils.interative_sensing(measurement_method=self.pm_robot_utils.get_laser_measurement,
    #                                                     measurement_valid_function = self.pm_robot_utils._check_for_valid_laser_measurement,
    #                                                     length = (-3.0, 0.0, 0.0),
    #                                                     step_inc = step_inc,
    #                                                     total_time = 8.0)
        
    #     if x_joint_result is None or y_joint_result is None:
    #         self.get_logger().error("Failed to get laser measurement...")
    #         response.success = False
    #         return response
        
    #     self.get_logger().error("Current X joint: " + str(x_joint_result))
    #     self.get_logger().error("Current Y joint: " + str(y_joint_result))

    
    #     #move to result position      
    #     self.pm_robot_utils.send_xyz_trajectory_goal_absolut(x_joint_result,
    #                                                         y_joint_result,
    #                                                         z,
    #                                                         time=1.0)
        
    #     time.sleep(1.0)
        
    #     # second iteration
    #     self.pm_robot_utils.send_xyz_trajectory_goal_relative(x_joint_rel=0.0002, 
    #                                                           y_joint_rel=0.0002, 
    #                                                           z_joint_rel=0, 
    #                                                           time=0.5)
        
    #     time.sleep(1.0)
    #     cal_height = self.pm_robot_utils.get_laser_measurement(unit='m')

    #     self.pm_robot_utils.send_xyz_trajectory_goal_relative(0.0, 0.0, -cal_height, 0.5)

    #     time.sleep(1.0)
        
    #     step_inc = 0.01
    #     self.get_logger().error("STARTING Y DIRECTION FINE")

    #     x, y_joint_result_2, z = self.pm_robot_utils.interative_sensing(measurement_method=self.pm_robot_utils.get_laser_measurement,
    #                                                 measurement_valid_function = self.pm_robot_utils._check_for_valid_laser_measurement,
    #                                              length = (0.0, -1.0, 0.0),
    #                                              step_inc = step_inc,
    #                                              total_time = 8.0)
        
    #     self.pm_robot_utils.send_xyz_trajectory_goal_relative(  x_joint_rel=0, 
    #                                                             y_joint_rel=0.0002, 
    #                                                             z_joint_rel=0, 
    #                                                             time=0.5)
        
    #     self.get_logger().error("STARTING X DIRECTION FINE")

    #     time.sleep(1)

    #     x_joint_result_2, y, z = self.pm_robot_utils.interative_sensing(measurement_method=self.pm_robot_utils.get_laser_measurement,
    #                                                     measurement_valid_function = self.pm_robot_utils._check_for_valid_laser_measurement,
    #                                                     length = (-1.0, 0.0, 0.0),
    #                                                     step_inc = step_inc,
    #                                                     total_time = 8.0)

        
    #     if x_joint_result_2 is None or y_joint_result_2 is None:
    #         self.get_logger().error("Failed to get laser measurement...")
    #         response.success = False
    #         return response
        
    #     self.get_logger().error("Current X joint: " + str(x_joint_result_2))
    #     self.get_logger().error("Current Y joint: " + str(y_joint_result_2))

    #     # move to result position
    #     self.pm_robot_utils.send_xyz_trajectory_goal_absolut(x_joint_result_2,
    #                                                         y_joint_result_2,
    #                                                         z_joint=z,
    #                                                         time=1.0)
        
    #     time.sleep(1.0)
        
    #     relative_transform = get_rel_transform_for_frames(scene=self.pm_robot_utils.object_scene,
    #                                             from_frame='CAL_Calibration_Qube_Cam_Top_Vision_Dynamic',
    #                                             to_frame=f'CAL_Laser_Toolhead_TCP_to_Calibration_Qube',
    #                                             tf_buffer=self.pm_robot_utils.tf_buffer,
    #                                             logger=self._logger)
        
    #     self._logger.error("Relative pose: " + str(relative_transform))

    #     if relative_transform is None:
    #         self.get_logger().error("Failed to get relative pose...")
    #         response.success = False
    #         return response
        
    #     success = self.save_joint_config('Laser_Toolhead_TCP_Joint', 
    #                                      relative_transform,
    #                                      overwrite=False)
        
    #     if not success:
    #         self.get_logger().error("Failed to save joint config...")
    #         response.success = False
    #         return response    
                
    #     response.success = True
    #     

    #     return response
    
    
    ###########################################
    ### Confocal_top ##########################
    ###########################################
        
    # def calibrate_confocal_top_callback(self, request:EmptyWithSuccess.Request, response:EmptyWithSuccess.Response):
    #     

    #     if not is_frame_from_scene(self.pm_robot_utils.object_scene, 
    #                                'CAL_Calibration_Qube_Cam_Top_Vision_Dynamic'):
            
    #         self.get_logger().error("Missing calibration frame. Exeucte 'calibrate_calibration_cube_to_cam_top' first...")
    #         response.success =False
    #         return response
        
    #     

    #     spawn_success = self.spawn_calibration_frames('CF_Confocal_to_Calibration_Qube.json')
        
    #     unique_identifier = self.get_unique_identifier('CF_Confocal_to_Calibration_Qube.json')

    #     if not spawn_success:
    #         self.get_logger().error("Failed to spawn calibration frames...")
    #         response.success = False
    #         return response
        
    #     # Move laser to calibration cube
    #     move_request = MoveToFrame.Request()
    #     move_request.target_frame = 'CAL_Calibration_Qube_Cam_Top_Vision_Dynamic'
    #     move_request.execute_movement = True
    #     move_request.translation.x = 0.5*1e-3
    #     move_request.translation.y = 0.5*1e-3

    #     if not self.pm_robot_utils.client_move_robot_confocal_top_to_frame.wait_for_service(timeout_sec=1.0):
    #         self.get_logger().error('Laser move service not available...')
    #         response.success = False
    #         return response
        
    #     # move to intial position
    #     response_move:MoveToFrame.Response = self.pm_robot_utils.client_move_robot_confocal_top_to_frame.call(move_request)
        
    #     if not response_move.success:
    #         self.get_logger().error("Failed to move laser to calibration cube")
    #         response.success = False
    #         return response
                
    #     # set the laser to the zero height
    #     initial_z_height = self.pm_robot_utils.get_confocal_top_measurement(unit='m')
        
    #     self.get_logger().error(f'Heigt {initial_z_height}')

    #     self.pm_robot_utils.send_xyz_trajectory_goal_relative(0.0, 0.0, -initial_z_height, 0.5)

    #     time.sleep(2)

    #     step_inc = 0.1 # mm
    #     self.get_logger().error("STARTING Y DIRECTION ROUGTH")

    #     # sense the y direction
    #     x, y_joint_result, z = self.interative_sensing(measurement_method=self.pm_robot_utils.get_confocal_top_measurement,
    #                                                     measurement_valid_function = self.pm_robot_utils.check_confocal_top_measurement_in_range,
    #                                              length = (0.0, -3.0, 0.0),
    #                                              step_inc = step_inc,
    #                                              total_time = 8.0)

        
    #     # move back to initial state
    #     response_move:MoveToFrame.Response = self.pm_robot_utils.client_move_robot_confocal_top_to_frame.call(move_request)

    #     if not response_move.success:
    #         self.get_logger().error("Failed to move laser to calibration cube")
    #         response.success = False
    #         return response     

    #     self.get_logger().error("STARTING X DIRECTION ROUGTH")

    #     x_joint_result, y, z = self.interative_sensing(measurement_method=self.pm_robot_utils.get_confocal_top_measurement,
    #                                                     measurement_valid_function = self.pm_robot_utils.check_confocal_top_measurement_in_range,
    #                                                     length = (-3.0, 0.0, 0.0),
    #                                                     step_inc = step_inc,
    #                                                     total_time = 8.0)
        
    #     if x_joint_result is None or y_joint_result is None:
    #         self.get_logger().error("Failed to get laser measurement...")
    #         response.success = False
    #         return response
        
    #     self.get_logger().error("Current X joint: " + str(x_joint_result))
    #     self.get_logger().error("Current Y joint: " + str(y_joint_result))

    
    #     #move to result position      
    #     self.pm_robot_utils.send_xyz_trajectory_goal_absolut(x_joint_result,
    #                                                         y_joint_result,
    #                                                         z,
    #                                                         time=1.0)
        
    #     time.sleep(1.0)

        
        
    #     # second iteration
    #     self.pm_robot_utils.send_xyz_trajectory_goal_relative(x_joint_rel=0.0002, 
    #                                                           y_joint_rel=0.0002, 
    #                                                           z_joint_rel=0, 
    #                                                           time=0.5)
        
    #     time.sleep(1.0)

    #     cal_height = self.pm_robot_utils.get_confocal_top_measurement(unit='m')

    #     self.pm_robot_utils.send_xyz_trajectory_goal_relative(0.0, 0.0, -cal_height, 0.5)
        
    #     time.sleep(1.0)

    #     step_inc = 0.01 # mm
    #     self.get_logger().error("STARTING Y DIRECTION FINE")

    #     x, y_joint_result_2, z = self.interative_sensing(measurement_method=self.pm_robot_utils.get_confocal_top_measurement,
    #                                                     measurement_valid_function = self.pm_robot_utils.check_confocal_top_measurement_in_range,
    #                                                     length = (0.0, -1.0, 0.0),
    #                                                     step_inc = step_inc,
    #                                                     total_time = 8.0)
        
    #     time.sleep(1.0)
        
    #     self.pm_robot_utils.send_xyz_trajectory_goal_relative(  x_joint_rel=0, 
    #                                                             y_joint_rel=0.0002, 
    #                                                             z_joint_rel=0, 
    #                                                             time=0.5)
        
    #     time.sleep(1.0)
        
    #     self.get_logger().error("STARTING X DIRECTION FINE")

    #     x_joint_result_2, y, z = self.interative_sensing(measurement_method=self.pm_robot_utils.get_confocal_top_measurement,
    #                                                     measurement_valid_function = self.pm_robot_utils.check_confocal_top_measurement_in_range,
    #                                                     length = (-1.0, 0.0, 0.0),
    #                                                     step_inc = step_inc,
    #                                                     total_time = 8.0)
        
    #     time.sleep(1.0)

    #     if x_joint_result_2 is None or y_joint_result_2 is None:
    #         self.get_logger().error("Failed to get laser measurement...")
    #         response.success = False
    #         return response
        
    #     self.get_logger().error("Current X joint: " + str(x_joint_result_2))
    #     self.get_logger().error("Current Y joint: " + str(y_joint_result_2))
        
    #     # move to result position
    #     self.pm_robot_utils.send_xyz_trajectory_goal_absolut(   x_joint_result_2,
    #                                                             y_joint_result_2,
    #                                                             z_joint=z,
    #                                                             time=1.0)
            
    #     time.sleep(1.0)
        
    #     relative_transform = get_rel_transform_for_frames(scene=self.pm_robot_utils.object_scene,
    #                                             from_frame='CAL_Calibration_Qube_Cam_Top_Vision_Dynamic',
    #                                             to_frame=f'CAL_Confocal_TCP_to_Calibration_Qube',
    #                                             tf_buffer=self.pm_robot_utils.tf_buffer,
    #                                             logger=self._logger)
        
    #     relative_transform_2 = get_rel_transform_for_frames(scene=self.pm_robot_utils.object_scene,
    #                                     from_frame='CAL_Calibration_Qube_Cam_Top_Vision_Dynamic',
    #                                     to_frame=f'CAL_Confocal_TCP_2_to_Calibration_Qube',
    #                                     tf_buffer=self.pm_robot_utils.tf_buffer,
    #                                     logger=self._logger)
        
    #     if relative_transform is None or relative_transform_2 is None:
    #         self.get_logger().error("Failed to get relative pose...")
    #         response.success = False
    #         return response
        
    #     if abs(relative_transform.translation.x) > abs(relative_transform_2.translation.x):
    #         transfrom = relative_transform_2
    #         self._logger.info(f"Confocal TCP is at position 2.")


    #     else:
    #         transfrom = relative_transform
    #         self._logger.info(f"Confocal TCP is at position 1.")


    #     # This affects both TCPs at it uses the same joint calibration. Is is no issue as only one tcp is uesed at a time
    #     success = self.save_joint_config('Confocal_Sensor_Top_TCP_Joint', 
    #                             transfrom,
    #                             overwrite=False)

    #     if not success:
    #         self.get_logger().error("Failed to save joint config...")
    #         response.success = False
    #         return response    
        
    #     response.success = True

    #     self.get_logger().error(f"Results x: {transfrom.translation.x*1e6} um, y: {transfrom.translation.y*1e6} um, z: {transfrom.translation.z*1e6} um")

    #     time.sleep(4.0)

    #     # Move confocal away
    #     move_request = MoveToFrame.Request()
    #     move_request.target_frame = 'CAL_Calibration_Qube_Cam_Top_Vision_Dynamic'
    #     move_request.execute_movement = True
    #     move_request.translation.z = 0.03

    #     if not self.pm_robot_utils.client_move_robot_confocal_top_to_frame.wait_for_service(timeout_sec=1.0):
    #         self.get_logger().error('Laser move service not available...')
    #         response.success = False
    #         return response
        
    #     # move to intial position
    #     response_move:MoveToFrame.Response = self.pm_robot_utils.client_move_robot_confocal_top_to_frame.call(move_request)
        
    #     if not response_move.success:
    #         self.get_logger().error("Failed to move laser to calibration cube")
    #         response.success = False
    #         return response
        
    #     return response
    
    ############################################
    ### Calibration Siemens Gripper to cube ####
    ############################################

    # def calibrate_sim_gripper_on_cube(self, request:EmptyWithSuccess.Request, response:EmptyWithSuccess.Response):
    #     # We currently need this calbiration function. However we would like to get rid of this calibration as it is only possible to do with the siemens gripper

    #     self.pm_robot_utils.pm_robot_config.tool.reload_config()
    #     current_tool = self.pm_robot_utils.pm_robot_config.tool.get_tool().get_current_tool()

    #     if current_tool != 'Siemens_Vacuum_Array_short':
    #         response.success = False
    #         self.get_logger().error(f'Current tool is not "Siemens_Vacuum_Array_short"! Executing this funciton is not possible!')
    #         return response

    #     set_success = self.pm_robot_utils.set_force_sensor_bias()
        
    #     if not set_success:
    #         response.success = False
    #         return response
        
    #     # Move gripper away
    #     move_request = MoveToFrame.Request()
    #     move_request.target_frame = 'Calibration_Qube'
    #     move_request.execute_movement = True
    #     move_request.translation.z = 0.001 # 1 mm above the cube

    #     if not self.pm_robot_utils.client_move_robot_tool_to_frame.wait_for_service(timeout_sec=1.0):
    #         self.get_logger().error(f'Client {self.pm_robot_utils.client_move_robot_tool_to_frame.srv_name} not available!')
    #         response.success = False
    #         return response
        
    #     # move to intial position
    #     response_move:MoveToFrame.Response = self.pm_robot_utils.client_move_robot_tool_to_frame.call(move_request)
        
    #     if not response_move.success:
    #         self.get_logger().error("Failed to move laser to calibration cube")
    #         response.success = False
    #         return response
        
    #     # check client avaliable
    #     if not self.pm_robot_utils.client_get_force_measurement.wait_for_service(1):
    #         self.get_logger().error(f'Client {self.pm_robot_utils.client_get_force_measurement.srv_name} not available!')
    #         response.success = False
    #         return response
    #     # get force measurement
    #     request_force = ForceSensorGetMeasurement.Request()

    #     # Rougth sensing
    #     step_size = 0.0001 # m
    #     distance = 0.0013 # m
    #     max_steps = distance //step_size 

    #     for iterator in range(int(max_steps)):
            
    #         response_measurement: ForceSensorGetMeasurement.Response = self.pm_robot_utils.client_get_force_measurement.call(request_force)
    #         z_force = response_measurement.data[2]
    #         self.get_logger().error(f"Z Force is: {z_force}")

    #         if abs(z_force) > 0.1:
    #             self.get_logger().error(f"Force is exeeded!")
    #             break
            
    #         else:
    #             self.pm_robot_utils.send_xyz_trajectory_goal_relative(  x_joint_rel=0, 
    #                                                             y_joint_rel=0.0, 
    #                                                             z_joint_rel=step_size, 
    #                                                             time=0.5)
    #             time.sleep(1.0)

    #         self.get_logger().error(f"Running iteration: {iterator}")

    #     if (iterator >= max_steps):
    #         self.get_logger().error(f"Sensing failed. Max steps reached!")
    #         response.success = False
    #         return response
        
    #     time.sleep(1.0)

    #     self.pm_robot_utils.send_xyz_trajectory_goal_relative(  x_joint_rel=0, 
    #                                                             y_joint_rel=0.0, 
    #                                                             z_joint_rel=-(0.00015), 
    #                                                             time=0.5)
        
    #     time.sleep(1.0)
    #     self.get_logger().error(f"Starting fine search!")
    #     # Fine Sensing
    #     step_size = 0.00001 # m
    #     distance = 0.00025 # m
    #     max_steps = distance //step_size 

    #     for iterator in range(int(max_steps)):
            
    #         response_measurement: ForceSensorGetMeasurement.Response = self.pm_robot_utils.client_get_force_measurement.call(request_force)
    #         z_force = response_measurement.data[2]
    #         self.get_logger().error(f"Z Force is: {z_force}")

    #         if abs(z_force) > 0.1:
    #             self.get_logger().error(f"Force is exeeded!")
    #             break
            
    #         else:
    #             self.pm_robot_utils.send_xyz_trajectory_goal_relative(  x_joint_rel=0, 
    #                                                             y_joint_rel=0.0, 
    #                                                             z_joint_rel=step_size, 
    #                                                             time=0.5)
    #             time.sleep(1.0)

    #         self.get_logger().error(f"Running iteration: {iterator}")

    #     if (iterator >= max_steps):
    #         self.get_logger().error(f"Sensing failed. Max steps reached!")
    #         response.success = False
    #         return response
        
    #     time.sleep(1.0)
    #     transform = self.pm_robot_utils.get_transform_for_frame('PM_Robot_Tool_TCP', 'Calibration_Qube')
    #     self.get_logger().error(f"Transform: {str(transform)}")

    #     time.sleep(5.0)

    #     # Move gripper away
    #     move_request = MoveToFrame.Request()
    #     move_request.target_frame = 'Calibration_Qube'
    #     move_request.execute_movement = True
    #     move_request.translation.z = 0.03 # 30 mm above the cube

    #     if not self.pm_robot_utils.client_move_robot_tool_to_frame.wait_for_service(timeout_sec=1.0):
    #         self.get_logger().error(f'Client {self.pm_robot_utils.client_move_robot_tool_to_frame.srv_name} not available!')
    #         response.success = False
    #         return response
        
    #     # move to intial position
    #     response_move:MoveToFrame.Response = self.pm_robot_utils.client_move_robot_tool_to_frame.call(move_request)

    #     response.success = True

    #     return response
    
    ###########################################
    ### Gonio left chuck ######################
    ###########################################
    
    def calibrate_gonio_left_chuck_callback(self, request:EmptyWithSuccess.Request, response:EmptyWithSuccess.Response):
        
        # To be implemented...
        self.get_logger().error("Gonio left chuck not implemented yet...")   

        return response
    
    ###########################################
    ### Gonio right chuck #####################
    ###########################################
    
    def calibrate_gonio_right_chuck_callback(self, request:EmptyWithSuccess.Request, response:EmptyWithSuccess.Response):

        # To be implemented...
        self.get_logger().error("Gonio right chuck not implemented yet...")
        
        return response
    

    def calibrate_laser_xy_on_camera_bottom(self, request:EmptyWithSuccess.Request, response:EmptyWithSuccess.Response):
        """
        The functionality of the laser calibration has been verified by manually moving the laser to the camera tcp. By experts!!!
        """

        self._logger.warn(f"Starting calibration 'calibrate_laser_xy_on_camera_bottom'...")

        CAMERA_TARGET_HEIGHT = 1.6 #    mm - this is not needed as the fiducials are on top of the platelet

        forward_request = EmptyWithSuccess.Request()
        backward_request = EmptyWithSuccess.Request()
        forward_response:EmptyWithSuccess.Response = self.client_move_calibration_target_forward.call(forward_request)

        if not forward_response.success:
            self._logger.error("Failed to move calibration target forward")
            response.success = False
            return response

        move_success = self.pm_robot_utils.move_laser_to_frame('Calibration_Platelet_Calibration_Frame',
                                                               z_offset=0.05)

        if not move_success:
            self._logger.error("Failed to move laser to frame")
            response.success = False
            backward_response:EmptyWithSuccess.Response = self.client_move_calibration_target_backward.call(backward_request)
            return response
        
        move_success = self.pm_robot_utils.move_laser_to_frame('Calibration_Platelet_Calibration_Frame',
                                                               z_offset=0.00)

        if not move_success:
            self._logger.error("Failed to move laser to frame")
            response.success = False
            backward_response:EmptyWithSuccess.Response = self.client_move_calibration_target_backward.call(backward_request)
            return response
        

        if not self.pm_robot_utils._check_for_valid_laser_measurement():
            self._logger.error("Could not get a valid laser measurement!")
            response.success = False
            return response
        
        initial_z_height = self.pm_robot_utils.get_laser_measurement(unit='m')
        
        self.get_logger().error(f'Heigt {initial_z_height}')

        self.pm_robot_utils.send_xyz_trajectory_goal_relative(0.0, 0.0, -initial_z_height, 1.0)

        time.sleep(1)

        # move to a position where the laser is visible
        z_joint_target = self.pm_robot_utils.get_current_joint_state(self.pm_robot_utils.Z_Axis_JOINT_NAME)
        x_joint_target = -0.4447
        y_joint_target = -0.0317

        move_success = self.pm_robot_utils.send_xyz_trajectory_goal_absolut(   
                                                                x_joint= x_joint_target,
                                                                y_joint = y_joint_target,
                                                                z_joint = z_joint_target,
                                                                time=0.5)

        if not move_success:
            response.success = False
            return response
        
        request_execute_vision_bottom = ExecuteVision.Request()
        request_execute_vision_bottom.camera_config_filename = self.pm_robot_utils.get_cam_file_name_bottom()
        request_execute_vision_bottom.image_display_time = -1
        request_execute_vision_bottom.process_filename = "PM_Robot_Calibration/Calibration_Laser_xy_on_Camera_Bottom.json"
        request_execute_vision_bottom.process_uid = f"Laser_xy_on_Cam_Bottom"

        if not self.pm_robot_utils.client_execute_vision.wait_for_service(timeout_sec=1.0):
            self._logger.error('Vision Manager not available...')
            response.success = False
            return response
        
        response_execute_vision_bottom:ExecuteVision.Response = self.pm_robot_utils.client_execute_vision.call(request_execute_vision_bottom)
        
        if not response_execute_vision_bottom.success:
            self._logger.error("Failed to execute vision for bottom camera")
            response.success = False
            return response
        
        if len(response_execute_vision_bottom.vision_response.results.circles) != 1:
            self._logger.error("Vision did not find a single circle!")
            response.success = False
            return response
        
        circle:vision_msg.VisionCircle = response_execute_vision_bottom.vision_response.results.circles[0]

        x_offset = circle.center_point.axis_value_1
        y_offset = circle.center_point.axis_value_2

        rel_trans = Transform()

        time.sleep(2.0)

        transfrom_camera_TCP = self.pm_robot_utils.get_transform_for_frame(frame_name=self.pm_robot_utils.TCP_LASER,
                                                            parent_frame=self.pm_robot_utils.TCP_CAMERA_BOTTOM)


        #self._logger.error(f"Transform x: {transfrom.translation.x}")
        #self._logger.error(f"Transform y: {transfrom.translation.y}")

        rel_trans.translation.x = transfrom_camera_TCP.translation.x - x_offset * 1e-6
        rel_trans.translation.y = transfrom_camera_TCP.translation.y - y_offset * 1e-6

        self._logger.warn(f"Correction value x: {rel_trans.translation.x*1e6} um")
        self._logger.warn(f"Correction value y: {rel_trans.translation.y*1e6} um")

        rel_trans.translation.x = -1*rel_trans.translation.x
        rel_trans.translation.y = -1*rel_trans.translation.y

        rel_trans_camera_b_tcp = Transform()

        # this seems to be the right solution, but it does not work. it seems that the camera focus point is closer to the top surface of the camera calibration platelet
        #rel_trans_camera_b_tcp.translation.z = transfrom_camera_TCP.translation.z - CAMERA_TARGET_HEIGHT*1e-3
        empirical_value = 0.0005       # verified by experts
        rel_trans_camera_b_tcp.translation.z = transfrom_camera_TCP.translation.z + empirical_value

        
        save_success = self.save_joint_config ( joint_name='Laser_Toolhead_TCP_Joint',
                                                rel_transformation=rel_trans,
                                                overwrite=False)
        
        save_success = self.save_joint_config ( joint_name='Camera_Station_TCP_Joint',
                                                rel_transformation=rel_trans_camera_b_tcp,
                                                overwrite=False)
        
        # log results
        self.log_calibration(file_name="calibrate_laser_xy_on_camera_bottom",
                             calibration_dict=self._transform_to_dict(rel_trans))
        
        self.log_calibration(file_name="calibrate_camera_bottom_tcp_z_on_laser_z",
                             calibration_dict=self._transform_to_dict(rel_trans_camera_b_tcp))

        if not save_success:
            self._logger.error("Saving of configuration failed!")
            response.success = False
            return response         

        response.success = True

        # move out of danger zone
        self.pm_robot_utils.send_xyz_trajectory_goal_relative(0.0, 0.0, -0.02, 1.0)

        backward_response:EmptyWithSuccess.Response = self.client_move_calibration_target_backward.call(backward_request)
        
        return response

    def calibrate_confocal_top_xy_on_camera_bottom(self, request:EmptyWithSuccess.Request, response:EmptyWithSuccess.Response):

        self._logger.warn(f"Starting calbiration 'calibrate_confocal_top_xy_on_camera_bottom'...")

        

        forward_request = EmptyWithSuccess.Request()
        backward_request = EmptyWithSuccess.Request()
        forward_response:EmptyWithSuccess.Response = self.client_move_calibration_target_forward.call(forward_request)

        if not forward_response.success:
            self._logger.error("Failed to move calibration target forward")
            response.success = False
            return response
        
        request_move_to_frame = MoveToFrame.Request()
        request_move_to_frame.target_frame = 'Calibration_Platelet_Calibration_Frame'
        request_move_to_frame.execute_movement = True
        request_move_to_frame.translation.z = 0.00
        #request_move_to_frame.translation.z = 0.001

        #request_move_to_frame.translation.x = 0.0

        if not self.pm_robot_utils.client_move_robot_confocal_top_to_frame.wait_for_service(timeout_sec=1.0):
            self._logger.error('Camera move service not available...')
            response.success = False
            backward_response:EmptyWithSuccess.Response = self.client_move_calibration_target_backward.call(backward_request)
            return response
        
        response_move_to_frame:MoveToFrame.Response = self.pm_robot_utils.client_move_robot_confocal_top_to_frame.call(request_move_to_frame)
        
        if not response_move_to_frame.success:
            self._logger.error("Failed to move laser to frame")
            response.success = False
            backward_response:EmptyWithSuccess.Response = self.client_move_calibration_target_backward.call(backward_request)
            return response

        if not self.pm_robot_utils.check_confocal_top_measurement_in_range():
            
            # move 3 mm up
            move_success = self.pm_robot_utils.send_xyz_trajectory_goal_relative(0, 0, -3.0*1e-3,time=1)
                                            
            if not move_success:
                    response.success = False
                    return response
            
            step_inc = 1.0 # in mm
            self._logger.warn(f"Laser measurement not valid! Trying to iteratively find a valid value!")                

            x, y, final_z = self.pm_robot_utils.interative_sensing(measurement_method=self.pm_robot_utils.get_confocal_top_measurement,
                                            measurement_valid_function = self.pm_robot_utils.check_confocal_top_measurement_in_range,
                                            length = (0.0, 0.0, 4.0),
                                            step_inc = step_inc,
                                            total_time = 2.0)
            
            if x is None:
                response.success = False
                self._logger.warn(f"Laser measurement not valid! OUT OF RANGE")
                return response
        
        initial_z_height = self.pm_robot_utils.get_confocal_top_measurement(unit='m')
        
        self.get_logger().error(f"Measured confocal heigt '{initial_z_height*1e-6}' um")

        self.pm_robot_utils.send_xyz_trajectory_goal_relative(0.0, 0.0, -initial_z_height, 1.0)

        time.sleep(1)

        initial_z_height = self.pm_robot_utils.get_confocal_top_measurement(unit='m')
        
        self.get_logger().error(f"Measured confocal heigt corrected'{initial_z_height*1e-6}' um")

        z_joint_target = self.pm_robot_utils.get_current_joint_state(self.pm_robot_utils.Z_Axis_JOINT_NAME)
        x_joint_target = -0.44687985
        y_joint_target = 0.00261

        move_success = self.pm_robot_utils.send_xyz_trajectory_goal_absolut(   
                                                                x_joint= x_joint_target,
                                                                y_joint = y_joint_target,
                                                                z_joint = z_joint_target,
                                                                time=0.1)

        if not move_success:
            response.success = False
            return response
        
        request_execute_vision_bottom = ExecuteVision.Request()
        request_execute_vision_bottom.camera_config_filename = self.pm_robot_utils.get_cam_file_name_bottom()
        request_execute_vision_bottom.image_display_time = -1
        request_execute_vision_bottom.process_filename = "PM_Robot_Calibration/Calibration_Confocal_Top_xy_on_Camera_Bottom.json"
        request_execute_vision_bottom.process_uid = f"Confocal_xy_on_Cam_Bottom"

        if not self.pm_robot_utils.client_execute_vision.wait_for_service(timeout_sec=1.0):
            self._logger.error('Vision Manager not available...')
            response.success = False
            return response
        
        response_execute_vision_bottom:ExecuteVision.Response = self.pm_robot_utils.client_execute_vision.call(request_execute_vision_bottom)
        
        if not response_execute_vision_bottom.success:
            self._logger.error("Failed to execute vision for bottom camera")
            response.success = False
            return response
        
        if len(response_execute_vision_bottom.vision_response.results.circles) != 1:
            self._logger.error("Vision did not find a single circle!")
            response.success = False
            return response
        
        circle:vision_msg.VisionCircle = response_execute_vision_bottom.vision_response.results.circles[0]

        x_offset = circle.center_point.axis_value_1
        y_offset = circle.center_point.axis_value_2

        time.sleep(2.0)

        rel_trans = Transform()

        transfrom = self.pm_robot_utils.get_transform_for_frame(frame_name=self.pm_robot_utils.TCP_CONFOCAL_TOP_2,
                                                            parent_frame=self.pm_robot_utils.TCP_CAMERA_BOTTOM)
        
        #self._logger.error(f"Transform x: {transfrom.translation.x}")
        #self._logger.error(f"Transform y: {transfrom.translation.y}")

        rel_trans.translation.x = transfrom.translation.x - x_offset * 1e-6
        rel_trans.translation.y = transfrom.translation.y - y_offset * 1e-6

        self._logger.warn(f"Correction value x: {rel_trans.translation.x*1e6} um")
        self._logger.warn(f"Correction value y: {rel_trans.translation.y*1e6} um")

        rel_trans.translation.x = -1*rel_trans.translation.x
        rel_trans.translation.y = -1*rel_trans.translation.y
        
        save_success = self.save_joint_config ( joint_name='Confocal_Sensor_Top_TCP_Joint',
                                                rel_transformation=rel_trans,
                                                overwrite=False)

        if not save_success:
            self._logger.error("Saving of configuration failed!")
            response.success = False
            return response         

        response.success = True

        # move out of danger zone
        self.pm_robot_utils.send_xyz_trajectory_goal_relative(0.0, 0.0, -0.02, 1.0)

        backward_response:EmptyWithSuccess.Response = self.client_move_calibration_target_backward.call(backward_request)
            
        return response    
    
    def calibrate_confocal_bottom_xy_on_cam_top(self, request:EmptyWithSuccess.Request, response:EmptyWithSuccess.Response):
        

        self._logger.warn(f"Starting calbiration 'calibrate_confocal_bottom_xy_on_cam_top'...")
        
        forward_request = EmptyWithSuccess.Request()
        backward_request = EmptyWithSuccess.Request()
        forward_response:EmptyWithSuccess.Response = self.client_move_calibration_target_forward.call(forward_request)

        if not forward_response.success:
            self._logger.error("Failed to move calibration target forward")
            response.success = False
            return response
        
        request_move_to_frame = MoveToFrame.Request()
        request_move_to_frame.target_frame = 'Laser_Height_Calibration_Frame'
        request_move_to_frame.execute_movement = True
        request_move_to_frame.translation.z = -0.00387


        if not self.pm_robot_utils.client_move_robot_cam1_to_frame.wait_for_service(timeout_sec=1.0):
            self._logger.error('Camera move service not available...')
            response.success = False
            backward_response:EmptyWithSuccess.Response = self.client_move_calibration_target_backward.call(backward_request)
            return response
        
        response_move_to_frame:MoveToFrame.Response = self.pm_robot_utils.client_move_robot_cam1_to_frame.call(request_move_to_frame)

        if not response_move_to_frame.success:
            self._logger.error("Failed to move laser to frame")
            response.success = False
            backward_response:EmptyWithSuccess.Response = self.client_move_calibration_target_backward.call(backward_request)
            return response
        
        time.sleep(1)
        z_joint_target = 0.0010185
        x_joint_target = -0.2091909
        y_joint_target = -0.05865595

        
        move_success = self.pm_robot_utils.send_xyz_trajectory_goal_absolut(   
                                                                x_joint= x_joint_target,
                                                                y_joint = y_joint_target,
                                                                z_joint = z_joint_target,
                                                                time=0.1)


        if not move_success:
            response.success = False
            return response
        
        x_offset, y_offset = self._get_circle_from_vision(process_file_name="Calibration_Confocal_Bottom_xy_on_Camera_Top.json",
                                                          camera_file_name=self.pm_robot_utils.get_cam_file_name_top(),
                                                          process_name="Confocal_xy_on_Cam_Top")
        
        if x_offset is None:
            response.success = False
            return response
        
        #self._logger.error(f"Camera measurement {x_offset} um, {y_offset} um")

        time.sleep(2.0)

        rel_trans = Transform()

        transfrom = self.pm_robot_utils.get_transform_for_frame(frame_name=self.pm_robot_utils.TCP_CONFOCAL_BOTTOM,
                                                             parent_frame=self.pm_robot_utils.TCP_CAMERA_TOP)
        
        #self._logger.error(f"Transform x: {transfrom.translation.x}")
        #self._logger.error(f"Transform y: {transfrom.translation.y}")

        rel_trans.translation.x = transfrom.translation.x - x_offset * 1e-6
        rel_trans.translation.y = transfrom.translation.y - y_offset * 1e-6

        self._logger.warn(f"Result x: {rel_trans.translation.x*1e6} um")
        self._logger.warn(f"Result y: {rel_trans.translation.y*1e6} um")

        rel_trans.translation.x = -1*rel_trans.translation.x
        rel_trans.translation.y = -1*rel_trans.translation.y
        
        save_success = self.save_joint_config ( joint_name='Confocal_Sensor_Bottom_TCP_Joint',
                                                rel_transformation=rel_trans,
                                                overwrite=False)

        if not save_success:
            self._logger.error("Saving of configuration failed!")
            response.success = False
            return response         

        # move out of danger zone
        #self.pm_robot_utils.send_xyz_trajectory_goal_relative(0.0, 0.0, -0.05, 1.0)
        #backward_response:EmptyWithSuccess.Response = self.client_move_calibration_target_backward.call(backward_request)

        response.success = True
        
        return response  
    
    def calibrate_confocal_bottom_z_on_laser(self, request:EmptyWithSuccess.Request, response:EmptyWithSuccess.Response):
        
        
        self._logger.warn(f"Starting calbiration 'calibrate_confocal_bottom_z_on_laser'...")

        forward_request = EmptyWithSuccess.Request()
        backward_request = EmptyWithSuccess.Request()
        forward_response:EmptyWithSuccess.Response = self.client_move_calibration_target_forward.call(forward_request)

        if not forward_response.success:
            self._logger.error("Failed to move calibration target forward")
            response.success = False
            return response
                
        move_success = self.pm_robot_utils.move_laser_to_frame(frame_name='TCP_Confocal_Sensor_Bottom',
                                                               z_offset=0.004,
                                                               y_offset=-0.003)

        if not move_success:
            self._logger.error("Failed to move laser to frame")
            response.success = False
            backward_response:EmptyWithSuccess.Response = self.client_move_calibration_target_backward.call(backward_request)
            return response

        if not self.pm_robot_utils.check_confocal_bottom_measurement_in_range():
            self._logger.error("Could not get a valid confocal bottom measurement!")
            response.success = False
            return response
        
        if not self.pm_robot_utils._check_for_valid_laser_measurement():

            move_success = self.pm_robot_utils.send_xyz_trajectory_goal_relative(0, 0, -3.0*1e-3,time=0.5)
                                            
            if not move_success:
                    response.success = False
                    return response
            
            step_inc = 0.4 # in mm
            self._logger.warn(f"Laser measurement not valid! Trying to iteratively find a valid value!")                

            x, y, final_z = self.pm_robot_utils.interative_sensing(measurement_method=self.pm_robot_utils.get_laser_measurement,
                                            measurement_valid_function = self.pm_robot_utils._check_for_valid_laser_measurement,
                                            length = (0.0, 0.0, 5.0),
                                            step_inc = step_inc,
                                            total_time = 8.0)
            
            if x is None:
                response.success = False
                self._logger.warn(f"Laser measurement not valid! OUT OF RANGE")
                return response
                
        initial_z_height = self.pm_robot_utils.get_laser_measurement(unit='m')
        
        self.get_logger().warn(f"Measured laser heigt '{initial_z_height*1e6}' um")

        self.pm_robot_utils.send_xyz_trajectory_goal_relative(0.0, 0.0, -initial_z_height, 1.0)

        confocal_bottom_measurement = self.pm_robot_utils.get_confocal_bottom_measurement(unit='m')

        self.get_logger().warn(f"Measured confocal bottom heigt '{confocal_bottom_measurement*1e6}' um")

        time.sleep(1.0)

        # beginning the calculations

        rel_trans = Transform()
        
        transfrom = self.pm_robot_utils.get_transform_for_frame(frame_name=self.pm_robot_utils.TCP_LASER,
                                                             parent_frame=self.pm_robot_utils.TCP_CONFOCAL_BOTTOM)
        
        self._logger.warn(f"Transform from Confocal bottom to laser - z: {transfrom.translation.z*1e6} um.")

        rel_trans.translation.z = transfrom.translation.z

        distance_between_laser_confocal_bottom = LASER_CALIBRATION_TARGET_THICKNESS*1e-3 - confocal_bottom_measurement

        #rel_trans.translation.z = -1*(distance_between_laser_confocal_bottom - rel_trans.translation.z )

        rel_trans.translation.z = rel_trans.translation.z - distance_between_laser_confocal_bottom

        self._logger.warn(f"Calibration result z: {rel_trans.translation.z*1e6} um.")

        save_success = self.save_joint_config ( joint_name ='Confocal_Sensor_Bottom_TCP_Joint',
                                                rel_transformation=rel_trans,
                                                overwrite=False)

        if not save_success:
            self._logger.error("Saving of configuration failed!")
            response.success = False
            return response    
        
        # move out of danger zone
        self.pm_robot_utils.send_xyz_trajectory_goal_relative(0.0, 0.0, -0.05, 1.0)

        backward_response:EmptyWithSuccess.Response = self.client_move_calibration_target_backward.call(backward_request)
        calibration_dict = {}
        calibration_dict["transform"] = self._transform_to_dict(rel_trans)
        calibration_dict["confocal_bottom_measurement"] = round(confocal_bottom_measurement*1e6,1)

        self.log_calibration(file_name='calibrate_confocal_bottom_z_on_laser',
                             calibration_dict=calibration_dict)

        response.success = True
        
        return response  
        

    def calibrate_confocal_top_z_on_laser(self, request:EmptyWithSuccess.Request, response:EmptyWithSuccess.Response):
        self._logger.warn(f"Starting calbiration 'calibrate_confocal_top_z_on_laser'...")

        forward_request = EmptyWithSuccess.Request()
        backward_request = EmptyWithSuccess.Request()
        forward_response:EmptyWithSuccess.Response = self.client_move_calibration_target_forward.call(forward_request)

        if not forward_response.success:
            self._logger.error("Failed to move calibration target forward")
            response.success = False
            return response
        
        # First approach the position
        request_move_to_frame = MoveToFrame.Request()
        #request_move_to_frame.target_frame = 'Laser_Height_Calibration_Frame'
        request_move_to_frame.target_frame = 'Calibration_Platelet_Calibration_Frame'
        request_move_to_frame.execute_movement = True
        request_move_to_frame.translation.z = 0.03
        #request_move_to_frame.translation.z = 0.001

        #request_move_to_frame.translation.x = 0.0

        if not self.pm_robot_utils.client_move_robot_laser_to_frame.wait_for_service(timeout_sec=1.0):
            self._logger.error('Camera move service not available...')
            response.success = False
            backward_response:EmptyWithSuccess.Response = self.client_move_calibration_target_backward.call(backward_request)
            return response
        
        response_move_to_frame:MoveToFrame.Response = self.pm_robot_utils.client_move_robot_laser_to_frame.call(request_move_to_frame)
        
        if not response_move_to_frame.success:
            self._logger.error("Failed to move laser to frame")
            response.success = False
            backward_response:EmptyWithSuccess.Response = self.client_move_calibration_target_backward.call(backward_request)
            return response
        
        request_move_to_frame.translation.z = 0.0 

        response_move_to_frame:MoveToFrame.Response = self.pm_robot_utils.client_move_robot_laser_to_frame.call(request_move_to_frame)
        
        if not response_move_to_frame.success:
            self._logger.error("Failed to move laser to frame")
            response.success = False
            backward_response:EmptyWithSuccess.Response = self.client_move_calibration_target_backward.call(backward_request)
            return response

        # we want to make sure that we get the top measurement of the platelet
        # move 2 mm up
        move_success = self.pm_robot_utils.send_xyz_trajectory_goal_relative(0, 0, -2.0*1e-3,time=0.5)
        
        if not move_success:
                response.success = False
                return response
        
        step_inc = 0.4 # in mm
        self._logger.warn(f"Laser measurement not valid! Trying to iteratively find a valid value!")                

        x, y, final_z = self.pm_robot_utils.interative_sensing(measurement_method=self.pm_robot_utils.get_laser_measurement,
                                        measurement_valid_function = self.pm_robot_utils._check_for_valid_laser_measurement,
                                        length = (0.0, 0.0, 3.0),
                                        step_inc = step_inc,
                                        total_time = 4.0)
        
        if x is None:
            response.success = False
            self._logger.warn(f"Laser measurement not valid! OUT OF RANGE")
            return response
                
        initial_z_height = self.pm_robot_utils.get_laser_measurement(unit='m')
        
        self.get_logger().error(f"Measured laser heigt measurement'{initial_z_height*1e6}' um")

        self.pm_robot_utils.send_xyz_trajectory_goal_relative(0.0, 0.0, -initial_z_height, 0.5)

        time.sleep(2.0)

        laser_transfrom = self.pm_robot_utils.get_transform_for_frame(frame_name = self.pm_robot_utils.TCP_LASER,
                                                             parent_frame = request_move_to_frame.target_frame)
        
        laser_transform_z = laser_transfrom.translation.z

        self.get_logger().error(f"Laser offset value' {laser_transform_z*1e6}' um")

        if not self.pm_robot_utils.client_move_robot_confocal_top_to_frame.wait_for_service(timeout_sec=1.0):
            self._logger.error('Camera move service not available...')
            response.success = False
            backward_response:EmptyWithSuccess.Response = self.client_move_calibration_target_backward.call(backward_request)
            return response
        
        response_move_to_frame:MoveToFrame.Response = self.pm_robot_utils.client_move_robot_confocal_top_to_frame.call(request_move_to_frame)
        
        if not response_move_to_frame.success:
            self._logger.error("Failed to move laser to frame")
            response.success = False
            backward_response:EmptyWithSuccess.Response = self.client_move_calibration_target_backward.call(backward_request)
            return response
        
        
        if not self.pm_robot_utils.check_confocal_top_measurement_in_range():
            
            # move 3 mm up
            move_success = self.pm_robot_utils.send_xyz_trajectory_goal_relative(0, 0, -3.0*1e-3,time=0.3)
                                            
            if not move_success:
                    response.success = False
                    return response
            
            step_inc = 1.0 # in mm
            self._logger.warn(f"Laser measurement not valid! Trying to iteratively find a valid value!")                

            x, y, final_z = self.pm_robot_utils.interative_sensing(measurement_method=self.pm_robot_utils.get_confocal_top_measurement,
                                            measurement_valid_function = self.pm_robot_utils.check_confocal_top_measurement_in_range,
                                            length = (0.0, 0.0, 4.0),
                                            step_inc = step_inc,
                                            total_time = 2.0)
            
            if x is None:
                response.success = False
                self._logger.warn(f"Laser measurement not valid! OUT OF RANGE")
                return response

        confocal_top_measurement = self.pm_robot_utils.get_confocal_top_measurement('m')

        self.get_logger().warn(f"Measured confocal heigt measurement '{confocal_top_measurement*1e6}' um")

        self.pm_robot_utils.send_xyz_trajectory_goal_relative(0.0, 0.0, -confocal_top_measurement, 0.5)

        time.sleep(2.0)

        confocal_transfrom = self.pm_robot_utils.get_transform_for_frame(frame_name=self.pm_robot_utils.TCP_CONFOCAL_TOP_2,
                                                             parent_frame=request_move_to_frame.target_frame)
        
        confocal_transfrom_z = confocal_transfrom.translation.z

        self.get_logger().warn(f"Confocal offset value' {confocal_transfrom_z*1e6}' um")

        rel_trans = Transform()

        transform_z = -1*(confocal_transfrom_z - laser_transform_z)

        rel_trans.translation.z = transform_z
    
        self._logger.warn(f"Calibration value z: {rel_trans.translation.z*1e6} um.")

        save_success = self.save_joint_config ( joint_name='Confocal_Sensor_Top_TCP_Joint',
                                                rel_transformation=rel_trans,
                                                overwrite=False)
        
        if not save_success:
            self._logger.error("Saving of configuration failed!")
            response.success = False
            return response         

        # move out of danger zone
        self.pm_robot_utils.send_xyz_trajectory_goal_relative(0.0, 0.0, -0.05, 1.0)

        backward_response:EmptyWithSuccess.Response = self.client_move_calibration_target_backward.call(backward_request)

        response.success = True
        
        return response  

    def calibrate_calibration_cube_z_on_laser(self, request:EmptyWithSuccess.Request, response:EmptyWithSuccess.Response):
        
        self._logger.warn(f"Starting calbiration 'calibrate_calibration_cube_z_on_laser'...")


        spawn_success = self.spawn_calibration_frames('CF_Calibration_Cube_on_Laser.json')
        
        if not spawn_success:
            self.get_logger().error(f"Spawning of frames failed!")

            response.success = False
            return response
        
        unique_identifier = self.get_unique_identifier('CF_Calibration_Cube_on_Laser.json')

        correct_frame_request_1 = CorrectFrame.Request()
        correct_frame_request_2 = CorrectFrame.Request()
        correct_frame_request_3 = CorrectFrame.Request()
        correct_frame_request_4 = CorrectFrame.Request()

        correct_frame_request_1.remeasure_after_correction = True
        correct_frame_request_1.use_iterative_sensing = True

        correct_frame_request_2.remeasure_after_correction = True
        correct_frame_request_2.use_iterative_sensing = True

        correct_frame_request_3.remeasure_after_correction = True
        correct_frame_request_3.use_iterative_sensing = True

        correct_frame_request_4.remeasure_after_correction = True
        correct_frame_request_4.use_iterative_sensing = True

        correct_frame_request_1.frame_name = f"{unique_identifier}Laser_1"
        correct_frame_request_2.frame_name = f"{unique_identifier}Laser_2"
        correct_frame_request_3.frame_name = f"{unique_identifier}Laser_3"
        correct_frame_request_4.frame_name = f"{unique_identifier}Laser_4"

        request_list = [correct_frame_request_1,
                        correct_frame_request_2,
                        correct_frame_request_3,
                        correct_frame_request_4]

        for request in request:
            pass
                
        return response  
    
    def _get_circle_from_vision(self, process_file_name:str, camera_file_name:str, process_name:str):

        request_execute_vision_bottom = ExecuteVision.Request()
        request_execute_vision_bottom.camera_config_filename = camera_file_name
        request_execute_vision_bottom.image_display_time = -1
        request_execute_vision_bottom.process_filename = f"PM_Robot_Calibration/{process_file_name}"
        request_execute_vision_bottom.process_uid = process_name

        if not self.pm_robot_utils.client_execute_vision.wait_for_service(timeout_sec=1.0):
            self._logger.error('Vision Manager not available...')
            return None, None
        
        response_execute_vision_bottom:ExecuteVision.Response = self.pm_robot_utils.client_execute_vision.call(request_execute_vision_bottom)
        
        if not response_execute_vision_bottom.success:
            self._logger.error("Failed to execute vision for bottom camera")
            return None, None
        
        if len(response_execute_vision_bottom.vision_response.results.circles) != 1:
            self._logger.error("Vision did not find a single circle!")
            return None, None
        
        circle:vision_msg.VisionCircle = response_execute_vision_bottom.vision_response.results.circles[0]

        x_offset = circle.center_point.axis_value_1
        y_offset = circle.center_point.axis_value_2

        return x_offset, y_offset
    
    def measure_frame(self, frame_id:str)->tuple[bool, Vector3]:
        
        if not self.client_measure_frame_cam.wait_for_service(timeout_sec=1.0):
            self._logger.error('Vision measure frame service not available...')
            return False, None
        
        request = MeasureFrame.Request()
        request.frame_name = frame_id
        response:MeasureFrame.Response = self.client_measure_frame_cam.call(request)
        
        return response.success, response.result_vector
    
    def correct_frame_vison(self, frame_id:str)->bool:
        
        if not self.client_correct_frame_vision.wait_for_service(timeout_sec=1.0):
            self._logger.error('Vision correct frame service not available...')
            return False
        
        request = CorrectFrame.Request()
        request.frame_name = frame_id
        response:CorrectFrame.Response = self.client_correct_frame_vision.call(request)
        
        return response.success
    

    def correct_frame_confocal_bottom(self, frame_id:str)->bool:
        
        if not self.client_correct_frame_confocal_bottom.wait_for_service(timeout_sec=1.0):
            self._logger.error(f"Client '{self.client_correct_frame_confocal_bottom.srv_name}' not available...")
            return False
        
        request = CorrectFrame.Request()
        request.frame_name = frame_id
        response:CorrectFrame.Response = self.client_correct_frame_confocal_bottom.call(request)
        
        return response.success
    
    def modify_pose_from_frame(self, modify_pose_request:ami_srv.ModifyPoseFromFrame.Request)->bool:
        if not self.client_modify_pose_from_frame.wait_for_service(timeout_sec=1.0):
            self._logger.error('Modify pose from frame service not available...')
            return False
        
        request = ami_srv.ModifyPoseFromFrame.Request()
        
        response:ami_srv.ModifyPoseFromFrame.Response = self.client_modify_pose_from_frame.call(request)
        
        return response.success
    
    def _log_calibration_result(self, calibration_values:Transform, unit = 'm',):
        _calibration_values = copy.deepcopy(calibration_values)
        if unit == 'm':
            _calibration_values.translation.x = _calibration_values.translation.x * 1e6
            _calibration_values.translation.y = _calibration_values.translation.y * 1e6
            _calibration_values.translation.z = _calibration_values.translation.z * 1e6
        elif unit == 'mm':
            _calibration_values.translation.x = _calibration_values.translation.x * 1e3
            _calibration_values.translation.y = _calibration_values.translation.y * 1e3
            _calibration_values.translation.z = _calibration_values.translation.z * 1e3
        elif unit == 'um':
            pass
        else:
            raise ValueError(f"Invalid unit given {unit}!")
        
        threshold = 2 # um
        
        if abs(_calibration_values.translation.x) > threshold:
            self._logger.warn(f"Calibration value for 'x' changed significantly ({_calibration_values.translation.x} um). You need to restart the PM Robot!")

        if abs(_calibration_values.translation.y) > threshold:
            self._logger.warn(f"Calibration value for 'y' changed significantly ({_calibration_values.translation.y} um). You need to restart the PM Robot!")

        if abs(_calibration_values.translation.z) > threshold:
            self._logger.warn(f"Calibration value for 'z' changed significantly ({_calibration_values.translation.z} um). You need to restart the PM Robot!")

    def log_calibration(self, 
                        file_name: str,
                        calibration_dict: dict) -> bool:
        # Ensure the log directory exists
        if not os.path.exists(self.calibration_log_dir):
            os.makedirs(self.calibration_log_dir)

        self._calibration_history_log(file_name)

        # Ensure .json extension
        if not file_name.endswith(".json"):
            file_name += ".json"

        file_path = os.path.join(self.calibration_log_dir, file_name)

        # Add a timestamp to the calibration data
        calibration_dict["timestamp"] = datetime.datetime.now().isoformat()

        # Default structure
        calibration_history = {"calibration_history": []}

        if os.path.exists(file_path):
            try:
                with open(file_path, 'r') as file:
                    calibration_history = json.load(file)

                    # Fallback if structure is wrong
                    if "calibration_history" not in calibration_history:
                        calibration_history["calibration_history"] = []
            except json.JSONDecodeError as e:
                self._logger.warn(
                    f"JSON decode error in '{file_path}': {e}. Overwriting with new log structure."
                )
            except Exception as e:
                self._logger.error(
                    f"Error reading calibration file '{file_path}': {e}"
                )
                return False

        # Append the new calibration entry
        calibration_history["calibration_history"].append(calibration_dict)

        # Write the final JSON back to file
        try:
            with open(file_path, 'w') as file:
                json.dump(calibration_history, file, indent=2)
            self._logger.info(f"Calibration data logged to {file_path}")
            return True
        except Exception as e:
            self._logger.error(f"Failed to write JSON calibration log to '{file_path}': {e}")
            return False

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

        self._log_calibration_result(rel_transformation, unit=unit)

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
                new_transform = get_relative_transform_for_transforms_calibration(  base_transform = current_transformation, 
                                                                                    additional_transform = rel_transformation)
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

    def _calibration_history_log(self, history_entry:str):
        self._load_last_calibrations_data()
        self._last_calibrations_data[history_entry] = f'{datetime.datetime.now()}'
        self._save_last_calibrations_data()

    def _load_last_calibrations_data(self):
        path = self.calibration_log_dir
        file_name = 'last_calibrations.yaml'

        if os.path.exists(path + '/' + file_name):
            with open(path + '/' + file_name, 'r') as file:
                self._last_calibrations_data = yaml.load(file, Loader=yaml.FullLoader)
        else:
            pass

    def _save_last_calibrations_data(self):
        path = self.calibration_log_dir
        file_name = 'last_calibrations.yaml'

        with open(path + '/' + file_name, 'w') as file:
            yaml.dump(self._last_calibrations_data, file)
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
