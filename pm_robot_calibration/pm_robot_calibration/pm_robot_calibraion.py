from fileinput import filename
from urllib import response
from pathlib import Path
from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor
import yaml
import sys
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
import pm_skills_interfaces.srv as skills_srv
import pm_skills_interfaces.action as skills_action

from pm_robot_calibration.py_modules.hexapod_calibration_classes import MeasurementSet

from pm_moveit_interfaces.srv import MoveToPose,  MoveToFrame
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, StaticTransformBroadcaster
from pm_vision_interfaces.srv import ExecuteVision, CalibrateAngle, CalibratePixelPerUm
import pm_vision_interfaces.msg as vision_msg
from geometry_msgs.msg import Vector3, TransformStamped, Pose, PoseStamped, Quaternion, Transform
from rclpy.action import ActionServer, GoalResponse, CancelResponse

from scipy.spatial.transform import Rotation as R

import assembly_manager_interfaces.srv as ami_srv
import assembly_manager_interfaces.msg as ami_msg

from assembly_scene_publisher.py_modules.AssemblyScene import AssemblyManagerScene
from assembly_manager_interfaces.srv import SpawnFramesFromDescription, ModifyPoseFromFrame
from pm_msgs.srv import EmptyWithSuccess, ForceSensorGetMeasurement
import numpy as np
from circle_fit import circle_fit
import matplotlib.pyplot as plt

# import get_package_share_directory
from ament_index_python.packages import get_package_share_directory

from pm_skills.py_modules.PmRobotUtils import PmRobotUtils
from pm_robot_primitive_skills.py_modules.PmRobotError import PmRobotError
from pm_robot_primitive_skills.py_modules.PmRobotMeasurementError import PmRobotMeasurementError

from pm_robot_calibration.py_modules.hexapod_calibration.sphere_calibration import SphereCalibration
from pm_robot_calibration.py_modules.hexapod_calibration.calibration_analysis import CalibrationAnalysis
from assembly_scene_publisher.py_modules.geometry_functions import (multiply_ros_transforms, inverse_ros_transform)
import time
import os
import datetime
import copy
import math
import json
import shutil

from assembly_scene_publisher.py_modules.scene_functions import (get_rel_transform_for_frames)

from assembly_scene_publisher.py_modules.geometry_type_functions import (get_relative_transform_for_transforms, 
                                                                         get_relative_transform_for_transforms_calibration)


from assembly_scene_publisher.py_modules.scene_errors import (RefAxisNotFoundError, 
                                                              RefFrameNotFoundError, 
                                                              RefPlaneNotFoundError, 
                                                              ComponentNotFoundError)
TOOL_VACUUM_IDENT = 'pm_robot_vacuum_tools'
TOOL_GRIPPER_1_JAW_IDENT = 'pm_robot_tool_parallel_gripper_1_jaw'
TOOL_GRIPPER_2_JAW_IDENT = 'pm_robot_tool_parallel_gripper_2_jaws'
DISPENSER_TRAVEL_DISTANCE = 0.04

LASER_CALIBRATION_TARGET_THICKNESS =  3.87 # mm

CAMERA_CALIBRATION_JOINT_VALUES_X = -0.266460 # in m
CAMERA_CALIBRATION_JOINT_VALUES_Y = -0.045949 # in m
CAMERA_CALIBRATION_JOINT_VALUES_Z = 0.002535 # in m

class HexapodConstants:
    FIXED_CS_SMARPOD_FRAME = "Smarpod_Station_Base_Calibration"
    CALIBRATED_CS_SMARPOD_FRAME = "Smarpod_Station_Base"
    BALL_ENDEFFECTOR = "Smarpod_Part_Spawn"
    BALL_DIAMETER = 6.35 #mm
    CALIBRATION_FILE_JOINT_NAME = "Smarpod_Station"
    CALIBRATION_FILE_JOINT_NAME_SPHERE = "CAL_Smarpod_J__t__P"
    SMARPOD_CS_PIVOT_BASE_NAME = "Smarpod_Pivot_Base_Origin"
class CancelCalibrationException(Exception):
    pass
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
        
        self.gripper_frames_spawned = False
        # clients
        self.client_spawn_frames_from_description = self.create_client(SpawnFramesFromDescription, '/assembly_manager/spawn_frames_from_description')
        self.client_create_ref_frame = self.create_client(ami_srv.CreateRefFrame, '/assembly_manager/create_ref_frame')
        self.client_measure_frame_cam = self.create_client(skills_srv.MeasureFrameVision, '/pm_skills/vision_measure_frame')
        self.client_correct_frame_vision = self.create_client(skills_srv.CorrectFrameVision, '/pm_skills/vision_correct_frame')
        self.client_modify_pose_from_frame = self.create_client(ami_srv.ModifyPoseFromFrame, '/assembly_manager/modify_frame_from_frame')
        self.client_calibrate_camera_pixel = self.create_client(CalibratePixelPerUm, '/pm_vision_manager/SetCameraPixelPerUm')
        self.client_calibrate_camera_angle = self.create_client(CalibrateAngle, '/pm_vision_manager/SetCameraAngle')
        self.client_correct_frame_confocal_bottom = self.create_client(skills_srv.CorrectFrameLaser, '/pm_skills/correct_frame_with_confocal_bottom')
        self.client_correct_frame_with_laser = self.create_client(skills_srv.CorrectFrameLaser, '/pm_skills/correct_frame_with_laser')
        self.client_correct_frame_with_confocal_top = self.create_client(skills_srv.CorrectFrameLaser, '/pm_skills/correct_frame_with_confocal_top')

        # services
        self.calibrate_cameras_srv = self.create_service(EmptyWithSuccess, '/pm_robot_calibration/calibrate_cameras', self.calibrate_cameras_callback, callback_group=self.callback_group)
        self.calibrate_calibration_cube_xy_on_cam_top_srv = self.create_service(EmptyWithSuccess, '/pm_robot_calibration/calibrate_calibration_cube_xy_on_camera_top', self.calibrate_calibration_cube_to_cam_top, callback_group=self.callback_group)
        self.calibrate_calibration_cube_z_laser_srv = self.create_service(EmptyWithSuccess, '/pm_robot_calibration/calibrate_calibration_cube_z_on_confocal_top', self.calibrate_calibration_cube_z_on_confocal_top, callback_group=self.callback_group)

        #self.calibrate_laser_on_calibration_cube_srv = self.create_service(EmptyWithSuccess, '/pm_robot_calibration/calibrate_laser_on_calibration_cube', self.calibrate_laser_on_calibration_cube_callback, callback_group=self.callback_group)
        #self.calbirate_confocal_top_srv = self.create_service(EmptyWithSuccess, '/pm_robot_calibration/calibrate_confocal_top', self.calibrate_confocal_top_callback, callback_group=self.callback_group)
        self.calibrate_laser_head_on_camera_srv = self.create_service(EmptyWithSuccess, '/pm_robot_calibration/calibrate_laser_xy_on_camera_bottom', self.calibrate_laser_xy_on_camera_bottom, callback_group=self.callback_group)
        self.calibrate_confocal_head_on_camera_srv = self.create_service(EmptyWithSuccess, '/pm_robot_calibration/calibrate_confocal_top_xy_on_camera_bottom', self.calibrate_confocal_top_xy_on_camera_bottom, callback_group=self.callback_group)
        self.calibrate_confocal_bottom_z_srv = self.create_service(EmptyWithSuccess, '/pm_robot_calibration/calibrate_confocal_bottom_z_on_laser', self.calibrate_confocal_bottom_z_on_laser, callback_group=self.callback_group)
        self.calibrate_confocal_top_z_srv = self.create_service(EmptyWithSuccess, '/pm_robot_calibration/calibrate_confocal_top_z_on_laser', self.calibrate_confocal_top_z_on_laser, callback_group=self.callback_group)
        self.calibrate_confocal_bottom_xy_srv = self.create_service(EmptyWithSuccess, '/pm_robot_calibration/calibrate_confocal_bottom_xy_on_cam_top', self.calibrate_confocal_bottom_xy_on_cam_top, callback_group=self.callback_group)
        self.assess_hexapod_calibration_file_srv = self.create_service(skills_srv.AssessHexapodCalibration, '/pm_robot_calibration/assess_hexapod_calibration_file', self.assess_hexapod_calibration, callback_group=self.callback_group)

        #self.calibrate_siemens_gripper_on_cal_cube = self.create_service(EmptyWithSuccess, '/pm_robot_calibration/calibrate_siemens_gripper_on_cal_cube', self.calibrate_sim_gripper_on_cube, callback_group=self.callback_group)
        self.calbibrate_gonio_left_chuck_srv = self.create_service(EmptyWithSuccess, '/pm_robot_calibration/calibrate_gonio_left_chuck', self.calibrate_gonio_left_chuck_callback, callback_group=self.callback_group)
        self.calbibrate_gonio_right_chuck_srv = self.create_service(EmptyWithSuccess, '/pm_robot_calibration/calibrate_gonio_right_chuck', self.calibrate_gonio_right_chuck_callback, callback_group=self.callback_group)
        self.calibrate_gripper_srv = self.create_service(EmptyWithSuccess, '/pm_robot_calibration/calibrate_gripper_xyt_on_camera_bottom', self.calibrate_gripper_xyt_on_camera_bottom, callback_group=self.callback_group)
        self.calibrate_gripper_plane_srv = self.create_service(EmptyWithSuccess, '/pm_robot_calibration/calibrate_gripper_plane', self.calibrate_gripper_plane, callback_group=self.callback_group)
        
        self.calibrate_1K_dispenser_xy_on_cam_bottom_srv = self.create_service(EmptyWithSuccess, '/pm_robot_calibration/calibrate_1K_dispenser_xy_on_cam_bottom', self.calibrate_1K_dispenser_xy_on_cam_bottom, callback_group=self.callback_group)
        self.calibrate_1K_dispenser_z_on_calibration_cube_srv = self.create_service(EmptyWithSuccess, '/pm_robot_calibration/calibrate_1K_dispenser_z_on_calibration_cube', self.calibrate_1K_dispenser_z_on_calibration_cube, callback_group=self.callback_group)

        #self.calibrate_smarpod_srv = self.create_service(EmptyWithSuccess, '/pm_robot_calibration/calibrate_smarpod', self.calibrate_smarpod, callback_group=self.callback_group)
        

        self.calibrate_smarpod_action_srv = ActionServer(self,
            skills_action.SmarpodCalibration,
            f'/pm_robot_calibration/calibrate_smarpod',
            execute_callback=self.calibrate_smarpod_V3,
            goal_callback=self._goal_calibration_callback,
            cancel_callback=self._cancel_calibration_callback
        )

        # paths
        self.calibration_frame_dict_path = get_package_share_directory('pm_robot_description') + '/urdf/urdf_configs/calibration_frame_dictionaries'
        self.calibration_log_dir = get_package_share_directory('pm_robot_calibration') + '/calibration_logs/'

        #self.update_pm_robot_config()
        self.get_logger().info(self.INFO_TEXT)
        self._last_calibrations_data = {}    
        
    def get_gripper_calibration_frame_dictionary(self)->dict:
        calibration_frame_dict = {}
        file_path = None

        self.pm_robot_utils.update_pm_robot_config()
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
        
        if not self.client_spawn_frames_from_description.wait_for_service(timeout_sec=1.0):
            self._logger.error('Assembly manager not available...')
            return False, None
        
        self._logger.info('Calibration: Spawning frames for gripper...')
        
        response:SpawnFramesFromDescription.Response = self.client_spawn_frames_from_description.call(request=request)
        self.gripper_frames_spawned = True
        return response.success, frames_list, unique_identifier

    
    def spawn_calibration_frames(self, calibration_file_name:str):
        # spawn the calibration frames
        request = SpawnFramesFromDescription.Request()
        request.dict_or_path = self.calibration_frame_dict_path + '/' + calibration_file_name

        if not self.client_spawn_frames_from_description.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Assembly manager not available...')
            return False
        
        spawn_response:SpawnFramesFromDescription.Response = self.client_spawn_frames_from_description.call(request)  

        if not spawn_response.success:
            raise PmRobotError("Failed to spawn calibration frames")

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
        self._logger.warning("Starting Camera calibration!")
        
        try:

            self.set_calibration_platelet_forward()

            request_move_to_frame = MoveToFrame.Request()
            request_move_to_frame.target_frame = 'Camera_Station_TCP'
            request_move_to_frame.execute_movement = True
            request_move_to_frame.translation.z = 0.003
            request_move_to_frame.translation.x = 0.00

            if not self.pm_robot_utils.client_move_robot_cam1_to_frame.wait_for_service(timeout_sec=1.0):
                raise PmRobotError(f'Client "{self.pm_robot_utils.client_move_robot_cam1_to_frame.srv_name}" not available...')
            
            response_move_to_frame:MoveToFrame.Response = self.pm_robot_utils.client_move_robot_cam1_to_frame.call(request_move_to_frame)
            
            if not response_move_to_frame.success:
                raise PmRobotError("Failed to move to camera station for calibration")

            move_success = self.pm_robot_utils.send_xyz_trajectory_goal_absolut(    x_joint = CAMERA_CALIBRATION_JOINT_VALUES_X,
                                                                                    y_joint = CAMERA_CALIBRATION_JOINT_VALUES_Y,
                                                                                    z_joint = CAMERA_CALIBRATION_JOINT_VALUES_Z,
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
                    raise PmRobotError(f'Client "{self.pm_robot_utils.client_execute_vision.srv_name}" not available...')
                
                response_execute_vision_bottom:ExecuteVision.Response = self.pm_robot_utils.client_execute_vision.call(request_execute_vision_bottom)
                
                if not response_execute_vision_bottom.success:
                    raise PmRobotError("Failed to execute vision for bottom camera")
                
                # measure frame with top cam
                request_execute_vision_top = ExecuteVision.Request()
                request_execute_vision_top.camera_config_filename = self.pm_robot_utils.get_cam_file_name_top()
                request_execute_vision_top.image_display_time = -1
                request_execute_vision_top.process_filename = "PM_Robot_Calibration/Camera_Calibration_Top_Process.json"
                request_execute_vision_top.process_uid = f"Cam_Cal_T_{iter}"

                if not self.pm_robot_utils.client_execute_vision.wait_for_service(timeout_sec=1.0):
                    raise PmRobotError(f'Client "{self.pm_robot_utils.client_execute_vision.srv_name}" not available...')

                response_execute_vision_top:ExecuteVision.Response = self.pm_robot_utils.client_execute_vision.call(request_execute_vision_top)

                if not response_execute_vision_top.success:
                    raise PmRobotError("Failed to execute vision for top camera")
                
                # Only for the first iteration
                if iter == 0:
                    process_success = self._process_calibrate_cameras(vision_result_top=response_execute_vision_top.vision_response.results,
                                            vision_result_bottom=response_execute_vision_bottom.vision_response.results)
                    
                    if not process_success:
                        raise PmRobotError("Failed to calibrate cameras")

            calibrate_cs_success = self._process_calibrate_coordinate_systems(vision_result_top=response_execute_vision_top.vision_response.results,
                                                                            vision_result_bottom=response_execute_vision_bottom.vision_response.results)
            
            if not calibrate_cs_success:
                raise PmRobotError("Failed to calibrate coordinate systems")

            response.success = process_success
            self.set_calibration_platelet_backward()
            
        except PmRobotError as e:
            self._logger.error(f"Error occurred during camera calibration: {e}")
            response.success = False

        finally:
            self._logger.info(f"Camera calibration process completed with success: {response.success}")
            try:
                #self.set_calibration_platelet_backward()
                pass

            except PmRobotError as e2:
                self._logger.error(f"Error occurred while moving calibration target backward: {e2}")
                response.success = False
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

        #rel_trans.translation.x = (rel_trans.translation.x + trans.translation.x)
        #rel_trans.translation.y = (rel_trans.translation.y + trans.translation.y)
        rel_trans.translation.x = -1*trans.translation.x - rel_trans.translation.x
        rel_trans.translation.y = -1*trans.translation.y - rel_trans.translation.y


        self._logger.error(f"Result trans x: {rel_trans.translation.x}")
        self._logger.error(f"Result trans y: {rel_trans.translation.y}")


        save_success = self.save_joint_config ( joint_name='Cam1_Toolhead_TCP_Joint',
                                                rel_transformation=rel_trans,
                                                overwrite=False)

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

        self.get_logger().warning(f"Average radius: {average_radius} um")
        self.get_logger().warning(f"max radius: {max_radius} um")
        self.get_logger().warning(f"min radius: {min_radius} um")
        self.get_logger().warning(f"Radius: {average_radius} um")
        self.get_logger().warning(f"Radius: [{circle_0.radius}, {circle_1.radius}, {circle_2.radius}, {circle_3.radius}] um")
        self.get_logger().warning(f"Distances: [{length_01}, {length_13}, {length_32}, {length_20}] um")
        self.get_logger().warning(f"Angles: [{angle_01}, {angle_13}, {angle_32}, {angle_20}] um")

        average_angle = (angle_01 + angle_13 + angle_20 + angle_32)/4

        average_length = (length_01 + length_13 + length_32 + length_20)/4

        self.get_logger().warning(f"Avg angle: {average_angle} um")
        self.get_logger().warning(f"Avg length: {average_length} um")

        return (average_angle, average_length)
        
    ###########################################
    ### Calibrate 1K_dispenser ################
    ###########################################
    
    def calibrate_1K_dispenser_xy_on_cam_bottom(self, request:EmptyWithSuccess.Request, response:EmptyWithSuccess.Response):
        
        #frame = '1K_Dispenser_TCP'
        try:
            self.pm_robot_utils.move_1k_dispenser_to_frame(frame_name=self.pm_robot_utils.TCP_CAMERA_BOTTOM,
                                                        z_offset=DISPENSER_TRAVEL_DISTANCE + 0.01)

            self.pm_robot_utils.open_protection()

            time.sleep(1.0)

            self.pm_robot_utils.extend_dispenser()

            time.sleep(1.0)

            time_to_wait = 20
            self.get_logger().warninging(f"You have {time_to_wait} seconds to clean the dispenser needle.")

            for t in range(time_to_wait):
                self.get_logger().info(f"You have {time_to_wait - t} seconds to clean the dispenser needle.")
                time.sleep(1.0)

            self.pm_robot_utils.move_1k_dispenser_to_frame(frame_name=self.pm_robot_utils.TCP_CAMERA_BOTTOM)

            frame_name = self.spawn_1k_dispenser_cal_frame()
            
            time.sleep(1.0)
            
            success_correct_frame = self.correct_frame_vison(frame_name).success

            if not success_correct_frame:
                seconds = 100
                self.get_logger().error(f"Failed to correct 1K dispenser vision frame. You have {seconds} seconds to fix this issue.")

                for t in range(seconds):
                    self.get_logger().info(f"You have {seconds - t} seconds to fix the issue.")
                    time.sleep(1.0)

                success_correct_frame_2 = self.correct_frame_vison(frame_name).success

                if not success_correct_frame_2:
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
                pass
            except PmRobotError as e2:
                self.get_logger().error(f"Error occurred while resetting 1K dispenser: {e2.message}")
                response.success = False
            pass

        return response

    def spawn_1k_dispenser_cal_frame(self)->str:

        spawn_request = ami_srv.CreateRefFrame.Request()

        # get current dispenser tip
        self.pm_robot_utils.update_pm_robot_config()
        dispenser_tip = self.pm_robot_utils.pm_robot_config.dispenser_1k.get_current_dispenser_tip()

        frame_name = f"CAL_{dispenser_tip}_Vision_Frame"

        spawn_request.ref_frame.frame_name = frame_name
        spawn_request.ref_frame.parent_frame = self.pm_robot_utils.TCP_1K_DISPENSER


        if not self.pm_robot_utils.client_create_ref_frame.wait_for_service(timeout_sec=1.0):
            raise PmRobotError(f"Client '{self.pm_robot_utils.client_create_ref_frame.srv_name}' not available")

        response:ami_srv.CreateRefFrame.Response = self.pm_robot_utils.client_create_ref_frame.call(spawn_request)

        if not response.success:
            raise PmRobotError(f"Failed to create reference frame: {spawn_request.ref_frame.frame_name}")

        return spawn_request.ref_frame.frame_name


    ###########################################
    ### calibrate_1K_dispenser_z_on_calibration_cube ################
    ###########################################
    
    def calibrate_1K_dispenser_z_on_calibration_cube(self, request:EmptyWithSuccess.Request, response:EmptyWithSuccess.Response):
        
        #frame = '1K_Dispenser_TCP'

        try:
            frame_name = 'Calibration_Cube_Bottom'
            self.pm_robot_utils.move_1k_dispenser_to_frame(frame_name=frame_name,
                                                        z_offset=DISPENSER_TRAVEL_DISTANCE + 0.01)
            
            self.pm_robot_utils.open_protection()

            time.sleep(1.0)

            self.pm_robot_utils.extend_dispenser()

            time.sleep(1.0)

            time_to_wait = 2
            self.get_logger().warninging(f"You have {time_to_wait} seconds to clean the dispenser needle.")

            for t in range(time_to_wait):
                self.get_logger().info(f"You have {time_to_wait - t} seconds to clean the dispenser needle.")
                time.sleep(1.0)

            self.pm_robot_utils.move_1k_dispenser_to_frame(frame_name=frame_name,
                                                           z_offset =0.003)

            step_inc = 0.5  

            self.get_logger().info("Starting coarse approach...")
            x, y, final_z = self.pm_robot_utils.interative_sensing(measurement_method=None,
                                measurement_valid_function = self.pm_robot_utils.check_reference_cube_pressed,
                                length = (0.0, 0.0, 5.0),
                                step_inc = step_inc,
                                total_time = 8.0)
            
            self.pm_robot_utils.send_xyz_trajectory_goal_relative(0.0, 0.0, -0.0005, 1.0)

            self.get_logger().info("Starting fine approach...")
            step_inc = 0.05
            x, y, final_z = self.pm_robot_utils.interative_sensing(measurement_method=None,
                                measurement_valid_function = self.pm_robot_utils.check_reference_cube_pressed,
                                length = (0.0, 0.0, 1.0),
                                step_inc = step_inc,
                                total_time = 8.0)
            
            
            relative_transform:Transform = self.pm_robot_utils.get_transform_for_frame(frame_name=self.pm_robot_utils.TCP_1K_DISPENSER,
                                                                                        parent_frame=frame_name)

            rel_transform = Transform()
            rel_transform.translation.z = -1*relative_transform.translation.z

            self.save_joint_config('1K_Dispenser_TCP_Joint', rel_transform)

            cal_dict = self._transform_to_dict(rel_transform)

            self.log_calibration(file_name='calibrate_1K_dispenser_z_on_calibration_cube', calibration_dict=cal_dict)
            
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
                # move out of danger zone
                self.pm_robot_utils.send_xyz_trajectory_goal_relative(0.0, 0.0, -0.05, 1.0)

            except PmRobotError as e2:
                self.get_logger().error(f"Error occurred while resetting 1K dispenser: {e2.message}")
                response.success = False
            pass

        return response

    ###############################################
    ### Calibrate gripper #########################
    ###############################################

    def calibrate_gripper_xyt_on_camera_bottom(self, request:EmptyWithSuccess.Request, response:EmptyWithSuccess.Response):
        
        try:
            spawn_success, frames, unique_identifier = self.spawn_frames_for_current_gripper()
            
            if not spawn_success:
                raise PmRobotError("Spawning of gripper frames failed!")
            
            move_success = self.pm_robot_utils.send_t_trajectory_goal_absolut(0.0, 2.0)

            if not move_success:
                raise PmRobotError("Failed to move gripper to rotation 0.0")
            
            # maybe make this a service input?? so that the user can dynamically decide on the number of rotations?

            #rotations = [0.0, 20, 30, 40, 50, 60, 80]
            #rotations = [90, 65, 55, 45, 35, 25, 0]
            # rotations = [90, 60, 30, 0]
            rotations = [0]

            #rotations = [60, 40, 20, 0]

            # move gripper close to camera to calibration start position
            move_to_start_success, move_msg = self.pm_robot_utils.move_camera_top_to_frame(frame_name=self.pm_robot_utils.TCP_CAMERA_BOTTOM,
                                                                                endeffector_override=self.pm_robot_utils.TCP_TOOL,
                                                                                z_offset=-0.05)
            
            if not move_to_start_success:
                raise PmRobotError(f"Failed to move to start position: {move_msg}")

            # convert to rad
            rotations = [r * np.pi / 180.0 for r in rotations]
            
            if len(rotations) == 0:
                raise PmRobotError("No angles for the gripper calibration specified")
            
            distance_list: list[float] = []  
            frame_poses_list:list[Pose] = []      

            for index, rotation in enumerate(rotations):
                move_success = self.pm_robot_utils.send_t_trajectory_goal_absolut(rotation, 2.0)
                self.get_logger().error("Gripper rotation: " + str(rotation))
                
                if not move_success:
                    raise PmRobotError("Failed to move gripper to rotation: " + str(rotation))
         
                for frame in frames:
                    
                    if 'Vision' in frame or 'vision' in frame:
                        correct_frame_success = self.correct_frame_vison(frame).success
                        #correct_frame_success = self.measure_frame(frame)
                        
                        if not correct_frame_success:
                            raise PmRobotError("Failed to correct frame: " + frame)
                
                ref_frame = self.pm_robot_utils.assembly_scene_analyzer.get_ref_frame_by_name(f'{unique_identifier}CALIBRATION_PM_Robot_Tool_TCP')
                
                if ref_frame is None:
                    raise PmRobotError("Failed to get reference frame...")
                
                if index !=0:
                    pose_1 = frame_poses_list[index-1]
                    pose_2 = ref_frame.pose
                    distance = np.sqrt((pose_1.position.x - pose_2.position.x)**2 + (pose_1.position.y - pose_2.position.y)**2)
                    distance_list.append(distance)

                frame_poses_list.append(copy.deepcopy(ref_frame.pose))
            
            relative_transform:Transform = get_rel_transform_for_frames(scene=self.pm_robot_utils.assembly_scene_analyzer._get_scene(),
                                        from_frame=f'{unique_identifier}CALIBRATION_PM_Robot_Tool_TCP',
                                        to_frame=f'{unique_identifier}CALIBRATION_PM_Robot_Tool_TCP_initial',
                                        tf_buffer=self.pm_robot_utils.tf_buffer,
                                        logger=self._logger)
            
            # Check if calibration was done without rotation (single rotation of 0)
            if len(rotations) > 1 and len(distance_list) > 0:
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
                    rel_t_joint.translation.x = (relative_transform.translation.x - x)
                    rel_t_joint.translation.y = (relative_transform.translation.y - y)
                    
                    relative_transform.translation.x = x
                    relative_transform.translation.y = y
                    
                    self._logger.warning(f"T-Axis has offset: {x* 1e6}, {y* 1e6} um")
                    
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
                    self._logger.warning("T-Axis has no offset...")
            else:
                # Single rotation calibration (no multi-rotation axis correction needed)
                self._logger.info("Calibrating gripper without rotation - single pose calibration")

            # log relative pose
            #self._logger.error("Relative pose: " + str(relative_transform))
            self._logger.error("Translation of the gripper tip")
            self._logger.error(f"x offset: {relative_transform.translation.x * 1e6} um")
            self._logger.error(f"y offset: {relative_transform.translation.y * 1e6} um")
            
            self.save_joint_config('PM_Robot_Tool_TCP_Joint', relative_transform)
            self.log_calibration(file_name='PM_Robot_Tool_TCP_Joint', calibration_dict=self._transform_to_dict(relative_transform))

            response.success = True

        except PmRobotError as e:
            self._logger.error(f"Calibrate Gripper failed: {e}")
            response.success = False

        finally:
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
        
        self.pm_robot_utils.assembly_scene_analyzer.wait_for_initial_scene_update()
        
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
                
                ref_frame: ami_msg.RefFrame = self.pm_robot_utils.assembly_scene_analyzer.get_ref_frame_by_name(frame)

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

        
        relative_transform_angle:Transform =    get_rel_transform_for_frames(scene=self.pm_robot_utils.assembly_scene_analyzer._get_scene(),
                                        from_frame=f'{unique_identifier}CALIBRATION_PM_Robot_Tool_TCP_initial',
                                        to_frame=f'{unique_identifier}CALIBRATION_PM_Robot_Tool_TCP',
                                        tf_buffer=self.pm_robot_utils.tf_buffer,
                                        logger=self._logger)
        
        roll, pitch, yaw = R.from_quat([relative_transform_angle.rotation.x, 
                                        relative_transform_angle.rotation.y,
                                        relative_transform_angle.rotation.z,
                                        relative_transform_angle.rotation.w,]).as_euler('xyz', degrees=True)
        
        self._logger.warning(f"Results - Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}")

        result_dict['results'] = result_list
        result_dict["rx"] = round(roll, 5)
        result_dict["ry"] = round(pitch, 5)
        result_dict["rz"] = round(yaw, 5)
        result_dict["z"] = round(relative_transform_angle.translation.z*1e6, 1)

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
    ### calibrate_calibration_cube_to_cam_top #
    ###########################################
    
    def calibrate_calibration_cube_to_cam_top(self, request:EmptyWithSuccess.Request, response:EmptyWithSuccess.Response):

        self._logger.warning(f"Starting calibration 'calibrate_calibration_cube_to_cam_top'...")

        try:
            # Spawn the frames

            self.spawn_calibration_frames('CF_Calibration_Qube_Cam_Top.json')

            unique_identifier = self.get_unique_identifier('CF_Calibration_Qube_Cam_Top.json')

            vision_request = ExecuteVision.Request()
            vision_request.process_filename = f"Assembly_Manager/Frames/{unique_identifier}Vision_Dynamic.json"
            vision_request.process_uid = "Cal_Calibrate_Cube"
            vision_request.image_display_time = -1
            vision_request.camera_config_filename = self.pm_robot_utils.get_cam_file_name_top()


            if not self.pm_robot_utils.client_execute_vision.wait_for_service(timeout_sec=1.0):
                raise PmRobotError("Service 'ExecuteVision' not available...")

            frame_name = f'{unique_identifier}Vision_Dynamic'
            move_success_offset, move_offset_msg = self.pm_robot_utils.move_camera_top_to_frame(frame_name=frame_name,
                                                                                            z_offset=0.05)

            if not move_success_offset:
                raise PmRobotError(f"Could not move the camera to frame {frame_name}: {move_offset_msg}")
            
            move_success, move_msg = self.pm_robot_utils.move_camera_top_to_frame(frame_name=frame_name)

            if not move_success:
                raise PmRobotError(f"Could not move the camera to frame {frame_name}: {move_msg}")

            # Measure the frame
            result:ExecuteVision.Response = self.pm_robot_utils.client_execute_vision.call(vision_request)

            if not result.success:
                raise PmRobotError("Vision process failed...")
                
            #detected_point:vision_msg.VisionPoint = result.vision_response.results.points[0]

            circle:vision_msg.VisionCircle = result.vision_response.results.circles[0]
            detected_point:vision_msg.VisionPoint = circle.center_point

            
            rel_transform = Transform()
            rel_transform.translation.x = detected_point.axis_value_1 * 1e-6
            rel_transform.translation.y = detected_point.axis_value_2 * 1e-6

            success = self.save_joint_config('Calibration_Qube_Joint', 
                                             rel_transform)
            
            self.log_calibration(file_name='calibrate_calibration_cube_to_cam_top', 
                                 calibration_dict=self._transform_to_dict(rel_transform))
            
            if not success:
                raise PmRobotError("Failed to save joint config...")
            
            response.success = True
        
        except PmRobotError as e:
            response.success = False
            self.get_logger().error(f"Calibration failed: {e}")

        finally:
            # move out of danger zone
            self.pm_robot_utils.send_xyz_trajectory_goal_relative(0.0, 0.0, -0.05, 0.5)

        return response
    
    ###########################################
    ### laser_on_calibration_cube ############
    ###########################################
    
    # def calibrate_laser_on_calibration_cube_callback(self, request:EmptyWithSuccess.Request, response:EmptyWithSuccess.Response):
    #     # To be implemented...
    #     self.get_logger().warning("Laser on calibration cube not fully implemented yet...")
        
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
    
        
    ###########################################
    ### calibrate_laser_xy_on_camera_bottom ###
    ###########################################

    def calibrate_laser_xy_on_camera_bottom(self, request:EmptyWithSuccess.Request, response:EmptyWithSuccess.Response):
        """
        The functionality of the laser calibration has been verified by manually moving the laser to the camera tcp. By experts!!!
        """
        try:
            self._logger.warning(f"Starting calibration 'calibrate_laser_xy_on_camera_bottom'...")

            CAMERA_TARGET_HEIGHT = 1.6 #    mm - this is not needed as the fiducials are on top of the platelet
            
            self.set_calibration_platelet_forward()

            move_success = self.pm_robot_utils.move_laser_to_frame('Calibration_Platelet_Calibration_Frame',
                                                                z_offset=0.05)

            if not move_success:
                raise PmRobotError("Failed to move laser to frame")
            
            move_success = self.pm_robot_utils.move_laser_to_frame('Calibration_Platelet_Calibration_Frame',
                                                                z_offset=0.00)

            if not move_success:
                raise PmRobotError("Failed to move laser to frame")           

            if not self.pm_robot_utils._check_for_valid_laser_measurement():
                raise PmRobotError("Could not get a valid laser measurement!")
            
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
                raise PmRobotError('Vision Manager not available...')

            response_execute_vision_bottom:ExecuteVision.Response = self.pm_robot_utils.client_execute_vision.call(request_execute_vision_bottom)
            
            if not response_execute_vision_bottom.success:
                raise PmRobotError("Failed to execute vision for bottom camera")
            
            if len(response_execute_vision_bottom.vision_response.results.circles) != 1:
                raise PmRobotError("Vision did not find a single circle!")
            
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

            self._logger.warning(f"Correction value x: {rel_trans.translation.x*1e6} um")
            self._logger.warning(f"Correction value y: {rel_trans.translation.y*1e6} um")

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
                raise PmRobotError("Saving of configuration failed!")

            # move out of danger zone
            self.pm_robot_utils.send_xyz_trajectory_goal_relative(0.0, 0.0, -0.02, 1.0)
            
            response.success = True
        
        except PmRobotError as e:
            self._logger.error(f"Error occurred during 'calibrate_confocal_bottom_xy_on_cam_top': {e}")
            response.success = False

        finally:
            self._logger.info(f"Camera calibration process completed with success: {response.success}")
            try:
                self.set_calibration_platelet_backward()

            except PmRobotError as e2:
                self._logger.error(f"Error occurred while moving calibration target backward: {e2}")
                response.success = False
        return response
    
    ##################################################
    ### calibrate_confocal_top_xy_on_camera_bottom ###
    ##################################################

    def calibrate_confocal_top_xy_on_camera_bottom(self, request:EmptyWithSuccess.Request, response:EmptyWithSuccess.Response):

        try:
            self._logger.warning(f"Starting calibration 'calibrate_confocal_top_xy_on_camera_bottom'...")

            move_success, move_msg = self.pm_robot_utils.move_confocal_top_to_frame(self.pm_robot_utils.TCP_CAMERA_BOTTOM,
                                                                          z_offset=0.05)

            if not move_success:
                raise PmRobotError(f"Failed to move confocal top to frame: {move_msg}")
            
            self.set_calibration_platelet_forward()
            
            move_success, move_msg = self.pm_robot_utils.move_confocal_top_to_frame(self.pm_robot_utils.TCP_CAMERA_BOTTOM,
                                                                          z_offset=0.002)

            if not move_success:
                raise PmRobotError(f"Failed to move confocal top to frame: {move_msg}")
            
            if not self.pm_robot_utils.check_confocal_top_measurement_in_range():
                            
                step_inc = 1.0 # in mm
                self._logger.warning(f"Laser measurement not valid! Trying to iteratively find a valid value!")                

                x, y, final_z = self.pm_robot_utils.interative_sensing(measurement_method=self.pm_robot_utils.get_confocal_top_measurement,
                                                measurement_valid_function = self.pm_robot_utils.check_confocal_top_measurement_in_range,
                                                length = (0.0, 0.0, 4.0),
                                                step_inc = step_inc,
                                                total_time = 2.0)
                
                if x is None:
                    raise PmRobotError(f"Laser measurement not valid! OUT OF RANGE")
            
            initial_z_height = self.pm_robot_utils.get_confocal_top_measurement(unit='m')
            
            self.get_logger().error(f"Measured confocal heigt '{initial_z_height*1e-6}' um")

            self.pm_robot_utils.send_xyz_trajectory_goal_relative(0.0, 0.0, -initial_z_height, 1.0)

            time.sleep(1)

            initial_z_height = self.pm_robot_utils.get_confocal_top_measurement(unit='m')
            
            request_execute_vision_bottom = ExecuteVision.Request()
            request_execute_vision_bottom.camera_config_filename = self.pm_robot_utils.get_cam_file_name_bottom()
            request_execute_vision_bottom.image_display_time = -1
            request_execute_vision_bottom.process_filename = "PM_Robot_Calibration/Calibration_Confocal_Top_xy_on_Camera_Bottom.json"
            request_execute_vision_bottom.process_uid = f"Confocal_xy_on_Cam_Bottom"

            if not self.pm_robot_utils.client_execute_vision.wait_for_service(timeout_sec=1.0):
                raise PmRobotError('Vision Manager not available...')

            # first try
            response_execute_vision_bottom:ExecuteVision.Response = self.pm_robot_utils.client_execute_vision.call(request_execute_vision_bottom)
            
            if not response_execute_vision_bottom.success:
                raise PmRobotError("Failed to execute vision for bottom camera")
            
            if len(response_execute_vision_bottom.vision_response.results.circles) != 1:
                raise PmRobotError("Vision did not find a single circle!")

            circle:vision_msg.VisionCircle = response_execute_vision_bottom.vision_response.results.circles[0]

            x_offset = circle.center_point.axis_value_1
            y_offset = circle.center_point.axis_value_2

            time.sleep(1.0)

            self.pm_robot_utils.send_xyz_trajectory_goal_relative(x_joint_rel=-x_offset*1e-6,
                                                                  y_joint_rel=-y_offset*1e-6,
                                                                  z_joint_rel=0.0, 
                                                                  time=0.5)

            # second try
            response_execute_vision_bottom:ExecuteVision.Response = self.pm_robot_utils.client_execute_vision.call(request_execute_vision_bottom)
            
            if not response_execute_vision_bottom.success:
                raise PmRobotError("Failed to execute vision for bottom camera")
            
            if len(response_execute_vision_bottom.vision_response.results.circles) != 1:
                raise PmRobotError("Vision did not find a single circle!")

            circle:vision_msg.VisionCircle = response_execute_vision_bottom.vision_response.results.circles[0]

            x_offset = circle.center_point.axis_value_1
            y_offset = circle.center_point.axis_value_2

            time.sleep(1.0)

            rel_trans = Transform()
    
            transform = self.pm_robot_utils.get_transform_for_frame(frame_name=self.pm_robot_utils.TCP_CONFOCAL_TOP,
                                                                parent_frame=self.pm_robot_utils.TCP_CAMERA_BOTTOM)
            
            rel_trans.translation.x = transform.translation.x - x_offset * 1e-6
            rel_trans.translation.y = transform.translation.y - y_offset * 1e-6

            self._logger.warning(f"Correction value x: {rel_trans.translation.x*1e6} um")
            self._logger.warning(f"Correction value y: {rel_trans.translation.y*1e6} um")

            rel_trans.translation.x = -1*rel_trans.translation.x
            rel_trans.translation.y = -1*rel_trans.translation.y
            
            save_success = self.save_joint_config ( joint_name='Confocal_Sensor_Top_TCP_Joint',
                                                    rel_transformation=rel_trans,
                                                    overwrite=False)

            self.log_calibration(file_name="calibrate_confocal_top_xy_on_camera_bottom",
                                 calibration_dict=self._transform_to_dict(rel_trans))

            if not save_success:
                raise PmRobotError("Saving of configuration failed!")    

            response.success = True
    
        except PmRobotError as e:
            self._logger.error(f"Error occurred during 'calibrate_confocal_top_xy_on_camera_bottom': {e}")
            response.success = False

        finally:
            self._logger.info(f"Camera calibration process completed with success: {response.success}")
            try:
                self.set_calibration_platelet_backward()
                # move out of danger zone
                self.pm_robot_utils.send_xyz_trajectory_goal_relative(0.0, 0.0, -0.02, 1.0)      

            except PmRobotError as e2:
                self._logger.error(f"Error occurred while moving calibration target backward: {e2}")
                response.success = False
        return response
    
    ##################################################
    ### calibrate_confocal_bottom_xy_on_cam_top ######
    ##################################################

    def calibrate_confocal_bottom_xy_on_cam_top(self, request:EmptyWithSuccess.Request, response:EmptyWithSuccess.Response):
        
        try:
            self._logger.warning(f"Starting calibration 'calibrate_confocal_bottom_xy_on_cam_top'...")
            
            self.set_calibration_platelet_forward()
            
            request_move_to_frame = MoveToFrame.Request()
            request_move_to_frame.target_frame = 'Laser_Height_Calibration_Frame'
            request_move_to_frame.execute_movement = True
            request_move_to_frame.translation.z = -0.00387

            if not self.pm_robot_utils.client_move_robot_cam1_to_frame.wait_for_service(timeout_sec=1.0):
                raise PmRobotError('Camera move service not available...')
            
            response_move_to_frame:MoveToFrame.Response = self.pm_robot_utils.client_move_robot_cam1_to_frame.call(request_move_to_frame)

            if not response_move_to_frame.success:
                raise PmRobotError("Failed to move laser to frame")

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
                raise PmRobotError("TBD")
            
            x_offset, y_offset = self._get_circle_from_vision(process_file_name="Calibration_Confocal_Bottom_xy_on_Camera_Top.json",
                                                            camera_file_name=self.pm_robot_utils.get_cam_file_name_top(),
                                                            process_name="Confocal_xy_on_Cam_Top")
            
            if x_offset is None:
                raise PmRobotError("TBD")
            
            #self._logger.error(f"Camera measurement {x_offset} um, {y_offset} um")

            time.sleep(2.0)

            rel_trans = Transform()

            transfrom = self.pm_robot_utils.get_transform_for_frame(frame_name=self.pm_robot_utils.TCP_CONFOCAL_BOTTOM,
                                                                parent_frame=self.pm_robot_utils.TCP_CAMERA_TOP)
            
            #self._logger.error(f"Transform x: {transfrom.translation.x}")
            #self._logger.error(f"Transform y: {transfrom.translation.y}")

            rel_trans.translation.x = transfrom.translation.x - x_offset * 1e-6
            rel_trans.translation.y = transfrom.translation.y - y_offset * 1e-6

            self._logger.warning(f"Result x: {rel_trans.translation.x*1e6} um")
            self._logger.warning(f"Result y: {rel_trans.translation.y*1e6} um")

            rel_trans.translation.x = -1*rel_trans.translation.x
            rel_trans.translation.y = -1*rel_trans.translation.y
            
            save_success = self.save_joint_config ( joint_name='Confocal_Sensor_Bottom_TCP_Joint',
                                                    rel_transformation=rel_trans,
                                                    overwrite=False)

            if not save_success:
                raise PmRobotError("Saving of configuration failed!")

            # move out of danger zone
            #self.pm_robot_utils.send_xyz_trajectory_goal_relative(0.0, 0.0, -0.05, 1.0)
            #backward_response:EmptyWithSuccess.Response = self.pm_robot_utils.client_move_calibration_target_backward.call(backward_request)

            response.success = True
            
        except PmRobotError as e:
            self._logger.error(f"Error occurred during 'calibrate_confocal_bottom_xy_on_cam_top': {e}")
            response.success = False

        finally:
            self._logger.info(f"Camera calibration process completed with success: {response.success}")
            try:
                self.set_calibration_platelet_backward()

            except PmRobotError as e2:
                self._logger.error(f"Error occurred while moving calibration target backward: {e2}")
                response.success = False
        return response
    

    ##################################################
    ### calibrate_confocal_bottom_xy_on_cam_top ######
    ##################################################
    
    def calibrate_confocal_bottom_z_on_laser(self, request:EmptyWithSuccess.Request, response:EmptyWithSuccess.Response):
        
        try:

            self._logger.warning(f"Starting calibration 'calibrate_confocal_bottom_z_on_laser'...")
            
            self.set_calibration_platelet_forward()
                    
            move_success = self.pm_robot_utils.move_laser_to_frame(frame_name='TCP_Confocal_Sensor_Bottom',
                                                                z_offset=0.004,
                                                                y_offset=-0.003)

            if not move_success:
                raise PmRobotError("Failed to move laser to frame")

            if not self.pm_robot_utils.check_confocal_bottom_measurement_in_range():
                raise PmRobotError("Could not get a valid confocal bottom measurement!")
            
            if not self.pm_robot_utils._check_for_valid_laser_measurement():

                move_success = self.pm_robot_utils.send_xyz_trajectory_goal_relative(0, 0, -3.0*1e-3,time=0.5)
                                                
                if not move_success:
                    raise PmRobotError("TBD")
                
                step_inc = 0.4 # in mm
                self._logger.warning(f"Laser measurement not valid! Trying to iteratively find a valid value!")                

                x, y, final_z = self.pm_robot_utils.interative_sensing(measurement_method=self.pm_robot_utils.get_laser_measurement,
                                                measurement_valid_function = self.pm_robot_utils._check_for_valid_laser_measurement,
                                                length = (0.0, 0.0, 8.0),
                                                step_inc = step_inc,
                                                total_time = 8.0)
                
                if x is None:
                    raise PmRobotError(f"Laser measurement not valid! OUT OF RANGE")
                    
            initial_z_height = self.pm_robot_utils.get_laser_measurement(unit='m')
            
            self.get_logger().warning(f"Measured laser heigt '{initial_z_height*1e6}' um")

            self.pm_robot_utils.send_xyz_trajectory_goal_relative(0.0, 0.0, -initial_z_height, 1.0)

            confocal_bottom_measurement = self.pm_robot_utils.get_confocal_bottom_measurement(unit='m')

            self.get_logger().warning(f"Measured confocal bottom heigt '{confocal_bottom_measurement*1e6}' um")

            time.sleep(1.0)

            # beginning the calculations

            rel_trans = Transform()
            
            transfrom = self.pm_robot_utils.get_transform_for_frame(frame_name=self.pm_robot_utils.TCP_LASER,
                                                                parent_frame=self.pm_robot_utils.TCP_CONFOCAL_BOTTOM)
            
            self._logger.warning(f"Transform from Confocal bottom to laser - z: {transfrom.translation.z*1e6} um.")

            rel_trans.translation.z = transfrom.translation.z

            distance_between_laser_confocal_bottom = LASER_CALIBRATION_TARGET_THICKNESS*1e-3 - confocal_bottom_measurement

            #rel_trans.translation.z = -1*(distance_between_laser_confocal_bottom - rel_trans.translation.z )

            rel_trans.translation.z = rel_trans.translation.z - distance_between_laser_confocal_bottom

            self._logger.warning(f"Calibration result z: {rel_trans.translation.z*1e6} um.")

            save_success = self.save_joint_config ( joint_name ='Confocal_Sensor_Bottom_TCP_Joint',
                                                    rel_transformation=rel_trans,
                                                    overwrite=False)

            if not save_success:
                raise PmRobotError(f"Saving of configuration failed!")

            # move out of danger zone
            self.pm_robot_utils.send_xyz_trajectory_goal_relative(0.0, 0.0, -0.05, 1.0)

            calibration_dict = {}
            calibration_dict["transform"] = self._transform_to_dict(rel_trans)
            calibration_dict["confocal_bottom_measurement"] = round(confocal_bottom_measurement*1e6,1)

            self.log_calibration(file_name='calibrate_confocal_bottom_z_on_laser',
                                calibration_dict=calibration_dict)

            response.success = True
                
        except PmRobotError as e:
            self._logger.error(f"Error occurred during 'calibrate_confocal_bottom_xy_on_cam_top': {e}")
            response.success = False

        finally:
            self._logger.info(f"Camera calibration process completed with success: {response.success}")
            try:
                self.set_calibration_platelet_backward()

            except PmRobotError as e2:
                self._logger.error(f"Error occurred while moving calibration target backward: {e2}")
                response.success = False
        return response
        
    
    #################################################
    ## calibrate_confocal_top_z_on_laser ############
    #################################################

    def calibrate_confocal_top_z_on_laser(self, request:EmptyWithSuccess.Request, response:EmptyWithSuccess.Response):
        
        try:
            self._logger.warning(f"Starting calibration 'calibrate_confocal_top_z_on_laser'...")

            #frame_name = 'Calibration_Platelet_Calibration_Frame'
            frame_name = 'Calibration_Qube'

            x_offset = -0.003
            y_offset = -0.003

            move_success = self.pm_robot_utils.move_laser_to_frame(frame_name=frame_name,
                                                                    z_offset=0.05,
                                                                    x_offset=x_offset,
                                                                    y_offset=y_offset)

            if not move_success:
                raise PmRobotError("Failed to move laser to frame")

            #self.set_calibration_platelet_forward()

            move_success = self.pm_robot_utils.move_laser_to_frame(frame_name=frame_name,
                                                                    z_offset=0.002,
                                                                    x_offset=x_offset,
                                                                    y_offset=y_offset)

            if not move_success:
                raise PmRobotError("Failed to move laser to frame")
            
            step_inc = 0.4 # in mm
            self._logger.warning(f"Laser measurement not valid! Trying to iteratively find a valid value!")                

            x, y, final_z = self.pm_robot_utils.interative_sensing(measurement_method=self.pm_robot_utils.get_laser_measurement,
                                            measurement_valid_function = self.pm_robot_utils._check_for_valid_laser_measurement,
                                            length = (0.0, 0.0, 3.0),
                                            step_inc = step_inc,
                                            total_time = 4.0)
            
            if x is None:
                raise PmRobotError(f"Laser measurement not valid! OUT OF RANGE")
                    
            initial_z_height = self.pm_robot_utils.get_laser_measurement(unit='m')

            #self.get_logger().error(f"Measured laser height measurement'{initial_z_height*1e6}' um")

            self.pm_robot_utils.send_xyz_trajectory_goal_relative(0.0, 0.0, -initial_z_height, 0.5)

            time.sleep(2.0)

            laser_transform = self.pm_robot_utils.get_transform_for_frame(frame_name=self.pm_robot_utils.TCP_LASER,
                                                                parent_frame='world')

            laser_transform_z = laser_transform.translation.z

            self.get_logger().error(f"Laser offset value' {laser_transform_z*1e6}' um")

            if not self.pm_robot_utils.client_move_robot_confocal_top_to_frame.wait_for_service(timeout_sec=1.0):
                raise PmRobotError('Camera move service not available...')
            
            move_success, move_msg = self.pm_robot_utils.move_confocal_top_to_frame(frame_name=frame_name,
                                                                          x_offset=x_offset,
                                                                          y_offset=y_offset)

            if not move_success:
                raise PmRobotError(f"Failed to move laser to frame: {move_msg}")
           
            if not self.pm_robot_utils.check_confocal_top_measurement_in_range():
                
                # move 3 mm up
                move_success = self.pm_robot_utils.send_xyz_trajectory_goal_relative(0, 0, -3.0*1e-3,time=0.3)
                                                
                if not move_success:
                    raise PmRobotError("TBD")
                
                step_inc = 1.0 # in mm
                self._logger.warning(f"Laser measurement not valid! Trying to iteratively find a valid value!")                

                x, y, final_z = self.pm_robot_utils.interative_sensing(measurement_method=self.pm_robot_utils.get_confocal_top_measurement,
                                                measurement_valid_function = self.pm_robot_utils.check_confocal_top_measurement_in_range,
                                                length = (0.0, 0.0, 4.0),
                                                step_inc = step_inc,
                                                total_time = 2.0)
                
                if x is None:
                    raise PmRobotError(f"Laser measurement not valid! OUT OF RANGE")

            confocal_top_measurement = self.pm_robot_utils.get_confocal_top_measurement('m')

            #self.get_logger().warning(f"Measured confocal heigt measurement '{confocal_top_measurement*1e6}' um")

            self.pm_robot_utils.send_xyz_trajectory_goal_relative(0.0, 0.0, -confocal_top_measurement, 0.5)

            time.sleep(2.0)

            confocal_transform = self.pm_robot_utils.get_transform_for_frame(frame_name=self.pm_robot_utils.TCP_CONFOCAL_TOP,
                                                                            parent_frame='world')

            confocal_transform_z = confocal_transform.translation.z

            self.get_logger().error(f"Confocal offset value' {confocal_transform_z*1e6}' um")

            rel_trans = Transform()
            
            transform_z = (confocal_transform_z - laser_transform_z)

            rel_trans.translation.z = transform_z

            self._logger.warning(f"Calibration value z: {rel_trans.translation.z*1e6} um.")

            save_success = self.save_joint_config ( joint_name='Confocal_Sensor_Top_TCP_Joint',
                                                    rel_transformation=rel_trans,
                                                    overwrite=False)

            self.log_calibration(file_name='calibrate_confocal_top_z_on_laser', 
                                 calibration_dict=self._transform_to_dict(rel_trans))

            if not save_success:
                raise PmRobotError("Saving of configuration failed!") 

            response.success = True
        
        except PmRobotError as e:
            self._logger.error(f"Error occurred during 'calibrate_confocal_top_z_on_laser': {e}")
            response.success = False

        finally:
            self._logger.info(f"Camera calibration process completed with success: {response.success}")
            try:
                self.set_calibration_platelet_backward()
                # Move out of the danger zone
                self.pm_robot_utils.send_xyz_trajectory_goal_relative(0.0, 0.0, -0.05, 1.0)

            except PmRobotError as e2:
                self._logger.error(f"Error occurred while moving calibration target backward: {e2}")
                response.success = False
        return response

    #################################################
    

    def calibrate_calibration_cube_z_on_confocal_top(self, request:EmptyWithSuccess.Request, response:EmptyWithSuccess.Response):
        
        try:
            self._logger.warning(f"Starting calibration 'calibrate_calibration_cube_z_on_confocal_top'...")

            move_confocal_success, move_msg = self.pm_robot_utils.move_confocal_top_to_frame(frame_name='Calibration_Cube_Bottom',
                                                                                   z_offset=0.05)
            
            if not move_confocal_success:
                raise PmRobotError(f"Moving confocal top to frame failed: {move_msg}")

            move_confocal_success, move_msg = self.pm_robot_utils.move_confocal_top_to_frame(frame_name='Calibration_Cube_Bottom',
                                                                                   z_offset=0.002)

            if not move_confocal_success:
                raise PmRobotError(f"Moving confocal top to frame failed: {move_msg}")

            step_inc = 0.5
            x, y, final_z = self.pm_robot_utils.interative_sensing(measurement_method=self.pm_robot_utils.get_confocal_top_measurement,
                    measurement_valid_function = self.pm_robot_utils.check_confocal_top_measurement_in_range,
                    length = (0.0, 0.0, 4.0),
                    step_inc = step_inc,
                    total_time = 2.0)
                
            if x is None:
                raise PmRobotError(f"Laser measurement not valid! OUT OF RANGE")

            confocal_measurement = self.pm_robot_utils.get_confocal_top_measurement('m')

            self.pm_robot_utils.send_xyz_trajectory_goal_relative(0.0, 0.0, -confocal_measurement, 1.0)

            confocal_transform = self.pm_robot_utils.get_transform_for_frame(frame_name=self.pm_robot_utils.TCP_CONFOCAL_TOP,
                                                                parent_frame='Calibration_Cube_Bottom')
            
            result_transform =Transform()
            result_transform.translation.z = confocal_transform.translation.z

            save_success = self.save_joint_config ( joint_name='Calibration_Cube_Bottom_Joint',
                                        rel_transformation=result_transform)

            if not save_success:
                raise PmRobotError(f"Failed to save joint configuration for 'Calibration_Cube_Bottom_Joint'")

            self.log_calibration(file_name='calibrate_calibration_cube_z_on_confocal_top',
                                calibration_dict=self._transform_to_dict(result_transform))
            
            response.success = True

        except PmRobotError as e:
            self._logger.error(f"Error occurred while calibrating: {e}")
            response.success = False
        
        finally:
            # move up
            self.pm_robot_utils.send_xyz_trajectory_goal_relative(0.0, 0.0, -0.05, 1.0)
            pass

        return response

    # Service implementation    
    # def calibrate_smarpod(self, request:EmptyWithSuccess.Request, response:EmptyWithSuccess.Response):
    #     calibration_data = []

    #     move_up = False

    #     use_confocal_top = True

    #     if use_confocal_top:
    #         correction_method = self.correct_frame_confocal_top
    #     else:
    #         correction_method = self.correct_frame_laser

    #     try:
    #         self._logger.warning(f"Starting calibration 'calibrate_smarpod'...")
            
            
    #         if self.pm_robot_utils.get_mode() == self.pm_robot_utils.REAL_MODE:
    #             self.pm_robot_utils.pm_robot_config.set_to_real_HW()
    #             self._logger.info(f"Using real hardware bringup configuration for smarpod calibration.")
    #         else:
    #             self.pm_robot_utils.pm_robot_config.set_to_simulation_HW()
    #             self._logger.info(f"Using simulation hardware bringup configuration for smarpod calibration.")

            
    #         if not (self.pm_robot_utils.pm_robot_config.smarpod_station.get_activate_status()):
    #             raise PmRobotError("Smarpod station is not activated in the configuration!")
            
    #         if (self.pm_robot_utils.pm_robot_config.smarpod_station.get_current_chuck_center() != "empty" and
    #             self.pm_robot_utils.pm_robot_config.smarpod_station.get_current_chuck() != "empty"):
    #              raise PmRobotError("Smarpod station has already a chuck and a chuck center assigned. Please remove them before calibrating the smarpod station!") 
            

    #         self.spawn_calibration_frames('CF_Smarpod_Calibration.json')

    #         unique_identifier = self.get_unique_identifier('CF_Smarpod_Calibration.json')

    #         # we can hardcode the frame names here, because this hopefully never changes

    #         vision_frame_name_1 = f"{unique_identifier}Vision_1"
    #         vision_frame_name_2 = f"{unique_identifier}Vision_2"
    #         vision_frame_name_3 = f"{unique_identifier}Vision_3"

    #         laser_frame_1 = f"{unique_identifier}Laser_1"
    #         laser_frame_2 = f"{unique_identifier}Laser_2"
    #         laser_frame_3 = f"{unique_identifier}Laser_3"
    #         laser_frame_4 = f"{unique_identifier}Laser_4"

    #         laser_frame_1_initial = f"{unique_identifier}Laser_1_initial"
    #         laser_frame_2_initial = f"{unique_identifier}Laser_2_initial"
    #         laser_frame_3_initial = f"{unique_identifier}Laser_3_initial"
    #         laser_frame_4_initial = f"{unique_identifier}Laser_4_initial"

    #         laser_frames = [
    #             laser_frame_1,
    #             laser_frame_2,
    #             laser_frame_3,
    #             laser_frame_4]
            
    #         laser_initial_frames = [
    #             laser_frame_1_initial,
    #             laser_frame_2_initial,
    #             laser_frame_3_initial,
    #             laser_frame_4_initial]

    #         ###  MOVE HEXAPOD TO ZERO
    #         self.pm_robot_utils.send_smarpod_trajectory_goal_absolut(x_joint=0.0, y_joint=0.0, z_joint=0.0, time=1.0)

            
    #         #self.pm_robot_utils.send_smarpod_trajectory_goal_relative(x_joint_rel=0.001, y_joint_rel=0.001, z_joint_rel=0.0, time=1.0)

    #         if use_confocal_top:
    #             self.pm_robot_utils.move_confocal_top_to_frame(frame_name=laser_frame_1, z_offset=0.01)
    #         else:
    #             self.pm_robot_utils.move_laser_to_frame(frame_name=laser_frame_1, z_offset=0.01)

    #         move_up = True
            
    #         for l_frame_name in laser_initial_frames:
    #             res = correction_method(frame_id=l_frame_name,
    #                                            remeasure_after_correction=True)
                                
    #             cv = res.correction_values.z
    #             self._logger.warning(f"Correction value for frame '{l_frame_name}' is z: {cv*1e6} um")         

    #         for l_frame_name in laser_frames:
    #             res = correction_method(frame_id=l_frame_name)
    #             cv = res.correction_values.z
    #             self._logger.warning(f"Correction value for frame '{l_frame_name}' is z: {cv*1e6} um")         
            
    #         def collect_frame_data(angle, pose_name, rx_cmd, ry_cmd):
    #             frame_data = {}
                
    #             pose_id = f"{pose_name}_rx{rx_cmd}_ry{ry_cmd}"

    #             for index, frame in enumerate(laser_frames):
                
    #                 res = correction_method(
    #                     frame_id=frame,
    #                     remeasure_after_correction=False,
    #                     use_iterative_sensing=True,
    #                 )
    #                 cv = res.correction_values.z

    #                 time.sleep(1.0)

    #                 trans_fixed_point = self.pm_robot_utils.get_transform_for_frame(
    #                     frame_name=frame,
    #                     parent_frame="SmarPod_Origin",
    #                 )

    #                 trans_rot_point = self.pm_robot_utils.get_transform_for_frame(
    #                     frame_name=frame,
    #                     parent_frame="Smarpod_Top_Plate",
    #                 )
                    
    #                 trans_to_initial_point = self.pm_robot_utils.get_transform_for_frame(
    #                     frame_name=f"CAL_Smarpod_Laser_{index + 1}",
    #                     parent_frame=f"CAL_Smarpod_Laser_{index + 1}_initial",
    #                 )
    #                 trans_static = self.pm_robot_utils.get_transform_for_frame(
    #                     frame_name="Smarpod_Top_Plate",
    #                     parent_frame="SmarPod_Origin",
    #                 )

    #                 frame_data[frame] = {
    #                     "correction_z_um": cv * 1e6,
    #                     "transform_fixed": self._transform_to_dict(trans_fixed_point),
    #                     "transform_rot": self._transform_to_dict(trans_rot_point),
    #                     "transform_to_initial": self._transform_to_dict(trans_to_initial_point),
    #                     "transform_static": self._transform_to_dict(trans_static),
    #                 }

    #                 self._logger.warning(
    #                     f"{frame} | pose={pose_id} | rx={rx_cmd} | ry={ry_cmd} | z={cv*1e6:.2f} um"
    #                 )

    #             return {
    #                 "angle": angle,
    #                 "pose_id": pose_id,
    #                 "rx_cmd": rx_cmd,
    #                 "ry_cmd": ry_cmd,
    #                 "frames": frame_data,
    #             }

    #         #angles = [0.5, 1.0]
    #         #angles = [0.5, 1.0, 1.5, 2.0]
    #         angles = [0.5, 1.0]

    #         pose_sequence = [
    #             ("rx", 1, 0),
    #             ("rx", -1, 0),
    #             ("ry", 0, 1),
    #             ("ry", 0, -1),
    #             ("rxry", 1, 1),
    #             ("rxry", -1, -1),
    #             ("rxry", 1, -1),
    #             ("rxry", -1, 1),
    #         ]

    #         total_iterations = len(angles) * len(pose_sequence)
    #         current_iteration = 0

    #         for angle in angles:
    #             for pose_name, rx, ry in pose_sequence:
    #                 self._logger.warning(
    #                     f"Starting iteration {current_iteration + 1}/{total_iterations}"
    #                 )   

    #                 rx_cmd = angle * rx
    #                 ry_cmd = angle * ry

    #                 # the hexapod takes quite a while to move. so ~4 sec is mandatory.
    #                 self.pm_robot_utils.send_smarpod_trajectory_goal_absolut(
    #                     x_joint=0.0,
    #                     y_joint=0.0,
    #                     z_joint=0.0,
    #                     rx_joint_deg=rx_cmd,
    #                     ry_joint_deg=ry_cmd,
    #                     rz_joint_deg=0,
    #                     time=4.0,
    #                 )

    #                 time.sleep(2.0)

    #                 calibration_data.append(
    #                     collect_frame_data(angle, pose_name, rx_cmd, ry_cmd)
    #                 )
    #                 current_iteration += 1
            
    #         response.success = True

    #     except PmRobotError as e:
    #         self._logger.error(f"Error occurred while calibrating smarpod: {e}")
    #         response.success = False
        
    #     finally:
    #         # save the calibration data
    #         filename = f"{self.calibration_log_dir}calibration_data_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.json"

    #         with open(filename, "w") as f:
    #             json.dump(calibration_data, f, indent=2)

    #         self._logger.info(f"Calibration data saved to {filename}")

    #         if move_up:
    #             self.pm_robot_utils.send_xyz_trajectory_goal_relative(0.0, 0.0, -0.05, 1.0)
    #         # Move smarpod back to zero position
    #         self.pm_robot_utils.send_smarpod_trajectory_goal_absolut(x_joint=0.0, y_joint=0.0, z_joint=0.0, time=2.0)
        
    #     return response
    
    async def calibrate_smarpod(self, goal_handle: skills_action.SmarpodCalibration.Goal):

        calibration_data = []
        calibration_run_timestamp = datetime.datetime.now().isoformat()
        calibration_goal_inputs = {
            "use_confocal_over_laser": bool(goal_handle.request.use_confocal_over_laser),
        }
        
        result = skills_action.SmarpodCalibration.Result()
        result.success = False

        move_up = False

        use_confocal_top = goal_handle.request.use_confocal_over_laser
        remeasure_after_correction = False

        if use_confocal_top:
            correction_method = self.correct_frame_confocal_top
        else:
            correction_method = self.correct_frame_laser

        try:
            self._logger.warning(f"Starting calibration 'calibrate_smarpod'...")
            
            
            if self.pm_robot_utils.get_mode() == self.pm_robot_utils.REAL_MODE:
                self.pm_robot_utils.pm_robot_config.set_to_real_HW()
                self._logger.info(f"Using real hardware bringup configuration for smarpod calibration.")
            else:
                self.pm_robot_utils.pm_robot_config.set_to_simulation_HW()
                self._logger.info(f"Using simulation hardware bringup configuration for smarpod calibration.")

            
            if not (self.pm_robot_utils.pm_robot_config.smarpod_station.get_activate_status()):
                raise PmRobotError("Smarpod station is not activated in the configuration!")
            
            if (self.pm_robot_utils.pm_robot_config.smarpod_station.get_current_chuck_center() != "empty" and
                self.pm_robot_utils.pm_robot_config.smarpod_station.get_current_chuck() != "empty"):
                 raise PmRobotError("Smarpod station has already a chuck and a chuck center assigned. Please remove them before calibrating the smarpod station!") 
            

            self.spawn_calibration_frames('CF_Smarpod_Calibration.json')

            unique_identifier = self.get_unique_identifier('CF_Smarpod_Calibration.json')

            # we can hardcode the frame names here, because this hopefully never changes

            vision_frame_name_1 = f"{unique_identifier}Vision_1"
            vision_frame_name_2 = f"{unique_identifier}Vision_2"
            vision_frame_name_3 = f"{unique_identifier}Vision_3"

            laser_frame_1 = f"{unique_identifier}Laser_1"
            laser_frame_2 = f"{unique_identifier}Laser_2"
            laser_frame_3 = f"{unique_identifier}Laser_3"
            laser_frame_4 = f"{unique_identifier}Laser_4"

            laser_frame_1_initial = f"{unique_identifier}Laser_1_initial"
            laser_frame_2_initial = f"{unique_identifier}Laser_2_initial"
            laser_frame_3_initial = f"{unique_identifier}Laser_3_initial"
            laser_frame_4_initial = f"{unique_identifier}Laser_4_initial"

            laser_frames = [
                laser_frame_1,
                laser_frame_2,
                laser_frame_3,
                laser_frame_4]
            
            laser_initial_frames = [
                laser_frame_1_initial,
                laser_frame_2_initial,
                laser_frame_3_initial,
                laser_frame_4_initial]

            ###  MOVE HEXAPOD TO ZERO
            self.pm_robot_utils.send_smarpod_trajectory_goal_absolut(x_joint=0.0, y_joint=0.0, z_joint=0.0, time=1.0)

            
            #self.pm_robot_utils.send_smarpod_trajectory_goal_relative(x_joint_rel=0.001, y_joint_rel=0.001, z_joint_rel=0.0, time=1.0)

            if use_confocal_top:
                self.pm_robot_utils.move_confocal_top_to_frame(frame_name=laser_frame_1, z_offset=0.01)
            else:
                self.pm_robot_utils.move_laser_to_frame(frame_name=laser_frame_1, z_offset=0.01)

            move_up = True
            
            for l_frame_name in laser_initial_frames:
                res = correction_method(frame_id=l_frame_name,
                                        remeasure_after_correction=remeasure_after_correction)
                                
                cv = res.correction_values.z
                self._logger.warning(f"Correction value for frame '{l_frame_name}' is z: {cv*1e6} um")         

            for l_frame_name in laser_frames:
                res = correction_method(frame_id=l_frame_name, remeasure_after_correction=remeasure_after_correction)
                cv = res.correction_values.z
                self._logger.warning(f"Correction value for frame '{l_frame_name}' is z: {cv*1e6} um")         
            
            def collect_frame_data(angle, pose_name, rx_cmd, ry_cmd, rz_cmd, x_cmd, y_cmd):
                frame_data = {}
                
                pose_id = f"{pose_name}_rx{rx_cmd}_ry{ry_cmd}_rz{rz_cmd}_x{x_cmd}_y{y_cmd}"

                for index, frame in enumerate(laser_frames):
                    
                    if goal_handle.is_cancel_requested:
                        self._logger.warning("Calibration cancelled.")
                        raise PmRobotError("Calibration cancelled.")
        
                    res = correction_method(
                        frame_id=frame,
                        remeasure_after_correction=False,
                        use_iterative_sensing=True,
                    )
                    cv = res.correction_values.z
                    
                    time.sleep(0.5)

                    trans_fixed_point = self.pm_robot_utils.get_transform_for_frame(
                        frame_name=frame,
                        parent_frame="SmarPod_Origin",
                    )

                    trans_rot_point = self.pm_robot_utils.get_transform_for_frame(
                        frame_name=frame,
                        parent_frame="Smarpod_Top_Plate",
                    )
                    
                    trans_to_initial_point = self.pm_robot_utils.get_transform_for_frame(
                        frame_name=f"CAL_Smarpod_Laser_{index + 1}",
                        parent_frame=f"CAL_Smarpod_Laser_{index + 1}_initial",
                    )
                    trans_static = self.pm_robot_utils.get_transform_for_frame(
                        frame_name="Smarpod_Top_Plate",
                        parent_frame="SmarPod_Origin",
                    )

                    frame_data[frame] = {
                        "correction_z_um": cv * 1e6,
                        "transform_fixed": self._transform_to_dict(trans_fixed_point),
                        "transform_rot": self._transform_to_dict(trans_rot_point),
                        "transform_to_initial": self._transform_to_dict(trans_to_initial_point),
                        "transform_static": self._transform_to_dict(trans_static),
                    }

                    self._logger.warning(
                        f"{frame} | pose={pose_id} | rx={rx_cmd} | ry={ry_cmd} | rz={rz_cmd} | x={x_cmd} | y={y_cmd} | z={cv*1e6:.2f} um"
                    )

                return {
                    "angle": angle,
                    "pose_id": pose_id,
                    "rx_cmd": rx_cmd,
                    "ry_cmd": ry_cmd,
                    "rz_cmd": rz_cmd,
                    "x_cmd": x_cmd,
                    "y_cmd": y_cmd,
                    "frames": frame_data,
                }
            
            # do not do more than 1.8 degrees, as the gripper might collide with the calibration wafer when using the laser for measuring
            #angles = [0.5, 1.0]
            #angles = [0.5, 1.0, 1.5, 2.0]
            angles = [1.0, 1.8]

            pose_sequence = [
                ("rx", 1, 0),
                ("rx", -1, 0),
                ("ry", 0, 1),
                ("ry", 0, -1),
                ("rxry", 1, 1),
                ("rxry", -1, -1),
                ("rxry", 1, -1),
                ("rxry", -1, 1),
            ]
            
            rz_angles =[0]

            # in mm
            x_positions = [0, 2]
            y_positions = [0, 2]

            total_iterations = len(angles) * len(pose_sequence) * len(rz_angles) * len(x_positions) * len(y_positions)
            current_iteration = 0

            for x_pos in x_positions:
                for y_pos in y_positions:
                    for rz_angle in rz_angles:
                        for angle in angles:
                            for pose_name, rx, ry in pose_sequence:
                                self._logger.warning(
                                    f"Starting iteration {current_iteration + 1}/{total_iterations}"
                                )   

                                rx_cmd = angle * rx
                                ry_cmd = angle * ry
                                rz_cmd = rz_angle
                                x_cmd = x_pos
                                y_cmd = y_pos

                                # the hexapod takes quite a while to move. so ~4 sec is mandatory.
                                self.pm_robot_utils.send_smarpod_trajectory_goal_absolut(
                                    x_joint=x_cmd*1e-3,
                                    y_joint=y_cmd*1e-3,
                                    z_joint=0.0,
                                    rx_joint_deg=rx_cmd,
                                    ry_joint_deg=ry_cmd,
                                    rz_joint_deg=rz_cmd,
                                    time=4.0,
                                )

                                time.sleep(2.0)

                                calibration_data.append(
                                    collect_frame_data(angle = angle, 
                                                       pose_name = pose_name, 
                                                       rx_cmd = rx_cmd, 
                                                       ry_cmd = ry_cmd, 
                                                       rz_cmd = rz_cmd,
                                                       x_cmd = x_cmd,
                                                       y_cmd = y_cmd)
                                )
                                current_iteration += 1
            
            result.success = True
            goal_handle.succeed()
        
        except PmRobotError as e:
            self._logger.error(f"Error occurred while calibrating smarpod: {e}")
            result.success = False

        except CancelCalibrationException as e:
            self._logger.warning("Calibration cancelled.")
            result.success = False
            goal_handle.canceled()

        finally:
            # save the calibration data
            filename = f"{self.calibration_log_dir}calibration_data_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
            try:
                calibration_output = {
                    "timestamp": calibration_run_timestamp,
                    "goal_handle": calibration_goal_inputs,
                    "calibration_data": calibration_data,
                }
                with open(filename, "w") as f:
                    json.dump(calibration_output, f, indent=2)
                self._logger.info(f"Calibration data saved to {filename}")
            except Exception as e:
                self._logger.error(f"Could not save calibration data: {e}")

            if move_up:
                self.pm_robot_utils.send_xyz_trajectory_goal_relative(0.0, 0.0, -0.05, 1.0)

            # Move smarpod back to zero position
            self.pm_robot_utils.send_smarpod_trajectory_goal_absolut(x_joint=0.0, y_joint=0.0, z_joint=0.0, time=2.0)
        
        return result


    def _get_sensor_transform(self, use_confocal_top):
        frame = (
            self.pm_robot_utils.TCP_CONFOCAL_TOP
            if use_confocal_top
            else self.pm_robot_utils.TCP_LASER
        )

        return self.pm_robot_utils.get_transform_for_frame(
            frame_name=frame,
            parent_frame="SmarPod_Origin",
        )
    
    def _get_measurement(self, use_confocal_top):
        if use_confocal_top:
            if not self.pm_robot_utils.check_confocal_top_measurement_in_range():
                raise PmRobotMeasurementError("Confocal top measurement is out of range.")
            return self.pm_robot_utils.get_confocal_top_measurement(unit="mm")

        if not self.pm_robot_utils._check_for_valid_laser_measurement():
            raise PmRobotMeasurementError("Laser measurement is out of range.")

        return self.pm_robot_utils.get_laser_measurement(unit="mm")

    async def calibrate_smarpod_V2(self, goal_handle: skills_action.SmarpodCalibration.Goal):
        
        goal = goal_handle.request
        use_confocal_top = goal.use_confocal_over_laser

        calibration_data = []
        distances_dict = {}
        calibration_run_timestamp = datetime.datetime.now().isoformat()
        calibration_goal_inputs = {
            "use_confocal_over_laser": bool(use_confocal_top),
        }
        
        result = skills_action.SmarpodCalibration.Result()
        result.success = False

        move_up = False

        remeasure_after_correction = False

        if use_confocal_top:
            correction_method = self.correct_frame_confocal_top
        else:
            correction_method = self.correct_frame_laser

        try:
            self._logger.warning(f"Starting calibration 'calibrate_smarpod'...")
            
            
            if self.pm_robot_utils.get_mode() == self.pm_robot_utils.REAL_MODE:
                self.pm_robot_utils.pm_robot_config.set_to_real_HW()
                self._logger.info(f"Using real hardware bringup configuration for smarpod calibration.")
            else:
                self.pm_robot_utils.pm_robot_config.set_to_simulation_HW()
                self._logger.info(f"Using simulation hardware bringup configuration for smarpod calibration.")

            
            if not (self.pm_robot_utils.pm_robot_config.smarpod_station.get_activate_status()):
                raise PmRobotError("Smarpod station is not activated in the configuration!")
            
            if (self.pm_robot_utils.pm_robot_config.smarpod_station.get_current_chuck_center() != "empty" and
                self.pm_robot_utils.pm_robot_config.smarpod_station.get_current_chuck() != "empty"):
                 raise PmRobotError("Smarpod station has already a chuck and a chuck center assigned. Please remove them before calibrating the smarpod station!") 
            

            self.spawn_calibration_frames('CF_Smarpod_Calibration.json')

            unique_identifier = self.get_unique_identifier('CF_Smarpod_Calibration.json')

            # we can hardcode the frame names here, because this hopefully never changes

            vision_frame_name_1 = f"{unique_identifier}Vision_1"
            vision_frame_name_2 = f"{unique_identifier}Vision_2"
            vision_frame_name_3 = f"{unique_identifier}Vision_3"

            laser_frame_1 = f"{unique_identifier}Laser_1"
            laser_frame_2 = f"{unique_identifier}Laser_2"
            laser_frame_3 = f"{unique_identifier}Laser_3"
            laser_frame_4 = f"{unique_identifier}Laser_4"

            laser_frame_1_initial = f"{unique_identifier}Laser_1_initial"
            laser_frame_2_initial = f"{unique_identifier}Laser_2_initial"
            laser_frame_3_initial = f"{unique_identifier}Laser_3_initial"
            laser_frame_4_initial = f"{unique_identifier}Laser_4_initial"

            laser_frames = [
                laser_frame_1,
                laser_frame_2,
                laser_frame_3,
                laser_frame_4]
            
            laser_initial_frames = [
                laser_frame_1_initial,
                laser_frame_2_initial,
                laser_frame_3_initial,
                laser_frame_4_initial]

            ###  MOVE HEXAPOD TO ZERO
            self.pm_robot_utils.send_smarpod_trajectory_goal_absolut(x_joint=0.0, y_joint=0.0, z_joint=0.0, time=1.0)

            #self.pm_robot_utils.send_smarpod_trajectory_goal_relative(x_joint_rel=0.001, y_joint_rel=0.001, z_joint_rel=0.0, time=1.0)

            if use_confocal_top:
                self.pm_robot_utils.move_confocal_top_to_frame(frame_name=laser_frame_1, z_offset=0.01)
            else:
                self.pm_robot_utils.move_laser_to_frame(frame_name=laser_frame_1, z_offset=0.01)

            move_up = True
                  
            for l_frame_name in laser_frames:
                res = correction_method(frame_id=l_frame_name, remeasure_after_correction=remeasure_after_correction)
                cv = res.correction_values.z
                self._logger.warning(f"Correction value for frame '{l_frame_name}' is z: {cv*1e6} um")         
            
            poses = []

            for frame in laser_frames:
                transform = self.pm_robot_utils.get_transform_for_frame(
                    frame_name=frame,
                    parent_frame="world",
                    )
                pose = Pose()
                pose.position.x = transform.translation.x
                pose.position.y = transform.translation.y
                pose.position.z = transform.translation.z
                poses.append(pose)

            pose_1 = poses[0]
            pose_2 = poses[1]
            pose_3 = poses[2]
            pose_4 = poses[3]

            distances_dict = {}
            distance_x_12 = pose_2.position.x - pose_1.position.x
            distance_y_12 = pose_2.position.y - pose_1.position.y
            distance_z_12 = pose_2.position.z - pose_1.position.z
            distance_12 = math.sqrt(distance_x_12**2 + distance_y_12**2 + distance_z_12**2)
            distance_x_23 = pose_3.position.x - pose_2.position.x
            distance_y_23 = pose_3.position.y - pose_2.position.y
            distance_z_23 = pose_3.position.z - pose_2.position.z
            distance_23 = math.sqrt(distance_x_23**2 + distance_y_23**2 + distance_z_23**2)
            distance_x_34 = pose_4.position.x - pose_3.position.x
            distance_y_34 = pose_4.position.y - pose_3.position.y
            distance_z_34 = pose_4.position.z - pose_3.position.z
            distance_34 = math.sqrt(distance_x_34**2 + distance_y_34**2 + distance_z_34**2)
            distance_x_41 = pose_1.position.x - pose_4.position.x
            distance_y_41 = pose_1.position.y - pose_4.position.y
            distance_z_41 = pose_1.position.z - pose_4.position.z
            distance_41 = math.sqrt(distance_x_41**2 + distance_y_41**2 + distance_z_41**2)          
            
            distances_dict = {
                "distance_x_12_mm": distance_x_12 * 1e3,
                "distance_y_12_mm": distance_y_12 * 1e3,
                "distance_z_12_mm": distance_z_12 * 1e3,
                "distance_12_mm": distance_12 * 1e3,
                "distance_x_23_mm": distance_x_23 * 1e3,
                "distance_y_23_mm": distance_y_23 * 1e3,
                "distance_z_23_mm": distance_z_23 * 1e3,
                "distance_23_mm": distance_23 * 1e3,
                "distance_x_34_mm": distance_x_34 * 1e3,
                "distance_y_34_mm": distance_y_34 * 1e3,
                "distance_z_34_mm": distance_z_34 * 1e3,
                "distance_34_mm": distance_34 * 1e3,
                "distance_x_41_mm": distance_x_41 * 1e3,
                "distance_y_41_mm": distance_y_41 * 1e3,
                "distance_z_41_mm": distance_z_41 * 1e3,
                "distance_41_mm": distance_41 * 1e3,
            }

            # do not do more than 1.8 degrees, as the gripper might collide with the calibration wafer when using the laser for measuring
            #angles = [0.5, 1.0]
            #angles = [0.5, 1.0, 1.5, 2.0]
            angles = [0.5, 0.8]

            pose_sequence = [
                ("rx", 1, 0),
                ("rx", -1, 0),
                ("ry", 0, 1),
                ("ry", 0, -1),
                ("rxry", 1, 1),
                ("rxry", -1, -1),
                ("rxry", 1, -1),
                ("rxry", -1, 1),
            ]
            
            rz_angles =[0]

            # in mm
            x_positions = [0, 2]
            y_positions = [0, 2]

            total_iterations = len(angles) * len(pose_sequence) * len(rz_angles) * len(x_positions) * len(y_positions) * 4
            current_iteration = 0

            for x_pos in x_positions:
                for y_pos in y_positions:
                    for pose in poses:
                        
                        _pose = copy.deepcopy(pose)
                        _pose.position.x += x_pos * 1e-3
                        _pose.position.y += y_pos * 1e-3

                        move_request = MoveToPose.Request()
                        move_request.move_to_pose = _pose
                        move_request.execute_movement = True
                        
                        self._logger.warning(f"Moving smarpod to the next pose on the wafer...")
                        
                        if use_confocal_top:
                            self.pm_robot_utils.move_confocal_top_to_pose(move_request)
                        else:
                            self.pm_robot_utils.move_laser_to_pose(move_request)

                        for pose_name, rx, ry in pose_sequence:
                            for angle in angles:

                                if goal_handle.is_cancel_requested:
                                    self._logger.warning("Calibration cancelled.")
                                    raise PmRobotError("Calibration cancelled.")

                                self._logger.warning(
                                    f"Starting iteration {current_iteration + 1}/{total_iterations}"
                                )   

                                rx_cmd = angle * rx
                                ry_cmd = angle * ry
                                x_cmd = x_pos
                                y_cmd = y_pos

                                pose_id = f"rx{rx_cmd}_ry{ry_cmd}_x{x_cmd}_y{y_cmd}"

                                # the hexapod takes quite a while to move. so ~4 sec is mandatory.
                                self.pm_robot_utils.send_smarpod_trajectory_goal_absolut(
                                    x_joint=x_cmd*1e-3,
                                    y_joint=y_cmd*1e-3,
                                    z_joint=0.0,
                                    rx_joint_deg=rx_cmd,
                                    ry_joint_deg=ry_cmd,
                                    rz_joint_deg=0,
                                    time=2.0,
                                )
                                
                                time.sleep(2.0)

                                measurement = self._get_measurement(use_confocal_top)

                                self._logger.warning(f"Measurement at pose={pose_id} | measurement={measurement:.3f} mm"
                                    )
                                time.sleep(0.5)

                                trans_endeffector = self._get_sensor_transform(use_confocal_top)

                                current_mes_dict = {
                                    "pose_id": pose_id,
                                    "angle": angle,
                                    "rx_cmd": rx_cmd,
                                    "ry_cmd": ry_cmd,
                                    "x_cmd": x_cmd,
                                    "y_cmd": y_cmd,
                                    "measurement_mm": measurement,
                                    "transform_endeffector": self._transform_to_dict(trans_endeffector),
                                    "current_iteration": current_iteration + 1,
                                }

                                calibration_data.append(current_mes_dict)
                                current_iteration += 1

            
            result.success = True
            goal_handle.succeed()
        
        except PmRobotError as e:
            self._logger.error(f"Error occurred while calibrating smarpod: {e}")
            result.success = False

        except CancelCalibrationException as e:
            self._logger.warning("Calibration cancelled.")
            result.success = False
            goal_handle.canceled()

        finally:
            # save the calibration data
            filename = f"{self.calibration_log_dir}calibration_data_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
            try:
                calibration_output = {
                    "timestamp": calibration_run_timestamp,
                    "goal_handle": calibration_goal_inputs,
                    "calibration_data": calibration_data,
                    "distances": distances_dict
                }
                if not calibration_data  == []:
                    with open(filename, "w") as f:
                        json.dump(calibration_output, f, indent=2)
                    self._logger.info(f"Calibration data saved to {filename}")
                else:
                    self._logger.info(f"Calibration data empty. No file saved!")
            except Exception as e:
                self._logger.error(f"Could not save calibration data: {e}")

            if move_up:
                self.pm_robot_utils.send_xyz_trajectory_goal_relative(0.0, 0.0, -0.05, 1.0)

            # Move smarpod back to zero position
            self.pm_robot_utils.send_smarpod_trajectory_goal_absolut(x_joint=0.0, y_joint=0.0, z_joint=0.0, time=2.0)
        
        return result


    def spawn_ball_frames(self, 
                          reference_frame:str, 
                          reference_parent_frame: str,
                          ball_diameter_mm = 6.35)->list:
        # spawn the calibration frames
        request = ami_srv.CreateRefFrame.Request()
        name_stem = "CAL_Smarpod_Ball_"
        grid_distance = 1.0 # mm
        z_curve = 0.2 # mm
        frame_name_list = []
        ref_pose:Transform = self.pm_robot_utils.get_transform_for_frame(frame_name=reference_frame,
                                                               parent_frame=reference_parent_frame)

        ref_pose.translation.x

        sequence = [
                (0, 0),
                ( 1, 0),
                (-1, 0),
                (0, 1),
                (0, -1),
                (1, 1),
                (-1, -1),
                (1, -1),
                (-1, 1),
            ]
        
        if not self.client_create_ref_frame.wait_for_service(timeout_sec=1.0):
            raise PmRobotError(f"Client {self.client_create_ref_frame.srv_name} not available...")
        
        for index, entry in enumerate(sequence):
            _name = f"{name_stem}{(index+1)}"
            request.ref_frame.frame_name = _name
            frame_name_list.append(_name)
            request.ref_frame.parent_frame = reference_parent_frame
            request.ref_frame.pose.position.x = ref_pose.translation.x + (entry[0]*grid_distance*1e-3)
            request.ref_frame.pose.position.y = ref_pose.translation.y + (entry[1]*grid_distance*1e-3)
            # case
            mult = 0
            if (abs(entry[0]) >=1) and  (abs(entry[1]) >=1):
                mult = 1
            if ((abs(entry[0]) >=1) and  (abs(entry[1]) ==0) or (abs(entry[0]) ==0) and  (abs(entry[1]) >=1)):
                mult = 1
            if (abs(entry[0]) ==0) and  (abs(entry[1]) == 0):
                mult = 0 
            request.ref_frame.pose.position.z = ref_pose.translation.z + ball_diameter_mm/2*1e-3 - mult * z_curve*1e-3

            spawn_response:ami_srv.CreateRefFrame.Response = self.client_create_ref_frame.call(request)  

            if not spawn_response.success:
                raise PmRobotError("Failed to spawn calibration frames")
            
        return frame_name_list

    def measure_frame_list (self, 
                            frame_names_list: list[str], 
                            use_confocal_top:bool,
                            fixed_frame_name : str,
                            goal_handle):
        
        #move_success = False

        if use_confocal_top:
            correction_method = self.correct_frame_confocal_top
        else:
            correction_method = self.correct_frame_laser
        
        for frame_name in frame_names_list:

            # if use_confocal_top:
            #     move_success, msg = self.pm_robot_utils.move_confocal_top_to_frame(frame_name=frame_name)
            # else:
            #     move_success = self.pm_robot_utils.move_laser_to_frame(frame_name=frame_name)

            # if not move_success:
            #     raise PmRobotError(f"Could not move to frame {frame_name}")

            # measurement_mm = self._get_measurement(use_confocal_top)

            # self._logger.warn(f"Measurement {measurement_mm} mm")

            if goal_handle.is_cancel_requested:
                self._logger.warning("Calibration cancelled.")
                raise PmRobotError("Calibration cancelled.")

            response = correction_method(frame_id=frame_name, use_iterative_sensing=True)

        time.sleep(1)
        
        results = []
        for frame_name in frame_names_list:
            ref_pose:Transform = self.pm_robot_utils.get_transform_for_frame(frame_name=frame_name,
                                                        parent_frame=fixed_frame_name)
            transform_dict = self._transform_to_dict(ref_pose)
            result_dict = {f"{frame_name}": transform_dict}
            results.append(result_dict) 
        
        if not use_confocal_top:
            # move up to ensure hexapod can move feely, but only if laser is used as we have less clearance
            self.pm_robot_utils.send_xyz_trajectory_goal_relative(0.0, 0.0, -0.005, 1.0)

        return results
    
    async def calibrate_smarpod_V3(self, goal_handle: skills_action.SmarpodCalibration.Goal):
        
        goal = goal_handle.request
        use_confocal_top = goal.use_confocal_over_laser

        calibration_data = []
        calibration_run_timestamp = datetime.datetime.now().isoformat()
        calibration_goal_inputs = {
            "use_confocal_over_laser": bool(use_confocal_top),
        }
        
        result = skills_action.SmarpodCalibration.Result()
        result.success = False

        move_up = False

        remeasure_after_correction = False

        if use_confocal_top:
            correction_method = self.correct_frame_confocal_top
        else:
            correction_method = self.correct_frame_laser

        try:
            self._logger.warning(f"Starting calibration 'calibrate_smarpod'...")
            
            
            if self.pm_robot_utils.get_mode() == self.pm_robot_utils.REAL_MODE:
                self.pm_robot_utils.pm_robot_config.set_to_real_HW()
                self._logger.info(f"Using real hardware bringup configuration for smarpod calibration.")
            else:
                self.pm_robot_utils.pm_robot_config.set_to_simulation_HW()
                self._logger.info(f"Using simulation hardware bringup configuration for smarpod calibration.")

            
            if not (self.pm_robot_utils.pm_robot_config.smarpod_station.get_activate_status()):
                raise PmRobotError("Smarpod station is not activated in the configuration!")
            
            self._logger.info(f"Chuck: {self.pm_robot_utils.pm_robot_config.smarpod_station.get_current_chuck()}")


            if (self.pm_robot_utils.pm_robot_config.smarpod_station.get_current_chuck_center() != "empty" and
                self.pm_robot_utils.pm_robot_config.smarpod_station.get_current_chuck() != "empty"):
                 raise PmRobotError("Smarpod station has already a chuck and a chuck center assigned. Please remove them before calibrating the smarpod station!") 
            
            #self.spawn_calibration_frames('CF_Smarpod_Calibration.json')

            #unique_identifier = self.get_unique_identifier('CF_Smarpod_Calibration.json')

            main_frame_name = ""

            current_cal_transfrom:Transform = self.get_current_joint_calibration_transform(
                HexapodConstants.CALIBRATION_FILE_JOINT_NAME
            )
            current_cal_transfrom_dict = self._transform_to_dict(current_cal_transfrom)

            ###  MOVE HEXAPOD TO ZERO
            self.pm_robot_utils.send_smarpod_trajectory_goal_absolut(x_joint=0.0, y_joint=0.0, z_joint=0.0, time=1.0)


            # do not do more than 1.8 degrees, as the gripper might collide with the calibration wafer when using the laser for measuring
            #angles = [0.5, 1.0]
            #angles = [0.5, 1.0, 1.5, 2.0]
            angles = [1.5, 2.5]

            pose_sequence = [
                ("rx", 1, 0),
                ("rx", -1, 0),
                ("ry", 0, 1),
                ("ry", 0, -1),
                #("rxry", 1, 1),
                #("rxry", -1, -1),
                #("rxry", 1, -1),
                #("rxry", -1, 1),
            ]
            
            rz_angles =[0, 2]
            
            # in mm
            #x_positions = [0, 2]
            #y_positions = [0, 2]
            x_positions = [0, 4]
            y_positions = [0, 3]

            positions = [
                (x, y)
                for x in x_positions
                for y in y_positions
                if not (x > 0 and y > 0)
            ]

            total_iterations = (
                len(positions)
                * len(rz_angles)
                * len(pose_sequence)
                * len(angles)
            )

            current_iteration = 0

            start_z_offset = 5 * 1e-3

            # move to initial state to check if we get a measurment value
            move_success = False

            if use_confocal_top:
                move_success, msg = self.pm_robot_utils.move_confocal_top_to_frame(HexapodConstants.BALL_ENDEFFECTOR, 
                                                                                   z_offset=((HexapodConstants.BALL_DIAMETER/2*1e-3)))
            else:
                move_success = self.pm_robot_utils.move_laser_to_frame(HexapodConstants.BALL_ENDEFFECTOR, 
                                                                       z_offset=((HexapodConstants.BALL_DIAMETER/2*1e-3)))

            if not move_success:
                raise PmRobotError(f"Could not move to desired position!")
            
            try:
                # this will throw an exeption if fails
                measurement_mm = self._get_measurement(use_confocal_top)

                self._logger.warning(f"Initial measurement SUCCESSED! Current value {measurement_mm} mm.")

            except PmRobotMeasurementError as e:
                raise PmRobotError(f"Initial Measurement on calibration ball failed. Make sure the hexapod is already routhgly calibrated so that the distance sensor hits the top of the ball and shows a measurement value of 0.0!")

            for x_pos, y_pos in positions:
                for rz in rz_angles:                
                    
                    #move_success = False

                    # if use_confocal_top:
                    #     move_success, msg = self.pm_robot_utils.move_confocal_top_to_frame(BALL_ENDEFFECTOR, z_offset=((BALL_DIAMETER/2*1e-3)+start_z_offset))
                    # else:
                    #     move_success = self.pm_robot_utils.move_laser_to_frame(BALL_ENDEFFECTOR, z_offset=((BALL_DIAMETER/2*1e-3)+start_z_offset))

                    # if not move_success:
                    #     raise PmRobotError(f"Could not move to desired position!")
                    
                    for pose_name, rx, ry in pose_sequence:
                        for angle in angles:

                            if goal_handle.is_cancel_requested:
                                self._logger.warning("Calibration cancelled.")
                                raise PmRobotError("Calibration cancelled.")

                            self._logger.warning(f"Starting iteration {current_iteration + 1}/{total_iterations}")   

                            rx_cmd = angle * rx
                            ry_cmd = angle * ry
                            rz_cmd = rz
                            x_cmd = x_pos
                            y_cmd = y_pos

                            pose_id = f"rx{rx_cmd}_ry{ry_cmd}_rz{rz_cmd}_x{x_cmd}_y{y_cmd}"

                            # the hexapod takes quite a while to move. so ~4 sec is mandatory.
                            self.pm_robot_utils.send_smarpod_trajectory_goal_absolut(
                                x_joint=x_cmd*1e-3,
                                y_joint=y_cmd*1e-3,
                                z_joint=0.0,
                                rx_joint_deg=rx_cmd,
                                ry_joint_deg=ry_cmd,
                                rz_joint_deg=rz_cmd,
                                time=4.0,
                            )
                            
                            time.sleep(2.0)

                            name_list = self.spawn_ball_frames(reference_frame=HexapodConstants.BALL_ENDEFFECTOR,
                                            reference_parent_frame=HexapodConstants.FIXED_CS_SMARPOD_FRAME,
                                            ball_diameter_mm=HexapodConstants.BALL_DIAMETER)
                            
                            results_list = self.measure_frame_list(frame_names_list=name_list,
                                                    use_confocal_top=use_confocal_top,
                                                    fixed_frame_name=HexapodConstants.FIXED_CS_SMARPOD_FRAME,
                                                    goal_handle = goal_handle)
                            
                            move_up = True
                            
                            current_mes_dict = {
                                "pose_id": pose_id,
                                #"angle": angle,
                                "rx_cmd": rx_cmd,
                                "ry_cmd": ry_cmd,
                                "rz_cmd": rz_cmd,
                                "x_cmd": x_cmd,
                                "y_cmd": y_cmd,
                                "results_list": results_list,
                                "current_iteration": current_iteration + 1,
                            }

                            calibration_data.append(current_mes_dict)
                            current_iteration += 1
            
            result.success = True
            result.message = "Calibration succeded!"
            goal_handle.succeed()
        
        except PmRobotError as e:
            message = f"Error occurred while calibrating smarpod: {e}"
            self._logger.error(message)
            result.success = False
            result.message = message

        except PmRobotMeasurementError as e:
            message = f"Error measuring the calibration ball: {e}"
            self._logger.error(message)
            result.success = False
            result.message = message

        except CancelCalibrationException as e:
            message = f"Calibration cancelled: {e}"
            self._logger.warning(message)
            result.success = False
            result.message = message
            goal_handle.canceled()

        finally:
            # save the calibration data
            Path(self.calibration_log_dir).mkdir(parents=True, exist_ok=True)

            file_path = f"{self.calibration_log_dir}calibration_data_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
            
            try:
                calibration_output = {
                    "timestamp": calibration_run_timestamp,
                    "calibration_fixed_reference_frame": HexapodConstants.FIXED_CS_SMARPOD_FRAME,
                    "calibration_reference_frame": HexapodConstants.CALIBRATED_CS_SMARPOD_FRAME,
                    "current_calibration_transformation": current_cal_transfrom_dict, 
                    "goal_handle": calibration_goal_inputs,
                    "calibration_data": calibration_data,
                }
                if not calibration_data  == []:
                    with open(file_path, "w") as f:
                        json.dump(calibration_output, f, indent=2)
                    self._logger.info(f"Calibration data saved to {file_path}")
                else:
                    self._logger.info(f"Calibration data empty. No file saved!")

            except Exception as e:
                self._logger.error(f"Could not save calibration data: {e}")

            if move_up:
                self.pm_robot_utils.send_xyz_trajectory_goal_relative(0.0, 0.0, -0.05, 1.0)

            # Move smarpod back to zero position
            self.pm_robot_utils.send_smarpod_trajectory_goal_absolut(x_joint=0.0, y_joint=0.0, z_joint=0.0, time=2.0)
        
        return result
    
    def assess_hexapod_calibration(self, request:skills_srv.AssessHexapodCalibration.Request, response:skills_srv.AssessHexapodCalibration.Response):
        
        try:
            sc = SphereCalibration.load_file(request.results_file_path)

            self.get_logger().warn(f"Start calculating the Smarpot Pivot Point!")

            analysis:CalibrationAnalysis = sc.run_calibration()
            
            analysis.save_results(file_path=self.calibration_log_dir)
            
            analysis.plot_results(file_path=self.calibration_log_dir)
            
            euler_angles = analysis.get_B_T_P_euler()

            translation_pivot = analysis.get_B_T_P_translation()

            translation_ball = analysis.get_J_t_P_translation(unit='m')

            B__T__J = analysis.get_B_T_P_ros_transform()

            #self.get_logger().warn(f"euler: {str(euler_angles)}")
            #self.get_logger().warn(f"translation pivot: {str(translation_pivot)}")
            #self.get_logger().warn(f"translation ball: {str(translation_ball)}")
            
            current_transform_sphere = self.get_current_joint_calibration_transform(HexapodConstants.CALIBRATION_FILE_JOINT_NAME_SPHERE)
            
            current_transform_sphere.translation.x = translation_ball[0]
            current_transform_sphere.translation.y = translation_ball[1]
            current_transform_sphere.translation.z = translation_ball[2]

            #self.get_logger().warn(f"current transform: {str(current_transform_sphere)}")
            
            default_ball_transform:Transform = self.pm_robot_utils.get_transform_for_frame(frame_name=HexapodConstants.BALL_ENDEFFECTOR,
                                                            parent_frame=HexapodConstants.SMARPOD_CS_PIVOT_BASE_NAME)
            
            self.get_logger().warn(f"current transform: {str(default_ball_transform)}")


            trans_fixed_calibrated:Transform = self.pm_robot_utils.get_transform_for_frame(frame_name=HexapodConstants.FIXED_CS_SMARPOD_FRAME,
                                                                        parent_frame=HexapodConstants.CALIBRATED_CS_SMARPOD_FRAME)
            
            C__T__J:Transform = self.pm_robot_utils.get_transform_for_frame(frame_name=HexapodConstants.SMARPOD_CS_PIVOT_BASE_NAME,
                                                                        parent_frame=HexapodConstants.CALIBRATED_CS_SMARPOD_FRAME)
            
            #self.get_logger().warn(f"Current Calibration Value: {str(trans_fixed_calibrated)}")
            #self.get_logger().warn(f"Calibration Pivot: {str(C__T__J)}")

            self.save_joint_config(joint_name=HexapodConstants.CALIBRATION_FILE_JOINT_NAME_SPHERE,
                                   rel_transformation=current_transform_sphere,
                                   overwrite=True)
            
            J__T__C = inverse_ros_transform(C__T__J, output_type=Transform)
            B__T__C = multiply_ros_transforms(B__T__J, J__T__C, output_type=Transform)

            self.save_joint_config(joint_name=HexapodConstants.CALIBRATION_FILE_JOINT_NAME,
                        rel_transformation=B__T__C,
                        overwrite=True)
            
            self.get_logger().error(f"Calibration values written successfully to the joint calibration.")

            response.success = True
            
        except FileNotFoundError as e:
            message = (
                "Assessing the hexapod calibration file failed: "
                f"results file does not exist: {request.results_file_path}"
            )
            self.get_logger().error(message)
            self.get_logger().debug(str(e))
            response.message = message
            response.success = False

        except PmRobotError as e:
            message = f"Assessing the hexapod calibration file failed: {e}"
            self.get_logger().error(message)
            response.message = message
            response.success = False

        except Exception as e:
            message = f"Assessing the hexapod calibration file failed unexpectedly: {e}"
            self.get_logger().error(message)
            response.message = message
            response.success = False

        finally:
            pass

        return response
    
    def _goal_calibration_callback(self, goal_request):
        self.get_logger().info(f"Received goal: {str(goal_request)}")
        # Accept all goals
        return GoalResponse.ACCEPT

    def _cancel_calibration_callback(self, goal_handle):
        self.get_logger().info("Cancel request received")
        return CancelResponse.ACCEPT
    
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
        
        request = skills_srv.MeasureFrameVision.Request()
        request.frame_name = frame_id
        response:skills_srv.MeasureFrameVision.Response = self.client_measure_frame_cam.call(request)
        
        return response.success, response.result_vector
    
    def correct_frame_vison(self, frame_id:str)->skills_srv.CorrectFrameVision.Response:
        
        if not self.client_correct_frame_vision.wait_for_service(timeout_sec=1.0):
            self._logger.error('Vision correct frame service not available...')
            return False
        
        request = skills_srv.CorrectFrameVision.Request()
        request.frame_name = frame_id
        response:skills_srv.CorrectFrameVision.Response = self.client_correct_frame_vision.call(request)
        
        return response
    

    def correct_frame_confocal_bottom(self, frame_id:str)->bool:
        
        if not self.client_correct_frame_confocal_bottom.wait_for_service(timeout_sec=1.0):
            self._logger.error(f"Client '{self.client_correct_frame_confocal_bottom.srv_name}' not available...")
            raise PmRobotError(f"Client '{self.client_correct_frame_confocal_bottom.srv_name}' not available...")
        
        request = skills_srv.CorrectFrameLaser.Request()
        request.frame_name = frame_id
        response:skills_srv.CorrectFrameLaser.Response = self.client_correct_frame_confocal_bottom.call(request)
        
        return response.success
    
    def correct_frame_confocal_top(self, frame_id:str, 
                            remeasure_after_correction:bool=False,
                            use_iterative_sensing:bool=False)->skills_srv.CorrectFrameLaser.Response:
        
        if not self.client_correct_frame_with_confocal_top.wait_for_service(timeout_sec=1.0):
            raise PmRobotError(f"Client '{self.client_correct_frame_with_confocal_top.srv_name}' not available...")
        
        request = skills_srv.CorrectFrameLaser.Request()
        request.frame_name = frame_id
        request.remeasure_after_correction = remeasure_after_correction
        request.use_iterative_sensing = use_iterative_sensing
        response:skills_srv.CorrectFrameLaser.Response = self.client_correct_frame_with_confocal_top.call(request)
        
        if not response.success:
            raise PmRobotError(f"Correction of frame '{frame_id}' with confocal top failed!")
        
        return response
    
    def correct_frame_laser(self, frame_id:str, 
                            remeasure_after_correction:bool=False,
                            use_iterative_sensing:bool=False)->skills_srv.CorrectFrameLaser.Response:
        """
        Correct the frame using the laser.
        """
        
        if not self.client_correct_frame_with_laser.wait_for_service(timeout_sec=1.0):
            raise PmRobotError(f"Client '{self.client_correct_frame_with_laser.srv_name}' not available...")
        
        request = skills_srv.CorrectFrameLaser.Request()
        request.frame_name = frame_id
        request.remeasure_after_correction = remeasure_after_correction
        request.use_iterative_sensing = use_iterative_sensing
        response:skills_srv.CorrectFrameLaser.Response = self.client_correct_frame_with_laser.call(request)
        
        if not response.success:
            raise PmRobotError(f"Correction of frame '{frame_id}' with laser failed!")
        
        return response
    
    def modify_pose_from_frame(self, modify_pose_request:ami_srv.ModifyPoseFromFrame.Request)->bool:
        if not self.client_modify_pose_from_frame.wait_for_service(timeout_sec=1.0):
            self._logger.error('Modify pose from frame service not available...')
            return False
        
        request = ami_srv.ModifyPoseFromFrame.Request()
        
        response:ami_srv.ModifyPoseFromFrame.Response = self.client_modify_pose_from_frame.call(request)
        
        return response.success

    def set_calibration_platelet_forward(self):
        """Set the calibration plate forward.

        Raises:
            PmRobotError: If the service is not available or the call fails.
        """
        if not self.pm_robot_utils.client_move_calibration_target_forward.wait_for_service(timeout_sec=1.0):
            raise PmRobotError(f'Client {self.pm_robot_utils.client_move_calibration_target_forward.srv_name} not available...')
        forward_request = EmptyWithSuccess.Request()
        forward_response:EmptyWithSuccess.Response = self.pm_robot_utils.client_move_calibration_target_forward.call(forward_request)

        if not forward_response.success:
            raise PmRobotError("Failed to move calibration target forward")

    def set_calibration_platelet_backward(self):
        """Set the calibration plate backward.

        Raises:
            PmRobotError: If the service is not available or the call fails.
        """
        
        if not self.pm_robot_utils.client_move_calibration_target_backward.wait_for_service(timeout_sec=1.0):
            raise PmRobotError(f'Client {self.pm_robot_utils.client_move_calibration_target_backward.srv_name} not available...')
        backward_request = EmptyWithSuccess.Request()
        backward_response:EmptyWithSuccess.Response = self.pm_robot_utils.client_move_calibration_target_backward.call(backward_request)

        if not backward_response.success:
            raise PmRobotError("Failed to move calibration target backward")
        
        
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
            self._logger.warning(f"Calibration value for 'x' changed significantly ({_calibration_values.translation.x} um). You need to restart the PM Robot!")

        if abs(_calibration_values.translation.y) > threshold:
            self._logger.warning(f"Calibration value for 'y' changed significantly ({_calibration_values.translation.y} um). You need to restart the PM Robot!")

        if abs(_calibration_values.translation.z) > threshold:
            self._logger.warning(f"Calibration value for 'z' changed significantly ({_calibration_values.translation.z} um). You need to restart the PM Robot!")

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
                self._logger.warning(
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

    def get_joint_config_file_path_for_current_mode(self) -> str:
        if self.pm_robot_utils.get_mode() == self.pm_robot_utils.REAL_MODE:
            return self.pm_robot_utils.pm_robot_config.get_joint_config_path(use_real_HW=True)
        elif self.pm_robot_utils.get_mode() == self.pm_robot_utils.UNITY_MODE:
            return self.pm_robot_utils.pm_robot_config.get_joint_config_path(use_real_HW=False)
        else:
            return self.pm_robot_utils.pm_robot_config.get_joint_config_path(use_real_HW=False)

    def _joint_config_to_transform(self, joint_name: str, joint_config: dict) -> Transform:
        required_keys = (
            'x_offset',
            'y_offset',
            'z_offset',
            'rx_offset',
            'ry_offset',
            'rz_offset',
        )

        missing_keys = [key for key in required_keys if key not in joint_config]
        if missing_keys:
            raise PmRobotError(
                f"Joint '{joint_name}' calibration is missing keys: {missing_keys}"
            )

        current_transformation = Transform()
        current_transformation.translation.x = float(joint_config['x_offset']) * 1e-6
        current_transformation.translation.y = float(joint_config['y_offset']) * 1e-6
        current_transformation.translation.z = float(joint_config['z_offset']) * 1e-6

        quat = R.from_euler(
            'xyz',
            [
                float(joint_config['rx_offset']),
                float(joint_config['ry_offset']),
                float(joint_config['rz_offset']),
            ],
            degrees=True,
        ).as_quat()
        current_transformation.rotation.x = quat[0]
        current_transformation.rotation.y = quat[1]
        current_transformation.rotation.z = quat[2]
        current_transformation.rotation.w = quat[3]

        return current_transformation

    def get_current_joint_calibration_transform(self, joint_name: str) -> Transform:
        file_path = self.get_joint_config_file_path_for_current_mode()

        if not os.path.isfile(file_path):
            raise PmRobotError(f"Joint calibration file does not exist: {file_path}")

        with open(file_path, 'r') as file:
            calibration_config = yaml.load(file, Loader=yaml.FullLoader)

        if not isinstance(calibration_config, dict):
            raise PmRobotError(f"Joint calibration file is invalid or empty: {file_path}")

        joint_config = calibration_config.get(joint_name, None)
        if joint_config is None:
            raise PmRobotError(
                f"Joint '{joint_name}' does not exist in calibration file: {file_path}"
            )

        if not isinstance(joint_config, dict):
            raise PmRobotError(
                f"Joint '{joint_name}' calibration entry is invalid in file: {file_path}"
            )

        return self._joint_config_to_transform(joint_name, joint_config)

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
        
        file_path = self.get_joint_config_file_path_for_current_mode()

        self._log_calibration_result(rel_transformation, unit=unit)

        # Archive the original file before modifying it
        self._archive_joint_config_file(file_path)

        calibration_config = {}
        # check if the file exists
        try:
            with open(file_path, 'r') as file:
                calibration_config = yaml.load(file, Loader=yaml.FullLoader)
            
            joint_config = calibration_config.get(joint_name, None)
            current_transformation = Transform()
            current_transformation.rotation.w = 1.0
            
            if joint_config is not None:
                current_transformation = self._joint_config_to_transform(joint_name, joint_config)
            else:
                calibration_config[joint_name] = {}
            #new_transform = get_relative_transform_for_transforms(current_transformation, rel_transformation)
            if not overwrite:
                new_transform = get_relative_transform_for_transforms_calibration(  base_transform = current_transformation, 
                                                                                    additional_transform = rel_transformation)
            else:
                new_transform = rel_transformation
                self._logger.warning(f"Overwriting existing calibration data.")

            calibration_config[joint_name]['x_offset'] = float(new_transform.translation.x * 1e6)
            calibration_config[joint_name]['y_offset'] = float(new_transform.translation.y * 1e6)
            calibration_config[joint_name]['z_offset'] = float(new_transform.translation.z * 1e6)

            q = [float(new_transform.rotation.x),
                 float(new_transform.rotation.y),
                 float(new_transform.rotation.z),
                 float(new_transform.rotation.w)]

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

    def _archive_joint_config_file(self, file_path: str) -> bool:
        """Archive the original joint config file before modifying it.
        
        Creates an archive folder and copies the original file with a timestamp-based name.
        
        Args:
            file_path: Path to the joint configuration file to archive
            
        Returns:
            True if archiving was successful or file didn't exist, False if an error occurred
        """
        try:
            # Only archive if the file exists
            if not os.path.exists(file_path):
                return True
            
            # Create archive directory
            file_dir = os.path.dirname(file_path)
            archive_dir = os.path.join(file_dir, 'archive')
            
            if not os.path.exists(archive_dir):
                os.makedirs(archive_dir)
            
            # Get the base filename without extension
            base_name = os.path.basename(file_path)
            name_without_ext = os.path.splitext(base_name)[0]
            file_ext = os.path.splitext(base_name)[1]
            
            # Create filename with timestamp
            timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
            archive_filename = f"{name_without_ext}_{timestamp}{file_ext}"
            archive_path = os.path.join(archive_dir, archive_filename)
            
            # Copy the file to archive
            shutil.copy2(file_path, archive_path)
            self._logger.info(f"Archived original config file to: {archive_path}")
            
            return True
            
        except Exception as e:
            self._logger.error(f"Error archiving joint config file: {e}")
            return False

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
