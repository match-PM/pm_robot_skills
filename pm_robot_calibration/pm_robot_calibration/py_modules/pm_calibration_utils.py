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


class CancelCalibrationException(Exception):
    pass

class PmRobotCalibrationUtils():
    INFO_TEXT = """
    PM Robot Calibration Node
    This node is responsible for calibrating the PM robot.
    It provides services to calibrate the gripper, dispenser, cameras, and laser.
    """

    def __init__(self, node: Node):
        
        self.node = node
        self._logger = self.node.get_logger()

        self._logger.info(" Pm Robot Calibration Node started...")
        
        self.callback_group = MutuallyExclusiveCallbackGroup()
        self.pm_robot_utils = PmRobotUtils(self.node)
        self.pm_robot_utils.start_object_scene_subscribtion()
        
        # clients
        self.client_spawn_frames_from_description = self.node.create_client(SpawnFramesFromDescription, '/assembly_manager/spawn_frames_from_description')
        self.client_create_ref_frame = self.node.create_client(ami_srv.CreateRefFrame, '/assembly_manager/create_ref_frame')
        self.client_measure_frame_cam = self.node.create_client(skills_srv.MeasureFrameVision, '/pm_skills/vision_measure_frame')
        self.client_correct_frame_vision = self.node.create_client(skills_srv.CorrectFrameVision, '/pm_skills/vision_correct_frame')
        self.client_modify_pose_from_frame = self.node.create_client(ami_srv.ModifyPoseFromFrame, '/assembly_manager/modify_frame_from_frame')
        self.client_calibrate_camera_pixel = self.node.create_client(CalibratePixelPerUm, '/pm_vision_manager/SetCameraPixelPerUm')
        self.client_calibrate_camera_angle = self.node.create_client(CalibrateAngle, '/pm_vision_manager/SetCameraAngle')
        self.client_correct_frame_confocal_bottom = self.node.create_client(skills_srv.CorrectFrameLaser, '/pm_skills/correct_frame_with_confocal_bottom')
        self.client_correct_frame_with_laser = self.node.create_client(skills_srv.CorrectFrameLaser, '/pm_skills/correct_frame_with_laser')
        self.client_correct_frame_with_confocal_top = self.node.create_client(skills_srv.CorrectFrameLaser, '/pm_skills/correct_frame_with_confocal_top')
        
        # paths
        self.calibration_frame_dict_path = get_package_share_directory('pm_robot_description') + '/urdf/urdf_configs/calibration_frame_dictionaries'
        self.calibration_log_dir = get_package_share_directory('pm_robot_calibration') + '/calibration_logs/'

        #self.update_pm_robot_config()
        self._logger.info(self.INFO_TEXT)
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
            self._logger.error('Assembly manager not available...')
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

    
    def _goal_calibration_callback(self, goal_request):
        self._logger.info(f"Received goal: {str(goal_request)}")
        # Accept all goals
        return GoalResponse.ACCEPT

    def _cancel_calibration_callback(self, goal_handle):
        self._logger.info("Cancel request received")
        return CancelResponse.ACCEPT
    
    
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
                
        response:ami_srv.ModifyPoseFromFrame.Response = self.client_modify_pose_from_frame.call(modify_pose_request)
        
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
        sig_change_happend = False

        if abs(_calibration_values.translation.x) > threshold:
            self._logger.warning(f"Calibration value for 'x' changed significantly ({_calibration_values.translation.x} um). You need to restart the PM Robot!")
            sig_change_happend = True

        if abs(_calibration_values.translation.y) > threshold:
            self._logger.warning(f"Calibration value for 'y' changed significantly ({_calibration_values.translation.y} um). You need to restart the PM Robot!")
            sig_change_happend = True

        if abs(_calibration_values.translation.z) > threshold:
            self._logger.warning(f"Calibration value for 'z' changed significantly ({_calibration_values.translation.z} um). You need to restart the PM Robot!")
            sig_change_happend = True

        if not sig_change_happend:
            self._logger.warning(f"No significant change happend to the calibration data. You do NOT need to restart the PM Robot!")

    def _get_translation_difference_transform(self, current_transform: Transform, new_transform: Transform) -> Transform:
        difference = Transform()
        difference.translation.x = new_transform.translation.x - current_transform.translation.x
        difference.translation.y = new_transform.translation.y - current_transform.translation.y
        difference.translation.z = new_transform.translation.z - current_transform.translation.z
        difference.rotation.w = 1.0
        return difference

    def log_calibration(self, 
                        file_name: str,
                        calibration_dict: dict) -> bool:
        calibration_log_dir = self.get_calibration_log_dir_for_current_mode()

        # Ensure the log directory exists
        if not os.path.exists(calibration_log_dir):
            os.makedirs(calibration_log_dir)

        self._calibration_history_log(file_name)

        # Ensure .json extension
        if not file_name.endswith(".json"):
            file_name += ".json"

        file_path = os.path.join(calibration_log_dir, file_name)

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

    def get_calibration_log_dir_for_current_mode(self) -> str:
        if self.pm_robot_utils.get_mode() == self.pm_robot_utils.REAL_MODE:
            real_joint_config_path = self.pm_robot_utils.pm_robot_config.get_joint_config_path(use_real_HW=True)
            return os.path.join(
                os.path.dirname(os.path.abspath(real_joint_config_path)),
                'calibration_logs',
                ''
            )

        return self.calibration_log_dir

    def get_robot_configuration_dir_for_current_mode(self) -> str:
        if self.pm_robot_utils.get_mode() == self.pm_robot_utils.REAL_MODE:
            real_joint_config_path = self.pm_robot_utils.pm_robot_config.get_joint_config_path(use_real_HW=True)
            return os.path.join(os.path.dirname(os.path.abspath(real_joint_config_path)), '')

        return self.calibration_log_dir

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
        
        self._logger.warning(f"Saving joint calibration for joint '{joint_name}'!")

        rel_transformation_m = copy.deepcopy(rel_transformation)
        
        if unit == 'm':
            pass
        elif unit == 'mm':
            rel_transformation_m.translation.x = rel_transformation_m.translation.x * 1e-3
            rel_transformation_m.translation.y = rel_transformation_m.translation.y * 1e-3
            rel_transformation_m.translation.z = rel_transformation_m.translation.z * 1e-3
        elif unit == 'um':
            rel_transformation_m.translation.x = rel_transformation_m.translation.x * 1e-6
            rel_transformation_m.translation.y = rel_transformation_m.translation.y * 1e-6
            rel_transformation_m.translation.z = rel_transformation_m.translation.z * 1e-6
        else:
            self._logger.fatal('Fatal error in calibration program. Invalid unit!')
            return False
        
        file_path = self.get_joint_config_file_path_for_current_mode()

        # Archive the original file before modifying it
        if not self._archive_joint_config_file(file_path):
            self._logger.error("Saving joint calibration aborted because archiving failed.")
            return False

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
                                                                                    additional_transform = rel_transformation_m)
                calibration_change = rel_transformation_m
            else:
                new_transform = rel_transformation_m
                calibration_change = self._get_translation_difference_transform(
                    current_transform=current_transformation,
                    new_transform=new_transform,
                )
                self._logger.warning(f"Overwriting existing calibration data.")

            self._log_calibration_result(calibration_change, unit='m')

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
            timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
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
        path = self.get_calibration_log_dir_for_current_mode()
        file_name = 'last_calibrations.yaml'

        if os.path.exists(path + '/' + file_name):
            with open(path + '/' + file_name, 'r') as file:
                self._last_calibrations_data = yaml.load(file, Loader=yaml.FullLoader)
        else:
            pass

    def _save_last_calibrations_data(self):
        path = self.get_calibration_log_dir_for_current_mode()
        file_name = 'last_calibrations.yaml'

        if not os.path.exists(path):
            os.makedirs(path)

        with open(path + '/' + file_name, 'w') as file:
            yaml.dump(self._last_calibrations_data, file)
            pass
    

    
def main(args=None):
    pass


if __name__ == '__main__':
    main()
