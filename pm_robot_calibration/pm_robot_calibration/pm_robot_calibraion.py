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
from pm_vision_interfaces.srv import ExecuteVision
import pm_vision_interfaces.msg as vision_msg
from geometry_msgs.msg import Vector3, TransformStamped, Pose, PoseStamped, Quaternion, Transform

from scipy.spatial.transform import Rotation as R

import assembly_manager_interfaces.srv as ami_srv
import assembly_manager_interfaces.msg as ami_msg

from assembly_scene_publisher.py_modules.AssemblyScene import AssemblyManagerScene
from assembly_manager_interfaces.srv import SpawnFramesFromDescription
from pm_msgs.srv import EmptyWithSuccess

# import get_package_share_directory
from ament_index_python.packages import get_package_share_directory

from ros_sequential_action_programmer.submodules.pm_robot_modules.widget_pm_robot_config import VacuumGripperConfig, ParallelGripperConfig
from pm_skills.py_modules.PmRobotUtils import PmRobotUtils

TOOL_VACUUM_IDENT = 'pm_robot_vacuum_tools'
TOOL_GRIPPER_1_JAW_IDENT = 'pm_robot_tool_parallel_gripper_1_jaw'
TOOL_GRIPPER_2_JAW_IDENT = 'pm_robot_tool_parallel_gripper_2_jaws'

class PmRobotCalibrationNode(Node):

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
        self.client_move_calibration_target_forward = self.create_client(EmptyWithSuccess, '/pm_pneumatic_controller/Camera_Calibration_Platelet_Joint/MoveForward')
        self.client_move_calibration_target_backward = self.create_client(EmptyWithSuccess, '/pm_pneumatic_controller/Camera_Calibration_Platelet_Joint/MoveBackward')
        
        # services
        self.calibrate_gripper_srv = self.create_service(EmptyWithSuccess, '/pm_robot_calibration/calibrate_gripper', self.calibrate_gripper_callback, callback_group=self.callback_group)
        self.calibrate_dispenser_srv = self.create_service(EmptyWithSuccess, '/pm_robot_calibration/calibrate_1K_dispenser', self.calibrate_1K_dispenser_callback, callback_group=self.callback_group)
        self.calibrate_cameras_srv = self.create_service(EmptyWithSuccess, '/pm_robot_calibration/calibrate_cameras', self.calibrate_cameras_callback, callback_group=self.callback_group)
        
        # paths
        self.bringup_share_path = get_package_share_directory('pm_robot_bringup')
        self.calibration_frame_dict_path = get_package_share_directory('pm_robot_description') + '/urdf/urdf_configs/calibration_frame_dictionaries'
        self.pm_robot_config_path = self.bringup_share_path + '/config/pm_robot_bringup_config.yaml'

        self.update_pm_robot_config()

        # test_transform = Transform()
        # test_transform.translation.x = -0.1
        # test_transform.translation.y = 0.3
        # test_transform.translation.z = 0.1
        # test_transform.rotation.x = 1.0
        # self.save_joint_config('PM_Robot_Tool_TCP_Joint', test_transform)
        
    def update_pm_robot_config(self):
        with open(self.pm_robot_config_path, 'r') as file:
            self.pm_robot_config = yaml.load(file, Loader=yaml.FullLoader)
        self.vacuum_gripper_config = VacuumGripperConfig(TOOL_VACUUM_IDENT, self.pm_robot_config['pm_robot_tools'][TOOL_VACUUM_IDENT])
        self.parallel_gripper_1_jaw_config = ParallelGripperConfig(TOOL_GRIPPER_1_JAW_IDENT, self.pm_robot_config['pm_robot_tools'][TOOL_GRIPPER_1_JAW_IDENT])
        self.parallel_gripper_2_jaw_config = ParallelGripperConfig(TOOL_GRIPPER_2_JAW_IDENT, self.pm_robot_config['pm_robot_tools'][TOOL_GRIPPER_2_JAW_IDENT])
        
    def get_gripper_calibration_frame_dictionary(self)->dict:
        calibration_frame_dict = {}
        file_path = None
        
        if self.vacuum_gripper_config.get_activate_status():
            print("Vacuum gripper calibration frame dictionary")
            file_name = self.vacuum_gripper_config.get_calibration_frame_dict_file_name()
            file_path = self.calibration_frame_dict_path + '/' + file_name 
        
        elif self.parallel_gripper_1_jaw_config.get_activate_status():
            print("Parallel gripper 1 jaw calibration frame dictionary")
            file_name = self.parallel_gripper_1_jaw_config.get_calibration_frame_dict_file_name()
            file_path = self.calibration_frame_dict_path + '/' + file_name
            
        elif self.parallel_gripper_2_jaw_config.get_activate_status():
            print("Parallel gripper 2 jaws calibration frame dictionary")
            file_name = self.parallel_gripper_2_jaw_config.get_calibration_frame_dict_file_name()
            file_path = self.calibration_frame_dict_path + '/' + file_name
            
        # open the file and load the calibration frame dictionary
        try:
            with open(file_path, 'r') as file:
                calibration_frame_dict = yaml.load(file, Loader=yaml.FullLoader)
        
        except Exception as e:
            self._logger.error("Error: " + str(e))
            return {}
        
        return calibration_frame_dict, file_path
    
    def spawn_frames_for_current_gripper(self)->bool:
        if self.gripper_frames_spawned:
            return True
        
        self.update_pm_robot_config()
        calibration_frame_dict, file_path = self.get_gripper_calibration_frame_dictionary()
        if file_path is not None:
            request = SpawnFramesFromDescription.Request()
            request.dict_or_path = file_path
            
            while not self.client_spawn_frames.wait_for_service(timeout_sec=1.0):
                self._logger.error('Assembly manager not available, waiting again...')
            
            self._logger.info('Calibration: Spawning frames for gripper...')
            
            response:SpawnFramesFromDescription.Response = self.client_spawn_frames.call(request=request)
            self.gripper_frames_spawned = True
            return response.success
        else:
            return False    
    
    def calibrate_cameras_callback(self, request:EmptyWithSuccess.Request, response:EmptyWithSuccess.Response):
        forward_request = EmptyWithSuccess.Request()
        forward_response:EmptyWithSuccess.Response = self.client_move_calibration_target_forward.call(forward_request)

        if not forward_response.success:
            self._logger.error("Failed to move calibration target forward")
            response.success = False
            return response
        
        request_move_to_frame = MoveToFrame.Request()
        request_move_to_frame.target_frame = 'Camera_Station_TCP'
        request_move_to_frame.execute_movement = True

        while not self.pm_robot_utils.client_move_robot_cam1_to_frame.wait_for_service(timeout_sec=1.0):
            self._logger.error('Camera move service not available, waiting again...')
            response.success = False
            return response
        
        response_move_to_frame:MoveToFrame.Response = self.pm_robot_utils.client_move_robot_cam1_to_frame.call(request_move_to_frame)
        if not response_move_to_frame.success:
            self._logger.error("Failed to move camera to frame")
            response.success = False
            return response

        # measure frame with bottom cam
        request_execute_vision_bottom = ExecuteVision.Request()
        request_execute_vision_bottom.camera_config_filename = self.pm_robot_utils.get_cam_file_name_bottom()
        request_execute_vision_bottom.image_display_time = 5
        request_execute_vision_bottom.process_filename = "PM_Robot_Calibration/Camera_Calibration_Bottom_Process.json"

        while not self.pm_robot_utils.client_execute_vision.wait_for_service(timeout_sec=1.0):
            self._logger.error('Vision service not available, waiting again...')
            response.success = False
            return response
        
        response_execute_vision_bottom:ExecuteVision.Response = self.pm_robot_utils.client_execute_vision.call(request_execute_vision_bottom)
        
        if not response_execute_vision_bottom.success:
            self._logger.error("Failed to execute vision for bottom camera")
            response.success = False
            return response
        
        # measure frame with bottom cam
        request_execute_vision_top = ExecuteVision.Request()
        request_execute_vision_top.camera_config_filename = self.pm_robot_utils.get_cam_file_name_top()
        request_execute_vision_top.image_display_time = 5
        request_execute_vision_top.process_filename = "PM_Robot_Calibration/Camera_Calibration_Top_Process.json"

        while not self.pm_robot_utils.client_execute_vision.wait_for_service(timeout_sec=1.0):
            self._logger.error('Vision service not available, waiting again...')
            response.success = False
            return response
        
        response_execute_vision_top:ExecuteVision.Response = self.pm_robot_utils.client_execute_vision.call(request_execute_vision_top)

        if not response_execute_vision_top.success:
            self._logger.error("Failed to execute vision for top camera")
            response.success = False
            return response
        
        # measure_request = MeasureFrame.Request()
        # measure_request.frame_name = self.pm_robot_utils.get_cam_file_name_bottom()
        # measure_request.   
        
        backward_request = EmptyWithSuccess.Request()
        backward_response:EmptyWithSuccess.Response = self.client_move_calibration_target_backward.call(backward_request)

        if not backward_response.success:
            self._logger.error("Failed to move calibration target backward")
            response.success = False
            return response
        
        response.success = True
        return response
        


    def calibrate_1K_dispenser_callback(self, request:EmptyWithSuccess.Request, response:EmptyWithSuccess.Response):
        frame = '1K_Dispenser_TCP'
        
        measure_success, result_vector = self.measure_frame(frame)
        
        response.success = measure_success
        return response
     
    def calibrate_gripper_callback(self, request:EmptyWithSuccess.Request, response:EmptyWithSuccess.Response):
        spawn_success = self.spawn_frames_for_current_gripper()
        response.success = spawn_success
        return response        
    
    def measure_frame(self, frame_id:str)->tuple[bool, Vector3]:
        request = MeasureFrame.Request()
        request.frame_name = frame_id
        response:MeasureFrame.Response = self.client_measure_frame_cam.call(request)
        
        return response.success, response.result_vector
    
    def save_joint_config(self, joint_name:str, rel_transformation:Transform)->bool:
        print("Saving joint config for joint: " + joint_name)
        package_name = 'pm_robot_description'
        file_name = 'calibration_config/pm_robot_joint_calibration.yaml'
        file_path = get_package_share_directory(package_name) + '/' + file_name

        calibration_config = {}
        # check if the file exists
        try:
            with open(file_path, 'r') as file:
                calibration_config = yaml.load(file, Loader=yaml.FullLoader)

            calibration_config[joint_name]['x_offset'] = rel_transformation.translation.x /1e6
            calibration_config[joint_name]['y_offset'] = rel_transformation.translation.y /1e6
            calibration_config[joint_name]['z_offset'] = rel_transformation.translation.z /1e6

            q = [rel_transformation.rotation.x,
                 rel_transformation.rotation.y,
                 rel_transformation.rotation.z,
                 rel_transformation.rotation.w]

            # Convert to Euler angles (roll, pitch, yaw) in radians
            r = R.from_quat(q)
            roll, pitch, yaw = r.as_euler('xyz', degrees=True)
            
            calibration_config[joint_name]['rx_offset'] = float(roll)
            calibration_config[joint_name]['ry_offset'] = float(pitch)
            calibration_config[joint_name]['rz_offset'] = float(yaw)
            print(roll, pitch, yaw)
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
            print("Calibration config saved to: " + file_path)
            return True
        except Exception as e:
            self._logger.error("Error: " + str(e))
            return False
        return False
        
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