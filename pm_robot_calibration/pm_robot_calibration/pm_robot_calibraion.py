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
from geometry_msgs.msg import Vector3, TransformStamped, Pose, PoseStamped, Quaternion

import assembly_manager_interfaces.srv as ami_srv
import assembly_manager_interfaces.msg as ami_msg

from assembly_scene_publisher.py_modules.AssemblyScene import AssemblyManagerScene
from assembly_manager_interfaces.srv import SpawnFramesFromDescription
from pm_msgs.srv import EmptyWithSuccess

# import get_package_share_directory
from ament_index_python.packages import get_package_share_directory

from ros_sequential_action_programmer.submodules.pm_robot_modules.widget_pm_robot_config import VacuumGripperConfig, ParallelGripperConfig

TOOL_VACUUM_IDENT = 'pm_robot_vacuum_tools'
TOOL_GRIPPER_1_JAW_IDENT = 'pm_robot_tool_parallel_gripper_1_jaw'
TOOL_GRIPPER_2_JAW_IDENT = 'pm_robot_tool_parallel_gripper_2_jaws'

class PmRobotCalibrationNode(Node):

    def __init__(self):
        
        super().__init__('pm_robot_calibration')
        self._logger = self.get_logger()

        self._logger.info(" Pm Robot Calibration Node started...")
        
        self.callback_group = MutuallyExclusiveCallbackGroup()
        self.pm_robot_config = {}
        self.gripper_frames_spawned = False
        # clients
        self.client_spawn_frames = self.create_client(SpawnFramesFromDescription, '/assembly_manager/spawn_frames_from_description')
        self.client_measure_frame_cam = self.create_client(MeasureFrame, '/pm_skills/vision_measure_frame')
        
        # services
        self.calibrate_gripper_srv = self.create_service(EmptyWithSuccess, '/pm_robot_calibration/calibrate_gripper', self.calibrate_gripper_callback, callback_group=self.callback_group)
        self.calibrate_dispenser_srv = self.create_service(EmptyWithSuccess, '/pm_robot_calibration/calibrate_1K_dispenser', self.calibrate_1K_dispenser_callback, callback_group=self.callback_group)
        
        # paths
        self.bringup_share_path = get_package_share_directory('pm_robot_bringup')
        self.calibration_frame_dict_path = get_package_share_directory('pm_robot_description') + '/urdf/urdf_configs/calibration_frame_dictionaries'
        self.pm_robot_config_path = self.bringup_share_path + '/config/pm_robot_bringup_config.yaml'

        self.update_pm_robot_config()
        
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