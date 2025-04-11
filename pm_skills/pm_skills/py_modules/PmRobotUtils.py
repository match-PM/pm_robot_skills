
from rclpy.node import Node
import sys
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from pm_skills_interfaces.srv import MeasureFrame, CorrectFrame
from pm_moveit_interfaces.srv import MoveToPose,  MoveToFrame
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, StaticTransformBroadcaster
from pm_vision_interfaces.srv import ExecuteVision
import pm_vision_interfaces.msg as vision_msg
from geometry_msgs.msg import Vector3, TransformStamped, Pose, PoseStamped, Quaternion

from pm_vision_manager.va_py_modules.vision_assistant_class import VisionProcessClass

import assembly_manager_interfaces.srv as ami_srv
import assembly_manager_interfaces.msg as ami_msg

from assembly_scene_publisher.py_modules.scene_functions import is_frame_from_scene
from assembly_scene_publisher.py_modules.tf_functions import get_transform_for_frame_in_world
import time


class PmRobotUtils():

    REAL_MODE = 0
    UNITY_MODE = 1
    GAZEBO_MODE = 2

    def __init__(self, node:Node):
        self._node = node
        self.client_execute_vision = self._node.create_client(ExecuteVision, '/pm_vision_manager/ExecuteVision')
        self.client_move_robot_cam1_to_frame = self._node.create_client(MoveToFrame, '/pm_moveit_server/move_cam1_to_frame')
        self.client_move_robot_tool_to_frame = self._node.create_client(MoveToFrame, '/pm_moveit_server/move_tool_to_frame')
        self.client_adapt_frame_absolut = self._node.create_client(ami_srv.ModifyPoseAbsolut, '/assembly_manager/modify_frame_absolut')

        self.object_scene:ami_msg.ObjectScene = None

    def start_object_scene_subscribtion(self):
        self.objcet_scene_subscriber = self._node.create_subscription(ami_msg.ObjectScene, '/assembly_manager/scene', self.object_scene_callback, 10)

    def is_gazebo_running(self)->bool:
        """Check if the Gazebo node is active."""
        node_names = self._node.get_node_names()
        if 'gazebo' in node_names:
            return True
        return False
    
    def is_unity_running(self)->bool:
        """Check if the Unity node is active."""
        node_names = self._node.get_node_names()
        if 'ROS2UnityCam1Publisher' in node_names:
            return True
        return False
    
    def get_mode(self)->int:
        """Get the current mode of the robot."""
        """
        Returns:
        0 - REAL_MODE
        1 - UNITY_MODE
        2 - GAZEBO_MODE
        """

        if self.is_gazebo_running():
            return self.GAZEBO_MODE
        elif self.is_unity_running():
            return self.UNITY_MODE
        else:
            return self.REAL_MODE
    
        
    def wait_for_initial_scene_update(self):
        while self.object_scene is None:
            self._node.get_logger().warn("Waiting for object scene to be updated...")
            self._node.get_logger().warn("Make sure you started the scene subscribtion...")
            time.sleep(0.1)

    def object_scene_callback(self, msg:ami_msg.ObjectScene)-> str:
        self.object_scene = msg
    
    def get_cam_file_name_bottom(self)->str:
        mode = self.get_mode()

        if mode == self.REAL_MODE:
            return 'pm_robot_basler_bottom_cam_2.yaml'
        elif mode == self.UNITY_MODE:
            return 'pm_robot_bottom_cam_2_Unity.yaml'
        else:
            return 'NOT_AVAILABLE'
            
    def get_cam_file_name_top(self)->str:
        mode = self.get_mode()

        if mode == self.REAL_MODE:
            return 'pm_robot_basler_top_cam_1.yaml'
        elif mode == self.UNITY_MODE:
            return 'pm_robot_top_cam_1_Unity.yaml'
        else:
            return 'NOT_AVAILABLE'