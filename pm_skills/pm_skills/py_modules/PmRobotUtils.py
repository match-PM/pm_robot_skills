
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
    def __init__(self, node:Node):
        self._node = node
        self.client_execute_vision = self._node.create_client(ExecuteVision, '/pm_vision_manager/ExecuteVision')
        self.client_move_robot_cam1_to_frame = self._node.create_client(MoveToFrame, '/pm_moveit_server/move_cam1_to_frame')
        self.client_move_robot_tool_to_frame = self._node.create_client(MoveToFrame, '/pm_moveit_server/move_tool_to_frame')
        self.client_adapt_frame_absolut = self._node.create_client(ami_srv.ModifyPoseAbsolut, '/assembly_manager/modify_frame_absolut')

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