
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

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient

from builtin_interfaces.msg import Duration
from builtin_interfaces.msg import Duration as MsgDuration

from sensor_msgs.msg import JointState
from pm_msgs.srv import LaserGetMeasurement


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
        self.client_get_laser_mes = self._node.create_client(LaserGetMeasurement, '/pm_sensor_controller/Laser/GetMeasurement')
        self.client_move_robot_laser_to_frame = self._node.create_client(MoveToFrame, '/pm_moveit_server/move_laser_to_frame')

        self.object_scene:ami_msg.ObjectScene = None
        self.xyt_joint_client = ActionClient(self._node, FollowJointTrajectory, '/pm_robot_xyz_axis_controller/follow_joint_trajectory')

        self.joint_state_sub = self._node.create_subscription(
                                JointState,
                                '/joint_states',
                                self.joint_state_callback,
                                10
                                )
        
        self._current_joint_state_positions = {}

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
        
    def float_to_ros_duration(self, time_float):
        
        secs = int(time_float)
        nsecs = int((time_float - secs) * 1e9)
        return MsgDuration(sec=secs, nanosec=nsecs)

    def send_xyz_trajectory_goal_absolut(self,  x_joint:float, 
                                        y_joint:float, 
                                        z_joint:float,
                                        time:float = 5)->bool:
        """Send a goal to the robot."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['X_Axis_Joint', 'Y_Axis_Joint', 'Z_Axis_Joint']
        point = JointTrajectoryPoint()

        point.positions = [float(x_joint), float(y_joint), float(z_joint)]
        point.time_from_start =self.float_to_ros_duration(time)
        goal.trajectory.points.append(point)
        
        success = self._send_goal(goal)
        return success

    def send_xyz_trajectory_goal_relative(self, x_joint_rel:float, 
                                        y_joint_rel:float, 
                                        z_joint_rel:float,
                                        time:float = 5)->bool:
        """Send a goal to the robot."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['X_Axis_Joint', 'Y_Axis_Joint', 'Z_Axis_Joint']
        point = JointTrajectoryPoint()
        # Get the current joint state positions
        current_x = self.get_current_joint_state('X_Axis_Joint')
        current_y = self.get_current_joint_state('Y_Axis_Joint')
        current_z = self.get_current_joint_state('Z_Axis_Joint')
        
        point.positions = [float(current_x + x_joint_rel),
                            float(current_y + y_joint_rel), 
                            float(current_z + z_joint_rel)]
        
        point.time_from_start = self.float_to_ros_duration(time)
        goal.trajectory.points.append(point)
        success = self._send_goal(goal)
        return success
    
    def _send_goal(self, goal:FollowJointTrajectory.Goal)->bool:
            self.xyt_joint_client.wait_for_server()
            result= self.xyt_joint_client.send_goal(goal)
            result:FollowJointTrajectory.Result = result.result

            if result is None:
                self._node.get_logger().info('Goal rejected.')
                return False
            if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
                #self._node.get_logger().info('Goal accepted.')
                return True 
            if result.error_code == FollowJointTrajectory.Result.INVALID_GOAL:
                self._node.get_logger().info('Goal rejected.')
                return False
            if result.error_code == FollowJointTrajectory.Result.INVALID_JOINTS:
                self._node.get_logger().info('Goal rejected. Invalid joints.')
                return False
                
    def joint_state_callback(self, msg: JointState):
        for name, position in zip(msg.name, msg.position):
            self._current_joint_state_positions[name] = position

    def get_current_joint_state(self,joint_name:str)->float:
        """
        Get the current joint state of the robot.
        
        Args:
            joint_name (str): Name of the joint.
        
        Returns:
            float: Current joint state.
        """
        return self._current_joint_state_positions.get(joint_name, None)

    def get_laser_measurement(self, unit:str = "m")-> float:
        """
        Method to get the laser measurement from the laser sensor
        
        Returns:
            float: _description_
        """
        call_async = False

        if not self.client_get_laser_mes.wait_for_service(timeout_sec=1.0):
            self._node.get_logger().error("Service '/pm_sensor_controller/Laser/GetMeasurement' not available")
            return None
        
        req = LaserGetMeasurement.Request()

        if call_async:
            future = self.client_get_laser_mes.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is None:
                self._node.get_logger().error('Service call failed %r' % (future.exception(),))
                return None
            response= future.result()
            
        else:
            response:LaserGetMeasurement.Response = self.client_get_laser_mes.call(req)
        
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