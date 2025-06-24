
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
from pm_msgs.srv import LaserGetMeasurement, Cam2LightSetState, CoaxLightSetState
from pm_uepsilon_confocal_msgs.srv import GetValue

from pm_robot_modules.submodules.pm_robot_config import PmRobotConfig
class PmRobotUtils():

    REAL_MODE = 0
    UNITY_MODE = 1
    GAZEBO_MODE = 2
    
    X_Axis_JOINT_NAME = 'X_Axis_Joint'
    Y_Axis_JOINT_NAME = 'Y_Axis_Joint'
    Z_Axis_JOINT_NAME = 'Z_Axis_Joint'
    T_Acis_JOINT_NAME = 'T_Axis_Joint'

    def __init__(self, node:Node):
        self._node = node
        self.client_execute_vision = self._node.create_client(ExecuteVision, '/pm_vision_manager/ExecuteVision')
        self.client_move_robot_cam1_to_frame = self._node.create_client(MoveToFrame, '/pm_moveit_server/move_cam1_to_frame')
        self.client_move_robot_tool_to_frame = self._node.create_client(MoveToFrame, '/pm_moveit_server/move_tool_to_frame')
        self.client_adapt_frame_absolut = self._node.create_client(ami_srv.ModifyPoseAbsolut, '/assembly_manager/modify_frame_absolut')
        self.client_get_laser_mes = self._node.create_client(LaserGetMeasurement, '/pm_sensor_controller/Laser/GetMeasurement')
        self.client_move_robot_laser_to_frame = self._node.create_client(MoveToFrame, '/pm_moveit_server/move_laser_to_frame')
        self.client_get_confocal_bottom_measurement = self._node.create_client(GetValue, '/uepsilon_two_channel_controller/IFC2422/ch1/distance/srv')
        self.client_get_confocal_top_measurement = self._node.create_client(GetValue, '/uepsilon_two_channel_controller/IFC2422/ch2/distance/srv')
        self.client_set_cam2_coax_light = self._node.create_client(Cam2LightSetState, '/pm_lights_controller/Cam2Light/SetState')
        self.client_set_cam1_coax_light = self._node.create_client(CoaxLightSetState, '/pm_lights_controller/CoaxLight/SetState')

        self.object_scene:ami_msg.ObjectScene = None
        self.xyz_joint_client = ActionClient(self._node, FollowJointTrajectory, '/pm_robot_xyz_axis_controller/follow_joint_trajectory')
        self.t_joint_client = ActionClient(self._node, FollowJointTrajectory, '/pm_robot_t_axis_controller/follow_joint_trajectory')
        
        self.joint_state_sub = self._node.create_subscription(
                                JointState,
                                '/joint_states',
                                self.joint_state_callback,
                                10
                                )
        
        self._current_joint_state_positions = {}
        
        self.pm_robot_config = PmRobotConfig()
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self._node)

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
        
        success = self._send_goal_xyz(goal)
        return success

    def send_t_trajectory_goal_absolut(self, t_joint:float, time:float = 5)->bool:
        """Send a goal to the robot."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [self.T_Acis_JOINT_NAME]
        point = JointTrajectoryPoint()

        point.positions = [float(t_joint)]
        point.time_from_start =self.float_to_ros_duration(time)
        goal.trajectory.points.append(point)
        
        success = self._send_goal_t(goal)
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
        success = self._send_goal_xyz(goal)
        return success
    
    def _send_goal_xyz(self, goal:FollowJointTrajectory.Goal)->bool:
            self.xyz_joint_client.wait_for_server()
            result= self.xyz_joint_client.send_goal(goal)
            result:FollowJointTrajectory.Result = result.result

            if result is None:
                self._node.get_logger().info('Goal rejected.')
                return False
            if result.error_code == FollowJointTrajectory.Result.INVALID_GOAL:
                self._node.get_logger().info('Goal rejected.')
                return False
            if result.error_code == FollowJointTrajectory.Result.INVALID_JOINTS:
                self._node.get_logger().info('Goal rejected. Invalid joints.')
                return False
            
            if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
                wait_success = self.wait_for_joints_reached(
                    joint_names=[self.X_Axis_JOINT_NAME, self.Y_Axis_JOINT_NAME, self.Z_Axis_JOINT_NAME],
                    target_joint_values=[goal.trajectory.points[0].positions[0],
                                        goal.trajectory.points[0].positions[1],
                                        goal.trajectory.points[0].positions[2]],
                    tolerance=[0.000001],
                    timeout=5.0)
                return wait_success
    
    def _send_goal_t(self, goal:FollowJointTrajectory.Goal)->bool:
            self.t_joint_client.wait_for_server()
            result= self.t_joint_client.send_goal(goal)
            result:FollowJointTrajectory.Result = result.result

            if result is None:
                self._node.get_logger().info('Goal rejected.')
                return False
            if result.error_code == FollowJointTrajectory.Result.INVALID_GOAL:
                self._node.get_logger().info('Goal rejected.')
                return False
            if result.error_code == FollowJointTrajectory.Result.INVALID_JOINTS:
                self._node.get_logger().info('Goal rejected. Invalid joints.')
                return False
            
            
            if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
                #self._node.get_logger().info('Goal accepted.')
                
                target_joint_value = goal.trajectory.points[0].positions[0]
                self._node._logger.warn(f"Waiting for joints to be reached...{target_joint_value}")
                wait_success = self.wait_for_joints_reached(
                    joint_names=[self.T_Acis_JOINT_NAME],
                    target_joint_values=[target_joint_value],
                    tolerance=[0.0001],
                    timeout=5.0)
                
                return wait_success 
    
    def interative_sensing_laser(self)->True:
        # THIS METHOD IS NOT COLLISION SAVE
        # it will move max 1 mm
        iterations = 20
        increments = 0.0001 # 100 um


        self.send_xyz_trajectory_goal_relative(x_joint_rel=0,
                                                y_joint_rel=0,
                                                z_joint_rel = -iterations*increments,
                                                time=0.5)
        
        for iterator in range((iterations)*2):

            if self.get_mode() == self.REAL_MODE:
                # move up
                if not self._check_for_valid_laser_measurement():
                    self.send_xyz_trajectory_goal_relative(x_joint_rel=0,
                                                           y_joint_rel=0,
                                                           z_joint_rel=increments,
                                                           time=0.5)
                else:
                    return True

                self._node.get_logger().warn(f"{iterator}")
        
        return False
                # move to default
                


    def _check_for_valid_laser_measurement(self):
        # down movement
        value_1 = self.get_laser_measurement(unit='um')
        time.sleep(0.5)
        value_2 = self.get_laser_measurement(unit='um')
        time.sleep(0.5)
        value_3 = self.get_laser_measurement(unit='um')
        self._node.get_logger().warn(f"{value_1}")
        self._node.get_logger().warn(f"{value_2}")
        self._node.get_logger().warn(f"{value_3}")

        if ((value_1 == value_2) and (value_2 == value_3) and (value_1 == value_3)):
            self._node.get_logger().warn(f"False")
            return False
        else:
            self._node.get_logger().warn(f"True")
            return True

    def wait_for_joints_reached(self, 
                                joint_names:list[str], 
                                target_joint_values:list[float], 
                                tolerance:list[float],
                                timeout:float = 5.0)->bool:
        
        """
        Wait for the robot to reach the desired joint positions.
        Args:
            joint_names (list[str]): List of joint names.   
            joint_values (list[float]): List of joint values.
            tolerance (list[float]): List of tolerances for each joint.
            timeout (float): Timeout in seconds.
        Returns:

            bool: True if the robot reached the desired joint positions, False otherwise.
        """
        start_time = time.time()
        while True:
            current_joint_positions = [self.get_current_joint_state(name) for name in joint_names]
            if None in current_joint_positions:
                self._node.get_logger().error("Joint state not available.")
                return False
            
            reached = True
            for i, (current_position, target_position, tol) in enumerate(zip(current_joint_positions, target_joint_values, tolerance)):
                if abs(current_position - target_position) > tol:
                    reached = False
                    break
            
            if reached:
                self._node.get_logger().info("Robot reached the desired joint positions.")
                return True
            
            if time.time() - start_time > timeout:
                self._node.get_logger().error("Timeout waiting for joint positions.")
                return False
    
    def set_cam2_coax_light(self, intensity_value_pct: float)-> bool:

        if not self.client_set_cam2_coax_light.wait_for_service(timeout_sec=1.0):
            self._node.get_logger().error("Service '/pm_lights_controller/CoaxLight/SetState' not available")
            return False
        
        req = Cam2LightSetState.Request()
        req.intensity = intensity_value_pct

        response = self.client_set_cam2_coax_light.call(req)

        return True
    
    def set_cam1_coax_light(self, enable_state: bool)-> bool:

        if not self.client_set_cam1_coax_light.wait_for_service(timeout_sec=1.0):
            self._node.get_logger().error("Service '/pm_lights_controller/CoaxLight/SetState' not available")
            return False
        
        req = CoaxLightSetState.Request()
        req.turn_on = enable_state

        response = self.client_set_cam1_coax_light.call(req)

        return True


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
    
    def get_confocal_top_measurement(self, unit:str = "m")->float:
        self._node.get_logger().warn("Test1")
        req = GetValue.Request()

        if self.get_mode() == self.REAL_MODE:
            if not self.client_get_confocal_top_measurement.wait_for_service(timeout_sec=1.0):
                self._node.get_logger().error("Service '/uepsilon_two_channel_controller/IFC2422/ch1/distance/srv' not available!")
                return None
            
            response:GetValue.Response = self.client_get_confocal_top_measurement.call(req)
        
        elif self.get_mode() == self.UNITY_MODE:
            pass

        else:
            return None
    
        multiplier = 1.0
        
        if unit == "mm":
            multiplier = 1e-3
        elif unit == "cm":
            multiplier = 1e-2
        elif unit == "m":
            multiplier = 1e-6
        elif unit == "um":
            multiplier = 1.0
        self._node.get_logger().warn("Test2")

        return response.data * multiplier

    def get_confocal_bottom_measurement(self, unit:str = "m")->float:
        self._node.get_logger().warn("Test1")
        req = GetValue.Request()

        call_async = False


        if self.get_mode() == self.REAL_MODE:
            self._node.get_logger().warn("Test22")

            if not self.client_get_confocal_bottom_measurement.wait_for_service(timeout_sec=1.0):
                self._node.get_logger().error("Service '/uepsilon_two_channel_controller/IFC2422/ch1/distance/srv' not available!")
                return None
            
            if call_async:
                future = self.client_get_confocal_bottom_measurement.call_async(req)
                rclpy.spin_until_future_complete(self, future)
                if future.result() is None:
                    self._node.get_logger().error('Service call failed %r' % (future.exception(),))
                    return None
                response= future.result()

            else:
                response:GetValue.Response = self.client_get_confocal_bottom_measurement.call(req)
        
        elif self.get_mode() == self.UNITY_MODE:
            pass

        else:
            return None
        
        self._node.get_logger().warn("Test2")
        multiplier = 1.0
        
        if unit == "mm":
            multiplier = 1e-3
        elif unit == "cm":
            multiplier = 1e-2
        elif unit == "m":
            multiplier = 1e-6
        elif unit == "um":
            multiplier = 1.0
        self._node.get_logger().warn("Test3")

        return response.data * multiplier
