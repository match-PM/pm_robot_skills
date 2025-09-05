
from rclpy.node import Node
import sys
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from pm_skills_interfaces.srv import MeasureFrame, CorrectFrame
from pm_moveit_interfaces.srv import MoveToPose,  MoveToFrame, AlignGonio, MoveRelative
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, StaticTransformBroadcaster
from pm_vision_interfaces.srv import ExecuteVision
import pm_vision_interfaces.msg as vision_msg
from geometry_msgs.msg import Vector3, TransformStamped, Pose, PoseStamped, Quaternion, Transform
from example_interfaces.srv import SetBool

from pm_vision_manager.va_py_modules.vision_assistant_class import VisionProcessClass

import assembly_manager_interfaces.srv as ami_srv
import assembly_manager_interfaces.msg as ami_msg

from std_msgs.msg import Float64MultiArray

from assembly_scene_publisher.py_modules.scene_functions import is_frame_from_scene
from assembly_scene_publisher.py_modules.tf_functions import get_transform_for_frame_in_world
import time

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient

from builtin_interfaces.msg import Duration
from builtin_interfaces.msg import Duration as MsgDuration

from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import JointState
from pm_msgs.srv import LaserGetMeasurement, Cam2LightSetState, CoaxLightSetState, ForceSensorBias,ForceSensorGetMeasurement, EmptyWithSuccess, ReferenceCubeState
from pm_uepsilon_confocal_msgs.srv import GetValue

from pm_robot_modules.submodules.pm_robot_config import PmRobotConfig
from enum import Enum
from assembly_scene_publisher.py_modules.AssemblySceneAnalyzerAdv import AssemblySceneAnalyzerAdv
from assembly_scene_publisher.py_modules.AssemblySceneAnalyzer import UnInitializedScene

from assembly_scene_publisher.py_modules.scene_errors import (RefAxisNotFoundError, 
                                                              RefFrameNotFoundError, 
                                                              RefPlaneNotFoundError, 
                                                              ComponentNotFoundError)
# create new error

class PmRobotError(Exception):
    def __init__(self, message="Error occured controlling the robot."):
        self.message = message
        super().__init__(self.message)

class PmRobotTcps(Enum):
    TCP_LASER = 'Laser_Toolhead_TCP'
    TCP_TOOL = 'PM_Robot_Tool_TCP'
    TCP_CAMERA_TOP = 'Cam1_Toolhead_TCP'
    TCP_CONFOCAL_TOP = 'TCP_Confocal_Sensor_Top'
    TCP_CONFOCAL_TOP_2 = 'TCP_Confocal_Sensor_Top_2'
    TCP_CAMERA_BOTTOM = 'Cam1_Toolhead_TCP'
    TCP_CONFOCAL_BOTTOM = 'TCP_Confocal_Sensor_Bottom'
    
class PmRobotUtils():

    REAL_MODE = 0
    UNITY_MODE = 1
    GAZEBO_MODE = 2
    
    X_Axis_JOINT_NAME = 'X_Axis_Joint'
    Y_Axis_JOINT_NAME = 'Y_Axis_Joint'
    Z_Axis_JOINT_NAME = 'Z_Axis_Joint'
    T_Axis_JOINT_NAME = 'T_Axis_Joint'
    GONIO_RIGHT_STAGE_1 = 'Gonio_Right_Stage_1_Joint'
    GONIO_RIGHT_STAGE_2 = 'Gonio_Right_Stage_2_Joint'
    GONIO_LEFT_STAGE_1 = 'Gonio_Left_Stage_1_Joint'
    GONIO_LEFT_STAGE_2 = 'Gonio_Left_Stage_2_Joint'
    
    TCP_LASER = 'Laser_Toolhead_TCP'
    TCP_TOOL = 'PM_Robot_Tool_TCP'
    TCP_CAMERA_TOP = 'Cam1_Toolhead_TCP'
    TCP_CONFOCAL_TOP = 'TCP_Confocal_Sensor_Top'
    TCP_CONFOCAL_TOP_2 = 'TCP_Confocal_Sensor_Top_2'
    TCP_CAMERA_BOTTOM = 'Camera_Station_TCP'
    TCP_CONFOCAL_BOTTOM = 'TCP_Confocal_Sensor_Bottom'
    TCP_1K_DISPENSER = '1K_Dispenser_TCP'

    def __init__(self, node:Node):
        self._node = node
        self.client_execute_vision = self._node.create_client(ExecuteVision, '/pm_vision_manager/ExecuteVision')
        self.client_move_robot_cam1_to_frame = self._node.create_client(MoveToFrame, '/pm_moveit_server/move_cam1_to_frame')
        self.client_move_robot_tool_to_frame = self._node.create_client(MoveToFrame, '/pm_moveit_server/move_tool_to_frame')
        self.client_move_robot_confocal_top_to_frame = self._node.create_client(MoveToFrame, '/pm_moveit_server/move_confocal_head_to_frame')
        self.client_move_robot_1k_dispenser_to_frame = self._node.create_client(MoveToFrame, '/pm_moveit_server/move_1k_dispenser_to_frame')

        self.client_adapt_frame_absolut = self._node.create_client(ami_srv.ModifyPoseAbsolut, '/assembly_manager/modify_frame_absolut')
        self.client_get_laser_mes = self._node.create_client(LaserGetMeasurement, '/pm_sensor_controller/Laser/GetMeasurement')
        self.client_move_robot_laser_to_frame = self._node.create_client(MoveToFrame, '/pm_moveit_server/move_laser_to_frame')
        self.client_move_laser_relative = self._node.create_client(MoveRelative, '/pm_moveit_server/move_laser_relative')
        self.client_get_confocal_bottom_measurement = self._node.create_client(GetValue, '/pm_robot_primitive_skills/get_confocal_bottom_measurement')
        self.client_get_confocal_top_measurement = self._node.create_client(GetValue, '/pm_robot_primitive_skills/get_confocal_top_measurement')
        self.client_set_cam2_coax_light = self._node.create_client(Cam2LightSetState, '/pm_lights_controller/Cam2Light/SetState')
        self.client_set_cam1_coax_light = self._node.create_client(CoaxLightSetState, '/pm_lights_controller/CoaxLight/SetState')
        self.client_set_force_sensor_bias = self._node.create_client(ForceSensorBias,'/pm_sensor_controller/ForceSensor/Bias')
        self.client_get_force_measurement = self._node.create_client(ForceSensorGetMeasurement,'/pm_sensor_controller/ForceSensor/GetMeasurement')
        self.client_align_gonio_right = self._node.create_client(AlignGonio, '/pm_moveit_server/align_gonio_right')
        self.client_align_gonio_left = self._node.create_client(AlignGonio, '/pm_moveit_server/align_gonio_left')
        self.client_set_collision = self._node.create_client(ami_srv.SetCollisionChecking, '/moveit_component_spawner/set_collision_checking')
        self.client_create_ref_frame = self.create_client(ami_srv.CreateRefFrame, '/assembly_manager/create_ref_frame')
        self.client_recalculate_assembly_instruction = self._node.create_client(ami_srv.CalculateAssemblyInstructions, '/assembly_manager/calculate_assembly_instructions')


        self.client_check_reference_cube = self._node.create_client(ReferenceCubeState, '/pm_sensor_controller/ReferenceCube/State')

        self.client_turn_on_vacuum_tool_head = self._node.create_client(EmptyWithSuccess, '/pm_nozzle_controller/Head_Nozzle/Vacuum')
        self.client_turn_off_vacuum_tool_head = self._node.create_client(EmptyWithSuccess, '/pm_nozzle_controller/Head_Nozzle/TurnOff')


        self._open_protection_real_srv = self._node.create_client(EmptyWithSuccess, '/pm_pneumatic_controller/N1K_Dispenser_Protection_Joint/MoveBackward')
        self._close_protection_real_srv = self._node.create_client(EmptyWithSuccess, '/pm_pneumatic_controller/N1K_Dispenser_Protection_Joint/MoveForward')

        self._retract_dispenser_real_srv = self._node.create_client(EmptyWithSuccess, '/pm_pneumatic_controller/N1K_Dispenser_Joint/MoveBackward')
        self._extend_dispenser_real_srv = self._node.create_client(EmptyWithSuccess, '/pm_pneumatic_controller/N1K_Dispenser_Joint/MoveForward')

        self.client_turn_on_gonio_left_vacuum = self._node.create_client(EmptyWithSuccess, '/pm_nozzle_controller/Gonio_Nozzle/Vacuum')
        self.client_turn_off_gonio_left_vacuum = self._node.create_client(EmptyWithSuccess, '/pm_nozzle_controller/Gonio_Nozzle/TurnOff')

        self.client_turn_on_gonio_right_vacuum = self._node.create_client(EmptyWithSuccess, '/pm_nozzle_controller/Nest_Nozzle/Vacuum')
        self.client_turn_off_gonio_right_vacuum = self._node.create_client(EmptyWithSuccess, '/pm_nozzle_controller/Nest_Nozzle/TurnOff')

        self._current_force_sensor_data = Float64MultiArray()

        self.object_scene_un= UnInitializedScene()

        self.assembly_scene_analyzer = AssemblySceneAnalyzerAdv(self.object_scene_un, self._node.get_logger())

        self.xyz_joint_client = ActionClient(self._node, FollowJointTrajectory, '/pm_robot_xyz_axis_controller/follow_joint_trajectory')
        self.t_joint_client = ActionClient(self._node, FollowJointTrajectory, '/pm_robot_t_axis_controller/follow_joint_trajectory')
        
        self.joint_state_sub = self._node.create_subscription(
                                JointState,
                                '/joint_states',
                                self.joint_state_callback,
                                10
                                )
        
        self.client_force_sensor = self._node.create_subscription(Float64MultiArray, '/pm_sensor_controller/ForceSensor/Stream',self.force_sensor_callback, 10)

        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )

        self.tf_static_sub = self._node.create_subscription(TFMessage, '/tf_static', self.tf_static_callback, qos_profile)

        self._current_joint_state_positions = {}

        self._tf_static_msgs = []
        
        self.pm_robot_config = PmRobotConfig()
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self._node)


    def tf_static_callback(self, msg: TFMessage):
        self._tf_static_msgs.extend(msg.transforms)

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
        while self.object_scene_un.scene is None:
            self._node.get_logger().warn("Waiting for object scene to be updated...")
            self._node.get_logger().warn("Make sure you started the scene subscribtion...")
            time.sleep(0.5)

    def object_scene_callback(self, msg:ami_msg.ObjectScene)-> str:
        self.object_scene_un.scene = msg
    
    def set_force_sensor_bias(self)->bool:
        if not self.client_set_force_sensor_bias.wait_for_service(1):
            self._node.get_logger().error(f"Service {self.client_set_force_sensor_bias.srv_name} not available!")
            return False
        request = ForceSensorBias.Request() 

        request.bias = True

        response:ForceSensorBias.Response = self.client_set_force_sensor_bias.call(request)
        self._node.get_logger().info(f"Force Sensor Bias set!")

        return True

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
        goal.trajectory.joint_names = [self.T_Axis_JOINT_NAME]
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
        """
        Move the robot relative to its current position.
        z axis is defined with gravity vector !!! Use negative values to move the robot up!
        THIS IS NOT COLLISION SAVE!!! Be careful.
        
        """

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
                time.sleep(1)
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
                    joint_names=[self.T_Axis_JOINT_NAME],
                    target_joint_values=[target_joint_value],
                    tolerance=[0.0001],
                    timeout=5.0)
                
                return wait_success 
                    

    def _check_for_valid_laser_measurement(self):
        # down movement
        if self.get_mode() == self.REAL_MODE:
            values =[]
            time.sleep(0.2)
            time_sleep = 0.4
            values.append(self.get_laser_measurement(unit='um'))
            time.sleep(time_sleep)
            values.append(self.get_laser_measurement(unit='um'))
            time.sleep(time_sleep)
            values.append(self.get_laser_measurement(unit='um'))
            time.sleep(time_sleep)
            values.append(self.get_laser_measurement(unit='um'))
            time.sleep(time_sleep)
            values.append(self.get_laser_measurement(unit='um'))

            if all(v == values[0] for v in values):
                #self._node.get_logger().warn(f"Laser Measurement Valid - False")
                return False
            else:
                #self._node.get_logger().warn(f"Laser Measurement Valid - True")
                return True
            
        elif self.get_mode() == self.UNITY_MODE:
            value = self.get_laser_measurement(unit='um')
            if (value > 299.0) or(value  < -299.0):
                self._node.get_logger().warn(f"Laser Measurement Valid - False")
                return False
            else:
                return True
        else: 
            self._node.get_logger().warn(f"Get laser measurement not implmentet for this mode {self.get_mode()}")
            raise RuntimeError("This should not happen.")

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
                time.sleep(0.5)
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
            raise PmRobotError("Service '/pm_sensor_controller/Laser/GetMeasurement' not available")
        
        req = LaserGetMeasurement.Request()

        if call_async:
            future = self.client_get_laser_mes.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is None:
                self._node.get_logger().error('Service call failed %r' % (future.exception(),))
                raise PmRobotError('Service call failed %r' % (future.exception(),))
            response= future.result()
            
        else:
            response:LaserGetMeasurement.Response = self.client_get_laser_mes.call(req)
        
        multiplier = self._get_multiplier(unit)

        return response.measurement * multiplier
    
    def get_confocal_top_measurement(self, unit:str = "m")->float:
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
        
        #self._node.get_logger().warn(f"value {response.data} um")

        multiplier =  self._get_multiplier(unit)

        result = response.data * multiplier
        
        #self._node.get_logger().warn(f"value {result} m")

        return result

    def check_confocal_top_measurement_in_range(self)->bool:
        req = GetValue.Request()

        if self.get_mode() == self.REAL_MODE:
            if not self.client_get_confocal_top_measurement.wait_for_service(timeout_sec=1.0):
                self._node.get_logger().error("Service '/uepsilon_two_channel_controller/IFC2422/ch1/distance/srv' not available!")
                return None
            
            response:GetValue.Response = self.client_get_confocal_top_measurement.call(req)
            #self._node.get_logger().warn(f"{response.success}")

            return response.success
        
        elif self.get_mode() == self.UNITY_MODE:
            return False

        else:
            return False
    
    def get_confocal_bottom_measurement(self, unit:str = "m")->float:
        req = GetValue.Request()

        call_async = False

        if self.get_mode() == self.REAL_MODE:

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
        
        multiplier = self._get_multiplier(unit)

        return response.data * multiplier

    def check_confocal_bottom_measurement_in_range(self)->bool:
        req = GetValue.Request()

        if self.get_mode() == self.REAL_MODE:
            if not self.client_get_confocal_bottom_measurement.wait_for_service(timeout_sec=1.0):
                self._node.get_logger().error("Service '/uepsilon_two_channel_controller/IFC2422/ch1/distance/srv' not available!")
                return None
            
            response:GetValue.Response = self.client_get_confocal_bottom_measurement.call(req)

            return response.success
        
        elif self.get_mode() == self.UNITY_MODE:
            return False

        else:
            return False
        
    def force_sensor_callback(self, msg: Float64MultiArray):
        self._current_force_sensor_data = msg

    def set_collision(self, link_1, link_2, should_check):
        """
        Set collision checking between two links.
        Args:
            link_1 (str): The first link.
            link_2 (str): The second link.
            should_check (bool): Whether to enable or disable collision checking.
        Raises:
            PmRobotError: If the service is not available or the request fails.
        """
        if not self.client_set_collision.wait_for_service(1):
            raise PmRobotError(f"Client '{self.client_set_collision.srv_name}' not available!")

        set_request = ami_srv.SetCollisionChecking.Request()
        set_request.link_1 = link_1
        set_request.link_2 = link_2
        set_request.should_check = should_check

        set_response:ami_srv.SetCollisionChecking.Response = self.client_set_collision.call(set_request)

        if not set_response.success:
            raise PmRobotError(f"Could not set collision between '{link_1}' and '{link_2}' to '{should_check}'")
    
    def set_tool_vaccum(self, state:bool)->bool:
        if not self.client_turn_on_vacuum_tool_head.wait_for_service(1):
            self._node._logger.error(f"Client '{self.client_turn_on_vacuum_tool_head.srv_name}' not available!")
            return False
        
        if not self.client_turn_off_vacuum_tool_head.wait_for_service(1):
            self._node._logger.error(f"Client '{self.client_turn_off_vacuum_tool_head.srv_name}' not available!")
            return False
        
        request=EmptyWithSuccess.Request()

        if state:
            response:EmptyWithSuccess.Response = self.client_turn_on_vacuum_tool_head.call(request)
        else:
            response:EmptyWithSuccess.Response = self.client_turn_off_vacuum_tool_head.call(request)

        return response.success


    def set_gripper_link_collision(self, part_name:str, state:bool):
        """
        Enable or disable collision between the gripper link and a specified part.
        Args:
            part_name (str): The name of the part to set collision with.
            state (bool): True to enable collision, False to disable.
        Raises:
            PmRobotError: If the setting collision fails or if the tool type is unsupported.
        Returns:
            None
        """

        tool_type = self.pm_robot_config.tool.get_active_tool_type()

        if tool_type == self.pm_robot_config.tool._gripper_vacuum.TOOL_VACUUM_IDENT:
            gripper_tip = self.pm_robot_config.tool.get_tool().get_current_tool_attachment()

        elif tool_type == self.pm_robot_config.tool._gripper_1_jaw.TOOL_GRIPPER_1_JAW_IDENT:
            raise PmRobotError("Not yet implemented for 1 jaw gripper!")

        elif tool_type == self.pm_robot_config.tool._gripper_2_jaw.TOOL_GRIPPER_2_JAW_IDENT:
            raise PmRobotError("Not yet implemented for 2 jaw gripper!")

        tip_name = 'PM_Robot_Vacuum_Tool_Tip'

        self.set_collision(part_name, 
                            gripper_tip,
                            state)
            
    def set_gripper_component_collision(self, component_name:str, state:bool):
        """
        Enable or disable collision between the gripped component and a specified part.
        Args:
            part_name (str): The name of the part to set collision with.
            state (bool): True to enable collision, False to disable.
        Raises:
            PmRobotError: If the setting collision fails or if the tool type is unsupported.
            ComponentNotFoundError: If the gripped component is not found.
        Returns:
            None
        """

        self.assembly_scene_analyzer.wait_for_initial_scene_update()
        self.assembly_scene_analyzer.get_component_by_name(component_name)

        self.set_gripper_link_collision(component_name, state)

    def set_gonio_left_vacuum(self, state:bool)->bool:
        if not self.client_turn_on_gonio_left_vacuum.wait_for_service(1):
            self._node._logger.error(f"Client '{self.client_turn_on_gonio_left_vacuum.srv_name}' not available!")
            return False

        if not self.client_turn_off_gonio_left_vacuum.wait_for_service(1):
            self._node._logger.error(f"Client '{self.client_turn_off_gonio_left_vacuum.srv_name}' not available!")
            return False
        
        request=EmptyWithSuccess.Request()

        if state:
            response:EmptyWithSuccess.Response = self.client_turn_on_gonio_left_vacuum.call(request)
        else:
            response:EmptyWithSuccess.Response = self.client_turn_off_gonio_left_vacuum.call(request)

        return response.success
    
    def set_gonio_right_vacuum(self, state:bool)->bool:
        if not self.client_turn_on_gonio_right_vacuum.wait_for_service(1):
            self._node._logger.error(f"Client '{self.client_turn_on_gonio_right_vacuum.srv_name}' not available!")
            return False

        if not self.client_turn_off_gonio_right_vacuum.wait_for_service(1):
            self._node._logger.error(f"Client '{self.client_turn_off_gonio_right_vacuum.srv_name}' not available!")
            return False
        
        request=EmptyWithSuccess.Request()

        if state:
            response:EmptyWithSuccess.Response = self.client_turn_on_gonio_right_vacuum.call(request)
        else:
            response:EmptyWithSuccess.Response = self.client_turn_off_gonio_right_vacuum.call(request)

        return response.success
    
    def get_transform_for_frame(self, 
                                frame_name: str, 
                                parent_frame:str) -> Transform:
        # this function adapts the tf for parent_frame changes
        #transform:TransformStamped = tf_buffer.lookup_transform(frame_name, 'world',rclpy.time.Time())
        try:
            
            transform_st:TransformStamped = self.tf_buffer.lookup_transform(parent_frame, frame_name, rclpy.time.Time(),rclpy.duration.Duration(seconds=1.0))
            self._node.get_logger().debug(f"Frame '{frame_name}' found in TF!")

        except Exception as e:
            transform_st = None
            raise ValueError(f"Frame '{frame_name}' does not exist in TF! {str(e)}")
        
        transform = Transform()
        transform.translation.x = transform_st.transform.translation.x
        transform.translation.y = transform_st.transform.translation.y
        transform.translation.z = transform_st.transform.translation.z

        transform.rotation.x = transform_st.transform.rotation.x
        transform.rotation.y = transform_st.transform.rotation.y
        transform.rotation.z = transform_st.transform.rotation.z
        transform.rotation.w = transform_st.transform.rotation.w

        return transform

    def move_laser_to_frame(self, frame_name:str, z_offset=0.0, y_offset=0.0, x_offset=0.0)-> bool:
        """
        Move the laser to the specified frame with the given offsets (in m).
        """
        if not self.client_move_robot_laser_to_frame.wait_for_service(timeout_sec=1.0):
            self._node._logger.error("Service '/pm_moveit_server/move_laser_to_frame' not available")
            return False
        
        req = MoveToFrame.Request()
        req.target_frame = frame_name
        req.execute_movement = True

        req.translation.z = z_offset
        req.translation.y = y_offset
        req.translation.x = x_offset

        response:MoveToFrame.Response = self.client_move_robot_laser_to_frame.call(req)
        return response.success

    def recalculate_assembly_instruction(self, instruction_id:str):
        if not self.client_recalculate_assembly_instruction.wait_for_service(timeout_sec=1.0):
            raise PmRobotError(f"Service '{self.client_recalculate_assembly_instruction.srv_name}' not available")
        
        req = ami_srv.CalculateAssemblyInstructions.Request()
        req.instruction_id = instruction_id

        response:ami_srv.CalculateAssemblyInstructions.Response = self.client_recalculate_assembly_instruction.call(req)

        if not response.success:
            raise PmRobotError(f"Failed to recalculate assembly instruction: {instruction_id}")

    def move_confocal_top_to_frame(self, frame_name:str, z_offset=0.0, y_offset=0.0, x_offset=0.0)-> bool:
        """
        Move the confocal top to the specified frame with the given offsets (in m).
        """
        if not self.client_move_robot_confocal_top_to_frame.wait_for_service(timeout_sec=1.0):
            self._node._logger.error(f"Service '{self.client_move_robot_confocal_top_to_frame.srv_name}' not available")
            return False
        
        req = MoveToFrame.Request()
        req.target_frame = frame_name
        req.execute_movement = True

        req.translation.z = z_offset
        req.translation.y = y_offset
        req.translation.x = x_offset

        response:MoveToFrame.Response = self.client_move_robot_confocal_top_to_frame.call(req)

        return response.success
    
    def move_camera_top_to_frame(self, frame_name:str, 
                                 z_offset:float = 0.0, 
                                 y_offset:float = 0.0,
                                 x_offset:float = 0.0,
                                 endeffector_override: str = None)-> bool:
        """
        Move the camera top to the specified frame with the given offsets (in m).
        """
        
        if not self.client_move_robot_cam1_to_frame.wait_for_service(timeout_sec=1.0):
            self._node._logger.error(f"Service '{self.client_move_robot_cam1_to_frame.srv_name}' not available")
            return False
        
        req = MoveToFrame.Request()
        req.target_frame = frame_name
        req.execute_movement = True

        if endeffector_override is not None:
            req.endeffector_frame_override = endeffector_override

        req.translation.z = z_offset
        req.translation.y = y_offset
        req.translation.x = x_offset

        response:MoveToFrame.Response = self.client_move_robot_cam1_to_frame.call(req)
        
        return response.success
    
    def create_ref_frame(self, ref_frame_request: ami_srv.CreateRefFrame.Request)->ami_srv.CreateRefFrame.Response:
        """
        Create a reference frame in the assembly manager.
        param ref_frame_request: ami_srv.CreateRefFrame.Request
        return: ami_srv.CreateRefFrame.Response
        Raises PmRobotError if the service call fails or if the response indicates failure.
        """
        if not self.client_create_ref_frame.wait_for_service(timeout_sec=1.0):
            raise PmRobotError(f"Service '{self.client_create_ref_frame.srv_name}' not available")
        
        response:ami_srv.CreateRefFrame.Response = self.client_create_ref_frame.call(ref_frame_request)

        if not response.success:
            raise PmRobotError(f"Failed to create reference frame: {response.message}")

        return response
    
    def move_1k_dispenser_to_frame(self, frame_name:str, 
                                 z_offset:float = 0.0, 
                                 y_offset:float = 0.0,
                                 x_offset:float = 0.0):
        """
        Move the camera top to the specified frame with the given offsets (in m).
        """
        
        if not self.client_move_robot_1k_dispenser_to_frame.wait_for_service(timeout_sec=1.0):
            raise PmRobotError(f"Service '{self.client_move_robot_1k_dispenser_to_frame.srv_name}' not available")
        
        req = MoveToFrame.Request()
        req.target_frame = frame_name
        req.execute_movement = True

        req.translation.z = z_offset
        req.translation.y = y_offset
        req.translation.x = x_offset

        response:MoveToFrame.Response = self.client_move_robot_1k_dispenser_to_frame.call(req)

        if not response.success:
            raise PmRobotError(f"Failed to move 1k dispenser: {response.message}")

    def check_reference_cube_pressed(self)->bool:

        if not self.client_check_reference_cube.wait_for_service(timeout_sec=1.0):
            raise PmRobotError (f"Service '{self.client_check_reference_cube.srv_name}' not available")

        req = ReferenceCubeState.Request()
        response: ReferenceCubeState.Response = self.client_check_reference_cube.call(req)

        return response.pressed
    
    def update_pm_robot_config(self):
        if self.get_mode() == self.REAL_MODE:
            self.pm_robot_config.set_real_HW(True)
        else:
            self.pm_robot_config.set_real_HW(False)

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
        self._node._logger.warn("abs: " + str(abs_length))

        max_length = max(abs_length)
        self._node._logger.warn("max abs: " + str(max_length))

        total_steps = int(max_length / step_inc)
        self._node._logger.warn("Total steps: " + str(total_steps))

        step_time = total_time / total_steps
        
        x_step = length[0] / total_steps
        y_step = length[1] / total_steps
        z_step = length[2] / total_steps

        # log all the values
        # self._node._logger.warn("Step time: " + str(step_time))
        # self._node._logger.warn("X step: " + str(x_step))
        # self._node._logger.warn("Y step: " + str(y_step))
        # self._node._logger.warn("Z step: " + str(z_step))
        # self._node._logger.warn("Length: " + str(length))
        
        for i in range(total_steps):
            self._node._logger.warn(f"Executing iteration {i}/{total_steps}")
            move_success = self.send_xyz_trajectory_goal_relative(x_joint_rel = x_step*1e-3,
                                                                    y_joint_rel = y_step*1e-3, 
                                                                    z_joint_rel = z_step*1e-3, 
                                                                    time = step_time)
            
            if not move_success:
                self._node._logger.error("Failed to move laser ROUTINE 2")
                return False
            
            time.sleep(0.1)

            #laser_measurement = measurement_method(unit='um')

            if measurement_valid_function():
                current_x_joint_result = self.get_current_joint_state(PmRobotUtils.X_Axis_JOINT_NAME)
                current_y_joint_result = self.get_current_joint_state(PmRobotUtils.Y_Axis_JOINT_NAME)
                current_z_joint_result = self.get_current_joint_state(PmRobotUtils.Z_Axis_JOINT_NAME)

                if measurement_method is not None:
                    measurement = measurement_method(unit='um')
                    self._node._logger.info(f"Measurement is {measurement} um.")

                return (current_x_joint_result, current_y_joint_result, current_z_joint_result)

        return None, None, None

    def open_protection(self):

        if not self._open_protection_real_srv.wait_for_service(timeout_sec=1.0):
            raise PmRobotError(f"Service '{self._open_protection_real_srv.srv_name}' not available.")

        req = EmptyWithSuccess.Request()
        response:EmptyWithSuccess.Response = self._open_protection_real_srv.call(req)

        if not response.success:
            raise PmRobotError(f"Failed to open protection: {response.message}")

    def close_protection(self):
        if not self._close_protection_real_srv.wait_for_service(timeout_sec=1.0):
            raise PmRobotError(f"Service '{self._close_protection_real_srv.srv_name}' not available.")

        req = EmptyWithSuccess.Request()
        response:EmptyWithSuccess.Response = self._close_protection_real_srv.call(req)
        if not response.success:
            raise PmRobotError(f"Failed to close protection: {response.message}")

    def retract_dispenser(self):

        if not self._retract_dispenser_real_srv.wait_for_service(timeout_sec=1.0):
            raise PmRobotError(f"Service '{self._retract_dispenser_real_srv.srv_name}' not available.")

        req = EmptyWithSuccess.Request()
        response: EmptyWithSuccess.Response = self._retract_dispenser_real_srv.call(req)

        if not response.success:
            raise PmRobotError(f"Failed to retract dispenser: {response.message}")

    def extend_dispenser(self):

        if not self._extend_dispenser_real_srv.wait_for_service(timeout_sec=1.0):
            raise PmRobotError(f"Service '{self._extend_dispenser_real_srv.srv_name}' not available.")

        req = EmptyWithSuccess.Request()
        response:EmptyWithSuccess.Response = self._extend_dispenser_real_srv.call(req)

        if not response.success:
            raise PmRobotError(f"Failed to extend dispenser: {response.message}")

    @staticmethod
    def _transform_to_dict(transform):
        ''' 
        Converts a TransformStamped message to a dictionary format.
        '''
        return {
            'header': {
                'stamp': {
                    'sec': transform.header.stamp.sec,
                    'nanosec': transform.header.stamp.nanosec
                },
                'frame_id': transform.header.frame_id
            },
            'child_frame_id': transform.child_frame_id,
            'transform': {
                'translation': {
                    'x': transform.transform.translation.x,
                    'y': transform.transform.translation.y,
                    'z': transform.transform.translation.z
                },
                'rotation': {
                    'x': transform.transform.rotation.x,
                    'y': transform.transform.rotation.y,
                    'z': transform.transform.rotation.z,
                    'w': transform.transform.rotation.w
                }
            }
        }
    
    def _get_multiplier(self, unit:str)->float:
        """
        Get the multiplier for the unit.
        
        Args:
            unit (str): Unit of measurement.
        
        Returns:
            float: Multiplier for the unit.
        """
        if unit == "mm":
            return 1e-3
        elif unit == "m":
            return 1e-6
        elif unit == "um":
            return 1.0
        else:
            raise ValueError("Invalid unit used in method")
