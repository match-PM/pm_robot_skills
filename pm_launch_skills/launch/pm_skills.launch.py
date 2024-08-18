import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
import yaml
from yaml.loader import SafeLoader
from launch.actions import RegisterEventHandler, DeclareLaunchArgument
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
import sys


def generate_launch_description():

    sim_time = True

    # Configure the node
    launch_vision_skills = Node(
        package='pm_vision_skills',
        executable='measure_frame',
        name="pm_vision_skills",
        output='both',
        parameters=[
            {'use_sim_time': sim_time}],
        emulate_tty=True,
        arguments=[('__log_level:=debug')]
    )

    launch_vision_ = Node(
        package='pm_vision_skills',
        executable='measure_frame',
        name="pm_vision_skills",
        output='both',
        parameters=[
            {'use_sim_time': sim_time}],
        emulate_tty=True,
        arguments=[('__log_level:=debug')]
    )

    launch_vision = IncludeLaunchDescription(PythonLaunchDescriptionSource([
        PathJoinSubstitution([
            FindPackageShare('pm_vision_manager'),
            'launch',
            'pm_vision.launch.py'
            ])
        ])
        )
  
    launch_assembly_manager = IncludeLaunchDescription(PythonLaunchDescriptionSource([
        PathJoinSubstitution([
            FindPackageShare('assembly_manager'),
            'launch',
            'assembly_manager.launch.py'
            ])
        ])
        )

    pm_robot_calibration = Node(
        package='pm_robot_calibration',
        executable='pm_robot_calibraion',
        name="pm_robot_calibraion",
        output='both',
        parameters=[
            {'use_sim_time': sim_time}],
        emulate_tty=True,
        arguments=[('__log_level:=debug')]
    )

    ld = LaunchDescription()

    #ld.add_action(declare_world)
    ld.add_action(launch_vision_skills)
    ld.add_action(launch_vision)
    ld.add_action(launch_assembly_manager)
    ld.add_action(pm_robot_calibration)

    return ld
