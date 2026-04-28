from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os




from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue

import os
import yaml

# log launch file with debug
import logging
logging.root.setLevel(logging.INFO)


def load_yaml(file_path):
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"File not found: {file_path}")
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)


def generate_robot_nodes(context):

    nodes = []
    config_file = LaunchConfiguration('robot_config_file').perform(context)
    configs = load_yaml(config_file)


    spawn_robots = []
    if LaunchConfiguration('spawn_franka_main').perform(context).lower() == 'true':
        spawn_robots.append("franka_main")
    if LaunchConfiguration('spawn_franka_left').perform(context).lower() == 'true':
        spawn_robots.append("franka_left")
    if LaunchConfiguration('spawn_franka_right').perform(context).lower() == 'true':
        spawn_robots.append("franka_right")
    
    for item_name, config in configs.items():
        if item_name in spawn_robots: 
            print("Spawn safetly layer for", item_name)

            nodes.append(
                Node(
                    package='franka_safety_layer',
                    executable='safety_node',
                    name='safety_node',
                    namespace=config['namespace'],
                    output='screen',
                    parameters=[]
                )
            )
            nodes.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([
                            FindPackageShare('franka_safety_layer'), 'launch', 'move_group.launch.py'
                        ])
                    ),
                    launch_arguments={
                        'namespace': str(config['namespace']),
                        'robot_ip': str(config['robot_ip']),
                        'load_gripper': str(config['load_gripper']),
                        'use_fake_hardware': str(config['use_fake_hardware']),
                        'fake_sensor_commands': str(config['fake_sensor_commands']),
                    }.items(),
                )
            )


    return nodes


def generate_launch_description():
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            'robot_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('franka_launch'), 'config', 'franka.config.yaml'
            ]),
            description='Path to the robot configuration file to load',
        ),
        DeclareLaunchArgument(
            "spawn_franka_main",
            default_value="false",
            description="Spawn franka main",
        ),
        DeclareLaunchArgument(
            "spawn_franka_left",
            default_value="false",
            description="Spawn franka left",
        ),
        DeclareLaunchArgument(
            "spawn_franka_right",
            default_value="true",
            description="Spawn franka right",
        ),
        OpaqueFunction(function=generate_robot_nodes),
    ])

