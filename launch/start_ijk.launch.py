from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

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
    bypass_safety = LaunchConfiguration('bypass_safety').perform(context).lower() == 'true'
    safety_config_file = LaunchConfiguration('safety_config_file').perform(context)

    spawn_robots = []
    if LaunchConfiguration('spawn_franka_main').perform(context).lower() == 'true':
        spawn_robots.append("franka_main")
    if LaunchConfiguration('spawn_franka_left').perform(context).lower() == 'true':
        spawn_robots.append("franka_left")
    if LaunchConfiguration('spawn_franka_right').perform(context).lower() == 'true':
        spawn_robots.append("franka_right")
    
    for item_name, config in configs.items():
        if item_name in spawn_robots: 
            print("Spawn", item_name)

            nodes.append(
                Node(
                    package="franka_safety_layer",
                    executable="safety_node",
                    name="safety_node",
                    namespace=config['namespace'],
                    parameters=[
                        safety_config_file,
                        {
                            'arm_id': str(config['arm_id']),
                            'arm_prefix': str(config['namespace']),
                            'init_joint_position': config['init_joint_position'],
                            'bypass_safety': bypass_safety,
                        }
                    ],
                    output="screen",
                )
            )

    return nodes


def generate_launch_description():
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            'robot_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('franka_launch'), 'config', 'dfki_bimanual.yaml'
            ]),
            description='Path to the robot configuration file to load',
        ),
        DeclareLaunchArgument(
            'safety_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('franka_safety_layer'), 'config', 'safety_params.yaml'
            ]),
            description='Path to the safety parameters config file',
        ),
        DeclareLaunchArgument(
            "spawn_franka_main",
            default_value="false",
            description="Spawn franka main",
        ),
        DeclareLaunchArgument(
            "spawn_franka_left",
            default_value="true",
            description="Spawn franka left",
        ),
        DeclareLaunchArgument(
            "spawn_franka_right",
            default_value="true",
            description="Spawn franka right",
        ),
        DeclareLaunchArgument(
            "bypass_safety",
            default_value="false",
            description="Bypass safety layer",
        ),
        OpaqueFunction(function=generate_robot_nodes),
    ])
