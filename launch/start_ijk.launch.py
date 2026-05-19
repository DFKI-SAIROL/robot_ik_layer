import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

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
            ns = config['namespace']
            print(f"Spawning controllers for: {item_name} (Namespace: {ns})")

            # 1. The Safety Node (Arbiter & Safe Target Generator)
            nodes.append(
                Node(
                    package="robot_safety_layer",
                    executable="franka_safety_node",
                    name="safety_node",
                    namespace=ns,
                    parameters=[
                        safety_config_file,
                        {
                            'arm_prefix': ns,
                            'bypass_safety': bypass_safety,
                            'init_joint_position': config['init_joint_position'],
                        }
                    ],
                    output="screen",
                )
            )

            # 2. The IK Node
            nodes.append(
                Node(
                    package="robot_ik_layer",
                    executable="ik_node",
                    name="ik_node",
                    namespace=ns,
                    parameters=[
                        safety_config_file, # Shares some tuning params
                        {
                            'arm_prefix': ns,
                            'init_joint_position': config['init_joint_position'],
                            'end_effector_frame': str(config['end_effector_frame']),
                        }
                    ],
                    output="screen",
                )
            )

    return nodes

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('franka_launch'), 'config', 'dfki_bimanual.yaml'
            ]),
            description='Path to the robot configuration file',
        ),
        DeclareLaunchArgument(
            'safety_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('robot_safety_layer'), 'config', 'safety_params.yaml'
            ]),
            description='Path to the safety parameters config file',
        ),
        DeclareLaunchArgument("spawn_franka_main", default_value="false"),
        DeclareLaunchArgument("spawn_franka_left", default_value="true"),
        DeclareLaunchArgument("spawn_franka_right", default_value="true"),
        DeclareLaunchArgument("bypass_safety", default_value="false"),

        OpaqueFunction(function=generate_robot_nodes),
    ])
