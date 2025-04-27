import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, SetEnvironmentVariable, 
                            IncludeLaunchDescription, SetLaunchConfiguration)
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, FindExecutable, TextSubstitution, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue
from launch_ros.actions import Node
from ros_gz_bridge.actions import RosGzBridge

def generate_launch_description():
    bridge_configuration = PathJoinSubstitution(
        [
            FindPackageShare("wheeled_biped_description"),
            "config",
            "bridge.yaml",
        ]
    )
    ros_gz_bridge = RosGzBridge(
        bridge_name="ros_gz_bridge",
        config_file=bridge_configuration,
    )

    gz_launch_path = PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
    gz_model_path = PathJoinSubstitution([FindPackageShare('wheeled_biped_description'), 'models'])
    model_file_path = PathJoinSubstitution([gz_model_path, 'model.urdf.xacro'])
    empty_world = PathJoinSubstitution([FindPackageShare("wheeled_biped_description"), "models", "empty.sdf"])

    robot_description_config = Command(
        [PathJoinSubstitution([FindExecutable(name="xacro")]), " ", model_file_path]
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{
            "robot_description": robot_description_config,
            "use_sim_time": True,
        }],
    )

    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "robot",
            "-topic", "/robot_description",
            "-x", "0",
            "-y", "0",
            "-z", "0.35",
        ],
        output="both",
    )
    
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("wheeled_biped_control"),
            "config",
            "controllers.yaml",
        ]
    )
    
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "differential_controller",
            "--param-file",
            robot_controllers,
            "--controller-ros-args",
            "-r /differential_controller/cmd_vel:=/cmd_vel",
        ],
    )

    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    return LaunchDescription([
        ros_gz_bridge,
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_model_path),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch_path),
            launch_arguments={
                'gz_args': [empty_world, " -r"], # joint_state_broadcaster requires -r or it fails to init
                'on_exit_shutdown': 'True'
            }.items(),
        ),
        robot_state_publisher_node,
        spawn_entity,
        robot_controller_spawner,
        joint_state_broadcaster_spawner,
    ])
