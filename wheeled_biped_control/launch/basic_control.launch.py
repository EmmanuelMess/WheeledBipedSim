from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("wheeled_biped_control"),
            "config",
            "controllers.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
    )

    return LaunchDescription(
        [
            control_node,
        ]
    )