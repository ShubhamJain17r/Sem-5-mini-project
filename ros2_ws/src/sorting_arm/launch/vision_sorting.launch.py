from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch vision node and autonomous sorter for single xArm."""
    camera_device_arg = DeclareLaunchArgument(
        "camera_device",
        default_value="/dev/video0",
        description="Path to V4L2 camera device",
    )
    target_arm_arg = DeclareLaunchArgument(
        "target_arm",
        default_value="xarm_1",
        description="Target manipulator to command (xarm_1)",
    )
    auto_start_arg = DeclareLaunchArgument(
        "auto_start",
        default_value="false",
        description="Whether to immediately begin autonomous sorting without service call",
    )
    dry_run_arg = DeclareLaunchArgument(
        "dry_run",
        default_value="false",
        description="Run in simulation/dry-run mode without commanding physical servos",
    )

    vision_node = Node(
        package="sorting_arm",
        executable="vision_node",
        name="vision_node",
        output="screen",
        parameters=[
            {
                "camera_device": LaunchConfiguration("camera_device"),
                "config_dir": "config",
                "publish_rate_hz": 10.0,
                "scale_px_per_mm": 2.0,
            }
        ],
    )

    sorter_node = Node(
        package="sorting_arm",
        executable="autonomous_sorter",
        name="autonomous_sorter_node",
        output="screen",
        parameters=[
            {
                "arm_id": LaunchConfiguration("target_arm"),
                "config_dir": "config",
                "auto_start": LaunchConfiguration("auto_start"),
                "dry_run": LaunchConfiguration("dry_run"),
            }
        ],
    )

    return LaunchDescription(
        [
            camera_device_arg,
            target_arm_arg,
            auto_start_arg,
            dry_run_arg,
            vision_node,
            sorter_node,
        ]
    )
