from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch driver, vision pipeline, and autonomous sorter for single xArm 1S system."""
    port_arg = DeclareLaunchArgument(
        "port",
        default_value="auto",
        description="Serial or USB HID device path for xArm 1S controller (default 'auto')",
    )
    baudrate_arg = DeclareLaunchArgument(
        "baudrate",
        default_value="115200",
        description="Serial baud rate",
    )
    arm_id_arg = DeclareLaunchArgument(
        "arm_id",
        default_value="xarm_1",
        description="Manipulator instance identifier (defaults to 'xarm_1')",
    )
    camera_device_arg = DeclareLaunchArgument(
        "camera_device",
        default_value="/dev/video0",
        description="Path to V4L2 camera device",
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

    driver_node = Node(
        package="sorting_arm",
        executable="driver_node",
        name="xarm_driver_node",
        namespace=LaunchConfiguration("arm_id"),
        output="screen",
        parameters=[
            {
                "port": LaunchConfiguration("port"),
                "baudrate": LaunchConfiguration("baudrate"),
                "arm_id": LaunchConfiguration("arm_id"),
                "config_path": "config/xarm_hardware.yaml",
            }
        ],
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
                "arm_id": LaunchConfiguration("arm_id"),
                "config_dir": "config",
                "auto_start": LaunchConfiguration("auto_start"),
                "dry_run": LaunchConfiguration("dry_run"),
            }
        ],
    )

    return LaunchDescription(
        [
            port_arg,
            baudrate_arg,
            arm_id_arg,
            camera_device_arg,
            auto_start_arg,
            dry_run_arg,
            driver_node,
            vision_node,
            sorter_node,
        ]
    )
