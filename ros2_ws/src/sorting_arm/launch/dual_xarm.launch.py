from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch driver nodes for both xArm manipulators in isolated namespaces."""
    arm1_port_arg = DeclareLaunchArgument(
        "arm1_port",
        default_value="auto",
        description="Serial or USB HID device path for xarm_1 (defaults to auto-detection)",
    )
    arm2_port_arg = DeclareLaunchArgument(
        "arm2_port",
        default_value="auto",
        description="Serial or USB HID device path for xarm_2 (defaults to auto-detection)",
    )
    baudrate_arg = DeclareLaunchArgument(
        "baudrate",
        default_value="115200",
        description="Controller baud rate",
    )
    launch_arm1_arg = DeclareLaunchArgument(
        "launch_arm_1",
        default_value="true",
        description="Whether to launch driver for xarm_1 (Back / North arm)",
    )
    launch_arm2_arg = DeclareLaunchArgument(
        "launch_arm_2",
        default_value="true",
        description="Whether to launch driver for xarm_2 (Front / South arm)",
    )

    driver_node_arm1 = Node(
        package="sorting_arm",
        executable="driver_node",
        name="xarm_driver_node",
        namespace="xarm_1",
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_arm_1")),
        parameters=[
            {
                "port": LaunchConfiguration("arm1_port"),
                "baudrate": LaunchConfiguration("baudrate"),
                "arm_id": "xarm_1",
            }
        ],
    )

    driver_node_arm2 = Node(
        package="sorting_arm",
        executable="driver_node",
        name="xarm_driver_node",
        namespace="xarm_2",
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_arm_2")),
        parameters=[
            {
                "port": LaunchConfiguration("arm2_port"),
                "baudrate": LaunchConfiguration("baudrate"),
                "arm_id": "xarm_2",
            }
        ],
    )

    return LaunchDescription([
        arm1_port_arg,
        arm2_port_arg,
        baudrate_arg,
        launch_arm1_arg,
        launch_arm2_arg,
        driver_node_arm1,
        driver_node_arm2,
    ])
