from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        "port",
        default_value="auto",
        description="Serial or USB HID port for xArm 1S controller (default 'auto')",
    )
    baudrate_arg = DeclareLaunchArgument(
        "baudrate",
        default_value="115200",
        description="Serial baud rate",
    )
    arm_id_arg = DeclareLaunchArgument(
        "arm_id",
        default_value="xarm_1",
        description="Manipulator instance identifier",
    )

    driver_node = Node(
        package="sorting_arm",
        executable="driver_node",
        name="xarm_driver_node",
        output="screen",
        parameters=[
            {
                "port": LaunchConfiguration("port"),
                "baudrate": LaunchConfiguration("baudrate"),
                "arm_id": LaunchConfiguration("arm_id"),
            }
        ],
    )

    return LaunchDescription([
        port_arg,
        baudrate_arg,
        arm_id_arg,
        driver_node,
    ])
