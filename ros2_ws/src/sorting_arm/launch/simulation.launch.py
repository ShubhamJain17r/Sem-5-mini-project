import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def get_asset_path(rel_path: str) -> str:
    """Resolve file path checking package share directory and local workspace fallbacks."""
    try:
        share_dir = get_package_share_directory("sorting_arm")
        p = os.path.join(share_dir, rel_path)
        if os.path.exists(p):
            return os.path.abspath(p)
    except Exception:
        pass

    candidates = [
        os.path.join(os.getcwd(), "ros2_ws", "src", "sorting_arm", rel_path),
        os.path.join("/workspace", "ros2_ws", "src", "sorting_arm", rel_path),
        os.path.join(os.path.dirname(__file__), "..", rel_path),
    ]
    for c in candidates:
        if os.path.exists(c):
            return os.path.abspath(c)
    return rel_path


def generate_launch_description():
    """Launch simulation environment: mock driver, URDF robot_state_publisher, RViz2, and autonomous sorter."""
    arm_id_arg = DeclareLaunchArgument(
        "arm_id",
        default_value="xarm_1",
        description="Manipulator instance identifier (defaults to 'xarm_1')",
    )
    sim_arg = DeclareLaunchArgument(
        "sim",
        default_value="true",
        description="Run driver in mock simulation mode",
    )
    gui_arg = DeclareLaunchArgument(
        "gui",
        default_value="true",
        description="Launch RViz2 3D digital twin visualizer",
    )
    visualizer_arg = DeclareLaunchArgument(
        "visualizer",
        default_value="true",
        description="Launch arena 5cm grid and interactive cube visualizer",
    )
    mock_vision_arg = DeclareLaunchArgument(
        "mock_vision",
        default_value="false",
        description="Launch mock vision node to publish test object scenarios",
    )
    scenario_arg = DeclareLaunchArgument(
        "scenario",
        default_value="dual_cube",
        description="Synthetic vision scenario ('dual_cube', 'single_red', 'single_green', 'four_cubes', 'unreachable')",
    )
    auto_start_arg = DeclareLaunchArgument(
        "auto_start",
        default_value="false",
        description="Whether to immediately begin autonomous sorting without service call",
    )
    dry_run_arg = DeclareLaunchArgument(
        "dry_run",
        default_value="false",
        description="Run sorter in dry-run mode without publishing joint commands",
    )
    grid_mode_arg = DeclareLaunchArgument(
        "grid_mode",
        default_value="full_grid",
        description="Grid population mode: 'full_grid' (all 28 cells) or 'dual' (2 test cubes)",
    )
    auto_reseed_arg = DeclareLaunchArgument(
        "auto_reseed",
        default_value="true",
        description="Automatically reseed and re-run with new scattered pattern when all objects are sorted",
    )
    mirror_only_arg = DeclareLaunchArgument(
        "mirror_only",
        default_value="false",
        description="Mirror external joint_states and detected_objects in RViz2 without mock driver/sorter",
    )
    default_cubes_arg = DeclareLaunchArgument(
        "default_cubes",
        default_value="true",
        description="Spawn default simulation cubes on arena visualizer startup",
    )

    urdf_file = get_asset_path(os.path.join("urdf", "xarm_1s.urdf"))
    rviz_config_file = get_asset_path(os.path.join("rviz", "xarm_simulation.rviz"))

    robot_desc = ""
    if os.path.exists(urdf_file):
        with open(urdf_file, "r") as f:
            robot_desc = f.read()

    arm_id = LaunchConfiguration("arm_id")

    # 1. Robot State Publisher (publishes 3D /tf tree from URDF and joint_states)
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_desc,
            "publish_frequency": 30.0,
        }],
        remappings=[
            ("/joint_states", PythonExpression(["'/' + '", arm_id, "' + '/joint_states'"])),
        ],
    )

    # 2. Mock Hardware Driver Node (interpolates simulated servo positions at 20 Hz)
    driver_node = Node(
        package="sorting_arm",
        executable="driver_node",
        name="xarm_driver_node",
        namespace=arm_id,
        output="screen",
        condition=UnlessCondition(LaunchConfiguration("mirror_only")),
        parameters=[{
            "port": "mock",
            "arm_id": arm_id,
            "sim": LaunchConfiguration("sim"),
            "config_path": "config/xarm_hardware.yaml",
            "adaptive_timing": True,
        }],
    )

    # 3. Autonomous Sorting Executive Node
    sorter_node = Node(
        package="sorting_arm",
        executable="autonomous_sorter",
        name="autonomous_sorter_node",
        output="screen",
        condition=UnlessCondition(LaunchConfiguration("mirror_only")),
        parameters=[{
            "arm_id": arm_id,
            "config_dir": "config",
            "auto_start": LaunchConfiguration("auto_start"),
            "dry_run": LaunchConfiguration("dry_run"),
            "auto_reseed": LaunchConfiguration("auto_reseed"),
        }],
    )

    # 4. Mock Vision Perception Node
    mock_vision_node = Node(
        package="sorting_arm",
        executable="mock_vision_node",
        name="mock_vision_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("mock_vision")),
        parameters=[{
            "arm_id": arm_id,
            "config_dir": "config",
            "scenario": LaunchConfiguration("scenario"),
            "publish_rate_hz": 1.0,
            "loop": False,
        }],
    )

    # 5. Arena Grid & Cube Visualizer Node
    arena_visualizer_node = Node(
        package="sorting_arm",
        executable="arena_visualizer",
        name="arena_visualizer_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("visualizer")),
        parameters=[{
            "arm_id": arm_id,
            "config_dir": "config",
            "default_cubes": LaunchConfiguration("default_cubes"),
            "grid_mode": LaunchConfiguration("grid_mode"),
        }],
    )

    # 6. RViz2 3D Digital Twin Visualizer
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(LaunchConfiguration("gui")),
        arguments=["-d", rviz_config_file],
    )

    return LaunchDescription([
        arm_id_arg,
        sim_arg,
        gui_arg,
        visualizer_arg,
        mock_vision_arg,
        scenario_arg,
        auto_start_arg,
        dry_run_arg,
        grid_mode_arg,
        auto_reseed_arg,
        mirror_only_arg,
        default_cubes_arg,
        robot_state_publisher_node,
        driver_node,
        sorter_node,
        arena_visualizer_node,
        mock_vision_node,
        rviz_node,
    ])
