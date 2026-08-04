from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    # 基础参数与场景参数统一由 astra_bringup 包安装与提供。
    default_base_config = PathJoinSubstitution(
        [FindPackageShare("astra_bringup"), "config", "astra_nav.yaml"]
    )
    default_scenario_config = PathJoinSubstitution(
        [FindPackageShare("astra_bringup"), "config", "scenarios", "default.yaml"]
    )
    base_config = LaunchConfiguration("base_config")
    scenario_config = LaunchConfiguration("scenario_config")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "base_config",
                default_value=default_base_config,
                description="基础导航参数文件。",
            ),
            DeclareLaunchArgument(
                "scenario_config",
                default_value=default_scenario_config,
                description="仿真场景覆盖参数文件。",
            ),
            # 仿真世界节点（astra_simulation 包）。
            Node(
                package="astra_simulation",
                executable="sim_world_node",
                name="sim_world_node",
                output="screen",
                parameters=[base_config, scenario_config],
            ),
            # 里程计接口代理节点（astra_localization 包）。
            Node(
                package="astra_localization",
                executable="batch_liwo_proxy_node",
                name="batch_liwo_proxy_node",
                output="screen",
                parameters=[base_config, scenario_config],
            ),
            # 占据栅格 / 二维障碍图节点（astra_mapping 包）。
            Node(
                package="astra_mapping",
                executable="occupancy_node",
                name="occupancy_node",
                output="screen",
                parameters=[base_config, scenario_config],
            ),
            # 规划节点（astra_planning 包）。
            Node(
                package="astra_planning",
                executable="planner_node",
                name="planner_node",
                output="screen",
                parameters=[base_config, scenario_config],
            ),
            # 控制节点（astra_control 包）。
            Node(
                package="astra_control",
                executable="controller_node",
                name="controller_node",
                output="screen",
                parameters=[base_config, scenario_config],
            ),
        ]
    )
