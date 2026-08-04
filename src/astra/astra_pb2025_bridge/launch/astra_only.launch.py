# 仅启动 Astra 规划/控制层，接入已在运行的 pb2025 仿真导航栈。
#
# 前置：先启动 rmu_gazebo_simulator 并 unpause，再启动 pb2025 顶层
#   rm_navigation_simulation_launch.py（提供 odom/TF/terrain_map/fake_vel_transform/RViz）。
#
# 设计：Astra 节点运行在【全局命名空间】（这样 astra_nav.yaml 的裸键参数正常生效），
# 通过 remap 把 I/O 话题接到 pb2025 的命名空间话题上。
#   odometry(/ns/odometry) ─> batch_liwo_proxy ─> /astra/odom
#   terrain_map(/ns/terrain_map, odom系) ─> occupancy ─> /astra/obstacle_grid
#   /astra/obstacle_grid + /astra/goal_pose ─> planner ─> /astra/trajectory
#   /astra/trajectory + /astra/odom ─> controller ─> /ns/cmd_vel_nav2_result（接 fake_vel_transform）
# TF：pb2025 发布在 /ns/tf，故把 Astra 的 /tf 订阅 remap 到 /ns/tf。

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    ns = LaunchConfiguration("namespace").perform(context)
    use_rviz = LaunchConfiguration("use_rviz").perform(context)
    p = "/" + ns  # pb2025 命名空间前缀

    astra_base = PathJoinSubstitution(
        [FindPackageShare("astra_bringup"), "config", "astra_nav.yaml"])
    astra_override = PathJoinSubstitution(
        [FindPackageShare("astra_pb2025_bridge"), "config", "astra_in_pb2025.yaml"])
    params = [astra_base, astra_override]

    tf_remap = [("/tf", p + "/tf"), ("/tf_static", p + "/tf_static")]

    # odom 代理：输入 pb2025 的 odometry，输出全局 /astra/odom
    proxy = Node(
        package="astra_localization", executable="batch_liwo_proxy_node",
        name="batch_liwo_proxy_node", output="screen", parameters=params,
        remappings=tf_remap + [
            ("/sim/odom", p + "/odometry"),   # 覆盖默认 input（astra_nav.yaml 里默认 /sim/odom）
        ])

    # 占据图：输入 pb2025 的 terrain_map（odom 系）
    occupancy = Node(
        package="astra_mapping", executable="occupancy_node",
        name="occupancy_node", output="screen", parameters=params,
        remappings=tf_remap + [
            ("/points", p + "/terrain_map"),
        ])

    planner = Node(
        package="astra_planning", executable="planner_node",
        name="planner_node", output="screen", parameters=params,
        remappings=tf_remap)

    # 控制器输出接 pb2025 fake_vel_transform 的输入 cmd_vel_nav2_result
    controller = Node(
        package="astra_control", executable="controller_node",
        name="controller_node", output="screen", parameters=params,
        remappings=tf_remap + [
            ("/cmd_vel", p + "/cmd_vel_nav2_result"),
        ])

    nodes = [proxy, occupancy, planner, controller]

    # 可视化：用 Astra 专用 RViz 配置（派生自 pb2025，含 TF/点云/机器人 + Astra 轨迹/障碍图）。
    # 运行在命名空间下，TF remap 到 /<ns>/tf；建议 pb2025 顶层以 use_rviz:=False 启动，由此处出图。
    if use_rviz.lower() in ("true", "1"):
        rviz_cfg = os.path.join(
            get_package_share_directory("astra_pb2025_bridge"), "rviz", "astra_view.rviz")
        nodes.append(Node(
            package="rviz2", executable="rviz2", name="rviz2",
            namespace=ns, output="screen",
            arguments=["-d", rviz_cfg],
            remappings=[("/tf", p + "/tf"), ("/tf_static", p + "/tf_static")]))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="red_standard_robot1"),
        DeclareLaunchArgument("use_rviz", default_value="False"),
        OpaqueFunction(function=launch_setup),
    ])
