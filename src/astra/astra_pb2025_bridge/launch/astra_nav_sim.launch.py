# A 套：Astra 导航（自足启动，不含 nav2 planner/controller）。
#
# 复用 pb2025 的定位（point_lio + small_gicp + map_server）与感知 glue
# （ign_sim_pointcloud_tool + loam_interface + sensor_scan_generation + terrain_analysis）
# 提供 odom / TF / terrain_map；用 pb2025 的 fake_vel_transform 做小陀螺补偿；
# 规划/控制用 Astra（occupancy + planner + controller）；可视化用 Astra 版 RViz（2D Goal Pose 工具）。
#
# 与 B 套（pb2025 原生 nav2，rm_navigation_simulation_launch.py）互斥，二选一启动。
# 前置：先起 rmu_gazebo_simulator 并 unpause。
#
# 目标点：RViz 工具栏点 “2D Goal Pose”（发布 /<ns>/goal_pose），由 Astra planner 响应。
#
# 全部节点在命名空间 red_standard_robot1 下，standalone（非 composable）方式，
# map_server 直接传 yaml_filename，避开 composable 命名空间参数注入问题。

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def launch_setup(context, *args, **kwargs):
    ns = LaunchConfiguration("namespace").perform(context)
    world = LaunchConfiguration("world").perform(context)
    use_rviz = LaunchConfiguration("use_rviz").perform(context)

    pb_bringup = get_package_share_directory("pb2025_nav_bringup")
    pb_params = os.path.join(pb_bringup, "config", "simulation", "nav2_params.yaml")
    map_yaml = os.path.join(pb_bringup, "map", "simulation", world + ".yaml")
    prior_pcd = os.path.join(pb_bringup, "pcd", "simulation", world + ".pcd")

    tf_remap = [("/tf", "tf"), ("/tf_static", "tf_static")]

    # pb2025 节点参数：命名空间根键重写 + 注入 use_sim_time / yaml_filename。
    pb_configured = ParameterFile(
        RewrittenYaml(source_file=pb_params, root_key=ns,
                      param_rewrites={"use_sim_time": "true", "yaml_filename": map_yaml},
                      convert_types=True),
        allow_substs=True)

    def pb_node(pkg, exe, name, extra=None):
        params = [pb_configured] if extra is None else [pb_configured, extra]
        return Node(package=pkg, executable=exe, name=name, namespace=ns,
                    output="screen", parameters=params, remappings=tf_remap)

    # —— 定位：point_lio + small_gicp + map_server + lifecycle_manager（standalone）——
    point_lio = pb_node("point_lio", "pointlio_mapping", "point_lio",
                        {"prior_pcd.prior_pcd_map_path": prior_pcd})
    map_server = pb_node("nav2_map_server", "map_server", "map_server",
                         {"yaml_filename": map_yaml})
    small_gicp = pb_node("small_gicp_relocalization", "small_gicp_relocalization_node",
                         "small_gicp_relocalization", {"prior_pcd_file": prior_pcd})
    lifecycle_localization = Node(
        package="nav2_lifecycle_manager", executable="lifecycle_manager",
        name="lifecycle_manager_localization", namespace=ns, output="screen",
        parameters=[{"use_sim_time": True}, {"autostart": True},
                    {"node_names": ["map_server"]}],
        remappings=tf_remap)

    # —— 感知 glue：点云转换 + loam + sensor_scan_generation + terrain_analysis ——
    pointcloud_tool = pb_node("ign_sim_pointcloud_tool", "ign_sim_pointcloud_tool_node",
                              "ign_sim_pointcloud_tool")
    loam_interface = pb_node("loam_interface", "loam_interface_node", "loam_interface")
    sensor_scan_generation = pb_node("sensor_scan_generation", "sensor_scan_generation_node",
                                     "sensor_scan_generation")
    terrain_analysis = pb_node("terrain_analysis", "terrainAnalysis", "terrain_analysis")
    # 远距地形分析（发 terrain_map_ext，即 RViz 里满屏的紫粉色点云；与原生 pb 对齐）。
    terrain_analysis_ext = pb_node("terrain_analysis_ext", "terrainAnalysisExt",
                                   "terrain_analysis_ext")

    # —— 小陀螺速度补偿：Astra cmd_vel_nav2_result -> 底盘 cmd_vel ——
    fake_vel_transform = pb_node("fake_vel_transform", "fake_vel_transform_node",
                                 "fake_vel_transform")

    # —— Astra 规划/控制（全局命名空间，用 remap 接 pb2025 命名空间话题）——
    p = "/" + ns
    astra_base = PathJoinSubstitution(
        [FindPackageShare("astra_bringup"), "config", "astra_nav.yaml"])
    astra_override = PathJoinSubstitution(
        [FindPackageShare("astra_pb2025_bridge"), "config", "astra_in_pb2025.yaml"])
    astra_params = [astra_base, astra_override]
    astra_tf = [("/tf", p + "/tf"), ("/tf_static", p + "/tf_static")]

    # 全局障碍图节点：融合先验 /map + 实时 terrain_map -> 全场障碍图(map帧)，
    # 并把 odom 帧里程计重投影到 map 帧输出 /astra/odom。取代 occupancy_node + batch_liwo_proxy。
    global_map = Node(package="astra_pb2025_bridge", executable="global_obstacle_map_node",
                      name="global_obstacle_map_node", output="screen", parameters=astra_params,
                      remappings=astra_tf)
    planner = Node(package="astra_planning", executable="planner_node",
                   name="planner_node", output="screen", parameters=astra_params,
                   remappings=astra_tf)
    controller = Node(package="astra_control", executable="controller_node",
                      name="controller_node", output="screen", parameters=astra_params,
                      remappings=astra_tf + [("/cmd_vel", p + "/cmd_vel_nav2_result")])

    # —— 可视化：Astra 版 RViz（含 2D Goal Pose 工具，命名空间下发 /<ns>/goal_pose）——
    rviz_nodes = []
    if use_rviz.lower() in ("true", "1"):
        rviz_cfg = os.path.join(
            get_package_share_directory("astra_pb2025_bridge"), "rviz", "astra_view.rviz")
        rviz_nodes.append(Node(
            package="rviz2", executable="rviz2", name="rviz2", namespace=ns,
            output="screen", arguments=["-d", rviz_cfg], remappings=tf_remap))

    return [point_lio, map_server, small_gicp, lifecycle_localization,
            pointcloud_tool, loam_interface, sensor_scan_generation, terrain_analysis,
            terrain_analysis_ext, fake_vel_transform,
            global_map, planner, controller] + rviz_nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="red_standard_robot1"),
        DeclareLaunchArgument("world", default_value="rmuc_2025"),
        DeclareLaunchArgument("use_rviz", default_value="True"),
        OpaqueFunction(function=launch_setup),
    ])
