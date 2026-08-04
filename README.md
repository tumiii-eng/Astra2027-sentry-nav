# pb2025-astra-nav

把 MINCO 轨迹优化那套导航方案（2D ESDF + JPS 前端 + MINCO + L-BFGS 两步优化 + SE2 MPC）接到 RoboMaster 2025 哨兵仿真项目 [pb2025_sentry_nav](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav) 上，在 Gazebo Fortress 仿真里发目标点验证。

仿真、里程计、重定位、地形感知这些底层全部沿用 pb2025 原生的那一套，Astra 只替换「规划 + 控制」这一层。所以同一套仿真下可以起两种导航来对照：

| | 全局规划 | 控制 |
| --- | --- | --- |
| **A 套** Astra | JPS 前端 + MINCO 轨迹 + L-BFGS 优化 | SE2 MPC / preview 控制器 |
| **B 套** pb2025 原生 | nav2 + Theta\* | omni PID pursuit |

两套互斥，不能同时跑。

## 仓库里有什么

只放自己写的代码。上游那 8 个仓库（310MB）不进版本库，用 `upstream.repos` + 一个脚本拉取，这样上游更新和本地改动不会互相污染。

```
src/astra/                      Astra 导航算法（10 个 ROS 2 包）
├── astra_common/               通用数据结构与数学
├── astra_third_party/          GCOPTER / DDR-opt 派生头文件（MIT）
├── astra_perception/           点云预处理、3D 滚动占据栅格
├── astra_mapping/              障碍提取、2D ESDF、障碍图节点
├── astra_planning/             JPS、局部目标、清障、MINCO、L-BFGS、规划节点
├── astra_control/              轨迹跟踪控制器、底盘速度输出
├── astra_localization/         里程计接口代理
├── astra_simulation/           本地仿真世界节点
├── astra_bringup/              聚合 launch 与默认参数
└── astra_pb2025_bridge/        ★ 接入 pb2025 的胶水层（本工程新写）

Dockerfile / compose.yaml       统一开发容器（仿真 + 导航同一个容器）
docker/entrypoint.sh
upstream.repos                  上游依赖清单，锁定 commit
tools/setup_upstream.sh         拉取上游 + 打补丁
docs/DEBUG_NOTES.md             调试笔记：已解决的问题、还没修的 BUG
```

胶水层 `astra_pb2025_bridge` 是接入的关键，不改 Astra 也不改 pb2025 的源码：

- `launch/astra_nav_sim.launch.py` — A 套主 launch，一条命令起全套
- `src/global_obstacle_map_node.cpp` — 提供 map 帧的全场障碍图，并把 odom 重投影到 map 帧
- `config/astra_in_pb2025.yaml` — 参数覆盖
- `rviz/astra_view.rviz` — RViz 配置，加了 Astra 的障碍图、膨胀图、轨迹显示

## 环境

- Ubuntu 22.04 + ROS 2 Humble + Gazebo Sim 6.17.0（Fortress）
- NVIDIA 显卡 + nvidia-container-toolkit（仿真渲染要用）
- Docker 与 Docker Compose v2
- 宿主机需要 `python3-vcstool`（拉上游用）

开发验证环境是 RTX 4060 Laptop，镜像约 7.9GB。

## 部署

### 1. 克隆并拉上游

```bash
git clone https://github.com/tumiii-eng/pb2025-astra-nav.git pb2025_ws
cd pb2025_ws
sudo apt install python3-vcstool     # 没装的话
./tools/setup_upstream.sh
```

脚本做三件事：按 `upstream.repos` 锁定的 commit 拉 8 个上游仓库到 `src/`、初始化 pb2025_sentry_nav 的 7 个子模块、给 5 个 `CMakeLists.txt` 打补丁（删掉 `USE_SCOPED_HEADER_INSTALL_DIR`，上游用的 ament_cmake_auto 版本不支持这个选项，留着编译会失败）。脚本是幂等的，重复跑没问题。

### 2. 下载先验点云

point_lio 和 small_gicp 重定位要用先验 PCD。文件太大不放 git，从上游给的网盘下载：[FlowUs 链接](https://flowus.cn/lihanchen/share/87f81771-fc0c-4e09-a768-db01f4c136f4?code=4PP1RS)，取 `rmul_2024 / rmuc_2024 / rmul_2025 / rmuc_2025` 四个 `.pcd`，放到：

```
src/pb2025_sentry_nav/pb2025_nav_bringup/pcd/simulation/
```

只跑仿真的话，最少需要 `rmuc_2025.pcd`。

### 3. 核对 .env

`.env` 里几个值跟宿主机环境相关，UID/GID 不是 1000 或者显示服务不是 GDM 的话要改：

```bash
LOCAL_UID=1000        # id -u
LOCAL_GID=1000        # id -g
DISPLAY=:0
HOST_XAUTHORITY=/run/user/1000/gdm/Xauthority
ROS_DOMAIN_ID=0
```

### 4. 建镜像并编译

```bash
xhost +local:docker          # 让容器能开 GUI 窗口
docker compose up -d --build
docker compose exec dev bash -lc 'cd /workspace && colcon build --symlink-install'
```

首次构建镜像十几分钟（含从源码编 small_gicp），29 个包编译也要一会儿。

容器内工作空间挂在 `/workspace`，**不是** pb2025 上游 README 里写的 `/root/ros_ws`。compose 的服务名是 `dev`，容器名 `pb2025-dev`。

## 跑起来

下面命令都在宿主机普通终端敲，`docker exec` 会带你进容器。机器人命名空间 `red_standard_robot1` 已经写死在命令里了。

### 第 0 步 确认没有残留进程

应该输出 `0`。不是 0 就先执行文末的「关闭所有进程」。

```bash
docker exec pb2025-dev bash -lc 'ps aux | grep -E "ign gazebo|ros2 launch|pointlio|rviz2" | grep -v grep | wc -l'
```

### 第 1 步 起仿真，等 40 秒

```bash
docker exec pb2025-dev bash -lc 'source /opt/ros/humble/setup.bash; source /workspace/install/setup.bash; export DISPLAY=:0; setsid ros2 launch rmu_gazebo_simulator bringup_sim.launch.py > /tmp/sim.log 2>&1 &'
```

日志里 `simple_competition_1v1.py ... Exec format error` 是仿真自带的无害小 bug，忽略。`libEGL: failed to create dri2 screen` 是 mesa 探测的告警，也不影响。

### 第 2 步 解除仿真暂停

**这步必须做。** 仿真世界启动后默认是暂停的，此时 `/clock` 会走、真值里程计有数据，但激光雷达和 IMU 完全不发布，机器人看起来像「消失」了。

有 GUI 的话点 Gazebo 左下角橙红色的启动按钮；命令行方式：

```bash
docker exec pb2025-dev bash -lc 'source /opt/ros/humble/setup.bash; ign service -s /world/default/control --reqtype ignition.msgs.WorldControl --reptype ignition.msgs.Boolean --timeout 5000 --req "pause: false"'
```

返回 `data: true` 就成了。验证雷达出数据，应该在 17Hz 左右：

```bash
docker exec pb2025-dev bash -lc 'source /opt/ros/humble/setup.bash; source /workspace/install/setup.bash; timeout 6 ros2 topic hz /red_standard_robot1/livox/lidar 2>&1 | grep -m1 "average rate"'
```

### 第 3 步 起导航，二选一，等 40 秒

A 套（Astra MINCO）：

```bash
docker exec pb2025-dev bash -lc 'source /opt/ros/humble/setup.bash; source /workspace/install/setup.bash; export DISPLAY=:0; setsid ros2 launch astra_pb2025_bridge astra_nav_sim.launch.py world:=rmuc_2025 use_rviz:=True > /tmp/astra_a.log 2>&1 &'
```

B 套（原生 nav2，对照用）：

```bash
docker exec pb2025-dev bash -lc 'source /opt/ros/humble/setup.bash; source /workspace/install/setup.bash; export DISPLAY=:0; setsid ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py world:=rmuc_2025 slam:=False use_rviz:=True > /tmp/pb_nav.log 2>&1 &'
```

### 第 4 步 发目标点

图形方式最省事：RViz 工具栏上，A 套点 **2D Goal Pose**，B 套点 **Nav2 Goal**，然后地图上点一下拖个方向。

命令行发 map 帧目标点 (5, -5)，A 套：

```bash
docker exec pb2025-dev bash -lc 'source /opt/ros/humble/setup.bash; source /workspace/install/setup.bash; timeout 4 ros2 topic pub -r 1 /red_standard_robot1/goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: map}, pose: {position: {x: 5.0, y: -5.0, z: 0.0}, orientation: {w: 1.0}}}"'
```

B 套走 nav2 action：

```bash
docker exec pb2025-dev bash -lc 'source /opt/ros/humble/setup.bash; source /workspace/install/setup.bash; ros2 action send_goal /red_standard_robot1/navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 5.0, y: -5.0, z: 0.0}, orientation: {w: 1.0}}}}"'
```

命令行发目标**别用 `--once`**。`--once` 发完进程立刻退出，而 DDS 建连要几百毫秒到 1 秒，冷连接下那条消息在订阅者连上之前就发完退出了，消息直接丢，planner 收不到新目标却毫无报错。用 `-r 1` 持续发几秒，或者干脆用 RViz。这个坑排查过一次，详见调试笔记。

### 看机器人当前位置

判断有没有到目标，以 map 帧为准（odom 帧会漂）：

```bash
docker exec pb2025-dev bash -lc 'source /opt/ros/humble/setup.bash; source /workspace/install/setup.bash; timeout 6 ros2 run tf2_ros tf2_echo map chassis --ros-args -r /tf:=/red_standard_robot1/tf -r /tf_static:=/red_standard_robot1/tf_static 2>&1 | grep -m1 Translation'
```

发目标前建议先连采两次位置，确认 z ≈ 0.08 且 xy 不乱跳，再发目标。定位发散的时候（z 跑到 1.6m 之类）所有数据都不可信。

### 关闭所有进程

每轮测试完都要做，反复启停容易把环境搞脏。

```bash
docker exec pb2025-dev bash -lc 'ps aux | grep -E "ros2 launch|component_container|pointlio|ign_sim_pointcloud|loam_interface|sensor_scan|terrainAnalysis|fake_vel|global_obstacle|occupancy_node|planner_node|controller_node|batch_liwo|map_server|lifecycle|small_gicp|joy|rviz2" | grep -v grep | awk "{print \$2}" | xargs -r kill -9 2>/dev/null; sleep 2'
docker exec pb2025-dev bash -lc 'ps aux | grep -E "ign gazebo|parameter_bridge|rmua19|ruby.*ign|referee|robot_state_publisher" | grep -v grep | awk "{print \$2}" | xargs -r kill -9 2>/dev/null; sleep 2'
docker exec pb2025-dev bash -lc 'ps aux | grep -E "ign gazebo|pointlio|rviz2|global_obstacle|component_container" | grep -v grep | wc -l'   # 应输出 0
```

别用 `pkill -f ros2` 这种广播式清理，它会把你自己所在的 `docker exec` 父 shell 一起杀掉（exit 137），清理跑一半就断了。所以上面用精确进程名匹配 PID。真卡死了重启设备最干净。

## 数据流（A 套）

```
/map（先验，map 帧，含手动标注障碍）┐
                                    ├→ global_obstacle_map_node ─→ /astra/global_obstacle_grid（map 帧全场）
/terrain_map（实时感知，odom 帧）────┘                            └→ /astra/odom（map 帧，由 odometry 重投影）

/astra/global_obstacle_grid + /astra/odom + goal_pose
    → planner_node → /astra/trajectory
    → controller_node → /<ns>/cmd_vel_nav2_result
    → fake_vel_transform → /<ns>/cmd_vel → 仿真底盘
```

Astra 节点跑在全局命名空间（不加 ns 前缀，这样裸键 yaml 参数才生效），通过话题 remap 接到 pb2025 的 `/red_standard_robot1/*`。TF 也 remap 到 `/<ns>/tf`。

planner 内部的规划链：收到障碍图 → 转 `Grid2D` → 膨胀 → 建 ESDF → 目标经可达性判定/局部目标选择/端点投影处理 → JPS 前端搜路径 → MINCO 两步优化（L-BFGS）→ 发 `nav_msgs/Path`，时间信息编码在 `header.stamp` 里。

## 编译单个包

改了代码之后不用全量重编：

```bash
docker exec pb2025-dev bash -lc 'source /opt/ros/humble/setup.bash; source /workspace/install/setup.bash; cd /workspace && colcon build --packages-select astra_pb2025_bridge --symlink-install'
```

改 RViz 配置不用编译，重起 A 套就生效。

## 已知问题

调试过程、已解决的问题和还没修的 BUG 都在 [docs/DEBUG_NOTES.md](docs/DEBUG_NOTES.md)。当前还开着的：

- **飞车 + 路径不忠诚**：根因定位在 `global_obstacle_map_node.cpp` 摄入 terrain_map 时只设了 intensity 下限、没有上限，异常高 intensity 的点把可通行地面污染成障碍，进而触发正反馈崩溃。修法已定，等实测数据坐实。
- **目标处理链缺连通性判据**：只判目标单点离障碍够远，不判起点到目标是否连通，合法但不连通的远目标必然规划失败。

## 许可

`src/astra/` 下自写代码为 Apache-2.0，`astra_third_party` 是 GCOPTER / DDR-opt 的派生头文件，保留原 MIT 许可。上游 8 个仓库不在本仓库内，各自遵循其原有许可。
