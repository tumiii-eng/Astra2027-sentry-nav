# Astra2027-sentry-nav

把 MINCO 轨迹优化那套导航方案（连续代价场 + 空间 A\* 前端 + MINCO + L-BFGS 两步优化 + SE2 MPC）接到 RoboMaster 2025 哨兵仿真项目 [pb2025_sentry_nav](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav) 上，在 Gazebo Fortress 仿真里发目标点验证。

仿真、里程计、重定位、地形感知这些底层全部沿用 pb2025 原生的那一套，Astra 只替换「规划 + 控制」这一层。所以同一套仿真下可以起两种导航来对照：

| | 全局规划 | 控制 |
| --- | --- | --- |
| **A 套** Astra | 空间 A\* 前端 + MINCO 轨迹 + L-BFGS 优化 | SE2 MPC / preview 控制器 |
| **B 套** pb2025 原生 | nav2 + Theta\* | omni PID pursuit |

两套互斥，不能同时跑。

规划与控制这一层的实现方法以开源项目 [HWSentryNav26](https://github.com/Polyacetone/HWSentryNav26) 为参考基准：遇到 Astra 与之做法不同、且 Astra 那侧确有问题的地方，一律改成对齐 HWSentryNav26 的方案（ROS API 仍用容器里的 Humble，不跟随其 Jazzy 版接口）。源码注释里标了「对应上游 …」的地方就是这类对齐点。

## 仓库里有什么

只放自己写的代码。上游那 8 个仓库（310MB）不进版本库，用 `upstream.repos` + 一个脚本拉取，这样上游更新和本地改动不会互相污染。

```
src/astra/                      Astra 导航算法（10 个 ROS 2 包）
├── astra_common/               通用数据结构、数学、速度剖面时间分配
├── astra_third_party/          GCOPTER / DDR-opt 派生头文件（MIT）
├── astra_perception/           点云预处理、3D 滚动占据栅格
├── astra_mapping/              障碍提取、连续代价场、ESDF、代价场采样
├── astra_planning/             空间 A*、端点重定位、重规划策略、MINCO、L-BFGS、规划节点
├── astra_control/              进度观测器、轨迹跟踪控制器、底盘速度输出
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
git clone https://github.com/tumiii-eng/Astra2027-sentry-nav.git pb2025_ws
cd pb2025_ws
sudo apt install python3-vcstool     # 没装的话
./tools/setup_upstream.sh
```

脚本做三件事，幂等，重复跑没问题：

1. 按 `upstream.repos` 锁定的 commit 拉 8 个上游仓库到 `src/`
2. 初始化 `pb2025_sentry_nav` 的 7 个子模块
3. 给 9 个 `CMakeLists.txt` 打补丁

跑完之后 `src/` 下应该有 9 个目录（8 个上游 + `astra`）。

#### 脚本做了什么（手动执行的话照这个来）

不想用脚本，或者脚本某一步失败要单独重试，可以直接敲这三步。

拉上游，`vcs` 会读 `upstream.repos` 里锁定的 commit，所以拿到的版本和开发时完全一致：

```bash
vcs import src < upstream.repos
```

`pb2025_sentry_nav` 自己还带 7 个子模块（point_lio、small_gicp_relocalization、livox_ros_driver2、pointcloud_to_laserscan、pb_nav2_plugins、pb_omni_pid_pursuit_controller、pb_teleop_twist_joy），必须单独初始化，漏了会编译失败：

```bash
git -C src/pb2025_sentry_nav submodule update --init --recursive
```

打补丁——删掉 9 个 `CMakeLists.txt` 里的 `USE_SCOPED_HEADER_INSTALL_DIR`。上游用的 ament_cmake_auto 版本不认这个选项，留着编译直接报错。这是本工程对上游的**全部**改动，每个文件就删这一行。注意其中 4 个在子模块里，所以必须先跑完上一步的 `submodule update`：

```bash
sed -i '/USE_SCOPED_HEADER_INSTALL_DIR/d' \
  src/pb2025_sentry_nav/fake_vel_transform/CMakeLists.txt \
  src/pb2025_sentry_nav/ign_sim_pointcloud_tool/CMakeLists.txt \
  src/pb2025_sentry_nav/loam_interface/CMakeLists.txt \
  src/pb2025_sentry_nav/sensor_scan_generation/CMakeLists.txt \
  src/pb2025_sentry_nav/pb_nav2_plugins/CMakeLists.txt \
  src/pb2025_sentry_nav/pb_omni_pid_pursuit_controller/CMakeLists.txt \
  src/pb2025_sentry_nav/pb_teleop_twist_joy/CMakeLists.txt \
  src/pb2025_sentry_nav/small_gicp_relocalization/CMakeLists.txt \
  src/rmu_gazebo_simulator/rmu_gazebo_simulator/CMakeLists.txt
```

验证补丁生效，下面这条应该输出 `0`：

```bash
grep -rc USE_SCOPED_HEADER_INSTALL_DIR src/ | grep -v ':0$' | wc -l
```

#### 上游版本对照

`upstream.repos` 里锁的 commit：

| 仓库 | commit |
| --- | --- |
| pb2025_sentry_nav | `7ed5f10` |
| rmu_gazebo_simulator | `c950a93` |
| rmoss_core | `8209ec5` |
| rmoss_gazebo | `7443ff0` |
| rmoss_interfaces | `424f50c` |
| rmoss_gz_resources | `b5c759f` |
| pb2025_robot_description | `a0541dd` |
| sdformat_tools | `47c2d1a` |

想换成上游最新版，把 `version:` 改成分支名（比如 `main`）重新 `vcs import`，但上游接口变动可能需要相应改胶水层。

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

首次构建镜像十几分钟（含从源码编 small_gicp），39 个包（29 个上游 + 10 个 astra）编译也要一会儿。

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

每轮测试完都要做，反复启停容易把环境搞脏。三条按顺序敲：

```bash
docker exec pb2025-dev pkill -f 'ros2 launch|gz sim|gzserver|ign gazebo|planner_node|controller_node|global_obstacle_map_node|point_lio|pointlio_mapping|small_gicp|map_server|lifecycle_manager|terrain_analysis|terrainAnalysis|loam_interface|sensor_scan_generation|ign_sim_pointcloud_tool|fake_vel_transform|rviz2|referee_system|spawn_robot'
sleep 3
docker exec pb2025-dev pkill -9 -f 'gz sim|gzserver|ign gazebo|rviz2'
sleep 2
docker exec pb2025-dev pgrep -af 'gz sim|planner_node|controller_node|point_lio|rviz2|terrain_analysis|map_server|global_obstacle_map|small_gicp|loam_interface|fake_vel_transform'
```

第三条**没有任何输出就是干净的**（`pgrep` 没匹配时返回退出码 1，属正常，不是报错）。

两个坑，都踩过：

- `pkill` **必须直接作为 `docker exec` 的目标**，不能包在 `bash -lc '...'` 里。包一层的话，整串正则会出现在那个包装 shell 自己的 `cmdline` 里，第一个 `pkill -f 'ros2 launch'` 就把自己所在的 shell 杀了，后面十几个 `pkill` 一个都执行不到，清理跑一半静默断掉。直接 exec 时 `pkill` 会自动把自己排除，安全。`planner_nod[e]` 这种加方括号的规避写法在这里也不管用——同一条命令行里仍然含有未加括号的字面量。
- 正则不要放宽到裸 `ruby`（太广），`gz sim` 已经覆盖 Gazebo 启动器了。上面这串已经核对过不会匹配容器的 PID 1 与 `sleep infinity` 保活进程。

真卡死了重启设备最干净。

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

planner 内部的规划链：收到障碍图 → 转 `Grid2D` → 建**连续代价场**（硬阻断半径 + 软代价衰减，对齐上游 `map_server` 的 inflation）→ 建 ESDF → 起点/终点做 BFS 重定位（`endpoint_nudge`）→ **空间 A\*** 前端搜路径 → MINCO 两步优化（L-BFGS）→ 发 `nav_msgs/Path`，时间信息编码在 `header.stamp` 里。

轨迹的时间戳由**全路径速度剖面**给出，不是每个路标点单独安排一次「静止到静止」的梯形：先在速度² 上做前向（加速可达）/ 终点零速 / 后向（刹车可达）三趟传播，再按 `Δt = 2Δs/(v_i+v_{i+1})` 梯形积分。整条路径因此只有一次加速、一次巡航、一次刹车，速度跨路标点连续。重规划时还会继承当前速度在首段切向上的前向投影，避免 5 Hz 重规划每周期都从零速重新起步。这一套逐项对应上游 `speed_profile_optimizer.cpp` 的 `reachable_seed()` + `make_profile()`。

## 编译单个包

改了代码之后不用全量重编：

```bash
docker exec pb2025-dev bash -lc 'source /opt/ros/humble/setup.bash; source /workspace/install/setup.bash; cd /workspace && colcon build --packages-select astra_pb2025_bridge --symlink-install'
```

改 RViz 配置不用编译，重起 A 套就生效。

## 已知问题

调试过程、已解决的问题和还没修的 BUG 都在 [docs/DEBUG_NOTES.md](docs/DEBUG_NOTES.md)。

上一版记的两条已经修完了：飞车根因（`global_obstacle_map_node` 缺 intensity 上限）已加双边门限；目标处理链缺连通性判据已换成起终点 BFS 重定位。当前待办：

- **速度提升尚未做仿真实测**：本版把速度剖面改成全路径可达传播、上限提到上游 medium 档（2.2 / 2.0），单元测试里平均速度已从 vmax 的 ~25% 提到 93%，但控制增益（`contour_kp` / `lag_kp` / `velocity_damping`）还是按 1.5 m/s 标的，需要按实测的横向误差 / 终点过冲再标一轮。

## 许可

`src/astra/` 下自写代码为 Apache-2.0，`astra_third_party` 是 GCOPTER / DDR-opt 的派生头文件，保留原 MIT 许可。上游 8 个仓库不在本仓库内，各自遵循其原有许可。
