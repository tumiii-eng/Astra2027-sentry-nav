# 调试笔记

记录接入过程中踩过的坑。分三类：还没修的、已修的、以及查了半天发现不是 bug 的。

排查这套东西有两条经验：**每轮测试后一定要关干净所有进程**（残留进程会让下一轮的现象完全不可信），**问题解决后做减法**（把试错期加的诊断代码、半成品、被取代的实现全删掉，只留最终有效的那份）。

---

## 还没修

### 提速后的控制增益还没按实测标定

本版把速度剖面改成全路径可达传播、上限提到上游 medium 档（2.2 / 2.0，见「已修」里那条），但 `contour_kp` / `lag_kp` / `velocity_damping` / `arrival_*_tolerance` 全都是在 1.5 m/s 下标出来的，没动。

需要一轮仿真实测，看 `controller_node` 日志里的：超速量、横向误差、纵向误差、参考速度、命令线速度。横向误差偏大就调 `contour_kp`，终点过冲就调 `velocity_damping`。在拿到这组数之前不要盲调。

---

## 已修

### 全路径速度剖面：平均速度只有 vmax 的 25%

现象是「怎么调 `max_velocity` 都跑不快」。根因不在参数，在时间分配的结构：`cumulative_times` / `cumulative_times_with_turning` 给**每一段**路标点独立安排一个「静止到静止」的梯形，速度在每个路标点都被打回零。

算一下就知道调参救不了：`waypoint_spacing=0.35 m`，旧参数 `vmax=1.5 / amax=1.6` 下 `t_acc=0.938 s`、`s_acc=0.703 m`，而 `2*s_acc=1.406 m` 远大于 0.35 m，也就是每段都还没加速完就要开始刹车，段平均速度只有 **0.374 m/s ≈ vmax 的 25%**。抬高 `vmax` 只会让 `s_acc` 更大、这个比例更低。

改法是照抄上游 `speed_profile_optimizer.cpp` 的做法，把逐段梯形换成**整条路径一次可达传播**：

1. 前向（加速可达）`z[i+1] = min(vmax², z[i] + 2·amax·Δs)`
2. 终点零速 `z[n-1] = 0`
3. 后向（刹车可达）`z[i] = min(z[i], z[i+1] + 2·amax·Δs)`
4. 梯形积分 `t += 2Δs / (v_i + v_{i+1})`

对应上游的 `reachable_seed()` + `make_profile()`。速度就此跨路标点连续，整条路径只有一次加速、一次巡航、一次刹车。含转角的那个版本只是把转角折成等效路程 `s = s1 + k_turn * s2` 之后走同一个积分器，所以折角依然多分时间，但不会把速度打回零。

还补上了一处原来完全没有的东西：**继承起点速度**。`projected_start_speed()` 取当前速度在首段切向上的前向投影（逆向分量取 0），对应上游 `std::max(0.0, current_velocity_map.dot(tangent))`。没有它的话，5 Hz 重规划每个周期都生成一条「从零速起步」的剖面，机器人被永久钉在加速斜坡的起点上——这一条单独就能解释大半的「跑不快」。起点实测速度允许暂时超出 nominal 包络，随后由终点零速的后向传播裁回可达值，同样照上游。

参数同步提到上游 `capability_profiles.yaml` 的 **medium** 档（`mpc.follow.capability` 用的就是这一档）：`max_velocity: 2.2`、`max_acceleration: 2.0`。注意上游 `path_planner.yaml` 里的 `velocity_max: 3.0` 是几何塑形用的宽包络，不是速度剖面的约束，没有采用。

提速的同时**必须保住之前消除过冲的两条同源关系**，否则过冲会回来：控制器 `max_linear_velocity` 与规划器 `max_velocity` 严格相等；`linear_command_accel_limit` 不低于规划器 `max_acceleration`。上游这两对值本来就来自同一个 `capability.command_dynamics.velocity_rate_max`。顺带把 `controller_node` 里无时间戳路径兜底重拟合那处写死的 `1.5/1.6` 换成读控制器自身的上限，不然以后改速度这条兜底路径会悄悄留在旧值。

效果：`waypoint_spacing=0.35 m` 下平均速度从 vmax 的 ~25% 提到 **93%**，单元测试断言 >70%。

写这段代码时踩了自己一个坑：旧实现里 `std::max(dt, 0.05)` 那个时长下界被我顺手留下了，而在密路标点或高速下 `dt` 本身就小于 0.05 s，这个下界会反过来**拉长**总时长，把整个改动抵消掉。改成 `std::max(dt, ds < 1e-6 ? 0.05 : 1.0e-3)`——下界只为保证 MINCO 分段时间严格为正，退化零长段才给 0.05 s。

单元测试也顺带纠了一处：原来拿 3 个节点、1 m 间距的路径去断言总时长，跑出来 `assert(times.back() < 3.0)` 失败。梯形积分把段内速度当线性处理，节点越稀疏越**高估**时长（上游是密集细分节点，所以碰不到）。测试改成按真实间距探测：20×0.1 m 断言逼近物理值 3.0 s、`dense ≤ coarse`（密了更短，恰好和旧实现相反）、0.35 m + medium 档断言与解析梯形吻合到 5% 以内、以及继承起点速度后总时长更短。

### 飞车 + 路径不忠诚（同一条正反馈崩溃链）

两个现象是耦合的：机器人跟不住规划路径（蟹行、绕圈、实际运动方向偏目标方向 45° 左右），以及偶发飞车（z 冲到 5.7m）。飞车时有个关键线索：**原本可通行的地面被仿真点云扫过之后变成不可通行的灰色**。

因果链：

1. 机器人被脱困轨迹推到场地边缘或掩体，轻微骑上 2D 地图无法表达的真实 3D 几何，车体 z 抬升。
2. `terrain_analysis` 的判据是 `pointZ - vehicleZ ∈ (minRelZ=-1.5, maxRelZ=0.2)`。车体 z 抬升、vehicleZ 滞后，平地点云算出来的离地高度（intensity）偏大，地面被标成障碍——就是看到的「变灰」。
3. **A 套特有的放大器，也是根因**：`global_obstacle_map_node.cpp` 摄入 terrain_map 时只有 intensity 下限 `if (inten < dyn_intensity_min_) continue;`，**没有上限**。而 B 套的 nav2 IntensityVoxelLayer 既有 `min_obstacle_intensity: 0.1` 也有 `max_obstacle_intensity: 2.0`。抬升产生的异常高 intensity 点，B 套滤掉了，A 套全标成障碍，障碍图污染成一片灰。
4. 规划崩溃：起点四周全是障碍，ESDF 距离骤降到 0.5m 以下，低于 `obstacle_cost_radius=0.55`。planner 日志反复报「JPS 未找到可行路径」「规划起点清障失败」「发布起点清障脱困轨迹」。
5. 正反馈：脱困轨迹乱推 → 车更歪、z 更高 → 更多地面变灰 → 障碍更多 → 彻底飞。

「路径不忠诚」是同一个因：机器人平时跟的不是通往目标的完整路径，而是被污染的障碍图逼出来的一段段十几米的扭曲脱困轨迹，所以看着像蟹行绕圈。

为什么只有 A 套会：terrain_analysis、point_lio、small_gicp 都是两套共用的，B 套不飞车。唯一的差异就是 `global_obstacle_map_node` 缺 intensity 上限。

已采到的数据：静止基线状态下 terrain_map 的 intensity 全部小于 0.5，没有超过 2.0 的点，约 18.6% 的点大于等于 0.1（真实墙体），z 范围 [-0.035, 0.753]。**飞车时刻的 intensity 还没采到**，这是下一步要做的，先坐实飞车时确实有 intensity > 2.0 的点再改代码。

修法：`global_obstacle_map_node` 加 `dynamic_intensity_max` 参数（对齐 B 套的 2.0），判据变成双边门限 `inten < dyn_intensity_min_ || inten > dyn_intensity_max_` 都跳过。已实现。

早期把这个当成「偶发的定位漂移」处理是误判，它是可复现的正反馈崩溃，有明确的 A 套侧根因。

### 目标处理链只有单点判据，没有连通性判据

旧的 `goal_reachability_resolver` / `local_goal_selector` / `endpoint_projector` 都只判「目标这个点离障碍够不够远」，目标单点合法就原样送给前端，从不判起点到目标之间是不是连通的。合法但不连通的远目标必然规划失败。

修法：整条目标处理链换成 `endpoint_nudge`——起点和终点各自在代价场上做 BFS，落在阻断区里就重定位到 `endpoint_nudge_max_distance`（1.0 m）范围内最近的可达自由格，对齐上游 `path_planner` 的 endpoint nudge。三个旧组件已删除。

### 前端从 JPS 换成空间 A\*

JPS 的跳点规则建立在**均匀栅格**上，而对齐上游之后障碍图是**连续代价场**（硬阻断半径 + 软代价衰减），格子不再只有「占据/自由」两态，跳点剪枝的前提不成立。换成 `spatial_grid_astar`，边代价直接吃代价场的连续值，参数 `spatial_a_star.obstacle_weight` / `max_expansions`，对应上游 `path_planner/spatial_grid_astar`。`jps_planner` 已删除。

### 膨胀过度切断连通域

远目标 (12,2) 规划不出来，反复报「JPS 未找到可行路径」，而同样的起点/目标/地图/定位下 B 套 nav2 能走到 (11.98, 1.94)。

根因是双重硬占据膨胀：`global_obstacle_map_node` 先膨胀 0.30m，planner 内部又膨胀一次，总量 0.8m 左右，把窄走廊的连通域整个切断了。

改法是去掉双重膨胀，膨胀全交给 planner 单次做：`global_obstacle_map_node` 的 `inflation_radius` 从 0.30 改成 0.0，planner 覆盖段加 `robot_radius: 0.25`、`safety_margin: 0.0`，总膨胀 0.46m 降到 0.25m，约等于机器人半径（5 格 @ 0.05m）。

实测 (12,2) 到达 (11.977, 1.904)，前端失败 0 次；基线 (5,-5) 也正常。

后续按当时说的「治本」方向真做了：膨胀已换成对齐上游的**连续代价场**——`inflation.full_cost_radius_m`（硬阻断，覆盖车体半径）+ `cutoff_radius_m` / `decay_rate_per_m`（软代价衰减），动态障碍层另有一套独立参数。所以 `robot_radius` / `safety_margin` 这两个参数现在已经不存在了，机器人在连续代价场里是一个质点。上面那条连通性判据也已经用 `endpoint_nudge` 解决。

### planner 真正用的膨胀图看不见

RViz 里看到的 `/astra/global_obstacle_grid` 只膨胀了 0.3m，而 planner 内部搜索用的那张 `inflated` 图（0.8m）从来没发布过，导致「看到的」和「搜索用的」不是一张图，排查膨胀问题时全靠猜。

planner 现在发布 `/astra/planner_inflated_grid`（map 帧，5Hz），参数 `publish_inflated_grid` / `inflated_grid_topic` 控制，RViz 配置里加了对应的 display。膨胀所见即所得。

### 障碍图几乎全空（占据率 0.09%）

早期把 `terrain_map` 喂给旧的 `occupancy_node`，但 terrain_map 的障碍信息编码在 intensity 里，`occupancy_node` 只看 z 高程，语义不匹配。

改成用预建的 `/map` 作障碍源（就是现在的 `global_obstacle_map_node`），占据率恢复到 46%（含膨胀）。顺带解决了另外两个问题：Astra 原本不订阅 `/map`，所以不知道先验地图和手动标注的障碍；原本的滚动局部图只有 10×10m，没法全局规划。现在基于全场 `/map` 在 map 帧规划，代价场 / ESDF / 前端 / MINCO 自然覆盖全场。

`occupancy_node`（滚动局部图）和 `batch_liwo_proxy`（纯透传）已经被取代，不在 A 套里用了。

### 发目标前机器人就狂转、RViz 卡死

`controller_node` 原本在没有里程计、没有轨迹的时候发布零速 cmd_vel。但下游的 `fake_vel_transform` 只要收到任何 cmd_vel 就会叠加 `init_spin_speed=3.14` 的小陀螺自旋，于是发目标之前机器人就开始狂转，TF 抖动，RViz 卡死。

改法：`controller_node` 的 `on_timer` 在无 odom / 无轨迹时**不发布** cmd_vel。静默让 `fake_vel_transform` 判超时、不自旋。已验证发目标前完全静止。（这在当时是唯一改过的 Astra 源码，后来对齐上游改了不少，不再成立。）

### 停不稳、终点过冲

`preview_controller` 旧的控制律是 `cmd.vx = ff + kp*err_x_body - velocity_damping*last_cmd_.vx`——阻尼项作用在**控制器自己上一条命令**上，不是机器人真实速度，旧的 `compute()` 签名结构上就接不到真实速度。机器人带惯性冲过终点，只按位置误差回拉，欠阻尼，终点附近振荡过冲。

关键发现：`controller_node` 早就算出了真实世界速度 `world_velocity` 并传给了 MPC，唯独传给 preview 控制器的时候把它丢了。不是拿不到，是没接。

改法：`compute()` 签名加 `const Twist2D & world_velocity`，控制律换成 `cmd = v_ref + kp*e_p - kd*(v_actual_body - v_ref_body)`。巡航段 v_actual ≈ v_ref，阻尼项接近 0，不损跟踪精度；终点段 v_ref → 0，退化成 `kp*e_p - kd*v_actual` 的 PD 主动刹车，真实惯性被直接抵消。`velocity_damping` 从 0.15 改到 0.6（语义变了：旧值是软化系数，新值是 PD 刹车增益 kd）。单元测试补了「终点主动刹车」这一项。

实测去程 (0,0) → (17,2) 误差 5cm，返程误差 0.1m，末端实际线速度刹到 0.05~0.09 m/s，到点后 16 秒内位置抖动小于 4cm，无过冲振荡。

顺带把 escape 极限环的诱因消除了：过冲根因解决后机器人不再停在贴墙点（距障碍小于 0.55m），往返测试里没再触发 ping-pong 死循环。

### 参考点按「墙钟时间」取，5 Hz 重规划下天生带滞后

旧实现取参考点的方式是「本条轨迹发布以来过了多少墙钟时间」。这在 5 Hz 重规划下是错的：每来一条新轨迹计时就归零，参考点跳回轨迹起点；而且这个索引和机器人**实际走到哪儿了**没有任何关系，跟丢了也发现不了。1.5 m/s 下光是前瞻时间就天生带约 0.27 m 的位置误差，乘上位置增益直接变成常态超速指令。

改法是移植上游 `nav_executor::RouteTracker`：进度是有向路径上的**时序状态** `(s, ṡ)`，不是每帧独立算一次最近点。位置和地图系速度矢量共同观测 `(s, ṡ)`，规划速度只作为 `ṡ` 的弱先验；同时维护多个竞争的弧长假设（`hypothesis_spacing` / `max_hypotheses` / `hypothesis_prune_ratio`），错误分支被后续观测淘汰，不会因为单次错误最近点永久锁死；对外可见的进度单调不减，杜绝沿错误分支回退。参考点改由弧长进度换算得到，切/法向误差分解和速度前馈也都从 `s` 处的几何切线与剖面速度取，对应上游 `follow_problem::reference_frame`。

与上游唯一的实现差异：上游底盘非全向，只有前向标量速度，要用 `chassis_velocity*(cosθ,sinθ)` 重建地图系速度矢量；pb2025 是四全向轮底盘，地图系速度矢量本身可直接观测，所以直接传入。

顺带修了一个把问题藏起来的 bug：渐入宽限（`startup_velocity_error_grace_time`）原来挂在「单条轨迹发布时刻」上，5 Hz 重规划让它**全程为真**，真正的超速 WARN 被一路降级成 INFO。改成挂在 `new_task_goal_distance`（轨迹终点移动超过 0.5 m 才算新任务）上。

位置误差回正也从单一 `position_kp` 换成在参考点切/法向上分解后分别回正（`contour_kp` / `lag_kp`，对应上游 `follow.tracking_weights.contour/lag`）。到达判定换成双判据：终点距离与剩余弧长须同时满足（对应上游 `stop_threshold_dist` 与 `stop_threshold_remaining_distance`），不再用参考速度做判据——速度剖面在急弯处也会低速，会被误判成到达。

---

## 查完发现不是 bug

### L-BFGS「恒不收敛」（-1008）其实是设计的正常终止

一度以为 MINCO 没有忠实复现导致优化不收敛。读了技术报告 5.5.4.1 加上带梯度诊断的实测，证伪了这个前提：

- 报告自己写的收敛解法就是「参考 DDR-OPT 加 line search 退出条件 + 放松最终条件 + 达迭代上限退出」，**明确接受长轨迹走到迭代上限退出**（30m 轨迹单线程约 80ms）。那个 DDR-OPT 退出条件 `param.past>0 && fabs(finit-f)/(fabs(finit)+1.0) < param.delta/param.past` 在 `lbfgs.hpp` 里已经一字不差实现了。
- `-1008` 就是 `LBFGSERR_MAXIMUMITERATION`，在 `minco_optimizer.cpp` 的 `accepted` 判定里本来就被当成正常接受并输出轨迹。实测长目标 (12,2) 能正常到达。
- FINELY 阶段 grad ≠ ∇cost（valley / violaPos 替换）是报告 5.5.4.2 刻意的设计，防止控制点卡在峡谷中线，不是 bug。副作用是 `g_epsilon` 判据对长轨迹结构上失去意义,但报告本来就不靠 g_epsilon 收敛。
- 实测长轨迹末梯度 `||g||_inf` 在 20~89 之间，g_epsilon 就算放宽到 1e-3（阈值约 0.03）也差三个数量级，放宽没用。短轨迹经 `LBFGS_STOP` 正常收敛。

处置：把之前为「强行收敛」加的诊断代码全删了（cost/gnorm 日志、三个诊断字段共 5 处），`minco_g_epsilon` 回默认 1e-5，`lbfgs_delta=1e-4` 保留（符合报告「放松最终条件」），迭代数 200 保留（计算预算，实测耗时 20~48ms）。只保留一处真实修复：`trajectory_optimizer.cpp` 的 `make_minco_config` 补上 `cfg.g_epsilon = config_.minco_g_epsilon`，此前 g_epsilon 从来没从上层传进去过。日志措辞也改了，加 `lbfgs_status_text()`，打「预优化=达迭代上限正常退出(-1008)/212次」而不是裸错误码。

**教训：排查之前先读报告确认「目标行为」是什么，别把设计特性当 bug 追。**

### 「能过去但回不来」是命令行发消息丢了，不是地形检测失效

现象：去程 (0,0) → (17,2) 成功，发返程 (0,0) 之后车不动，planner 反复报「发布短保持轨迹：原因=规划起点与目标点距离过近，保持点=(17.000, 2.000)」——内部的 `goal_` 死抱旧目标，从来没更新到 (0,0)。

一度怀疑是地形可通行检测失效（能过去就应该能回来）。但数据证伪了：terrain_map 19Hz、447 个障碍点、global_grid 5Hz，全都正常。

真因是 `ros2 topic pub --once` 发布后进程立刻退出，而 ROS 2 的 DDS discovery 握手要几百毫秒到 1 秒。冷连接下那条消息在订阅者连上之前就发完退出了，消息丢失，planner 的 `goal_` 自然没更新。去程之所以成功，是那次恰好赶上连接已经建立。

两个对照实验坐实了：改用 `ros2 topic pub -r 1` 持续发 3~4 秒，planner 立刻响应「保持点=(0.000,0.000)」，车干净开回原点；连接焐热之后再用 `--once` 发 (17,2)，又成功了。成败取决于 discovery 时机，是竞争条件不是稳定 bug。goal 回调本身没缺陷，无条件更新 `goal_.x/y` 并置 `has_goal_=true`，没有 QoS 不兼容也没有静默拒绝。

所以命令行发目标一律用 `-r 1`，或者直接用 RViz 的 2D Goal Pose（图形方式没有这个竞争）。

### 坐标系没对齐？

Astra 和 nav2 里机器人在 map 帧的位置都约等于 (0,0)，map → odom 约等于恒等变换,这是正常的：map 帧原点就是机器人启动点，不是世界坐标真值 (3.4, 9.5)。

### RViz 里缺满屏紫色层？

那是 nav2 的 `global_costmap/costmap`，A 套不起 nav2 所以没有。另外 `terrain_map_ext`（远距点云）确实曾经漏起，已经在 A 套 launch 里补上了。

---

## 环境相关的坑

- 仿真世界启动后**默认暂停**。此时 `/clock` 会推进、`chassis_odometry_gt` 有数据，但激光雷达和 IMU 完全不发布。headless 跑的时候必须用 `ign service` 解除暂停。
- 反复启停容易把环境搞脏：parameter_bridge exit -9、机器人「消失」、`/clock` 或 livox 不出数据。实在卡死重启设备最干净。
- `docker exec` 里嵌套多个 `timeout` 的长复合命令容易超时，拆短分步跑。
- odom 帧个别时刻会漂到离谱的值再被 small_gicp 的 map ← odom 修正拉回，这是 odom 漂移伪影。判断位置一律以 map 帧的 `tf2_echo` 为准。
