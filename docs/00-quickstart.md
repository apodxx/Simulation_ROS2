# 5 分钟复现清单（纯命令版）

从零到「车在 Gazebo 里被 Nav2 导航」，全部是可直接复制的命令。

> 时间估算：装依赖约 2~5 分钟（看网速），克隆+构建约 1~2 分钟，其余即时完成。

---

## 0. 前提

| 项目 | 要求 |
|---|---|
| 系统 | Ubuntu 22.04 x86_64 |
| ROS 2 | Humble |
| 仿真器 | Gazebo Classic 11 |
| CPU | ≥ 4 vCPU 推荐；2 vCPU 可用但需执行 [第 8 步](#8-2-vcpu-减负可选) |
| 磁盘 | ≥ 2 GB |

已装 ROS 2 Humble 的验证：

```bash
source /opt/ros/humble/setup.bash
ros2 --version          # 应输出 ros2 cli version: 0.18.x
gazebo --version        # 应输出 Gazebo 11.x
```

---

## 1. 装依赖

```bash
sudo apt update
sudo apt install -y \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-joint-state-publisher \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-nav2-map-server \
  ros-humble-teleop-twist-keyboard
```

---

## 2. 克隆 + 构建

> ⚠️ **必须克隆到 `~/Desktop/leap_ros_ws`** —— `scripts/` 里的脚本把路径写死成这个了。

```bash
git clone https://github.com/apodxx/Simulation_ROS2.git ~/Desktop/leap_ros_ws
cd ~/Desktop/leap_ros_ws
colcon build
source install/setup.bash
chmod +x scripts/*.sh
```

**强烈建议**把 source 写进 `.bashrc`，否则每个新终端都要手动 source：

```bash
cat >> ~/.bashrc <<'RC'
source /opt/ros/humble/setup.bash
if [ -f ~/Desktop/leap_ros_ws/install/setup.bash ]; then
  source ~/Desktop/leap_ros_ws/install/setup.bash
fi
RC
source ~/.bashrc
```

---

## 3. 启动仿真

```bash
~/Desktop/leap_ros_ws/scripts/start_sim_all.sh
```

会启动：`gzserver` + `gzclient` + `xuegecar_bringup` + 雷达 + RViz。

等待约 20 秒让 Gazebo 完全起来。

---

## 4. 健康自检（三个频率都要有数值）

```bash
ros2 topic hz /clock        # 应 ≈ 10 Hz
ros2 topic hz /scan         # 应 ≈ 10 Hz
ros2 topic hz /odom         # 应 ≈ 50 Hz
```

一键体检（复制整块）：

```bash
echo "=== 仿真 ==="
for t in /clock /scan /odom; do
  printf "%-8s " "$t"
  timeout 8 ros2 topic hz $t 2>&1 | grep -m1 'average rate' || echo "❌ 无数据"
done
echo "=== 车位置 ==="
gz model -m leap_ros -p
echo "=== 负载（2 核应 < 4）==="
cat /proc/loadavg
```

三项都正常 → 继续第 5 步。任何一项失败 → 看 [第 9 步](#9-三个最常踩的坑)。

---

## 5. 启动导航

```bash
~/Desktop/leap_ros_ws/scripts/start_nav2.sh
```

这个脚本会自动完成：

1. 发零速（解除 Gazebo 的速度指令锁存）
2. 杀干净所有旧 Nav2 实例（防止多实例抢话题）
3. 停掉 gmapping（它和 AMCL 抢 `map→odom`）
4. 启动 `nav2_bringup`
5. 等全部节点激活
6. 启动导航版 RViz

看到 `所有节点激活成功` 即可。

> **初始位姿已自动设为 (0,0,0)**，不需要手动点 `2D Pose Estimate`。

---

## 6. 发导航目标

在 RViz 里点工具栏 `Nav2 Goal`，在地图上点目标点并拖出最终朝向。

或用命令行：

```bash
~/Desktop/leap_ros_ws/scripts/send_goal.sh 2.5 -2.0 0
```

观察 RViz 右侧面板：`Distance remaining` 应持续减少，`Recoveries` 应为 0。

---

## 7. 自己重建地图（可选）

想换房间布局或重新体验建图流程：

```bash
# ① 重启建图（会复位车 + 重启 gmapping + 重启 RViz）
~/Desktop/leap_ros_ws/scripts/restart_slam.sh

# ② 开车绕一圈（离墙保持 0.3 m 以上）
~/Desktop/leap_ros_ws/scripts/drive_keyboard.sh
#   或自动化：scripts/drive_robot.sh circle

# ③ 停掉 gmapping 再存图（注意 save_map_timeout）
pkill -f slam_gmapping
ros2 run nav2_map_server map_saver_cli \
  -f ~/Desktop/leap_ros_ws/xuegecar_navigation2/maps/my_room \
  --ros-args -p save_map_timeout:=20.0

# ④ 用新地图导航
~/Desktop/leap_ros_ws/scripts/start_nav2.sh \
  ~/Desktop/leap_ros_ws/xuegecar_navigation2/maps/my_room.yaml
```

---

## 8. 2 vCPU 减负（可选）

如果出现 `CRITICAL FAILURE: SERVER xxx IS DOWN after not receiving a heartbeat`，说明 CPU 被挤爆导致 Nav2 心跳超时。按优先级减负：

```bash
# ① 关掉 foxglove_bridge（省 ~21% CPU，仿真用不到）
pkill -f foxglove_bridge

# ② 关掉 RViz 做纯命令行测试
pkill -f 'rviz2 -d'

# ③ RViz 帧率已在配置里降到 10；如需再降，改 sim.rviz 的 Frame Rate

# ④ 降低 AMCL 粒子数（xuegebot.yaml）
#    min_particles: 300  →  200
#    max_particles: 800  →  400

# ⑤ 关掉浏览器（Firefox 占用不小）
```

实测负载参考：

| 组成 | load |
|---|---|
| Gazebo + bringup + gmapping | 2.5 |
| ↑ + RViz | 3.5 |
| ↑ + Nav2 | 3.5（勉强） |
| ↑ + foxglove_bridge | **9.6（必挂）** |

---

## 9. 三个最常踩的坑

### ① `Package 'xxx' not found`

```bash
# 错误信息里的 searching 只列了 /opt/ros/humble → 没 source 工作空间
source /opt/ros/humble/setup.bash
source ~/Desktop/leap_ros_ws/install/setup.bash
```

> `colcon build` 之后，**已开着的终端**也要重新 source。

### ② 目标被接受但车不动 / 原地抽搐

```bash
grep -c 'argument target_frame does not exist' /tmp/sim/nav2.log   # 一直涨 → 局部代价地图卡了
grep -c 'No valid trajectories' /tmp/sim/nav2.log                   # 大量 → 雷达扫到自身
grep -c 'Failed to make progress' /tmp/sim/nav2.log
```

处理：`scripts/start_nav2.sh` 重启即可。若 `No valid trajectories` 持续，检查雷达 `range_min` 是否为 0.25。

### ③ 杀掉 Nav2 后车还在自己开

Gazebo 会**锁存最后一条速度指令**。必须显式发零速：

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

`start_nav2.sh` 已内置这一步。

---

## 10. 房间坐标速查

地图原点 = 房间中心。

| 项目 | 坐标 |
|---|---|
| 房间内部 | x ∈ [-4.0, 4.0]，y ∈ [-3.0, 3.0]（8 × 6 m） |
| `obstacle_box1` | (1.5, 0.8)，0.8 × 0.8 m |
| `obstacle_box2` | (-1.8, -1.2)，0.6 × 1.2 m |
| `obstacle_cylinder` | (-2.5, 1.8)，半径 0.35 m |

发导航目标时避开障碍物周围 0.5 m。

车头朝 **+x**（摄像头/超声波那端），初始位置 `(0, 0)` 朝 `0°`。

---

## 11. 全部命令一次跑完（懒人版）

```bash
# 一次性：构建 → 起仿真 → 起导航 → 发目标
cd ~/Desktop/leap_ros_ws
colcon build && source install/setup.bash
chmod +x scripts/*.sh
scripts/start_sim_all.sh &
sleep 25
scripts/start_nav2.sh
scripts/send_goal.sh 2.5 -2.0 0
```

---

## 遇到问题

| 症状 | 章节 |
|---|---|
| 建图冻结 / 参数不生效 / 地图有毛刺 | [03-troubleshooting.md](03-troubleshooting.md) |
| 想了解各个参数为什么这么设 | [02-parameters.md](02-parameters.md) |
| 想搞懂 TF 树、话题、Nav2 概念 | [01-architecture.md](01-architecture.md) |
