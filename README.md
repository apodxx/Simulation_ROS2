# Simulation_ROS2 — leap_ros 小车 Gazebo 仿真 / SLAM / Nav2 导航

在 **Gazebo Classic** 中仿真 leap_ros 差速小车，用 **gmapping** 建图，用 **Nav2** 做静态地图导航。

本仓库基于硬件厂商提供的 `leap_ros_ws` 工作空间，记录并修复了把它完整跑通在仿真里所需的全部配置改动。

---

## 一、环境

| 项目 | 版本 |
|---|---|
| 操作系统 | Ubuntu 22.04 (x86_64) |
| ROS 2 | Humble |
| 仿真器 | Gazebo Classic 11.10.2 |
| 导航 | Nav2 (ros-humble-navigation2 + nav2-bringup) |
| RM | Fast-DDS |
| 测试机 | 2 vCPU / 8 GB RAM（能跑，但要按本仓库的参数降负载） |

### 依赖安装

```bash
sudo apt install -y \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-joint-state-publisher \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-nav2-map-server \
  ros-humble-teleop-twist-keyboard
```

### 构建

```bash
cd ~/Desktop/leap_ros_ws
colcon build
source install/setup.bash
```

> 建议把 `source` 写进 `~/.bashrc`，否则每次新开终端都会报 `Package 'xxx' not found`。

---

## 二、目录结构

```
leap_ros_ws/
├── xuegecar_description/       # URDF、mesh、Gazebo 世界、RViz 配置  ★核心
│   ├── urdf/leap_ros.urdf                  # 原始 URDF
│   ├── urdf/leap_ros.gazebo.urdf           # 生成的 Gazebo URDF
│   ├── urdf/leap_ros.gazebo.sdf            # ★ Gazebo 实际加载的 SDF
│   ├── worlds/leap_room.world              # ★ 仿真房间
│   ├── rviz/sim.rviz                       # ★ 建图用 RViz 配置
│   ├── launch/gazebo_sim.launch.py         # ★ 启动 Gazebo + 机器人
│   ├── model.config                        # 让 model:// 能找到 mesh
│   └── meshes/*.stl
├── xuegecar_navigation2/       # Nav2 配置与地图  ★核心
│   ├── param/xuegebot.yaml                 # ★ Nav2 全部参数
│   ├── maps/leap_room_v1.pgm/.yaml         # ★ 建好的房间地图
│   ├── rviz/xuege_navigation2.rviz         # ★ 导航用 RViz 配置
│   └── launch/*.launch.py
├── slam_gmapping/              # gmapping 的 ROS 2 封装（已修复多个 bug）
│   ├── src/slam_gmapping.cpp
│   ├── include/slam_gmapping/slam_gmapping.h
│   └── params/slam_gmapping.yaml
├── openslam_gmapping/          # gmapping 上游源码
├── xuegecar_bringup/           # 底盘驱动、里程计、TF
├── camsense_lidar/             # 真机雷达驱动（仿真用不到）
├── xuegecar_camera/            # 摄像头流
├── xuegecar_cartographer/      # Cartographer 备选方案
├── xuegecar_mavlink_bridge/    # MAVLink 桥接
├── leap_interfaces/            # 自定义消息
├── xuegeros_demo/              # 真机综合启动
├── scripts/                    # ★ 本仓库新增的一键脚本
├── tools/                      # ★ 本仓库新增的调试工具
└── docs/                       # ★ 说明文档
```

★ = 本项目重点关注的仿真相关文件

---

## 三、快速开始

### 场景 A：建图（SLAM）

```bash
# ① 启动仿真（Gazebo + 底盘 + 雷达 + RViz）
~/Desktop/leap_ros_ws/scripts/start_sim_all.sh

# ② 启动 gmapping
cd ~/Desktop/leap_ros_ws
ros2 launch slam_gmapping slam_gmapping.launch.py

# ③ 开一圈（离墙保持 0.3 m 以上）
scripts/drive_keyboard.sh          # 键盘遥控
# 或 scripts/drive_robot.sh circle  # 自动绕圈

# ④ 存地图
ros2 run nav2_map_server map_saver_cli \
  -f ~/Desktop/leap_ros_ws/xuegecar_navigation2/maps/my_room \
  --ros-args -p save_map_timeout:=20.0
```

**注意**：`map_saver_cli` 默认只等 2 秒，DDS 发现慢时会失败，必须传 `save_map_timeout`。

### 场景 B：导航（Nav2）

```bash
# ① 启动仿真
scripts/start_sim_all.sh

# ② 启动导航（自动停车 → 清场 → 起 Nav2 → 起 RViz → 自动初始位姿）
scripts/start_nav2.sh

# ③ 在 RViz 里点 "Nav2 Goal" 选目标点
#    初始位姿已由 set_initial_pose 自动设为 (0,0,0)，不用手动点 "2D Pose Estimate"
```

命令行发目标：

```bash
scripts/send_goal.sh 2.5 -2.0 0      # x y yaw(度)
```

### RViz 工具栏两个按钮的区别

| 按钮 | 作用 | 发到 | 车会动吗 |
|---|---|---|---|
| `2D Pose Estimate` | 告诉 AMCL 我在哪（定位初始化） | `/initialpose` | ❌ 不会 |
| `Nav2 Goal` | 命令车去目标点 | `/navigate_to_pose` 动作 | ✅ 会 |

**方向必须给准**：位置差 0.5 m 以内 AMCL 能自己纠正，但方向错 60° 以上**永远纠不回来**（粒子云里没有正确方向的粒子）。详见 [docs/03-troubleshooting.md](docs/03-troubleshooting.md)。

---

## 四、房间与机器人参数

### 房间（`worlds/leap_room.world`）

| 项目 | 数值 |
|---|---|
| 内部尺寸 | x ∈ [-4.0, 4.0]，y ∈ [-3.0, 3.0]（8 × 6 m） |
| 墙厚 | 0.3 m（**必须 ≥ 0.2 m**，0.1 m 时车会从墙缝挤出去） |
| `obstacle_box1` | (1.5, 0.8)，0.8 × 0.8 m |
| `obstacle_box2` | (-1.8, -1.2)，0.6 × 1.2 m |
| `obstacle_cylinder` | (-2.5, 1.8)，半径 0.35 m |

### 机器人

| 项目 | 数值 |
|---|---|
| 驱动形式 | 两轮差速 + 前万向轮 |
| 轮距 | 0.162 m |
| 轮径 | 0.0591 m |
| 车头方向 | **+x**（摄像头/超声波那端） |
| 激光雷达 | 360 点 / 1° 分辨率 / 10 Hz / **0.25 ~ 12 m** |
| 雷达安装位 | (-0.0227, 0.000062, 0.12575)（相对 base_footprint） |

### 地图（`maps/leap_room_v1.yaml`）

```yaml
image: leap_room_v1.pgm
mode: trinary
resolution: 0.05
origin: [-10, -10, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
```

384 × 384 格 @ 5 cm = 19.2 × 19.2 m。

---

## 五、脚本说明（`scripts/`）

> 这些脚本假设工作空间在 `~/Desktop/leap_ros_ws`，机器人名 `leap_ros`。换环境需改路径。

| 脚本 | 作用 |
|---|---|
| `start_sim_all.sh` | 启动 Gazebo + 底盘 bringup + 雷达 + RViz + 控制台 |
| `start_nav2.sh` | **发零速 → 清场 → 启动 Nav2 → 启动导航版 RViz** |
| `restart_slam.sh` | 停车 → 复位车 → 重启 gmapping → 重启 RViz |
| `set_initial_pose.sh x y [yaw]` | 给 AMCL 喂初始位姿 |
| `send_goal.sh x y [yaw]` | 命令行发导航目标 |
| `drive_robot.sh <动作>` | 开环自动行驶：`forward` / `circle` / `square` / `spin` / `figure8` / `stop` / `pose` |
| `drive_keyboard.sh` | `teleop_twist_keyboard` 键盘遥控 |
| `reset_robot.sh <命令>` | `reset` 复位到原点 / `where` 打印位置与离墙距离 |

### 为什么要"先发零速"

Gazebo 的 `diff_drive` 插件会**锁存最后一条速度指令**。如果直接杀掉 Nav2，机器人会继续按最后的速度一直开，直到撞墙。所以所有"停止"流程都必须显式发一次零速。

---

## 六、关键改动记录

本项目对原始工作空间做了 **22 处**修改，全部有明确原因。完整表格见 [docs/02-parameters.md](docs/02-parameters.md)。

挑最重要的 5 个：

| 问题 | 现象 | 修复 |
|---|---|---|
| gmapping 的 yaml **完全不生效** | 改任何参数都没反应 | 源码里一行 `get_parameter` 都没有，补上参数读取 |
| `tf2_ros::MessageFilter` 自死锁 | 建图跑 4 分钟后彻底冻结，进程还活着 | 改为直连 `/scan` 订阅 |
| 雷达 `range_min = 0.02` | 车"看见自己"，代价地图把车标成障碍，原地抽搐 | 改为 0.25 m |
| DWB `min_vel_x = -0.12` + 缺 `PreferForward` | 车一路倒着开 | `min_vel_x: 0.0` + 加 `PreferForward` 评分器 |
| 墙厚 0.1 m | 地图里出现跑到墙外几米的毛刺 | 墙加厚到 0.3 m |

---

## 七、文档索引

| 文档 | 内容 |
|---|---|
| **[docs/00-quickstart.md](docs/00-quickstart.md)** | **5 分钟复现清单（纯命令版）** |
| [docs/01-architecture.md](docs/01-architecture.md) | 节点图、TF 树、话题表、Nav2 概念解释 |
| [docs/02-parameters.md](docs/02-parameters.md) | 所有参数的含义、默认值、改动理由 |
| [docs/03-troubleshooting.md](docs/03-troubleshooting.md) | 12 个真实踩过的坑与排查流程 |
| **[docs/05-real-car-deployment.md](docs/05-real-car-deployment.md)** | **真车部署（Jetson Orin NX + micro-ROS）** |

---

## 八、当前状态

- ✅ 建图：房间完整闭合，无重影，`map→odom` 漂移 < 1 cm
- ✅ 存图：`maps/leap_room_v1.pgm` 可直接用于导航
- ✅ 导航：目标成功率 100%，恢复行为 0 次
- ✅ 自动初始位姿：启动即定位，无需手动点
- ⚠️ 2 vCPU 会吃力，建议 ≥ 4 vCPU

---

## 九、致谢

原始工作空间 `leap_ros_ws` 由硬件厂商提供。本仓库仅包含仿真适配、参数调优和 bug 修复。
