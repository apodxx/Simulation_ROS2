# 一键脚本

> 这些脚本假设：工作空间在 `~/Desktop/leap_ros_ws`，机器人实体名 `leap_ros`，Ubuntu 桌面环境（`DISPLAY=:0`）。
> 换机器需要改脚本内的路径变量 `WS`。

## 启动类

### `start_sim_all.sh`
启动 Gazebo 仿真全栈：`gzserver` + `gzclient` + `xuegecar_bringup` + 雷达 + RViz + 一个控制台终端。

```bash
scripts/start_sim_all.sh
```

### `start_nav2.sh [地图yaml]`
启动 Nav2 导航。执行顺序：

0. **发零速**（Gazebo 会锁存最后一条速度指令，不归零它会继续开）
1. 杀干净所有旧 Nav2 实例（否则多实例抢话题，心跳互相挤掉）
2. 停掉 gmapping（它和 AMCL 抢 `map→odom`）
3. 启动 `nav2_bringup`
4. 等全部节点激活
5. 启动导航版 RViz

```bash
scripts/start_nav2.sh                                        # 用默认地图
scripts/start_nav2.sh ~/maps/my_room.yaml                    # 指定地图
```

### `restart_slam.sh [--keep-pose|--no-rviz]`
重启建图流程：停车 → 复位车 → 重启 gmapping → 重启 RViz。

```bash
scripts/restart_slam.sh                # 完整重启（复位车）
scripts/restart_slam.sh --keep-pose    # 不动车，接着当前位置继续建图
scripts/restart_slam.sh --no-rviz      # 不重启 RViz
```

## 控制类

### `set_initial_pose.sh <x> <y> [yaw度]`
给 AMCL 喂初始位姿。等价于 RViz 里的 `2D Pose Estimate`。

```bash
scripts/set_initial_pose.sh 0 0 0
```

> **方向必须准**：位置差 0.5 m 内 AMCL 能自己纠正，方向错 60° 以上永远纠不回来。

### `send_goal.sh <x> <y> [yaw度]`
命令行发导航目标。等价于 RViz 里的 `Nav2 Goal`。

```bash
scripts/send_goal.sh 2.5 -2.0 0
```

### `drive_keyboard.sh`
键盘遥控（`teleop_twist_keyboard`）。建图手动开时用。

> 开导航前记得关掉它，否则会抢 `/cmd_vel`。

### `drive_robot.sh <动作>`
开环自动行驶（按时间控制，不精确，仅用于快速测试）。

| 动作 | 说明 |
|---|---|
| `forward` | 前进 1 米 |
| `circle` | 绕圈 |
| `square` | 走正方形 |
| `spin` | 原地旋转 |
| `figure8` | 走 8 字 |
| `stop` | 停车 |
| `pose` | 只打印当前位置 |

### `reset_robot.sh <命令>`
| 命令 | 说明 |
|---|---|
| `reset` | 删除并重新生成模型，复位到原点 |
| `where` | 打印当前位置和四面墙距离 |

> **重要**：`reset` 会重新生成 Gazebo 模型，`diff_drive` 的 odom 从 0 重来，而 bringup 的积分还在继续 → `odom→base_footprint` 会跳变。
> **只要 reset 了车，就必须同时重启 gmapping/Nav2。顺序：先 reset 车，再重启。**
