# 架构与话题

## 一、坐标系（TF 树）

```
map ──────► odom ──────► base_footprint ──► base_link ──┬─► laser_frame
 │            │                 │                        ├─► camera_link
 │            │                 │                        ├─► ultrasonic_link
 │            │                 │                        ├─► imu_link
 │            │                 │                        └─► ...（各部件）
 │            │                 │
 │            │                 └─ 机器人在地面的投影
 │            └─ 连续、不跳变、会缓慢漂移
 └─ 全局一致、AMCL 修正时会跳变
```

| 变换 | 发布者 | 特性 | 说明 |
|---|---|---|---|
| `map → odom` | **AMCL**（导航）或 **gmapping**（建图） | 会跳变 | 定位修正量，**只允许一个发布者** |
| `odom → base_footprint` | `xuegecar_bringup` | 连续 | 由轮速积分得到 |
| `base_footprint → base_link` | `robot_state_publisher` | 静态 | URDF 固定关节 |
| `base_link → laser_frame` 等 | `robot_state_publisher` | 静态 | URDF 固定关节 |

### 为什么局部代价地图挂在 odom，全局挂在 map

| | 参考坐标系 | 原因 |
|---|---|---|
| 全局代价地图 | `map` | 规划需要"世界绝对坐标"，目标点 (2,1) 永远是同一个地方 |
| 局部代价地图 | `odom` | 控制需要"连续不跳"，坐标系跳变会让障碍物瞬移，控制器急刹 |

**记忆方法**：`map` 用来"想"，`odom` 用来"动"。

---

## 二、话题表

### 仿真层

| 话题 | 类型 | 发布者 | 频率 |
|---|---|---|---|
| `/clock` | `rosgraph_msgs/Clock` | Gazebo | 10 Hz |
| `/scan` | `sensor_msgs/LaserScan` | Gazebo 雷达插件 | 10 Hz |
| `/odom` | `nav_msgs/Odometry` | `xuegecar_bringup` | 50 Hz |
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 / 遥控 | 10~20 Hz |
| `/joint_states` | `sensor_msgs/JointState` | `joint_state_publisher` + diff_drive | 30 Hz |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | 多个 | 50 Hz |

### SLAM 层（建图时）

| 话题 | 说明 |
|---|---|
| `/map` | gmapping 输出的栅格地图（Transient Local，0.8 Hz） |
| `/map_metadata` | 地图元数据 |
| `/entropy` | 位姿熵，越小越确定 |

### 导航层（Nav2）

| 话题 | 说明 |
|---|---|
| `/map` | map_server 加载的静态地图 |
| `/amcl_pose` | AMCL 估计的位姿 |
| `/particle_cloud` | AMCL 粒子云（RViz 里的绿色箭头） |
| `/global_costmap/costmap` | 全局代价地图（`map` 帧，1 Hz） |
| `/local_costmap/costmap` | 局部代价地图（`odom` 帧，5 Hz） |
| `/plan` | 全局路径 |
| `/local_plan` | 局部路径 |
| `/initialpose` | 手动给定的初始位姿 |
| `/navigate_to_pose` | **动作**，导航目标 |

---

## 三、节点图

```
                        ┌─────────────┐
                        │   Gazebo    │
                        │ gzserver    │
                        └──┬───┬───┬──┘
                    /scan  │   │   │ /clock
                           │   │ /odom(轮关节)
                           ▼   ▼   ▼
                    ┌──────────────────┐
                    │ xuegecar_bringup │──► odom→base_footprint (TF)
                    └──────────────────┘
                           │ /odom
                           ▼
     建图模式              导航模式
  ┌──────────┐        ┌──────────────┐
  │ gmapping │        │    AMCL      │──► map→odom (TF)
  └────┬─────┘        └──────────────┘
       │ /map
       ▼                   │ /amcl_pose
   RViz 显示           ┌───▼──────────┐
                      │  map_server  │──► /map
                      └──────────────┘
                           │
                  ┌────────▼─────────┐
                  │  global_costmap  │──► /global_costmap/costmap
                  └────────┬─────────┘
                           ▼
                  ┌──────────────────┐
                  │ planner_server   │──► /plan
                  └────────┬─────────┘
                           ▼
                  ┌──────────────────┐
                  │ bt_navigator     │  (行为树调度)
                  └────────┬─────────┘
                           ▼
                  ┌──────────────────┐
                  │ controller_server│──► /cmd_vel
                  └────────┬─────────┘   (DWB 局部规划)
                           ▼
                  ┌──────────────────┐
                  │ velocity_smoother│──► /cmd_vel (平滑后)
                  └──────────────────┘
```

---

## 四、Nav2 的四个核心概念

### 1. 动作（Action）

导航任务耗时几十秒，需要持续反馈、可中止、有结果。所以 Nav2 用 **Action** 而不是 Topic。

```
你 ──目标──► bt_navigator
   ◄─反馈───   "还剩 6.04 米，预计 50 秒"  (10 Hz)
   ◄─结果───   SUCCEEDED / ABORTED
   ──取消──►   (随时可打断)
```

### 2. 生命周期（Lifecycle）

节点有状态机：`unconfigured → inactive → active`。

启动时 `lifecycle_manager` 按顺序 configure → activate，任何一个失败就停下，避免"一半模块活着"的危险状态。

它还有**心跳（bond）机制**：每秒检查节点是否活着。CPU 被挤爆导致心跳超时，会触发 `CRITICAL FAILURE: SERVER xxx IS DOWN`，然后**把所有节点 reset 下线**（进程还在，但功能全停）。

### 3. 行为树（Behavior Tree）

不是硬编码 if-else，而是一棵树：

```
[Sequence]
  ├─ [Retry] ComputePathToPose     计算全局路径
  ├─ [Retry] FollowPath            跟踪路径
  └─ [Fallback] 卡住了怎么办
        ├─ Spin       原地转找路
        ├─ Backup     倒退一点
        └─ Wait       等 5 秒
```

RViz 里的 `Recoveries` 计数就是 `[Fallback]` 被触发的次数。

### 4. 代价地图（Costmap）

把"能不能走"从 0/1 变成有梯度的代价。

**三层叠加**：StaticLayer（静态地图）→ ObstacleLayer（激光实时）→ InflationLayer（膨胀）

**代价数值**（Nav2 内部 0~255，发布时压缩到 0~100）：

| 内部值 | 发布值 | 含义 | RViz 颜色 |
|---|---|---|---|
| 254 | 100 | LETHAL 致命，禁行 | 亮品红 |
| 253 | 99 | INSCRIBED 内切，机器人中心进去就撞 | 青色 |
| 252→1 | 98→1 | 膨胀梯度，越近越贵 | 粉红渐层 |
| 0 | 0 | FREE 自由 | 白色 |
| 255 | -1 | NO_INFORMATION 未知 | 不着色 |

> **调试陷阱**：`ros2 topic echo /global_costmap/costmap` 里永远看不到 254，因为发布时被压成 100 了。

**实测的代价剖面**（西墙往房间内，y=0）：

| 离墙面 | 代价 | 含义 |
|---|---|---|
| 0.00 ~ 0.30 m | 99~100 | 禁行 |
| 0.35 m | 89 | 高代价 |
| 0.45 m | 73 | 中代价 |
| 0.55 m | 60 | 中代价 |
| 0.60 m | 0 | 自由 |

---

## 五、一次导航的完整链路

```
① RViz 点 "Nav2 Goal"
        │ /navigate_to_pose 动作请求
        ▼
② bt_navigator 启动行为树
        ▼
③ planner_server 读 global_costmap
   NavFn 用 Dijkstra 在代价栅格上搜索   ──► /plan
        ▼
④ controller_server (DWB) 每 0.1 秒：
   · 生成 30×40 = 1200 条候选速度组合
   · 在 local_costmap 上给每条打分
     （BaseObstacle / PathAlign / PathDist /
       GoalAlign / GoalDist / PreferForward / RotateToGoal / Oscillation）
   · 选出最优                        ──► /cmd_vel
        ▼
⑤ velocity_smoother 限加速度平滑     ──► /cmd_vel
        ▼
⑥ Gazebo diff_drive 插件转轮子
        │
        ├──► /odom ──► odom→base_footprint (TF)
        └──► /scan ──► AMCL ──► map→odom (TF)  ← 闭环修正
        ▼
⑦ 到达 goal tolerance (0.25 m / 14°) → SUCCEEDED
   卡住 → 行为树 Fallback → Spin / Backup / Wait
```

---

## 六、AMCL 粒子云怎么看

RViz 里的绿色箭头就是粒子云（`/particle_cloud`）：

| 箭头属性 | 含义 |
|---|---|
| 根部位置 | 候选位姿 (x, y) |
| 指向 | 候选的车头朝向 |
| 长度 | 权重（0.02 ~ 0.3 m） |

| 画面 | 含义 | 处理 |
|---|---|---|
| 紧凑一小团、方向一致 | ✅ 收敛，定位可信 | 正常导航 |
| 散开一大片 | ⚠️ 不确定 | 开一段路让它收敛 |
| 分成两团 | ❌ 位置歧义（房间对称） | 手动重给位姿 |
| 数量顶满 `max_particles` | ⚠️ AMCL 认为自己很不确定 | 同上 |

**AMCL 只在移动时更新**：`update_min_d: 0.25` / `update_min_a: 0.2`，也就是走满 25 cm 或转满 11.5° 才算一次。车停着不动，粒子云会冻结在初始散布上。

---

## 七、为什么"跳变"是必要的

`map→odom` 会跳。举例：AMCL 发现"原来我偏了 3 cm"，它不会移动机器人，而是把这个偏差一口气写进 `map→odom` —— 机器人的物理位置没变，但在 `map` 坐标系里的表示位置瞬间挪了 3 cm。

**类比**：`odom` 像手机计步器（连续但会累积误差），`map` 像真实地图，`map→odom` 就是"计步器偏差值"。走到新地标时忽然发现偏了，就修正一次，你人没动但在地图上"瞬移"了。

这就是为什么：
- **局部代价地图必须挂在 odom**（跳变会让控制器看到障碍物瞬移 → 急刹）
- **全局代价地图必须挂在 map**（odom 会漂，地图就对不上了）
