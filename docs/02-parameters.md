# 参数说明与改动记录

本文档记录本项目相对原始 `leap_ros_ws` 的全部修改，以及每处改动的**原因**。

---

## 一、gmapping（SLAM）

### 1.1 源码修复（`slam_gmapping/src/slam_gmapping.cpp`）

原始代码存在 4 个严重 bug，导致"改参数无效""跑几分钟就冻结"。

| # | 位置 | 问题 | 修复 |
|---|---|---|---|
| 1 | 全文 | **一行读参数的代码都没有**，`params/slam_gmapping.yaml` 完全不生效；所有值是编译期写死的（恰好与 yaml 数值相同，极具欺骗性） | 在 `init()` 末尾补上 `declare_parameter` + `get_parameter` 读取全部参数，并打印 `Loaded params: ...` 便于核对 |
| 2 | `initMapper()` 内 | `maxRange_ = scan->range_max - 0.01;` 无条件覆盖，yaml 里的 `maxRange`/`maxUrange` 永远被冲成 11.99 | 改为仅在参数 ≤ 0 时才用 `range_max` 作默认值，并打印 `Range limits in use: ...` |
| 3 | `startLiveSlam()` | 用 `tf2_ros::MessageFilter` 包装 `/scan`。它会在单线程执行器里与 TF 回调形成**自死锁**：`setTransformImpl → testTransformableRequests → MessageFilter 回调 → 重入同一把 tf2 锁` | 改为直连订阅 `/scan`（原文件注释里就有正确写法） |
| 4 | `laserCallback()` | `last_map_update` 是**局部变量**，每次回调归零 → 发布节流完全失效 → `updateMap()` 每次扫描都跑（O(历史长度) 全轨迹重算） | 改为成员变量 `last_map_update_` |
| 5 | `getOdomPose()` | `buffer_->transform(..., 1.0s)` 在单线程执行器里**阻塞最长 1 秒**，期间 TF 回调无法执行 | 改为零阻塞：先按扫描时间戳查一次，失败立刻退回最新帧，全程不等待 |

> 对应头文件 `slam_gmapping.h` 新增成员：`sub_`（直连订阅）、`last_map_update_`。

### 1.2 参数（`slam_gmapping/params/slam_gmapping.yaml`）

| 参数 | 原值（写死） | 新值 | 原因 |
|---|---|---|---|
| `use_sim_time` | `false` | **`true`** | 原本 `map→odom` 用墙钟时间戳，而 `odom→base` 用仿真时间，两套时间混在一棵 TF 树里 |
| `linearUpdate` | `1.0` | **`0.2`** | 走 20 cm 就做一次扫描匹配（原来走 1 米才做，车卡住时地图完全冻结） |
| `angularUpdate` | `0.5` | **`0.2`** | 转 11.5° 就匹配一次，减少航向漂移 |
| `temporalUpdate` | `1.0` | **`0.5`** | 停着不动时也持续更新 |
| `particles` | `30` | **`60`** | 墙角等退化场景下 30 个粒子权重完全分不开（实测 `neff=29.96/30`） |
| `iterations` | `5` | **`10`** | 扫描匹配迭代更充分 |
| `srr / srt / str / stt` | `0.1/0.2/0.1/0.2` | **`0.01/0.02/0.01/0.02`** | 仿真里轮式里程计几乎无误差，应更相信 odom |
| `maxRange` | 被强制覆盖为 11.99 | **`12.0`** | 超过此距离的激光点丢弃 |
| `maxUrange` | 被强制覆盖为 11.99 | **`8.0`** | 参与扫描匹配的最大距离（房间最远角 5 m，8 m 足够） |
| `map_update_interval` | `0.5`（写死） | **`1.0`** | 地图发布频率 |

---

## 二、Gazebo 世界（`xuegecar_description/worlds/leap_room.world`）

| 项目 | 原值 | 新值 | 原因 |
|---|---|---|---|
| 墙厚 | `0.1 m` | **`0.3 m`** | 0.1 m 薄墙时，车会从墙缝"挤"出去，地图里出现跑到墙外 4 米的毛刺；加厚后内壁仍在 x=±4 / y=±3，房间内部尺寸不变 |
| 墙中心位置 | ±4.0 / ±3.0 | **±4.15 / ±3.15** | 配合加厚，保持内壁位置不变 |

---

## 三、机器人模型（`xuegecar_description/urdf/leap_ros.gazebo.sdf`）

Gazebo 加载的是 **SDF**，不是 URDF（URDF 会被转换成 SDF，且固定关节会被合并、`<sensor><pose>` 会丢失）。

| 参数 | 原值 | 新值 | 原因 |
|---|---|---|---|
| 雷达 `<range><min>` | `0.02 m` | **`0.25 m`** | **最关键的一条**：0.02 m 会让雷达扫到车体自身（实测房间正中心有 24/360 个点在 0.096~0.20 m），代价地图因此在车脚下生成致命障碍，DWB 所有前进轨迹判碰撞 → 车原地抽搐不动 |
| 雷达 `<visualize>` | `1` | `0` | 减少 Gazebo 渲染负担 |
| 雷达 `<pose>` | 丢失 | **`-0.0227 0.000062 0.12575`** | URDF→SDF 转换时 `<sensor><pose>` 会被丢弃，必须手工补 |
| 轮子 `<cylinder>` 的 `rpy` | 无 | **`1.5708 0 0`** | URDF 的 cylinder 是 Z 轴对齐，不转会导致车轮歪 9.3°、离地 19 mm |

---

## 四、Nav2（`xuegecar_navigation2/param/xuegebot.yaml`）

### 4.1 AMCL

| 参数 | 原值 | 新值 | 原因 |
|---|---|---|---|
| `min_particles` | `500` | **`300`** | 2 核 VM 减负 |
| `max_particles` | `2000` | **`800`** | 同上 |
| `transform_tolerance` | `1.0` | **`2.0`** | 容忍 TF 延迟，避免 `Transform data too old` |
| `set_initial_pose` | 无 | **`true`** | 仿真里车永远在原点，自动初始化，免手动点 `2D Pose Estimate` |
| `initial_pose.{x,y,z,yaw}` | 无 | **`0,0,0,0`** | 配合上一条 |

### 4.2 Controller (DWB)

| 参数 | 原值 | 新值 | 原因 |
|---|---|---|---|
| `controller_frequency` | `20.0` | **`10.0`** | 2 核 VM 上 20 Hz 一直 `missed its desired rate` |
| `min_vel_x` | **`-0.12`** | **`0.0`** | **关键**：允许倒车 + 缺 `PreferForward` → DWB 100% 时间以最大倒车速度行驶，表现为"车一直倒着走" |
| `critics` | 缺 `PreferForward` | **加上** | 增加正向偏好 |
| `PreferForward.scale` | 无 | **`5.0`** | 同上 |

### 4.3 Costmap

| 参数 | 原值 | 新值 | 原因 |
|---|---|---|---|
| `inflation_radius`（全局+局部） | `0.2` | **`0.35`** | 0.2 m 时车容易贴障碍物太近，扫描点把自身足迹判成碰撞，触发 `Collision Ahead` → 恢复行为 13 次。改 0.35 后降到 **0 次** |
| `cost_scaling_factor` | `3.0` | **`2.0`** | 配合上面的膨胀半径 |

---

## 五、RViz 配置

### 5.1 `xuegecar_description/rviz/sim.rviz`（建图用）

| 项目 | 改动 | 原因 |
|---|---|---|
| Map 显示 | **新增** `/map`（Transient Local, Alpha 0.9, Draw Behind） | 原本没有 Map 显示项，只能看到激光点，看不到建出来的地图 |
| Frame Rate | `30` → **`10`** | 省 ~20% CPU |

### 5.2 `xuegecar_navigation2/rviz/xuege_navigation2.rviz`（导航用）

| 项目 | 改动 | 原因 |
|---|---|---|
| TopDownOrtho `Scale` | `127.88` → **`90`** | 点 `Reset` 按钮后能看全房间 |
| TopDownOrtho `X` / `Y` | `-0.044, -0.387` → **`0, 0`** | 焦点归零到房间中心 |

---

## 六、当前生效的关键参数一览

### gmapping

```yaml
use_sim_time: true
linearUpdate: 0.2
angularUpdate: 0.2
temporalUpdate: 0.5
particles: 60
iterations: 10
srr: 0.01  srt: 0.02  str: 0.01  stt: 0.02
maxRange: 12.0
maxUrange: 8.0
delta: 0.05
map_update_interval: 1.0
```

### Nav2

```yaml
amcl:
  min_particles: 300
  max_particles: 800
  transform_tolerance: 2.0
  update_min_d: 0.25
  update_min_a: 0.2
  set_initial_pose: true
  initial_pose: {x: 0.0, y: 0.0, z: 0.0, yaw: 0.0}

controller_server:
  controller_frequency: 10.0
  FollowPath.DWBLocalPlanner:
    min_vel_x: 0.0
    max_vel_x: 0.3
    max_vel_theta: 1.2
    vx_samples: 30
    vtheta_samples: 40
    sim_time: 2.5
    critics: [RotateToGoal, Oscillation, BaseObstacle,
              GoalAlign, PathAlign, PathDist, GoalDist, PreferForward]

global_costmap / local_costmap:
  inflation_radius: 0.35
  cost_scaling_factor: 2.0
  robot_radius: 0.1
  resolution: 0.05
```

---

## 七、调参建议

| 想达到的效果 | 调什么 | 注意 |
|---|---|---|
| 车离墙更远 | ↑ `inflation_radius` | 太大会在窄通道里规划失败 |
| 路径更平滑自然 | ↑ `cost_scaling_factor` (5~10) | 越大代价衰减越快，膨胀区边缘越"软" |
| 建图更不容易漂 | ↓ `linearUpdate` / `angularUpdate` | 太小会吃 CPU |
| 建图更精细 | ↑ `particles`、↓ `delta` | 都吃 CPU 和内存 |
| 导航更快 | ↑ `max_vel_x`、↑ `controller_frequency` | 小车轮径小，太快会打滑 |
| 定位更稳 | ↓ AMCL 的 `update_min_d` | 吃 CPU |
| 省 CPU | ↓ `max_particles`、↓ RViz 帧率、关掉 foxglove_bridge | — |

---

## 八、性能参考（2 vCPU / 8 GB）

| 场景 | load average | 是否稳定 |
|---|---|---|
| Gazebo + bringup + gmapping | ~2.5 | ✅ 稳定 |
| 上面 + RViz | ~3.5 | ✅ 勉强 |
| 上面 + Nav2 | ~3.5 | ⚠️ 心跳可能超时 |
| 上面 + foxglove_bridge | **~9.6** | ❌ Nav2 心跳超时，全部节点被 reset |
| 上面 + Firefox | 更高 | ❌ 必挂 |

**结论**：2 vCPU 是下限。关掉 `foxglove_bridge` 和 RViz 能显著改善。
