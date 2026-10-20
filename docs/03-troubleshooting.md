# 故障排查

按症状查表，再跳到对应章节。

## 快速索引

| 症状 | 章节 |
|---|---|
| 建图跑几分钟后彻底冻结，进程还活着 | [1](#1-建图冻结tf2-自死锁) |
| 改 gmapping 参数完全没反应 | [2](#2-gmapping-参数完全不生效) |
| 地图里出现跑到墙外几米的毛刺 | [3](#3-地图出现墙外毛刺) |
| 车被画在墙外 / 地图上位置明显偏 | [4](#4-定位漂移车画在墙外) |
| RViz 里的地图是旧的、不更新 | [5](#5-rviz-显示旧地图) |
| Nav2 节点全部 inactive，日志 CRITICAL FAILURE | [6](#6-nav2-全部节点被-reset) |
| `Package 'xxx' not found` | [7](#7-package-not-found) |
| `ros2 topic echo /amcl_pose` 一直没输出 | [8](#8-amcl-不给初始位姿就不发布) |
| 目标被接受但车一动不动 / 抽搐 | [9](#9-目标被接受但车不动)、[10](#10-车原地抽搐) |
| 车一直倒着走 | [11](#11-车一直倒着走) |
| 杀掉 Nav2 后车还在自己开 | [12](#12-gazebo-锁存速度指令) |
| `map_saver_cli` 报 Failed to spin map subscription | [13](#13-存地图失败) |
| 脚本一运行就退出，报 unbound variable | [14](#14-set-u-与-ros-setupbash-冲突) |
| `ros2 node list` / `ros2 lifecycle get` 查不到节点 | [15](#15-dds-发现慢导致查不到节点) |
| 网页打得开但没有数据 / 提示连不上小车 | [16](#16-网页连不上小车后端在跑但页面空) |

---

## 1. 建图冻结（tf2 自死锁）

**症状**
- 建图跑了几分钟（约 100 帧）后，`/map` 再也不更新
- 进程还活着、CPU 只有 10%、无任何报错
- 车动了也不更新

**排查**
```bash
grep -c 'update frame' /tmp/sim/gmapping.log     # 数字不再增长
ros2 topic hz /map                                # 无输出
for t in /proc/$(pgrep -f slam_gmapping | head -1)/task/*; do cat $t/wchan; echo; done
# 全部是 futex_wait_queue → 死锁
```

**根因**
`tf2_ros::MessageFilter` 在单线程执行器里与 TF 回调形成自死锁：
`setTransformImpl → testTransformableRequests → MessageFilter 回调 → 重入同一把非递归 tf2 锁`。

gdb 抓到的栈：
```
#4  libtf2_ros.so
#5  tf2::BufferCore::testTransformableRequests()
#6  tf2::BufferCore::setTransformImpl()
#8  tf2_ros::TransformListener::subscription_callback
#14 rclcpp::executors::SingleThreadedExecutor::spin()
```

**修复**
去掉 `tf2_ros::MessageFilter`，改为直连订阅 `/scan`（见 `slam_gmapping/src/slam_gmapping.cpp` 的 `startLiveSlam()`）。

---

## 2. gmapping 参数完全不生效

**症状**
改 `params/slam_gmapping.yaml` 里任何参数都毫无效果，日志里的值始终一样。

**排查**
```bash
grep -rnE 'get_parameter|declare_parameter' slam_gmapping/src/
# 只有一条注释提到 parameter → 参数从来没被读取过
```

**根因**
这个 ROS 2 端口里**一行读参数的代码都没有**。所有值是编译期写死的，而写死的默认值恰好等于 yaml 里的数值（`particles 30`、`linearUpdate 1.0`、`angularUpdate 0.5`），所以看起来"生效了"，极具欺骗性。

**修复**
在 `init()` 末尾补 `declare_parameter` + `get_parameter`，启动时打印一行 `Loaded params: ...` 供核对。

**验证**
重启后日志应出现：
```
Loaded params: particles=60 linearUpdate=0.200 angularUpdate=0.200 maxRange=12.00 maxUrange=8.00 ...
Range limits in use: maxRange=12.00 m  maxUrange=8.00 m  (laser range_max=12.00)
```

---

## 3. 地图出现墙外毛刺

**症状**
地图上出现一坨延伸到墙外好几米的杂点（实测已知区域跑到 y=+7.00，而北墙在 y=3.00）。

**根因**
墙体只有 0.1 m 厚，车贴着墙时有一部分"挤"进墙体，激光穿过墙测到了外面的虚空。

**修复**
`worlds/leap_room.world` 墙厚 0.1 → **0.3 m**，中心外移到 ±4.15 / ±3.15，保持内壁位置不变。

**验证**
```bash
# 地图已知区域应全部落在房间内
已知范围 x -4.05~+4.05   y -3.05~+3.05   ✅
```

---

## 4. 定位漂移（车画在墙外）

**症状**
RViz 里车明明在房间内，却画在墙外。

**排查**
```bash
# 对比三个量
ros2 run tf2_ros tf2_echo map base_footprint   # AMCL 认为的位置
ros2 run tf2_ros tf2_echo odom base_footprint  # 里程计认为的位置
ros2 run tf2_ros tf2_echo map odom             # 修正量（理想应接近单位矩阵）
gz model -m leap_ros -p                        # Gazebo 真值
```

**判据**：`map→odom` 偏移超过 0.2 m 或 5° 就说明扫描匹配漂了。

**根因**
车贴墙太近时，平墙在"沿墙方向"没有任何约束，粒子滤波会自由滑动。

**修复**
- 建图时离墙保持 0.3 m 以上
- `linearUpdate`/`angularUpdate` 调小（0.2），更频繁地匹配
- `srr/srt/str/stt` 调小（更相信 odom）
- 加 `particles`（30→60）

**修复后实测**：`map→odom` 从 `(170 mm, 4.92°)` 降到 `(9 mm, 0.05°)`。

---

## 5. RViz 显示旧地图

**症状**
- RViz 里的地图是上一轮的，不更新
- 车和地图明显错位
- 但用命令行订阅 `/map` 一切正常

**根因**
gmapping 每重启一次就创建一个**全新的 `/map` 发布者**，RViz 有时无法重新匹配到新发布者，于是一直显示旧地图。

**铁律**
> **重启 gmapping → 必须同时重启 RViz。**

**验证**
```bash
ros2 topic info /map | grep 'Publisher count'   # 应 ≥1
ros2 topic hz /map                              # 应有稳定频率
```

数据层正常但画面不对 = RViz 的问题，重启它。

---

## 6. Nav2 全部节点被 reset

**症状**
- RViz 面板显示 `Navigation: inactive`
- 日志出现 `CRITICAL FAILURE: SERVER xxx IS DOWN after not receiving a heartbeat`
- `Managed nodes have been reset`
- 容器进程还活着

**根因**
`lifecycle_manager` 用 bond 心跳监控各节点（默认 4 秒超时）。CPU 被挤爆时心跳发不出去，管理器判定节点死了，把所有节点下线。

**排查**
```bash
grep -c 'CRITICAL FAILURE' /tmp/sim/nav2.log
pgrep -cf component_container      # >1 说明有多个实例
cat /proc/loadavg
```

**修复**
1. 降低负载：关 `foxglove_bridge`（实测省 21% CPU）、降低 RViz 帧率、降低 AMCL 粒子数
2. 确保**只有一个** Nav2 实例：`start_nav2.sh` 里已经加了彻底清场步骤

**实测负载阈值**

| 组成 | load | 结果 |
|---|---|---|
| Gazebo + bringup + gmapping | 2.5 | ✅ |
| 上面 + RViz | 3.5 | ✅ |
| 上面 + Nav2 | 3.5 | ⚠️ 勉强 |
| 上面 + foxglove_bridge | **9.6** | ❌ 必挂 |

---

## 7. Package not found

**症状**
```
Package 'slam_gmapping' not found: searching: ['/opt/ros/humble']
```

**根因**
没 source 工作空间。注意错误里只搜了 `/opt/ros/humble` —— 这就是线索。

**修复**
```bash
source /opt/ros/humble/setup.bash
source ~/Desktop/leap_ros_ws/install/setup.bash
```

**重要**：`~/.bashrc` 只对新终端生效。**每次 `colcon build` 之后，已开着的终端要重新 source**，否则还在用旧的可执行文件。这是"我明明改了怎么没反应"的头号来源。

---

## 8. AMCL 不给初始位姿就不发布

**症状**
`ros2 topic echo /amcl_pose` 一直无输出，但 `/amcl_pose` 有发布者。

**根因**
**AMCL 在收到初始位姿之前，既不发布 `/amcl_pose`，也不发布 `map→odom`**。

日志对应：
```
[WARN] [amcl]: AMCL cannot publish a pose or update the transform. Please set the initial pose...
```

**修复**
二选一：
```bash
# 手动给
scripts/set_initial_pose.sh 0 0 0
```
或直接在 `xuegebot.yaml` 里硬编码（本项目已做）：
```yaml
amcl:
  ros__parameters:
    set_initial_pose: true
    initial_pose: {x: 0.0, y: 0.0, z: 0.0, yaw: 0.0}
```

---

## 9. 目标被接受但车不动

**症状**
面板显示有目标、`Distance remaining` 不减少、`Recoveries` 一直涨、日志 `Failed to make progress`。

**排查**
```bash
# 1. 局部代价地图是不是坏的
grep -c 'argument target_frame does not exist' /tmp/sim/nav2.log
# 会一直增长 → 见下

# 2. 车周围代价
# 用 tools/ 里的探针脚本采样 /local_costmap/costmap

# 3. 定位准不准
ros2 topic echo /amcl_pose --once 对比 gz model -m leap_ros -p

# 4. 谁在发速度
ros2 topic info /cmd_vel
```

**根因**
局部代价地图配了 `StaticLayer`（Nav2 官方默认**没有**）。它需要 `map→odom` 变换，当 AMCL 重新初始化时那一瞬间拿不到，`StaticLayer` 会进入失败状态**且不会自愈**。

**修复**
重启 Nav2 即可恢复。根治方法：局部代价地图只保留 `ObstacleLayer + InflationLayer`（官方默认配置）。

---

## 10. 车原地抽搐

**症状**
车在原地小幅左右抖动，完全不前进。日志 `No valid trajectories` 和 `Failed to make progress` 大量出现，恢复行为次数飙升。

**根因**
**雷达 `range_min` 设成了 0.02 m，扫到了车体自身。**

实测（房间正中心，最近真实障碍 3 m）：
```
最近距离 0.096 m
距离 <0.20 m 的点: 24 / 360   ← 这些是车体
```

这些点让代价地图在车脚下生成致命障碍 → DWB 1200 条候选轨迹全部判碰撞 → 只能原地微转。

**排查**
```bash
# 把车放在房间正中心，测激光最近距离
ros2 topic echo /scan --once
# 若最近距离 < 0.2 m 且周围空旷 → 就是这个问题
```

**修复**
`leap_ros.gazebo.sdf`：`<min>0.02</min>` → **`<min>0.25</min>`**（大于车体半径 0.08 m），然后重新 spawn 机器人。

**验证**
```
range_min = 0.250
最近距离  = 1.170 m
距离 <0.25 m 的点: 0 / 360   ✅
```

**修复后**：`No valid trajectories` 16 → **0**，`Failed to make progress` 29 → **0**，恢复行为 14 → **0**。

---

## 11. 车一直倒着走

**症状**
导航时车总是以车尾朝前的方式移动。

**排查**
```bash
# 采样 /cmd_vel 的 linear.x 符号分布
python3 tools/cmdtest.py
```
实测修复前：`linear.x < 0` 占 **399/400 = 100%**，且一直顶在 `-0.12`（`min_vel_x` 的极限）。

**根因**
- `min_vel_x: -0.12` 允许 DWB 倒车
- critics 列表里**没有 `PreferForward`**，即没有任何"惩罚倒车"的评分项 → DWB 找不到前进优势

**修复**
```yaml
controller_server:
  FollowPath.DWBLocalPlanner:
    min_vel_x: 0.0                       # 禁止主动倒车
    critics: [..., "GoalDist", "PreferForward"]   # 加正向偏好
    PreferForward.scale: 5.0
```

**注意**：禁止 DWB 倒车**不影响脱困**。`behavior_server` 的 `backup` 恢复行为有独立的速度参数，卡住时照样能倒退。

**验证**
修复后：`linear.x < 0` 占 **0%**，`> 0` 占 71%，最大值到 `+0.300`。

---

## 12. Gazebo 锁存速度指令

**症状**
杀掉 Nav2 后，车还在按最后的速度自己开，一路撞到墙上才停。

**根因**
Gazebo 的 `diff_drive` 插件会**锁存最后一条 `cmd_vel`**，没有超时归零机制。

**修复**
任何"停止"流程都必须显式发零速。`scripts/start_nav2.sh` 的第一步就是：
```bash
timeout 12 python3 -c "
import rclpy, time
from rclpy.node import Node
from geometry_msgs.msg import Twist
rclpy.init(); n=Node('stop_'); p=n.create_publisher(Twist,'/cmd_vel',10)
t0=time.time()
while time.time()-t0<1.5: p.publish(Twist()); time.sleep(0.05)
rclpy.shutdown()"
```

---

## 13. 存地图失败

**症状**
```
[ERROR] [map_saver]: Failed to spin map subscription
```

**根因**
`map_saver_cli` 默认只等 2 秒，DDS 发现慢时会超时。

**修复**
```bash
ros2 run nav2_map_server map_saver_cli -f <路径> \
  --ros-args -p save_map_timeout:=20.0
```

注意没有 `-t` 这个命令行参数，必须用 ROS 参数形式。

---

## 14. set -u 与 ROS setup.bash 冲突

**症状**
脚本一运行就退出：
```
/opt/ros/humble/setup.bash: line 8: AMENT_TRACE_SETUP_FILES: unbound variable
```

**根因**
脚本开头的 `set -u`（遇未定义变量即退出）与 ROS 的 `setup.bash` 冲突。

**修复**
把 `set -u` 改成 `set +u`。

---

## 15. DDS 发现慢导致查不到节点

**症状**
- `ros2 node list` 只返回几个节点，或缺节点
- `ros2 lifecycle get /amcl` 报 `Node not found`
- 但 `ros2 topic echo` 正常

**根因**
这台 VM 的 Fast-DDS 发现较慢（多播受限），CLI 工具经常只看到部分节点。**这不是故障。**

**判据**
```bash
# 如果话题有发布者、数据正常，就说明系统是好的
ros2 topic info /amcl_pose      # Publisher count: 1 → 正常
```

同理，`/cmd_vel` 的发布者数量在被杀进程后会**短暂虚高**（DDS 僵尸端点），约 20~60 秒自行消失。真正要看的是话题有没有数据。

---

## 16. 网页"连不上小车"（后端在跑但页面空）

### 症状

- `http://<Jetson-IP>:5173/` 能打开，页面框架正常
- 但地图空白、没有雷达点、显示"未连接"/"连不上小车"
- 直觉会以为是小车掉线了

### 关键判据：先分清是"车的问题"还是"桥的问题"

网页的数据链路是三段串联，任何一段断了页面都是空的：

```
小车 ──micro-ROS/UDP──> Jetson(ROS2) ──foxglove_bridge:8765──> 网页
```

按这三层逐一确认：

```bash
# 第一层：小车在线吗（ROS 话题有没有数据）
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
ros2 topic hz /scan      # 有 ~8 Hz → 车和 micro-ROS 都好
ros2 topic hz /odom      # 有 ~40 Hz
ros2 topic hz /map       # 建图时 ~2 Hz

# 第二层：数据桥接在吗（这一层最容易挂）
pgrep -cf foxglove_bridge        # 应为 2；是 0 就是它挂了
ss -tln | grep 8765              # 必须 LISTEN

# 第三层：后端和页面
ss -tln | grep -E ':3000|:5173'
```

**如果第一层有数据、第二层是 0，那就是数据桥接丢了，小车本身完全正常。**

### 根因：foxglove_bridge 是 node server.js 的子进程

`foxglove_bridge` 由后端的 startup 任务拉起，是 `node server.js` 的**子进程**。
所以一旦你：

- 单独 `pkill -f 'node server.js'` 然后重启后端
- 只重启了后端而没重新触发 startup 任务
- 机器重启后只手动起了后端

`foxglove_bridge` 就一起被杀掉且**不会自动回来** —— 于是 ROS 数据明明在全速流动，页面却一片空白。

### 修复：后端末尾的自愈看门狗

`/home/nvidia/robot/Ros_Vue_arm/server.js` 文件末尾追加了一段 watchdog：
后端启动 5 秒后自己 `POST /api/run_task/startup`，把 `foxglove_bridge` 拉起来。

```js
// AUTO_STARTUP_WATCHDOG：后端重启后自动拉起 startup 任务（foxglove_bridge）
setTimeout(async () => {
    try {
        const res = await fetch('http://127.0.0.1:3000/api/run_task/startup', { method: 'POST' });
        console.log('[watchdog] 已触发 startup 任务，HTTP', res.status);
    } catch (e) {
        console.error('[watchdog] 触发失败:', e.message);
    }
}, 5000);
```

**两个坑，改 server.js 时务必注意：**

1. **这个文件是 ESM**（`package.json` 里有 `"type": "module"`），
   不能用 `require('http')`，否则运行时崩 `ReferenceError: require is not defined`。
   要用 Node 内置的全局 `fetch`（Node 18+ 可用，Jetson 上是 v22）。
2. **不要把代码插进 `app.listen(port, () => {...})` 的回调体内部** ——
   用 `find('});')` 找闭合括号会命中回调里嵌套的 `});`，
   直接导致 `SyntaxError: Unexpected end of input`。
   **追加到文件最末尾**最安全。

改完**先验证语法再重启**，否则你会把后端也搞挂：

```bash
node --check /home/nvidia/robot/Ros_Vue_arm/server.js &&   pkill -f 'node server.js'; cd /home/nvidia/robot/Ros_Vue_arm;   nohup setsid node server.js > /tmp/server.log 2>&1 < /dev/null &
```

### 已验证自愈

```bash
pkill -f foxglove_bridge                  # 手动杀掉桥
pgrep -cf foxglove_bridge                 # 0
pkill -f 'node server.js'; nohup setsid node server.js ... &   # 重启后端
sleep 25
pgrep -cf foxglove_bridge                 # 2   ← 自动回来
ss -tln | grep 8765                       # LISTEN
```

### 手动兜底（看门狗万一失效）

不用重启后端，直接让后端跑一次 startup 任务：

```bash
curl -X POST http://localhost:3000/api/run_task/startup
sleep 5
pgrep -cf foxglove_bridge    # 2
```

---

## 附：标准排查流程

出现"导航不动"时，按顺序查这 6 项：

```bash
# ① 仿真健康吗
ros2 topic hz /clock      # 应 ~10 Hz
ros2 topic hz /scan       # 应 ~10 Hz
ros2 topic hz /odom       # 应 ~50 Hz

# ② 定位正常吗
ros2 topic echo /amcl_pose --once
gz model -m leap_ros -p          # 两者应接近

# ③ TF 树完整吗
ros2 run tf2_ros tf2_echo map base_footprint

# ④ 代价地图健康吗
grep -c 'argument target_frame does not exist' /tmp/sim/nav2.log   # 不应持续增长

# ⑤ 谁在发速度
ros2 topic info /cmd_vel
ros2 topic echo /cmd_vel --once

# ⑥ 资源够吗
cat /proc/loadavg          # 2 核时应 < 4
grep -c 'CRITICAL FAILURE' /tmp/sim/nav2.log   # 应为 0
```

哪一项不对，就跳回对应章节。
