# 真车部署指南（NVIDIA Jetson Orin NX + leap_ros）

本文档记录把本项目从 Gazebo 仿真部署到**真车**的完整过程与踩过的坑。

---

## 一、硬件架构

```
┌─────────────────────────┐         WiFi 192.168.31.0/24
│  leap_ros 小车           │◄──────────────────────────┐
│  ESP32-S3 下位机         │   micro-ROS over UDP:8888 │
│  192.168.31.174          │   (车主动连接上位机)        │
│                          │                            │
│  ├─ 电机 + 编码器         │                            │
│  ├─ Camsense 激光雷达     │                            │
│  ├─ IMU                 │                            │
│  ├─ HC-SR04 超声波       │                            │
│  └─ 电池 ADC             │                            │
└─────────────────────────┘                            │
                                                        │
┌────────────────────────────────────────────┐         │
│  NVIDIA Jetson Orin NX Super               │         │
│  192.168.31.147  Ubuntu 22.04 aarch64      │         │
│                                            │         │
│  micro_ros_agent (UDP 8888) ───────────────┘         │
│         ↓ 桥接到 ROS 2 DDS                            │
│  xuegecar_bringup  →  odom→base_footprint TF          │
│  robot_state_publisher → 静态 TF                       │
│  slam_gmapping / Nav2                                 │
└────────────────────────────────────────────┘         │
                                                        │
┌─────────────────────────┐                            │
│  Maturo 图传相机         │   HTTP MJPEG :81 ──────────┘
│  192.168.31.39          │
└─────────────────────────┘
```

### 关键点：**车主动连接上位机**

ESP32 作为 micro-ROS 客户端，主动连接 `agent_ip:8888`。所以：
- 上位机**只需监听** UDP 8888，不需要能 ping 通小车
- 代理地址保存在 MCU 的 NVS 里，断电重启后仍生效

---

## 二、小车 Web 配置 API

小车的 ESP32-S3 固件提供了一个 HTTP 配置页面和 JSON API。

### 查询状态

```bash
curl http://192.168.31.174/api/status
```

返回示例（节选）：

```json
{
  "device_name": "Maturo_E072A1B4F2D8",
  "uptime_ms": 937830,
  "runtime": {
    "comm_mode": "micro_ros",
    "microros_agent_ip": "192.168.31.147",
    "microros_agent_port": 8888
  },
  "wifi": { "mode": "STA", "ssid": "Xiaomi", "rssi": -60, "channel": 8 },
  "lidar": { "valid": true, "valid_points": 334, "min_mm": 218, "max_mm": 7537 },
  "battery": { "voltage_v": 7.826, "percentage": 76 },
  "ultrasonic": { "distance_cm": 51.17, "obstacle": false },
  "motion": { "vx": 0.0, "wz": 0.0, "target_vx": 0.0, "target_wz": 0.0 }
}
```

### 修改通信模式与代理地址

```bash
curl -X POST http://192.168.31.174/api/runtime-config \
  -H "Content-Type: application/x-www-form-urlencoded" \
  -d "comm_mode=micro_ros&microros_agent_ip=192.168.31.147&microros_agent_port=8888"
```

返回 `{"ok":true,"restart":true}`，MCU 会重启应用配置。

可选 `comm_mode`：`micro_ros` / `mavlink_udp` / `uart_mavlink`

### 其他端点

| 端点 | 说明 |
|---|---|
| `GET /api/status` | 完整状态 |
| `POST /api/runtime-config` | 通信模式 + agent 地址 |
| `GET /api/scan` | WiFi 扫描 |
| `POST /api/provision` | WiFi 配网 |
| `POST /api/camera/provision` | 相机配网 |

> **注意**：`http://192.168.31.174/` 不带 `?lang=zh` 时可能返回空响应，加参数即可。

---

## 三、部署步骤

### 1. 环境检查

```bash
uname -m                      # 必须是 aarch64
ls /opt/ros/                  # 应看到 humble
ros2 pkg list | wc -l         # 应 >= 300
```

### 2. 安装依赖

```bash
sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup \
                    ros-humble-nav2-map-server ros-humble-micro-ros-msgs
```

> **`micro_ros_agent` 不在 ROS apt 源里**，必须从源码编译（见下一步）。

### 3. 编译 micro_ros_agent

```bash
mkdir -p ~/micro_ros_ws/src && cd ~/micro_ros_ws/src
git clone -b humble --depth 1 https://github.com/micro-ROS/micro-ROS-Agent.git
cd ~/micro_ros_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select micro_ros_agent
```

Jetson Orin NX 上约 1 分钟。

### 4. 部署本项目

```bash
mv ~/leap_ros_ws ~/leap_ros_ws.vendor        # 备份原有
mkdir -p ~/leap_ros_ws/src && cd ~/leap_ros_ws/src
git clone https://github.com/apodxx/Simulation_ROS2.git
cd ~/leap_ros_ws && colcon build --symlink-install
```

### 5. 统一 ROS_DOMAIN_ID

```bash
# .bashrc 里设置
export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash
source ~/leap_ros_ws/install/setup.bash
source ~/micro_ros_ws/install/setup.bash
```

### 6. 启动

```bash
~/leap_ros_ws/src/Simulation_ROS2/scripts/start_real.sh
```

脚本会：清场 → 起 agent → 等小车连接 → 起底盘 bringup → 自检 → 起 RViz

### 7. 建图

```bash
ros2 launch slam_gmapping slam_gmapping_real.launch.py    # 注意用 real 版
```

`start_real.sh` 里已包含。手动遥控：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## 四、真车与仿真的关键差异

| 项目 | 仿真 | 真车 |
|---|---|---|
| 时钟 | `/clock`（必须 `use_sim_time: true`） | 无 `/clock`（**必须 `use_sim_time: false`**） |
| 雷达 | Gazebo 插件 | ESP32 通过 micro-ROS 转发 |
| `/scan` frame | `laser_frame` | `laser_frame`（相同） |
| `/odom` child | `base_footprint` | **`base_link`**（bringup 转成 TF） |
| 雷达 range | 0.02–12 m | **0.02–12 m**（相同，但实测最小 0.218 m） |
| 相机 | 无 | Maturo MJPEG :81 |
| 底盘 | Gazebo diff_drive | ESP32 电机 PID |

> **最容易出错的一条**：仿真用的 `slam_gmapping.yaml` 里 `use_sim_time: true`，真车必须用 `slam_gmapping_real.yaml`（`use_sim_time: false`），否则 gmapping 会一直等 `/clock` 卡死。

---

## 五、踩过的坑

### 1. ROS_DOMAIN_ID 不匹配导致话题看不到

现象：agent 日志显示会话已建立、创建了 6 个发布者，但 `ros2 topic list` 只有 `/rosout`。

原因：系统 `.bashrc` 里设了 `ROS_DOMAIN_ID=42`，而 agent 实际发布在 **domain 0**。

解决：统一为 `ROS_DOMAIN_ID=0`。

**排查方法**：两个 domain 都试一遍
```bash
ROS_DOMAIN_ID=0  ros2 topic list
ROS_DOMAIN_ID=42 ros2 topic list
```

### 2. 相机流 + gmapping 同时跑会把 micro-ROS 链路挤崩

现象：
- `/scan` 从 10 Hz 掉到 **0.78 Hz**
- `/odom` 从 50 Hz 掉到 4.4 Hz
- agent 日志疯狂 `create_client` / `delete_client`（会话反复掉线）

根因：
- `slam_gmapping` 吃满 **一个 CPU 核（105%）**
- `http_video_publisher` MJPEG 解码占 **35% CPU**
- 相机 HTTP 流与 micro-ROS UDP **争抢同一个 WiFi**

解决：
- **建图时关掉相机**
- gmapping 粒子数从 60 降到 40
- 关掉后链路恢复：`/scan` 8 Hz、`/odom` 49 Hz、30 秒 0 次掉线

### 3. 短指令几乎不动（启动斜坡）

实测：

| 指令 | 时长 | 实际位移 | 期望 |
|---|---|---|---|
| +0.08 m/s | 1.0 s | **1.0 cm** | 8 cm |
| +0.08 m/s | 2.0 s | 25.9 cm | 16 cm |
| +0.15 m/s | 2.0 s | **29.65 cm** | 30 cm ✅ |
| +0.30 m/s | 2.0 s | **61.44 cm** | 60 cm ✅ |
| −0.10 m/s | 2.0 s | **−20.48 cm** | 20 cm ✅ |

**规律**：电机有明显的启动斜坡，**指令短于 1 秒基本不动**。Nav2 调参时要注意 `min_vel_x` 不能太小。

### 4. `/imu` 是 BEST_EFFORT

订阅 `/imu` 时用默认的 reliable QoS 会收不到数据，必须：

```python
from rclpy.qos import qos_profile_sensor_data
node.create_subscription(Imu, '/imu', cb, qos_profile_sensor_data)
```

`/scan` 同样。

### 5. 地图呈"星芒"状不是 bug

现象：地图是一个白色圆盘 + 放射状条纹，看不到房间轮廓。

检验：把当前激光端点投影到地图，**69.3% 落在障碍格上** → 地图与激光一致。

真正原因：
- 只走了 2.9 米、只处理 142 帧，覆盖太少
- **1° 雷达在 5 米处射线间距 8.7 cm > 5 cm 网格**，远处只能清出稀疏射线

解决：**多开、慢开**，让同一区域被不同角度多次观测。

---

## 六、相机接入

`xuegecar_camera` 包把 HTTP MJPEG 流转成 ROS 图像话题：

```bash
ros2 launch xuegecar_camera http_video_publisher.launch.py \
  url:=http://192.168.31.39:81/ fps:=10.0
```

发布：`/camera/image_raw/compressed`（`sensor_msgs/CompressedImage`，`frame_id=camera_link`）

配置在 `xuegecar_camera/config/http_video_publisher.yaml`。

> 默认 `publish_raw: false`（不发原始 RGB），因为原始图像带宽很大，会让 HTTP 流卡顿。

---

## 七、已知限制

| 项 | 状态 |
|---|---|
| 建图速率 | `/scan` 约 8 Hz（受 agent + CPU 限制） |
| 地图稀疏 | 1° 雷达固有，需多次观测 |
| 相机带宽 | 与 micro-ROS 抢 WiFi，建图时应关闭 |
| 磁盘 | 部署后剩余约 14 GB，注意监控 |

---

## 八、故障速查

```bash
# 链路是否活着
grep -c 'create_client' /tmp/agent.log      # 应该稳定不增

# 话题是否在发
for t in /scan /odom /imu; do ros2 topic hz $t; done

# 小车自身状态
curl -s http://192.168.31.174/api/status | python3 -m json.tool

# TF 链是否完整
ros2 run tf2_ros tf2_echo map base_footprint
```

**症状 → 处理**

| 症状 | 处理 |
|---|---|
| `ros2 topic list` 看不到小车话题 | 检查 `ROS_DOMAIN_ID` 是否为 0 |
| agent 会话反复掉线 | 关相机、降低 gmapping 粒子数 |
| 车不动 | 检查超声波是否触发 `obstacle: true`；确认前方 0.35 m 以上 |
| gmapping 卡住不初始化 | 确认用的是 `slam_gmapping_real.launch.py`（`use_sim_time: false`） |
