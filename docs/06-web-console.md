# 网页中控台部署

通过浏览器操控小车：摇杆遥控、电量监控、SLAM 地图可视化、建图/导航任务管理。

访问地址：**http://192.168.31.147:5173/**

---

## 一、架构

```
浏览器
  │
  ├── HTTP  :5173  ──► Vite 开发服务器（Vue 3 前端）
  ├── HTTP  :3000  ──► Express 后端（任务管理 API）
  └── WS    :8765  ──► foxglove_bridge（ROS 2 数据桥接）
                              │
                              ▼
                        ROS 2 (ROS_DOMAIN_ID=0)
                              │
                    ┌─────────┴─────────┐
                    ▼                   ▼
             micro_ros_agent      xuegecar_bringup
                    │                   │
                    ▼                   ▼
              小车 ESP32-S3         /scan /odom /tf ...
```

| 端口 | 服务 | 作用 |
|---|---|---|
| **5173** | Vite | 前端界面（浏览器访问） |
| **3000** | Express (`server.js`) | 任务启停、地图列表、配置 API |
| **8765** | foxglove_bridge | 把 ROS 2 话题推给网页 |

---

## 二、项目位置

```
/home/nvidia/robot/
├── Ros_Vue_arm/              # 中控台前端 + 后端
│   ├── src/                  # Vue 源码
│   ├── server.js             # Express 后端（端口 3000）
│   ├── ros_config.json       # ★ 主配置（话题、相机、任务）
│   ├── vite.config.js        # 端口 5173，host 0.0.0.0
│   ├── start_console.sh      # ★ 一键启动
│   └── node_modules/         # 已装好
├── task_mapping.sh           # ★ 建图任务脚本
├── task_navigation.sh        # ★ 导航任务脚本
├── config/
│   ├── slam_toolbox.yaml     # slam_toolbox 参数
│   └── nav2.yaml             # Nav2 参数
└── start_robot.launch.py     # robot_state_publisher 启动文件
```

---

## 三、启动

```bash
cd ~/robot/Ros_Vue_arm
./start_console.sh
```

脚本做的事：

1. 加载 nvm / Node.js 环境
2. 加载 ROS 2 和 `~/leap_ros_ws/install/setup.bash`
3. 启动 `node server.js`（端口 3000）
4. 轮询等后端就绪
5. `POST /api/run_task/startup` → 自动启动 `foxglove_bridge`（端口 8765）
6. 启动 `npm run dev`（Vite，端口 5173）
7. 退出时自动停止 ROS 任务

访问：**http://<Jetson_IP>:5173/**

---

## 四、本次部署所做的修改

原配置指向的是旧环境，需要改 4 处：

### 1. `ros_config.json`

| 项 | 原值 | 新值 |
|---|---|---|
| `camera.stream_url` | `http://192.168.0.104:81/` | **`http://192.168.31.39:81/`** |
| `topics.battery` | `/battery` | **`/battery_state`**（真车实际话题名） |
| `save_map.save_dir` | `/home/nvidia/robot/leap_ros_ws/...` | **`/home/nvidia/leap_ros_ws/src/Simulation_ROS2/xuegecar_navigation2/maps`** |
| `maps_dir` | 同上 | 同上 |

备份：`ros_config.json.bak_deploy`

### 2. `start_console.sh`

```diff
- ROBOT_WORKSPACE="/root/ROS/leap_ros_ws"
+ ROBOT_WORKSPACE="/home/nvidia/leap_ros_ws"
```

备份：`start_console.sh.bak_deploy`

### 3. `task_mapping.sh` / `task_navigation.sh`

```diff
- export ROS_DOMAIN_ID=42
+ export ROS_DOMAIN_ID=0        # 与 micro-ROS agent 一致
```

备份：`*.bak_deploy`

### 4. `task_mapping.sh` 增加 gmapping 清理

```diff
  pkill -f "$PAT" 2>/dev/null
+ pkill -f slam_gmapping 2>/dev/null   # 避免和 gmapping 抢 map->odom
```

---

## 五、后端 API

| 端点 | 方法 | 作用 |
|---|---|---|
| `/api/runtime_config` | GET | 返回话题表 + 相机地址 |
| `/api/run_task/:name` | POST | 启动任务（`startup` / `mapping` / `navigation`） |
| `/api/stop_task/:name` | POST | 停止任务 |
| `/api/task_status/:name` | GET | 查询任务是否在跑 |
| `/api/list_maps` | GET | 列出地图目录下的所有地图 |
| `/api/save_map` | POST | 保存当前地图 |
| `/api/start_mapping` | POST | → 重定向到 `run_task/mapping` |
| `/api/start_navigation` | POST | 启动导航 |
| `/api/stop_navigation` | POST | 停止导航 |

命令行测试：

```bash
curl http://192.168.31.147:3000/api/runtime_config | python3 -m json.tool
curl http://192.168.31.147:3000/api/list_maps
curl http://192.168.31.147:3000/api/task_status/startup
```

---

## 六、页面功能

| 功能 | 数据来源 | ROS 话题 |
|---|---|---|
| 摇杆遥控 | 后端 → ROS | `/cmd_vel` |
| 电量监控 | foxglove | `/battery_state` |
| 激光雷达 | foxglove | `/scan` |
| SLAM 地图 | foxglove | `/map`、`/map_metadata` |
| 里程计位姿 | foxglove | `/odom`、`/tf` |
| 相机画面 | HTTP 直连 | `http://192.168.31.39:81/` |
| 建图/导航按钮 | 后端任务 | 见 `ros_config.json` 的 `tasks` |

> **注意**：`/map` 依赖 `map → odom` 变换。如果没在跑 SLAM/Nav2，地图不会显示。

---

## 七、任务脚本说明

### `task_mapping.sh`（点"开始建图"触发）

1. 杀掉旧的 slam_toolbox **和 slam_gmapping**（避免抢 `map→odom`）
2. 启动 `slam_toolbox online_sync_launch.py`（`use_sim_time:=false`）
3. 重启 `robot_state_publisher` 重发 `/tf_static`
   （`/tf_static` 是 TRANSIENT_LOCAL 只发一次，晚加入的订阅者收不到）
4. 前台循环，保持"运行中"状态

**设计上故意不幂等**：每次触发都从空白重新建图。想保留先点"保存地图"。

### `task_navigation.sh`（点"开始导航"触发）

1. 杀掉已有 Nav2 实例（避免重复 costmap）
2. 启动 `nav2_bringup navigation_launch.py`，参数用 `config/nav2.yaml`

---

## 八、故障排查

```bash
# 三个端口是否在监听
ss -tlnp | grep -E ':(5173|3000|8765)'

# 进程
pgrep -af 'node server.js|vite|foxglove_bridge'

# 后端日志
tail -30 /tmp/console.log

# 全链路检查
curl -s -o /dev/null -w "%{http_code}\n" http://localhost:5173/
curl -s http://localhost:3000/api/task_status/startup
```

| 症状 | 原因 | 处理 |
|---|---|---|
| 页面打不开 | Vite 没起 | 看 `/tmp/console.log`，确认 `npm run dev` 在跑 |
| 页面能开但数据不动 | foxglove_bridge 没起 | `curl -X POST localhost:3000/api/run_task/startup` |
| 有数据但地图空白 | 没跑 SLAM/Nav2，无 `map→odom` | 点"开始建图" |
| 摇杆没反应 | 小车未连接 / domain 不对 | 查 `grep -c create_client /tmp/agent.log`；确认 `ROS_DOMAIN_ID=0` |
| 相机黑屏 | 相机 IP 变了 | 改 `ros_config.json` 的 `camera.stream_url` |
| 建图按钮报错 | `task_mapping.sh` 的 domain 或路径不对 | 检查 `ROS_DOMAIN_ID` 和 `$R` 路径 |

---

## 九、可选：开机自启

当前是手动启动（`nohup setsid`，SSH 断开不影响，但重启不自动恢复）。

需要自启可加 systemd 服务：

```bash
sudo tee /etc/systemd/system/robot-console.service > /dev/null <<'UNIT'
[Unit]
Description=Robot Web Console
After=network.target

[Service]
Type=simple
User=nvidia
WorkingDirectory=/home/nvidia/robot/Ros_Vue_arm
ExecStart=/bin/bash /home/nvidia/robot/Ros_Vue_arm/start_console.sh
Restart=on-failure
RestartSec=10

[Install]
WantedBy=multi-user.target
UNIT

sudo systemctl daemon-reload
sudo systemctl enable --now robot-console
```

---

## 十、重要：建图前必须有完整的里程计链

点"开始建图"后地图不更新，最常见的**不是 SLAM 的问题**，而是里程计 TF 链断了。

### 必需的四个环节

```
小车 /odom ──► gyro_odom_fixer ──► /odom_fixed ──► xuegecar_bringup ──► odom→base_footprint TF
                                                                              │
                                                    slam_toolbox ◄────────────┘
                                                          │
                                                          ▼
                                                   /map（地图）
```

| 环节 | 缺了会怎样 |
|---|---|
| `gyro_odom_fixer` | `/odom_fixed` 无发布者 → 底盘 bringup 收不到里程计 → **没有任何 TF** |
| `xuegecar_bringup_real` | `odom→base_footprint` 不存在 → slam_toolbox 拿不到位姿 → **地图冻结** |
| `robot_state_publisher` | `base_footprint→laser_frame` 不存在 → 扫描无法变换 → **丢帧** |
| `slam_toolbox` / `slam_gmapping` | 没有 `/map` |

### 诊断命令

```bash
export ROS_DOMAIN_ID=0
for t in /odom_fixed /scan /map; do
  printf "%-14s " $t; ros2 topic hz $t 2>&1 | grep -m1 'average rate'
done
# /odom_fixed 应该有值；如果没值 → gyro_odom_fixer 没跑

ros2 run tf2_ros tf2_echo odom base_footprint   # 应该能拿到
ros2 run tf2_ros tf2_echo map base_footprint    # 应该能拿到
```

> ⚠️ `tf2_echo` 启动时会有几秒的 `does not exist`，那是新节点的 DDS 发现延迟，**不代表 TF 真的缺失**。
> 判定时建议用 Python 脚本等待发现，或看 `/tf` 的实际内容：
> `ros2 topic echo /tf --once | grep child_frame_id`

### 本次修复

`task_mapping.sh` 原来只重启 `slam_toolbox` 和 `robot_state_publisher`，
**不会拉起 `gyro_odom_fixer` 和底盘 bringup**，所以只要 fixer 停过，点"开始建图"就是坏的。

已在其启动 `slam_toolbox` 之前加入两段检查：

```bash
# 确保 gyro_odom_fixer 在跑
if ! pgrep -f gyro_odom_fixer >/dev/null 2>&1; then
  setsid nohup python3 /home/nvidia/robot/gyro_odom_fixer.py \
    --ros-args -p odom_in:=/odom -p odom_out:=/odom_fixed -p gyro_yaw_sign:=1.0 \
    > $R/logs/fixer.log 2>&1 < /dev/null &
  sleep 4
fi

# 确保底盘 bringup 在跑（用 /odom_fixed）
if ! pgrep -f 'lib/xuegecar_bringup/xuegecar_bringup' >/dev/null 2>&1; then
  setsid nohup ros2 launch xuegecar_bringup xuegecar_bringup_real.launch.py \
    > $R/logs/bringup.log 2>&1 < /dev/null &
  sleep 6
fi
```

备份：`task_mapping.sh.bak_fixer`

### 验证方法

```bash
# 移动小车 0.3 m，看地图已知格子数是否增长
# 正常应该从几百增长到几千
```

实测：移动 0.3 m 后已知格子 929 → 5893（+4964），地图正常增长。
