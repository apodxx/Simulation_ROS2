#!/bin/bash
# ===== 启动 Nav2 静态地图导航 =====
# 前提：Gazebo + 底盘 bringup 已在运行（先用 start_sim_all.sh）
# 用法: ./start_nav2.sh [地图yaml路径]
set +u
WS=/home/demo/Desktop/leap_ros_ws
MAP=${1:-$WS/xuegecar_navigation2/maps/leap_room_v1.yaml}
Y=$WS/xuegecar_navigation2/param/xuegebot.yaml
LOG=/tmp/sim/nav2.log
mkdir -p /tmp/sim

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

echo "==> 0/4 先发零速（Gazebo 会锁存最后一条速度指令，不归零它会一直开）"
timeout 12 python3 -c "
import rclpy, time
from rclpy.node import Node
from geometry_msgs.msg import Twist
rclpy.init(); n=Node('stop_'); p=n.create_publisher(Twist,'/cmd_vel',10)
t0=time.time()
while time.time()-t0<1.5: p.publish(Twist()); time.sleep(0.05)
rclpy.shutdown()" 2>/dev/null || true

echo "==> 1/4 清场：停掉 gmapping 和所有旧 Nav2 实例"
pkill -f slam_gmapping 2>/dev/null || true
# 关键：必须杀干净所有旧 Nav2，否则多个实例会互相抢话题、
# 把对方心跳挤掉，触发 "CRITICAL FAILURE: SERVER xxx IS DOWN"
pkill -9 -f 'nav2_bringup'       2>/dev/null || true
pkill -9 -f 'nav2_container'     2>/dev/null || true
pkill -9 -f 'component_container' 2>/dev/null || true
pkill -9 -f 'lifecycle_manager'  2>/dev/null || true
sleep 4
if pgrep -f 'component_container' >/dev/null; then
  echo "    !! 仍有旧实例残留，请手动检查: pgrep -af component_container"
fi

echo "==> 2/4 启动 Nav2（地图: $MAP）"
: > "$LOG"
nohup setsid ros2 launch nav2_bringup bringup_launch.py \
  map:="$MAP" use_sim_time:=true params_file:="$Y" > "$LOG" 2>&1 &
for i in $(seq 1 90); do
  grep -q 'Managed nodes are active' "$LOG" 2>/dev/null && break
  sleep 1
done
if grep -q 'Managed nodes are active' "$LOG"; then
  echo "    所有节点激活成功"
else
  echo "    !! 激活超时，看日志 $LOG"
  tail -5 "$LOG"
fi

echo "==> 3/4 启动 RViz（导航视图）"
pkill -f 'rviz2 -d' 2>/dev/null || true
sleep 2
export DISPLAY="${DISPLAY:-:0}"
[ -n "${XAUTHORITY:-}" ] || XAUTHORITY=$(ls /run/user/1000/.mutter-Xwaylandauth.* 2>/dev/null | head -1)
export XAUTHORITY
RV=$WS/install/xuegecar_navigation2/share/xuegecar_navigation2/rviz/xuege_navigation2.rviz
[ -f "$RV" ] || RV=$WS/xuegecar_navigation2/rviz/xuege_navigation2.rviz
nohup setsid rviz2 -d "$RV" --ros-args -p use_sim_time:=true > /tmp/sim/rviz_nav.log 2>&1 &
sleep 8

echo "==> 4/4 完成"
cat <<'TIP'

 在 RViz 里操作：
   ① 点工具栏 "2D Pose Estimate"，在地图上点一下车的实际位置并拖动朝向
      （或用命令行：./set_initial_pose.sh <x> <y>）
   ② 点 "Nav2 Goal"，在地图上点目标点并拖动最终朝向
   ③ 车会自己规划路径并开过去，面板里能看 ETA / 剩余距离 / 恢复次数

 命令行发目标：./send_goal.sh <x> <y> [yaw度]
TIP
