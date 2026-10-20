#!/bin/bash
# ===== 真车启动（micro-ROS 底盘 + RViz）=====
# 用法: ./start_real.sh [--no-rviz]
set +u
WS=~/leap_ros_ws
AGENT_WS=~/micro_ros_ws
export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash
source $WS/install/setup.bash 2>/dev/null
source $AGENT_WS/install/setup.bash 2>/dev/null

RVIZ=1
[ "${1:-}" = "--no-rviz" ] && RVIZ=0

echo "==> 1/4 清场"
pkill -f micro_ros_agent 2>/dev/null || true
pkill -f 'xuegecar_bringup|robot_state_publisher|joint_state_publisher' 2>/dev/null || true
pkill -f 'slam_gmapping|nav2_|component_container' 2>/dev/null || true
sleep 2

echo "==> 2/4 启动 micro-ROS agent（UDP 8888）"
: > /tmp/agent.log
nohup setsid ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v4 > /tmp/agent.log 2>&1 &
sleep 5
if grep -q 'create_client' /tmp/agent.log 2>/dev/null; then
  echo "    小车已连接 ✅"
else
  echo "    等待小车连接（最多 20 秒）..."
  for i in $(seq 1 40); do
    grep -q 'create_client' /tmp/agent.log 2>/dev/null && { echo "    小车已连接 ✅"; break; }
    sleep 0.5
  done
  grep -q 'create_client' /tmp/agent.log 2>/dev/null || echo "    ⚠️ 小车未连接，检查小车配置页的 agent IP 是否为 $(hostname -I | awk '{print $1}')"
fi

echo "==> 3/4 启动底盘（robot_state_publisher + xuegecar_bringup）"
: > /tmp/bringup.log
nohup setsid ros2 launch xuegecar_bringup xuegecar_bringup.launch.py > /tmp/bringup.log 2>&1 &
sleep 8

echo "==> 4/4 状态自检"
echo -n "    /odom  : "; timeout 10 ros2 topic hz /odom 2>&1 | grep -m1 'average rate' || echo "无数据"
echo -n "    /scan  : "; timeout 10 ros2 topic hz /scan 2>&1 | grep -m1 'average rate' || echo "无数据"
echo -n "    电池   : "; timeout 8 ros2 topic echo /battery_state --once --field percentage 2>/dev/null || echo "无数据"

if [ "$RVIZ" = "1" ]; then
  echo "==> 启动 RViz"
  export DISPLAY="${DISPLAY:-:0}"
  pkill -f 'rviz2 -d' 2>/dev/null || true
  sleep 1
  RV=$WS/install/xuegecar_description/share/xuegecar_description/rviz/sim.rviz
  nohup setsid rviz2 -d "$RV" > /tmp/rviz_real.log 2>&1 &
  echo "    RViz 已启动（Fixed Frame 建议改成 odom 或 map）"
fi

echo
echo "完成。手动遥控： ros2 run teleop_twist_keyboard teleop_twist_keyboard"
