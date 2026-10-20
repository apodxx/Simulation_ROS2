#!/bin/bash
# ===== 停止真车所有节点（含发零速，防止车继续跑）=====
set +u
export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash
source ~/leap_ros_ws/install/setup.bash 2>/dev/null

echo "==> 1. 发零速（防止底盘继续执行上一条指令）"
timeout 10 python3 -c "
import rclpy, time
from rclpy.node import Node
from geometry_msgs.msg import Twist
rclpy.init(); n=Node('stop_'); p=n.create_publisher(Twist,'/cmd_vel',10)
t0=time.time()
while time.time()-t0<1.5: p.publish(Twist()); time.sleep(0.05)
rclpy.shutdown()" 2>/dev/null || true

echo "==> 2. 停止所有节点"
pkill -f 'slam_gmapping' 2>/dev/null || true
pkill -f 'nav2_|component_container|lifecycle_manager' 2>/dev/null || true
pkill -f 'rviz2 -d' 2>/dev/null || true
pkill -f 'xuegecar_bringup|robot_state_publisher|joint_state_publisher' 2>/dev/null || true
pkill -f 'micro_ros_agent' 2>/dev/null || true
sleep 2
echo "  剩余相关进程:"
pgrep -af 'micro_ros_agent|bringup|rviz2|nav2|gmapping' | cut -c1-90 | sed 's/^/    /' || echo "    (无)"
echo "完成"
