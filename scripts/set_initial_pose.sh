#!/bin/bash
# 给 AMCL 喂初始位姿：./set_initial_pose.sh <x> <y> [yaw度]
set +u
source /opt/ros/humble/setup.bash
source /home/demo/Desktop/leap_ros_ws/install/setup.bash
X=${1:-0}; Y=${2:-0}; YD=${3:-0}
timeout 30 python3 - "$X" "$Y" "$YD" <<'PY'
import rclpy, sys, math, time
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
x, y, yd = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
yaw = math.radians(yd)
rclpy.init(); n = Node('set_pose')
p = n.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
m = PoseWithCovarianceStamped(); m.header.frame_id = 'map'
m.pose.pose.position.x = x; m.pose.pose.position.y = y
m.pose.pose.orientation.z = math.sin(yaw/2); m.pose.pose.orientation.w = math.cos(yaw/2)
c = [0.0]*36; c[0] = 0.25; c[7] = 0.25; c[35] = 0.068
m.pose.covariance = c
for _ in range(15):
    m.header.stamp = n.get_clock().now().to_msg(); p.publish(m); time.sleep(0.2)
print(f'初始位姿已发布: ({x:+.2f}, {y:+.2f}, {yd:+.0f}°)')
rclpy.shutdown()
PY
