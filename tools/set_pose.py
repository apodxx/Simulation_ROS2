#!/usr/bin/env python3
import rclpy, sys, math, time
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

x, y, yaw = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
rclpy.init()
n = Node('init_pose')
p = n.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
msg = PoseWithCovarianceStamped()
msg.header.frame_id = 'map'
msg.pose.pose.position.x = x
msg.pose.pose.position.y = y
msg.pose.pose.orientation.z = math.sin(yaw/2)
msg.pose.pose.orientation.w = math.cos(yaw/2)
cov = [0.0]*36
cov[0]=0.25; cov[7]=0.25; cov[35]=0.068
msg.pose.covariance = cov
for i in range(15):
    msg.header.stamp = n.get_clock().now().to_msg()
    p.publish(msg)
    time.sleep(0.2)
print(f'  已发布初始位姿 ({x:+.3f}, {y:+.3f}, {math.degrees(yaw):+.1f}°)')
rclpy.shutdown()
