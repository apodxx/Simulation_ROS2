#!/usr/bin/env python3
"""按 odom 闭环走到一串路点，用来补全地图。"""
import rclpy, math, sys
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

WAYPOINTS = [(-1.80, -2.30), (0.80, -2.30), (2.20, -1.00)]

class Go(Node):
    def __init__(self):
        super().__init__('waypoint_goto')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.cb, 10)
        self.x = self.y = self.th = None

    def cb(self, m):
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        self.x, self.y = p.x, p.y
        self.th = math.atan2(2*(q.w*q.z), 1 - 2*q.z*q.z)

    def stop(self):
        self.pub.publish(Twist())

def main():
    rclpy.init()
    n = Go()
    while rclpy.ok() and n.x is None:
        rclpy.spin_once(n, timeout_sec=0.2)
    print(f'  起点 odom: ({n.x:+.2f}, {n.y:+.2f}) yaw={math.degrees(n.th):+.0f}°')

    for idx, (tx, ty) in enumerate(WAYPOINTS):
        print(f'  → 路点 {idx+1}/{len(WAYPOINTS)}: ({tx:+.2f}, {ty:+.2f})')
        last_move = n.get_clock().now()
        last_xy = (n.x, n.y, n.th)
        while rclpy.ok():
            rclpy.spin_once(n, timeout_sec=0.05)
            dx, dy = tx - n.x, ty - n.y
            dist = math.hypot(dx, dy)
            if dist < 0.12:
                break
            ang = math.atan2(dy, dx) - n.th
            while ang > math.pi:  ang -= 2*math.pi
            while ang < -math.pi: ang += 2*math.pi
            cmd = Twist()
            if abs(ang) > 0.5:
                cmd.angular.z = 0.7 if ang > 0 else -0.7
            else:
                cmd.linear.x = max(0.06, min(0.22, 0.7*dist))
                cmd.angular.z = max(-0.9, min(0.9, 2.0*ang))
            n.pub.publish(cmd)
            now = n.get_clock().now()
            if (now - last_move).nanoseconds/1e9 > 5.0:
                moved = math.hypot(n.x-last_xy[0], n.y-last_xy[1])
                turned = abs(n.th - last_xy[2])
                if moved < 0.05 and turned < 0.15:
                    print('    !! 卡住了，中止'); n.stop(); return
                last_move = now; last_xy = (n.x, n.y, n.th)
        print(f'    到达 ({n.x:+.2f}, {n.y:+.2f})')
    n.stop()
    for _ in range(10):
        n.pub.publish(Twist()); rclpy.spin_once(n, timeout_sec=0.05)
    print(f'  终点 odom: ({n.x:+.2f}, {n.y:+.2f})  已停车')
    rclpy.shutdown()

main()
