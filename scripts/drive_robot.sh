#!/bin/bash
# =====================================================================
#  drive_robot.sh —— 让 Gazebo 里的小车按预设动作跑起来
#
#  用法:
#     bash ~/Desktop/drive_robot.sh circle     # 画圆（默认）
#     bash ~/Desktop/drive_robot.sh square     # 走正方形
#     bash ~/Desktop/drive_robot.sh forward    # 直行 1 米
#     bash ~/Desktop/drive_robot.sh spin       # 原地转一圈
#     bash ~/Desktop/drive_robot.sh figure8    # 走 8 字
#     bash ~/Desktop/drive_robot.sh stop       # 停车
#     bash ~/Desktop/drive_robot.sh pose       # 只看当前位置
#
#  说明: 结束时一定会发零速，车不会一直跑。
# =====================================================================
source /opt/ros/humble/setup.bash 2>/dev/null
source /home/demo/Desktop/leap_ros_ws/install/setup.bash 2>/dev/null

MODE="${1:-circle}"

python3 - "$MODE" <<'PYEOF'
import sys, math, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

mode = sys.argv[1] if len(sys.argv) > 1 else 'circle'

class Driver(Node):
    def __init__(self):
        super().__init__('drive_robot')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom = None
        self.create_subscription(Odometry, '/odom', self.cb, 10)

    def cb(self, m):
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        yaw = math.atan2(2.0*(q.w*q.z), 1.0-2.0*q.z*q.z)
        self.odom = (p.x, p.y, yaw)

    def spin_for(self, t):
        end = time.time() + t
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def run(self, v, w, seconds):
        """以 (v, w) 跑 seconds 秒，10 Hz 持续发布"""
        t = Twist()
        t.linear.x = float(v)
        t.angular.z = float(w)
        end = time.time() + seconds
        while rclpy.ok() and time.time() < end:
            self.pub.publish(t)
            self.spin_for(0.1)

    def stop(self):
        t = Twist()          # 全零
        for _ in range(15):
            self.pub.publish(t)
            self.spin_for(0.1)

    def pose_str(self):
        if not self.odom:
            return '(还没收到 /odom)'
        x, y, yaw = self.odom
        # 累加展开：把 yaw 的跳变累加，避免显示成 ±180° 内的折叠值
        if not hasattr(self, '_prev_yaw'):
            self._prev_yaw = yaw
            self._yaw_acc = yaw
        else:
            d = yaw - self._prev_yaw
            while d > math.pi:  d -= 2*math.pi
            while d < -math.pi: d += 2*math.pi
            self._yaw_acc += d
            self._prev_yaw = yaw
        return f'x={x:+.3f} m, y={y:+.3f} m, 朝向={math.degrees(self._yaw_acc):+.1f}° (= {math.degrees(yaw):+.1f}°)'

rclpy.init()
n = Driver()
n.spin_for(1.5)          # 先收几帧 /odom
print(f'  起始位置: {n.pose_str()}')
t0 = time.time()

if mode == 'pose':
    pass
elif mode == 'stop':
    n.stop()
    print('  已发零速')
elif mode == 'forward':
    print('  直行 1 米 (0.25 m/s × 4s)...')
    n.run(0.25, 0.0, 4.0)
    n.stop()
elif mode == 'spin':
    print('  原地逆时针转一圈 (1.0 rad/s × 6.3s ≈ 360°)...')
    n.run(0.0, 1.0, 6.3)
    n.stop()
elif mode == 'circle':
    print('  画圆 (v=0.2 m/s, w=0.5 rad/s, 10s) ≈ 半径 0.4m 的整圆...')
    n.run(0.20, 0.5, 10.0)
    n.stop()
elif mode == 'square':
    side = 0.8          # 边长 0.8 m
    v, w = 0.25, 1.0
    t_side = side / v
    t_turn = (math.pi / 2) / w
    print(f'  走正方形：边长 {side} m，4 条边 + 4 个 90° 转弯...')
    for i in range(4):
        print(f'    第 {i+1} 边：直行 {t_side:.1f}s')
        n.run(v, 0.0, t_side)
        print(f'    第 {i+1} 个弯：左转 {t_turn:.1f}s')
        n.run(0.0, w, t_turn)
    n.stop()
elif mode == 'figure8':
    print('  走 8 字：左半圈 + 右半圈...')
    n.run(0.20, 0.6, 10.5)
    n.run(0.20, -0.6, 10.5)
    n.stop()
else:
    print(f'  未知模式: {mode}')

n.spin_for(1.0)
print(f'  结束位置: {n.pose_str()}')
print(f'  用时 {time.time()-t0:.1f}s，已停车')
n.destroy_node()
rclpy.shutdown()
PYEOF
