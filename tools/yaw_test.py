#!/usr/bin/env python3
import subprocess, math, time, sys
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped

class T(Node):
    def __init__(s):
        super().__init__('yaw_test')
        s.pose = None
        s.sub = s.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', s.cb, 10)
        s.pub_pose = s.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        s.pub_vel = s.create_publisher(Twist, '/cmd_vel', 10)
    def cb(s, m):
        p = m.pose.pose.position; q = m.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z), 1 - 2*q.z*q.z)
        s.pose = (p.x, p.y, yaw)

def truth():
    out = subprocess.run(['gz','model','-m','leap_ros','-p'], capture_output=True, text=True, timeout=10).stdout.split()
    return (float(out[0]), float(out[1]), float(out[5]))

def norm(a):
    while a > math.pi: a -= 2*math.pi
    while a < -math.pi: a += 2*math.pi
    return a

def setpose(n, x, y, yaw):
    m = PoseWithCovarianceStamped()
    m.header.frame_id = 'map'
    m.pose.pose.position.x = x; m.pose.pose.position.y = y
    m.pose.pose.orientation.z = math.sin(yaw/2); m.pose.pose.orientation.w = math.cos(yaw/2)
    c = [0.0]*36; c[0]=0.25; c[7]=0.25; c[35]=0.068
    m.pose.covariance = c
    for _ in range(15):
        m.header.stamp = n.get_clock().now().to_msg(); n.pub_pose.publish(m); time.sleep(0.15)

def report(n, tag):
    n.pose = None
    for _ in range(60):
        rclpy.spin_once(n, timeout_sec=0.1)
        if n.pose is not None: break
    tx, ty, tyaw = truth()
    if n.pose is None:
        print(f'  {tag:26s} 无 AMCL 数据'); return
    ax, ay, ayaw = n.pose
    de = math.hypot(ax-tx, ay-ty)
    dye = math.degrees(norm(ayaw-tyaw))
    print(f'  {tag:26s} 位置误差 {de:6.3f} m   方向误差 {dye:+7.1f}°')

def drive(n, lin, ang, dur):
    t = Twist(); t.linear.x = lin; t.angular.z = ang
    t0 = time.time()
    while time.time()-t0 < dur:
        n.pub_vel.publish(t); time.sleep(0.05)
    for _ in range(12): n.pub_vel.publish(Twist()); time.sleep(0.05)

rclpy.init()
n = T()
# AMCL 在收到初始位姿前不会发布 /amcl_pose，先播种
_tx, _ty, _tyaw = truth()
setpose(n, _tx, _ty, _tyaw)
time.sleep(6)
for _ in range(200): rclpy.spin_once(n, timeout_sec=0.1)
if n.pose is None:
    print('  ❌ 收不到 /amcl_pose'); sys.exit(1)
time.sleep(3)

print('=' * 78)
print('  实验一：位置给准，方向故意错 90°')
print('=' * 78)
report(n, '① 基线（正常）')
tx, ty, tyaw = truth()
setpose(n, tx, ty, tyaw + math.radians(90))
time.sleep(7)
report(n, '② 刚给完 +90° 方向')
drive(n, 0.15, 0.0, 2.5)
time.sleep(4)
report(n, '③ 前进一段后')
drive(n, 0.0, 1.0, 2.0)
time.sleep(4)
report(n, '④ 再原地转 2 秒后')
drive(n, 0.12, 0.5, 3.0)
time.sleep(4)
report(n, '⑤ 再走+转 3 秒后')

print()
print('=' * 78)
print('  实验二：位置给准，方向错 30°')
print('=' * 78)
tx, ty, tyaw = truth()
setpose(n, tx, ty, tyaw + math.radians(30))
time.sleep(7)
report(n, '⑥ 刚给完 +30° 方向')
drive(n, 0.15, 0.3, 3.0)
time.sleep(4)
report(n, '⑦ 走一段后')

print()
print('=' * 78)
print('  恢复正确位姿')
print('=' * 78)
tx, ty, tyaw = truth()
setpose(n, tx, ty, tyaw)
time.sleep(7)
report(n, '⑧ 复位后')
rclpy.shutdown()
