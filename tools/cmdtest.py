import subprocess, math, time, rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

def truth():
    o = subprocess.run(['gz','model','-m','leap_ros','-p'],capture_output=True,text=True,timeout=10).stdout.split()
    return float(o[0]), float(o[1]), float(o[5])

class N(Node):
    def __init__(s):
        super().__init__('cmdsample')
        s.lin = []; s.ang = []
        s.create_subscription(Twist, '/cmd_vel', s.cb, 10)
        s.ac = ActionClient(s, NavigateToPose, '/navigate_to_pose')
    def cb(s, m):
        s.lin.append(m.linear.x); s.ang.append(m.angular.z)

rclpy.init(); n = N()
for _ in range(40): rclpy.spin_once(n, timeout_sec=0.1)
b = truth()
print(f'  出发真值: ({b[0]:+.2f}, {b[1]:+.2f}) 朝向 {math.degrees(b[2]):+.0f}°')
if not n.ac.wait_for_server(timeout_sec=10):
    print('  ❌ 动作服务器没响应'); raise SystemExit
g = NavigateToPose.Goal()
g.pose.header.frame_id = 'map'
g.pose.pose.position.x = 0.0; g.pose.pose.position.y = -1.0
g.pose.pose.orientation.w = 1.0
n.ac.send_goal_async(g)
print('  已发目标 (0.0, -1.0)，采样 /cmd_vel 20 秒...')
t0 = time.time()
while rclpy.ok() and time.time()-t0 < 20:
    rclpy.spin_once(n, timeout_sec=0.05)
a = truth()
lin = n.lin; ang = n.ang
if lin:
    neg = sum(1 for v in lin if v < -0.005)
    pos = sum(1 for v in lin if v >  0.005)
    zero = len(lin)-neg-pos
    print(f'  采样 {len(lin)} 次:')
    print(f'    linear.x < 0 (倒车): {neg} 次  ({neg/len(lin)*100:.0f}%)')
    print(f'    linear.x > 0 (前进): {pos} 次  ({pos/len(lin)*100:.0f}%)')
    print(f'    linear.x ≈ 0 (转/停): {zero} 次 ({zero/len(lin)*100:.0f}%)')
    print(f'    linear.x 范围: {min(lin):+.3f} ~ {max(lin):+.3f}')
    print(f'    angular.z 范围: {min(ang):+.3f} ~ {max(ang):+.3f}')
else:
    print('  ❌ 没采到 /cmd_vel')
print(f'  结束真值: ({a[0]:+.2f}, {a[1]:+.2f}) 朝向 {math.degrees(a[2]):+.0f}°')
dx,dy = a[0]-b[0], a[1]-b[1]
fwd = dx*math.cos(b[2]) + dy*math.sin(b[2])
side = -dx*math.sin(b[2]) + dy*math.cos(b[2])
print(f'  实际位移: {math.hypot(dx,dy):.3f} m   沿车头分量 {fwd:+.3f}  侧向 {side:+.3f}')
print(f'  → ' + ('车是朝车头方向走的（正常）' if fwd > 0.05 else ('车是朝车尾方向走的（倒车！）' if fwd < -0.05 else '几乎没动')))
rclpy.shutdown()
