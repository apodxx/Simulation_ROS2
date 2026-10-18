#!/bin/bash
# =====================================================================
#  drive_keyboard.sh —— 用键盘操控 Gazebo 里的小车
#
#  在虚拟机桌面按 Ctrl+Alt+T 打开终端，然后：
#      bash ~/Desktop/drive_keyboard.sh
#
#  然后按键盘（不需要回车）：
#      i  前进        ,  后退
#      j  左转        l  右转
#      k  停车
#      u  o  m  .    斜向移动
#      q  z          整体加减速
#      Ctrl+C        退出
# =====================================================================
set +e

echo "==================================================="
echo "       键盘操控 Gazebo 小车"
echo "==================================================="

# ---------- 1. 加载环境 ----------
source /opt/ros/humble/setup.bash
if [ -f /home/demo/Desktop/leap_ros_ws/install/setup.bash ]; then
    source /home/demo/Desktop/leap_ros_ws/install/setup.bash
fi

# ---------- 2. 检查仿真是否在跑 ----------
echo ""
echo "[检查] 仿真状态..."
if ! pgrep -x gzserver > /dev/null; then
    echo "  ❌ Gazebo 没在运行！"
    echo ""
    echo "  请先启动仿真，在另一个终端里执行："
    echo "      bash ~/Desktop/start_sim_all.sh"
    echo ""
    read -r -p "  按回车退出..." _
    exit 1
fi
echo "  ✅ Gazebo 正在运行"

if ! command -v ros2 > /dev/null; then
    echo "  ❌ 找不到 ros2 命令，请先 source /opt/ros/humble/setup.bash"
    read -r -p "  按回车退出..." _
    exit 1
fi

# ---------- 3. 检查底盘数据是否在流 ----------
ODOM_HZ=$(timeout 6 ros2 topic hz /odom 2>&1 | grep -m1 'average rate' | grep -oP 'rate: \K[0-9.]+')
if [ -z "$ODOM_HZ" ]; then
    echo "  ⚠️  收不到 /odom 数据，仿真可能被暂停了"
    echo "      试试恢复： gz world -p 0"
    echo "      或者重启： bash ~/Desktop/start_sim_all.sh"
    echo ""
    read -r -p "  仍要继续吗？[y/N] " ans
    [ "$ans" != "y" ] && [ "$ans" != "Y" ] && exit 1
else
    echo "  ✅ 底盘数据正常 (/odom ${ODOM_HZ} Hz)"
fi

# ---------- 4. 检查驱动端是否在线 ----------
SUBS=$(timeout 8 ros2 topic info /cmd_vel 2>/dev/null | grep -oP 'Subscription count: \K\d+')
if [ "${SUBS:-0}" -eq 0 ]; then
    echo "  ⚠️  /cmd_vel 没有订阅者（车可能不会动），但先继续"
else
    echo "  ✅ 驱动端已连接 (/cmd_vel 订阅者 ${SUBS} 个)"
fi

# ---------- 5. 打印操作说明 ----------
cat <<'EOF'

===================================================
  现在按下面的键控制小车（不用按回车）
===================================================

         u    i    o          前 = i
         j    k    l          后 = ,
         m    ,    .          左 = j   右 = l

   i  前进          ,  后退
   j  左转          l  右转
   k  停车
   u o m .        斜向/边走边转
   q  加速          z  减速
   Ctrl+C         退出

  👉 切到 Gazebo 窗口就能看到车在动

===================================================

EOF
sleep 2

# ---------- 6. 启动键盘遥控 ----------
cleanup() {
    echo ""
    echo "[退出] 发送零速，确保小车停下..."
    timeout 3 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}" > /dev/null 2>&1
    echo "[退出] 完成"
}
trap cleanup EXIT INT TERM

ros2 run teleop_twist_keyboard teleop_twist_keyboard
