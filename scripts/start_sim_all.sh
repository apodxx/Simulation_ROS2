#!/bin/bash
# =====================================================================
#  start_sim_all.sh —— 一键启动「Gazebo 仿真 + RViz + Web 中控台」
#
#  用法：
#     bash ~/Desktop/start_sim_all.sh              # 全部启动（带 GUI）
#     bash ~/Desktop/start_sim_all.sh nogui        # 不启动 Gazebo 界面
#     bash ~/Desktop/start_sim_all.sh stop         # 全部停止
#     bash ~/Desktop/start_sim_all.sh status       # 查看状态
#
#  启动内容：
#     ① Gazebo 仿真（leap_room 场景，含墙壁和障碍物）
#     ② robot_state_publisher + joint_state_publisher
#     ③ xuegecar_bringup  ← leap_ros_ws 自带节点，负责 odom -> base_footprint 的 TF
#     ④ rviz2（显示机器人模型 + 激光雷达点云）
#     ⑤ Ros_Vue_x86 中控台（后端 3000 / 前端 5173 / foxglove 8765）
# =====================================================================
export DISPLAY=:0
export XAUTHORITY=$(ls /run/user/1000/.mutter-Xwaylandauth.* 2>/dev/null | head -1)
export DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/1000/bus
# export LIBGL_ALWAYS_SOFTWARE=1   # 已注释：这台 VM 支持 SVGA3D 硬件加速，强制软件渲染会慢 5~10 倍
export ROBOT_WORKSPACE=/home/demo/Desktop/leap_ros_ws

WS=/home/demo/Desktop/leap_ros_ws
VUE=/home/demo/Desktop/Ros_Vue_x86
LOGDIR=/tmp/sim
mkdir -p "$LOGDIR"

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

stop_all() {
    echo "[stop] 停止中控台..."
    curl -s -X POST http://localhost:3000/api/stop_task/startup >/dev/null 2>&1
    pkill -9 -f 'node server.js'       2>/dev/null
    pkill -9 -f 'vite --host'          2>/dev/null
    pkill -9 -f 'foxglove_bridge'      2>/dev/null
    pkill -9 -f 'start_console.sh'     2>/dev/null
    echo "[stop] 停止仿真..."
    pkill -9 -f 'gazebo_sim.launch.py' 2>/dev/null
    pkill -9 -f 'gzserver|gzclient'    2>/dev/null
    pkill -9 -f 'spawn_entity|robot_state_publisher|joint_state_publisher' 2>/dev/null
    pkill -9 -f 'lib/xuegecar_bringup' 2>/dev/null
    pkill -9 -f 'rviz2'                2>/dev/null
    sleep 5
    # ⚠️ 必须在所有进程停掉之后才清理，否则会破坏正在运行的 Fast DDS 传输
    rm -rf /tmp/.gazebo ~/.gazebo/lock 2>/dev/null
    rm -f /dev/shm/fastrtps_* 2>/dev/null
    echo "[stop] 完成"
}

status() {
    echo "==================== 状态 ===================="
    for p in gzserver gzclient robot_state_publisher xuegecar_bringup joint_state_publisher rviz2 'node server.js' 'vite --host' foxglove_bridge; do
        printf '  %-24s %s\n' "$p" "$([ "$(pgrep -cf "$p")" -gt 0 ] && echo '✅ 运行中' || echo '— 未运行')"
    done
    echo "  --- 端口 ---"
    ss -lntp 2>/dev/null | grep -E ':3000|:5173|:8765' | sed 's/^/  /'
    echo "  --- 话题 ---"
    for t in /clock /scan /odom /imu; do
        printf '  %-8s ' "$t"; timeout 6 ros2 topic hz $t 2>&1 | grep -m1 'average rate' || echo "(无数据)"
    done
    echo "  ★ 中控台: http://$(hostname -I | awk '{print $1}'):5173"
}

case "${1:-start}" in
    stop)   stop_all ;;
    status) status ;;
    *)
        GUI=true; [ "${1:-}" = "nogui" ] && GUI=false
        stop_all
        echo "[1/2] 启动 Gazebo 仿真（含 leap_ros_ws 的 bringup 节点和 RViz）..."
        setsid nohup ros2 launch xuegecar_description gazebo_sim.launch.py \
            > "$LOGDIR/gazebo_sim.log" 2>&1 < /dev/null &
        echo "      等待 60 秒让 Gazebo 和 RViz 起来..."
        sleep 60
        grep -E 'Successfully spawned' "$LOGDIR/gazebo_sim.log" | tail -1

        echo "[2/2] 启动 Ros_Vue_x86 中控台..."
        cd "$VUE"
        setsid nohup bash start_console.sh > "$LOGDIR/console.log" 2>&1 < /dev/null &
        echo "      等待 35 秒..."
        sleep 35
        status
        ;;
esac
