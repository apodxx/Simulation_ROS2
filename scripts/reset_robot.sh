#!/bin/bash
# =====================================================================
#  reset_robot.sh —— 把小车"传送"回原点（不用重启整个仿真）
#
#  用法:
#     bash ~/Desktop/reset_robot.sh          # 复位到原点
#     bash ~/Desktop/reset_robot.sh where    # 只看车在哪、离墙多远
#
#  原理: 删掉 Gazebo 里的模型，再按原始 SDF 重新生成一份。
#        这样连差速插件的里程计也会归零，比重启仿真快得多（约 10 秒）。
# =====================================================================
source /opt/ros/humble/setup.bash 2>/dev/null
source /home/demo/Desktop/leap_ros_ws/install/setup.bash 2>/dev/null

SDF=/home/demo/Desktop/leap_ros_ws/install/xuegecar_description/share/xuegecar_description/urdf/leap_ros.gazebo.sdf
MODE="${1:-reset}"

show_where() {
    local pose
    pose=$(timeout 8 gz model -m leap_ros -p 2>/dev/null | head -1)
    if [ -z "$pose" ]; then
        echo "  ❌ 读不到车的位置（Gazebo 没运行？或模型被删了）"
        return 1
    fi
    read -r x y z roll pitch yaw <<< "$pose"
    echo "  当前位置: x=${x}  y=${y}"
    echo "  朝向    : $(awk -v a="$yaw" 'BEGIN{printf "%.1f°", a*180/3.14159265}')"
    # 房间 x∈[-4,4] y∈[-3,3]，车体半径约 0.12m
    awk -v x="$x" -v y="$y" 'BEGIN{
        dxl = x - (-4.0); dxr = 4.0 - x; dyb = y - (-3.0); dyt = 3.0 - y;
        printf "  离墙距离: 西 %.2fm  东 %.2fm  南 %.2fm  北 %.2fm\n", dxl, dxr, dyb, dyt;
        min = dxl; side = "西";
        if (dxr < min) { min = dxr; side = "东" }
        if (dyb < min) { min = dyb; side = "南" }
        if (dyt < min) { min = dyt; side = "南" }
        if (dyt < min) { min = dyt; side = "北" }
        if (min < 0.25) printf "  ⚠️  离%s墙只有 %.2fm，快撞上了！\n", side, min;
        else printf "  ✅ 还有 %.2fm 的活动余量\n", min;
    }'
}

case "$MODE" in
    where)
        echo "=== 小车位置 ==="
        show_where
        ;;
    reset)
        echo "=== 1. 复位前位置 ==="
        show_where
        echo
        echo "=== 2. 删除旧模型 ==="
        timeout 15 ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "{name: leap_ros}" 2>&1 | tail -2
        sleep 2

        echo "=== 3. 重新生成到原点 ==="
        timeout 40 ros2 run gazebo_ros spawn_entity.py \
            -file "$SDF" -entity leap_ros -x 0 -y 0 -z 0.05 -timeout 30 2>&1 | tail -3
        sleep 3

        echo
        echo "=== 4. 复位后位置 ==="
        show_where

        echo
        echo "=== 5. 发一条零速，确保不动 ==="
        timeout 5 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}" > /dev/null 2>&1
        echo "  完成 ✅ 车已回到原点，可以直接继续操控"
        ;;
    *)
        sed -n '2,12p' "$0"
        ;;
esac
