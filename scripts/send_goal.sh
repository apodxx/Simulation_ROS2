#!/bin/bash
# 命令行发导航目标：./send_goal.sh <x> <y> [yaw度]
set +u
source /opt/ros/humble/setup.bash
source /home/demo/Desktop/leap_ros_ws/install/setup.bash
X=${1:-1}; Y=${2:-1}; YD=${3:-0}
Q=$(python3 -c "import math;y=math.radians($YD);print(f'{math.sin(y/2):.4f} {math.cos(y/2):.4f}')")
ZQ=$(echo $Q | cut -d' ' -f1); WQ=$(echo $Q | cut -d' ' -f2)
echo "目标: ($X, $Y) 朝向 ${YD}°"
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: $X, y: $Y, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: $ZQ, w: $WQ}}}}" \
  --feedback | grep -E 'status|SUCCEEDED|ABORTED|distance_remaining' | tail -5
