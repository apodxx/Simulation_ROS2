#!/bin/bash
# ============================================================
# restart_slam.sh —— 一键重启建图
#
# 用法:
#   ./restart_slam.sh              复位车到原点 + 重启 gmapping + 重启 RViz
#   ./restart_slam.sh --keep-pose  不复位车，接着当前位置继续建图
#   ./restart_slam.sh --no-rviz    不重启 RViz
#
# 为什么必须重启 RViz：
#   gmapping 重启后会创建一个全新的 /map 发布者，RViz 有时无法
#   重新匹配到新发布者，就会一直显示上一轮留下的旧地图 —— 表现为
#   "车明明在空地，却画在灰色/墙外""地图不实时更新"。
# ============================================================
set -u
WS=/home/demo/Desktop/leap_ros_ws
LOG=/tmp/sim/gmapping.log
RLOG=/tmp/sim/rviz.log
mkdir -p /tmp/sim

KEEP_POSE=0
RVIZ=1
for a in "$@"; do
  case "$a" in
    --keep-pose) KEEP_POSE=1 ;;
    --no-rviz)   RVIZ=0 ;;
    -h|--help)   sed -n '2,14p' "$0"; exit 0 ;;
    *) echo "未知参数: $a"; exit 1 ;;
  esac
done

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

echo "==> 1/5 停车"
timeout 10 "$HOME/Desktop/drive_robot.sh" stop >/dev/null 2>&1 || true

echo "==> 2/5 停掉旧 gmapping"
pkill -f slam_gmapping 2>/dev/null || true
for i in $(seq 1 10); do pgrep -f 'slam_gmapping/lib' >/dev/null || break; sleep 0.5; done
if pgrep -f 'slam_gmapping/lib' >/dev/null; then
  echo "    强制结束"; pkill -9 -f slam_gmapping 2>/dev/null || true; sleep 1
fi

if [ "$KEEP_POSE" = "0" ]; then
  echo "==> 3/5 复位车到原点"
  "$HOME/Desktop/reset_robot.sh" reset 2>&1 | grep -E '当前位置|离墙' | sed 's/^/    /'
else
  echo "==> 3/5 保持当前位置（--keep-pose）"
fi

echo "==> 4/5 启动 gmapping"
: > "$LOG"
cd "$WS" || exit 1
nohup setsid ros2 launch slam_gmapping slam_gmapping.launch.py > "$LOG" 2>&1 &
for i in $(seq 1 60); do grep -q 'Initialization complete' "$LOG" 2>/dev/null && break; sleep 0.5; done
if grep -q 'Loaded params' "$LOG"; then
  grep -E 'Loaded params|Range limits in use' "$LOG" | sed 's/^/    /'
else
  echo "    !! 参数行没出现，检查 $LOG"
  tail -5 "$LOG" | sed 's/^/    /'
fi

if [ "$RVIZ" = "1" ]; then
  echo "==> 5/5 重启 RViz（关键！否则会显示上一轮的旧地图）"
  pkill -f 'rviz2 -d' 2>/dev/null || true
  sleep 2
  export DISPLAY="${DISPLAY:-:0}"
  if [ -z "${XAUTHORITY:-}" ]; then
    XAUTHORITY=$(ls /run/user/1000/.mutter-Xwaylandauth.* 2>/dev/null | head -1)
    export XAUTHORITY
  fi
  nohup setsid rviz2 -d "$WS/install/xuegecar_description/share/xuegecar_description/rviz/sim.rviz" > "$RLOG" 2>&1 &
  sleep 8
else
  echo "==> 5/5 跳过 RViz（--no-rviz）"
fi

echo
echo "================ 自检 ================"
echo -n "  gmapping 进程数     : "; pgrep -cf 'slam_gmapping/lib'
echo -n "  rviz2 进程数        : "; pgrep -cf 'rviz2 -d'
echo -n "  /map 发布频率       : "
timeout 15 ros2 topic hz /map 2>&1 | grep -m1 'average rate' || echo "还没数据（再等几秒）"
echo -n "  最近处理帧数        : "; grep -c 'update frame' "$LOG" 2>/dev/null
echo "  日志: $LOG    $RLOG"
echo "======================================"
