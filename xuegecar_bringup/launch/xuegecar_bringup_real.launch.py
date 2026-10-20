"""真车底盘 bringup：使用陀螺修正后的 /odom_fixed 生成 odom->base_footprint TF

为什么必须这样：
  固件的 /odom yaw 是左右轮速差积分（odom_yaw_ += local_vw * dt），
  掉头/卡住时轮子打滑会虚报数倍（实测 4.7 倍），导致 SLAM 建出"两条走廊"。
  gyro_odom_fixer 用 IMU 陀螺角速度积分替换 yaw，实测与雷达真值差 1%。
"""
import os
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share = get_package_share_directory('xuegecar_description')
    urdf = os.path.join(share, 'urdf', 'leap_ros.urdf')
    robot_description = {'robot_description': Command(['cat ', urdf])}

    return LaunchDescription([
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             name='joint_state_publisher', parameters=[robot_description], output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[robot_description], output='screen'),
        Node(package='xuegecar_bringup', executable='xuegecar_bringup',
             name='xuegecar_bringup', output='screen',
             # ★ 关键：用陀螺修正后的里程计
             remappings=[('odom', '/odom_fixed')]),
    ])
