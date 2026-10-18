#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gazebo_sim.launch.py —— 用 Gazebo 仿真整台 Leap 机器人

一次拉起：
  ① Gazebo (gzserver + gzclient)   物理引擎 + 图形界面
  ② robot_state_publisher          用仿真 URDF 发静态 TF + 机器人模型
  ③ joint_state_publisher          发 /joint_states（轮子关节角）
  ④ xuegecar_bringup               ★ leap_ros_ws 自带节点：订阅 /odom → 发 odom->base_footprint TF
  ⑤ spawn_entity.py                把机器人从 robot_description 生成到 Gazebo
  ⑥ rviz2                          可视化

Gazebo 插件产生的数据：
  libgazebo_ros_diff_drive  → /odom    + 订阅 /cmd_vel
  libgazebo_ros_ray_sensor  → /scan
  libgazebo_ros_imu_sensor  → /imu
  Gazebo 本体                → /clock   （所以所有节点必须 use_sim_time:=true）

用法：
  ros2 launch xuegecar_description gazebo_sim.launch.py
  ros2 launch xuegecar_description gazebo_sim.launch.py gui:=false rviz:=false
  ros2 launch xuegecar_description gazebo_sim.launch.py rviz:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('xuegecar_description')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    urdf_path = os.path.join(pkg_share, 'urdf', 'leap_ros.gazebo.urdf')
    sdf_path = os.path.join(pkg_share, 'urdf', 'leap_ros.gazebo.sdf')
    rviz_cfg = os.path.join(pkg_share, 'rviz', 'sim.rviz')
    world_path = os.path.join(pkg_share, 'worlds', 'leap_room.world')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gui = LaunchConfiguration('gui', default='true')
    rviz = LaunchConfiguration('rviz', default='true')

    # ⚠️ 必须用 ParameterValue(..., value_type=str) 包一层：
    #    否则 launch 会把 URDF 文本当 YAML 解析，报
    #    "Unable to parse the value of parameter robot_description as yaml"
    robot_description = {
        'robot_description': ParameterValue(Command(['cat ', urdf_path]), value_type=str)
    }

    # ---------------- ① Gazebo ----------------
    # 这里不用 gazebo_ros/gazebo.launch.py，因为它只接受 gui/server 两个参数、
    # 没法指定世界文件；改为直接包含 gzserver + gzclient 两个 launch。
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world_path, 'verbose': 'false'}.items(),
    )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(gui),
    )

    # ---------------- ② robot_state_publisher ----------------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )

    # ---------------- ③ joint_state_publisher ----------------
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )

    # ---------------- ④ leap_ros_ws 自带的 TF 桥 ----------------
    # 订阅 /odom（Gazebo 差速插件发），广播 odom -> base_footprint。
    # 与真机架构完全一致：插件只发 /odom，TF 由这个节点负责。
    xuegecar_bringup = Node(
        package='xuegecar_bringup',
        executable='xuegecar_bringup',
        name='xuegecar_bringup',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ---------------- ⑤ 生成机器人到 Gazebo ----------------
    # ⚠️ 这里生成的是 .sdf 而不是 .urdf，原因：
    #    URDF→SDF 转换会合并 fixed joint，并丢掉 <sensor><pose>，
    #    导致雷达被放进底盘内部（360 点全在扫自己）。
    #    leap_ros.gazebo.sdf 是转换后用 gen_gazebo_sdf.py 修正过传感器位姿的版本。
    #    另外用 -file 而不是 -topic，绕开 DDS 传 robot_description 的不稳定问题。
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=['-file', sdf_path,
                   '-entity', 'leap_ros',
                   '-x', '0.0', '-y', '0.0', '-z', '0.05',
                   '-timeout', '60.0'],
        parameters=[{'use_sim_time': use_sim_time}],
    )
    spawn_delayed = TimerAction(period=8.0, actions=[spawn_entity])

    # ---------------- ⑥ rviz2 ----------------
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_cfg],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz),
    )
    rviz_delayed = TimerAction(period=10.0, actions=[rviz_node])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='仿真里必须为 true（Gazebo 发 /clock）'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='是否启动 Gazebo 图形界面 gzclient'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='是否启动 rviz2'),
        gzserver,
        gzclient,
        robot_state_publisher,
        joint_state_publisher,
        xuegecar_bringup,
        spawn_delayed,
        rviz_delayed,
    ])
