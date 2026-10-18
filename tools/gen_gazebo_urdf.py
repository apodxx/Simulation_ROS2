#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gen_gazebo_urdf.py —— 由原始 leap_ros.urdf 生成可用于 Gazebo 仿真的版本

原始 URDF 实测：<inertial> 1 个、<collision> 1 个、<gazebo>/<plugin>/<sensor> 全为 0
→ 只有可视化，没有物理和传感器，Gazebo 无法仿真

本脚本：
  1. base_link   : STL 网格碰撞 → 简单长方体（物理稳定、性能好）
  2. 左右驱动轮   : 补 <inertial> + 圆柱 <collision>
  3. 万向轮       : 补球形 <collision>（半径按几何算，不能把驱动轮顶离地面）
  4. 追加 Gazebo 插件（保留注释）：
       libgazebo_ros_diff_drive.so → /odom + /cmd_vel
       libgazebo_ros_ray_sensor.so → /scan
       libgazebo_ros_imu_sensor.so → /imu
     publish_odom_tf=false，TF 交给工程自带的 xuegecar_bringup 发（与真机架构一致）
"""
import xml.etree.ElementTree as ET

SRC = '/home/demo/Desktop/leap_ros_ws/xuegecar_description/urdf/leap_ros.urdf'
DST = '/home/demo/Desktop/leap_ros_ws/xuegecar_description/urdf/leap_ros.gazebo.urdf'

# ---- 从 URDF 实测值推算 ----
BASE_JOINT_Z = 0.06205          # base_footprint -> base_link
WHEEL_JOINT_Z = -0.02955        # base_link -> wheel（负值，在车体下方）
WHEEL_RADIUS = 0.02955          # 轮心离地 ≈ 轮半径
WHEEL_WIDTH = 0.020
WHEEL_SEPARATION = 0.081 * 2    # 左右轮 y 间距 = 0.162
CASTER_JOINT_Z = -0.05115
# 万向轮半径：让球底刚好与轮底同高（略高一点点，避免把驱动轮顶起来）
CASTER_RADIUS = round((WHEEL_JOINT_Z + -WHEEL_RADIUS) * -1 - abs(CASTER_JOINT_Z) - 0.0004, 4)


def sub(parent, tag, **attrs):
    return ET.SubElement(parent, tag, {k: str(v) for k, v in attrs.items()})


def set_inertial(link, mass, ixx, iyy, izz, origin='0 0 0'):
    for old in link.findall('inertial'):
        link.remove(old)
    inr = ET.Element('inertial')
    sub(inr, 'origin', xyz=origin, rpy='0 0 0')
    sub(inr, 'mass', value=mass)
    sub(inr, 'inertia', ixx=ixx, ixy='0.0', ixz='0.0', iyy=iyy, iyz='0.0', izz=izz)
    link.insert(0, inr)


def set_collision(link, shape, size, origin='0 0 0', rpy='0 0 0'):
    for old in link.findall('collision'):
        link.remove(old)
    col = ET.Element('collision')
    sub(col, 'origin', xyz=origin, rpy=rpy)
    geo = sub(col, 'geometry')
    if shape == 'box':
        sub(geo, 'box', size=size)
    elif shape == 'cylinder':
        sub(geo, 'cylinder', radius=size[0], length=size[1])
    elif shape == 'sphere':
        sub(geo, 'sphere', radius=size)
    link.append(col)


def main():
    tree = ET.parse(SRC)
    root = tree.getroot()
    links = {l.get('name'): l for l in root.findall('link')}

    print('原始 link (%d): %s' % (len(links), ', '.join(links)))

    # 打印轮子关节的旋转轴（差速驱动必须绕 y 轴）
    for j in root.findall('joint'):
        if j.get('name') in ('left_wheel_joint', 'right_wheel_joint'):
            ax = j.find('axis')
            print('  %-18s type=%-11s axis=%s' % (
                j.get('name'), j.get('type'),
                ax.get('xyz') if ax is not None else '(无)'))

    print('\n--- 修改 ---')
    if 'base_link' in links:
        set_collision(links['base_link'], 'box', '0.20 0.17 0.06', origin='0.0 0 0.005')
        set_inertial(links['base_link'], 2.0, 0.01, 0.02, 0.02, origin='0 0 0.02')
        print('  base_link    : box 0.20x0.17x0.06 + 2.0kg')

    m = 0.05
    i_axis = 0.5 * m * WHEEL_RADIUS ** 2
    i_perp = (1.0 / 12.0) * m * (3 * WHEEL_RADIUS ** 2 + WHEEL_WIDTH ** 2)
    for name in ('left_wheel_link', 'right_wheel_link'):
        if name in links:
            # ⚠️ URDF 的 <cylinder> 默认轴向是 Z，车轮必须绕 Y 轴滚动，
            #    所以要绕 X 轴转 90° 把圆柱"躺倒"成轮子。忘了这一步，
            #    轮子会变成平躺的圆盘，驱动轮离地 19mm，车体前倾约 9.3°。
            set_collision(links[name], 'cylinder', (WHEEL_RADIUS, WHEEL_WIDTH),
                          rpy='1.5708 0 0')
            set_inertial(links[name], m, i_axis, i_perp, i_perp)
            print(f'  {name:13s}: cylinder r={WHEEL_RADIUS} l={WHEEL_WIDTH} (rpy=1.5708 0 0) + {m}kg')

    if 'caster_link' in links:
        set_collision(links['caster_link'], 'sphere', CASTER_RADIUS)
        set_inertial(links['caster_link'], 0.02, 1e-5, 1e-5, 1e-5)
        print(f'  caster_link  : sphere r={CASTER_RADIUS} + 0.02kg  '
              f'(轮底 z={WHEEL_JOINT_Z - WHEEL_RADIUS:.5f}, 球底 z={CASTER_JOINT_Z - CASTER_RADIUS:.5f})')

    # ---- 序列化（先不含插件），再在 </robot> 前插入带注释的插件 XML ----
    if hasattr(ET, 'indent'):
        ET.indent(tree, space='  ')
    body = ET.tostring(root, encoding='unicode')

    plugins = f'''
  <!-- ==================== 差速驱动 ==================== -->
  <!-- 发布 /odom、订阅 /cmd_vel。
       publish_odom_tf=false：odom->base_footprint 的 TF 交给 leap_ros_ws 自带的
       xuegecar_bringup 节点来发，与真机架构完全一致 -->
  <gazebo>
    <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
      <update_rate>50</update_rate>
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>{WHEEL_SEPARATION}</wheel_separation>
      <wheel_diameter>{WHEEL_RADIUS * 2}</wheel_diameter>
      <max_wheel_torque>2.0</max_wheel_torque>
      <max_wheel_acceleration>5.0</max_wheel_acceleration>
      <publish_odom>true</publish_odom>
      <publish_odom_tf>false</publish_odom_tf>
      <publish_wheel_tf>false</publish_wheel_tf>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_footprint</robot_base_frame>
    </plugin>
  </gazebo>

  <!-- ==================== 激光雷达 ==================== -->
  <!-- 与固件一致：360 点、0~359 度、1 度步进、0.02~12m、10Hz -->
  <!-- ⚠️ 关键：传感器挂到 base_link 上并显式写出安装偏移，而不是 reference="laser_frame"。
       原因：laser_frame 是 fixed joint，Gazebo 做固定关节合并时会丢掉这一层，
       导致传感器实际落在 base_link 原点 —— 表现为雷达一直在扫自己的车体
       （360 个点里只有 30 多个有限值，距离 0.10~0.15m，正好是底盘盒子的尺寸）。
       安装偏移来自 URDF: laser_joint = base_link + (-0.0227, 0.000062, 0.0637) -->
  <gazebo reference="base_link">
    <sensor type="ray" name="lidar">
      <pose>-0.0227 0.000062 0.0637 0 0 0</pose>
      <visualize>false</visualize>
      <update_rate>10</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>360</samples>
            <resolution>1</resolution>
            <min_angle>0.0</min_angle>
            <max_angle>6.26573</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.02</min>
          <max>12.0</max>
          <resolution>0.01</resolution>
        </range>
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>
      </ray>
      <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
        <ros>
          <remapping>~/out:=scan</remapping>
        </ros>
        <output_type>sensor_msgs/LaserScan</output_type>
        <frame_name>laser_frame</frame_name>
      </plugin>
    </sensor>
  </gazebo>

  <!-- ==================== IMU ==================== -->
  <!-- 同理：挂到 base_link，偏移取 URDF 的 imu_joint = (0, 0, 0.03) -->
  <gazebo reference="base_link">
    <sensor name="imu_sensor" type="imu">
      <pose>0 0 0.03 0 0 0</pose>
      <always_on>true</always_on>
      <update_rate>50</update_rate>
      <visualize>false</visualize>
      <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
        <ros>
          <remapping>~/out:=imu</remapping>
        </ros>
        <initial_orientation_as_reference>false</initial_orientation_as_reference>
        <frame_name>imu_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>

  <!-- ==================== 物理与外观 ==================== -->
  <gazebo reference="left_wheel_link">
    <mu1>1.0</mu1><mu2>1.0</mu2><kp>1e6</kp><kd>1.0</kd>
    <material>Gazebo/Black</material>
  </gazebo>
  <gazebo reference="right_wheel_link">
    <mu1>1.0</mu1><mu2>1.0</mu2><kp>1e6</kp><kd>1.0</kd>
    <material>Gazebo/Black</material>
  </gazebo>
  <gazebo reference="caster_link">
    <mu1>0.0</mu1><mu2>0.0</mu2>
    <material>Gazebo/Grey</material>
  </gazebo>
  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
  </gazebo>
  <gazebo reference="laser_frame">
    <material>Gazebo/Red</material>
  </gazebo>
</robot>
'''
    body = body.replace('</robot>', plugins.lstrip('\n'))
    with open(DST, 'w', encoding='utf-8') as f:
        f.write('<?xml version="1.0"?>\n')
        f.write(body)
    print(f'\n已生成: {DST}')

    # 校验
    ET.parse(DST)
    print('XML 解析校验: OK')


if __name__ == '__main__':
    main()
