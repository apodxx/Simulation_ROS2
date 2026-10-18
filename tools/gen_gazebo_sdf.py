#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gen_gazebo_sdf.py —— 把 URDF 转成 Gazebo 原生 SDF，并修正传感器安装位置

为什么要多这一步：
  URDF → SDF 转换时，Gazebo 会把 fixed joint 的子连杆「合并」进父连杆。
  base_footprint / base_link / laser_frame / imu_link 全被合并成一个 SDF link
  （名字叫 base_footprint），而 <gazebo reference="xxx"><sensor><pose> 里的
  pose 会被丢掉，传感器最终落在合并后连杆的原点 —— 结果是：
    · 雷达被塞进了底盘盒子内部，360 个点全在扫自己（距离 0.06~0.14m）
    · IMU 位置也不对

  验证方法（重要技巧）：
      gz sdf -p leap_ros.gazebo.urdf | less
  直接看 Gazebo 到底把模型解析成了什么样。

解决办法：用 gz sdf 转换出 SDF，然后把传感器 <pose> 显式改写成
  相对合并后 base_footprint 的真实安装位置，再让 Gazebo 直接加载 SDF。
"""
import subprocess
import xml.etree.ElementTree as ET

URDF = '/home/demo/Desktop/leap_ros_ws/xuegecar_description/urdf/leap_ros.gazebo.urdf'
SDF = '/home/demo/Desktop/leap_ros_ws/xuegecar_description/urdf/leap_ros.gazebo.sdf'

# 相对「合并后的 base_footprint」的安装位置 = base_joint 偏移 + 各自关节偏移
BASE_LINK_Z = 0.06205
LASER_POSE = f'-0.0227 0.000062 {BASE_LINK_Z + 0.0637:.5f} 0 0 0'   # z = 0.12575
IMU_POSE = f'0 0 {BASE_LINK_Z + 0.03:.5f} 0 0 0'                      # z = 0.09205


def main():
    print('=== 1. gz sdf 转换 URDF -> SDF ===')
    out = subprocess.run(['gz', 'sdf', '-p', URDF],
                         capture_output=True, text=True)
    if out.returncode != 0:
        raise SystemExit('gz sdf 失败: ' + out.stderr)
    sdf_text = out.stdout
    print(f'  转换完成，{len(sdf_text)} 字符')

    print('=== 2. 修正传感器 <pose> ===')
    root = ET.fromstring(sdf_text)
    fixed = []
    for link in root.iter('link'):
        for sensor in link.findall('sensor'):
            name = sensor.get('name')
            pose = sensor.find('pose')
            if pose is None:
                pose = ET.SubElement(sensor, 'pose')
            if name == 'lidar':
                print(f'  lidar : {pose.text!r}  ->  {LASER_POSE!r}')
                pose.text = LASER_POSE
                fixed.append(name)
            elif name == 'imu_sensor':
                print(f'  imu   : {pose.text!r}  ->  {IMU_POSE!r}')
                pose.text = IMU_POSE
                fixed.append(name)
    if not fixed:
        raise SystemExit('❌ 没找到任何传感器，说明 URDF 里的插件没被正确解析')

    print('=== 3. 写出 SDF ===')
    ET.indent(root, space='  ')
    xml = ET.tostring(root, encoding='unicode')
    with open(SDF, 'w', encoding='utf-8') as f:
        f.write('<?xml version="1.0" ?>\n')
        f.write(xml)
    print(f'  已生成: {SDF}')

    print('=== 4. 复核 ===')
    tree2 = ET.parse(SDF)
    for link in tree2.getroot().iter('link'):
        for sensor in link.findall('sensor'):
            p = sensor.find('pose')
            print(f"  link={link.get('name'):16s} sensor={sensor.get('name'):12s} pose={p.text if p is not None else '(无)'}")


if __name__ == '__main__':
    main()
