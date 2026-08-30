#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P1 受控退化场景生成器 (M5) — 实验期生成夹具，不驻留包内 worlds/

生成 4 个 controlled degeneracy worlds（默认输出到 <workspace>/data/worlds/，
不再写入 src/get_urdf/worlds/，避免把"实验夹具"混入正式地图资源）:
  - normal_room.world   : 结构丰富的正常房间 (健康基线)
  - corridor.world      : 长直走廊 (longitudinal translation 弱约束)
  - single_plane.world  : 单一巨大平面 (plane degeneration)
  - open_ground.world   : 仅地面 (只余 ground plane 弱约束)

用法:
  python3 generate_worlds.py --out-dir data/experiments/experiment_001
  python3 generate_worlds.py --out-dir /tmp/worlds --scenario corridor
  # 不传 --scenario 时生成全部 4 个

生成后自动自检 (generate_one 内执行):
  1) SDF/XML 合法性解析（xml.dom.minidom）
  2) 世界内必须存在带碰撞的 ground_plane 模型（防止"无地面 → 机器人无限坠落"）
"""

import os

OUT_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))))), "data", "worlds")

HEADER = """<sdf version='1.7'>
  <world name='default'>
    <light name='sun' type='directional'>
      <cast_shadows>1</cast_shadows>
      <pose>0 0 10 0 -0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range><constant>0.9</constant><linear>0.01</linear><quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type='adiabatic'/>
    <physics type='ode'>
      <max_step_size>0.002</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>500</real_time_update_rate>
    </physics>
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>false</shadows>
    </scene>
"""

FOOTER = """  </world>
</sdf>
"""


def ground():
    return """    <model name='ground_plane'>
      <static>1</static>
      <link name='link'>
        <collision name='collision'>
          <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
          <surface><friction><ode><mu>100</mu><mu2>50</mu2></ode></friction></surface>
        </collision>
        <visual name='visual'>
          <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Grey</name></script></material>
        </visual>
      </link>
    </model>
"""


def box(name, cx, cy, cz, sx, sy, sz, yaw=0.0, color='Gazebo/Grey'):
    """静态 box 模型: 中心 (cx,cy,cz), 尺寸 (sx,sy,sz)"""
    return f"""    <model name='{name}'>
      <static>1</static>
      <pose>{cx} {cy} {cz} 0 0 {yaw}</pose>
      <link name='link'>
        <collision name='collision'>
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          <surface><friction><ode><mu>100</mu><mu2>50</mu2></ode></friction></surface>
        </collision>
        <visual name='visual'>
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>{color}</name></script></material>
        </visual>
      </link>
    </model>
"""


def wall(name, cx, cy, length, height, thick=0.2, along_x=True, yaw=0.0):
    """沿 x 或 y 方向的墙"""
    if along_x:
        sx, sy, sz = length, thick, height
    else:
        sx, sy, sz = thick, length, height
    return box(name, cx, cy, height / 2.0, sx, sy, sz, yaw=yaw)


def room_with_walls(name, wx, wy, height=3.0, thick=0.2):
    """四壁房间"""
    s = []
    s.append(wall('wall_north', 0, wy / 2.0, wx, height, thick, along_x=True))
    s.append(wall('wall_south', 0, -wy / 2.0, wx, height, thick, along_x=True))
    s.append(wall('wall_west', -wx / 2.0, 0, wy, height, thick, along_x=False))
    s.append(wall('wall_east', wx / 2.0, 0, wy, height, thick, along_x=False))
    return "\n".join(s)


def normal_room():
    parts = [HEADER, ground(), room_with_walls('normal_room', 12.0, 12.0)]
    # 室内结构: 4 根柱子 + 若干家具块 (丰富几何约束)
    for i, (px, py) in enumerate([(-3, -3), (3, -3), (-3, 3), (3, 3)]):
        parts.append(box(f'column_{i}', px, py, 1.5, 0.5, 0.5, 3.0, color='Gazebo/DarkGrey'))
    parts.append(box('sofa', -2, 4.5, 0.4, 2.0, 0.8, 0.8))
    parts.append(box('table', 2, 4.5, 0.4, 1.2, 1.2, 0.8, color='Gazebo/Brown'))
    parts.append(box('cabinet', -4, 0, 1.0, 1.5, 0.5, 2.0))
    parts.append(box('desk', 4, 0, 0.4, 1.0, 0.6, 0.8, yaw=0.5))
    parts.append(FOOTER)
    return "".join(parts)


def corridor():
    """长直走廊: 宽 2.5m, 长 40m, 高 3m. longitudinal (x) 方向约束弱"""
    parts = [HEADER, ground()]
    parts.append(wall('corridor_wall_north', 0, 1.25, 40.0, 3.0, along_x=True))
    parts.append(wall('corridor_wall_south', 0, -1.25, 40.0, 3.0, along_x=True))
    # 端墙留门洞, 用两段短墙
    parts.append(box('end_west_1', -20.0, 0.7, 1.5, 0.2, 1.1, 3.0))
    parts.append(box('end_west_2', -20.0, -0.7, 1.5, 0.2, 1.1, 3.0))
    parts.append(box('end_east_1', 20.0, 0.7, 1.5, 0.2, 1.1, 3.0))
    parts.append(box('end_east_2', 20.0, -0.7, 1.5, 0.2, 1.1, 3.0))
    parts.append(FOOTER)
    return "".join(parts)


def single_plane():
    """单一巨大平面 (竖直墙): 只有一个方向法向量"""
    parts = [HEADER, ground()]
    parts.append(box('big_plane', 5.0, 0.0, 5.0, 0.3, 30.0, 10.0))
    parts.append(FOOTER)
    return "".join(parts)


def open_ground():
    """开阔地面: 只有 ground plane 约束"""
    parts = [HEADER, ground()]
    parts.append(FOOTER)
    return "".join(parts)


WORLD_FACTORIES = {
    "normal_room": normal_room,
    "corridor": corridor,
    "single_plane": single_plane,
    "open_ground": open_ground,
}


def _validate_world(path):
    """自检: XML 合法性 + 世界内必须存在带碰撞的 ground_plane（防无限坠落）。"""
    import xml.dom.minidom
    try:
        doc = xml.dom.minidom.parse(path)
    except Exception as e:
        return False, f"SDF/XML 解析失败: {e}"
    models = doc.getElementsByTagName("model")
    names = [m.getAttribute("name") for m in models]
    if "ground_plane" not in names:
        return False, "世界缺少 ground_plane 模型（无地面 → 机器人无限坠落）"
    for m in models:
        if m.getAttribute("name") == "ground_plane":
            if not m.getElementsByTagName("collision"):
                return False, "ground_plane 缺少 <collision>（无碰撞地面）"
            if not m.getElementsByTagName("plane"):
                return False, "ground_plane 碰撞缺少 <plane> 几何"
    return True, "含 ground_plane 碰撞"


def generate_one(scenario, out_path):
    """生成单个场景并自检；失败抛 RuntimeError。"""
    if scenario not in WORLD_FACTORIES:
        raise RuntimeError(f"未知场景: {scenario}, 可选: {list(WORLD_FACTORIES)}")
    out_path = os.path.abspath(out_path)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    content = WORLD_FACTORIES[scenario]()
    with open(out_path, "w") as f:
        f.write(content)
    ok, reason = _validate_world(out_path)
    if not ok:
        raise RuntimeError(f"[generate_worlds] 自检失败: {reason} -> {out_path}")
    print(f"[generate_worlds] {scenario} -> {out_path} ({len(content)} bytes, {reason})")
    return out_path


def main():
    import argparse
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out-dir", default=None,
                    help="输出目录（默认 <workspace>/data/worlds，不写入包内 worlds/）")
    ap.add_argument("--scenario", choices=list(WORLD_FACTORIES), default=None,
                    help="只生成指定场景；缺省生成全部 4 个")
    args = ap.parse_args()
    out_dir = args.out_dir or OUT_DIR
    os.makedirs(out_dir, exist_ok=True)
    scenarios = [args.scenario] if args.scenario else list(WORLD_FACTORIES)
    for s in scenarios:
        generate_one(s, os.path.join(out_dir, f"{s}.world"))


if __name__ == "__main__":
    main()
