#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""lio_evaluation 实验编排 (M4/M5/M6)

子命令:
  prepare [--world W | --scenario S] [--algo baseline|degen] [--x X --y Y --z Z --yaw YAW]
      创建 experiment_XXX/ 目录 + config.yaml + fast_lio 配置快照
      --world      包内确认地图 (test_world / normal_indoor / bookstore / hospital)
      --scenario   受控退化场景 (normal_room / corridor / single_plane / open_ground)，实验期生成到实验目录
  run-online EXP_DIR [--duration S] [--world W] [--x X --y Y --z Z --yaw YAW]
      启动仿真栈 + fast_lio + lio_evaluation 采集, 到时自动停止
      (启动后自动做健康门禁: 世界真正加载 / 机器人落在地面 / 雷达有点云)
      运动驱动三选一: 默认脚本漫游+避障 / --teleop 手动遥控(WASD) / --no-drive 静止
  replay EXP_DIR --bag BAG [--duration S]
      回放 rosbag (同一 bag 对比 baseline / degen)
  eval EXP_DIR
      离线计算 ATE/RPE/runtime/退化统计 -> result.yaml
  all --world W | --scenario S [--algo X] [--duration S] [--bag BAG]
      prepare + run-online(或 replay) + eval 一键闭环
"""

import argparse
import os
import re
import signal
import subprocess
import sys
import time

import yaml

WS_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))
FASTLIO_CONFIG = os.path.join(WS_ROOT, "src/localization/FAST_LIO/config/mid360.yaml")
EXP_BASE = os.path.join(WS_ROOT, "data/experiments")

# ---- 地图白名单 / 受控场景 / 推荐出生位姿 ----
# 包内确认地图（P0/P0.1 已验证加载与模型），get_urdf 的 worlds/ 只保留这 5 个。
VALID_WORLDS = [
    "test_world",
    "normal_indoor",
    "bookstore",
    "hospital",
]

# 受控退化场景（实验期由 generate_worlds.py 生成到实验目录，不驻留包内 worlds/）
SCENARIOS = {
    "normal_room": "结构丰富的正常房间（健康基线）",
    "corridor": "长直走廊（longitudinal translation 弱约束）",
    "single_plane": "单一巨大平面（plane degeneration）",
    "open_ground": "仅地面（只余 ground plane 弱约束）",
}

# 包内确认地图的推荐出生位姿（未列出的用 get_urdf_launch 默认 x=0 y=0 z=0.30 yaw=0）
# normal_indoor 默认 (0,0) 会卡进家具，实测 x=5.0 可正常落地（y/z 沿用原推荐）；
# hospital 默认 (0,0) 落在夹缝里，实测 +1.0 并抬高 z 后更稳；
# bookstore 出生点来自 P0.1（原 turtlebot3 位）。
RECOMMENDED_SPAWN = {
    "normal_indoor": {"x": 5.0, "y": -1.5, "z": 0.30, "yaw": 1.57},
    "bookstore": {"x": -2.3, "y": 6.2, "z": 0.30, "yaw": -1.72},
    "hospital": {"x": 1.0, "y": 1.0, "z": 0.50, "yaw": 0.0},
}

GET_URDF_WORLDS_INSTALL = os.path.join(
    WS_ROOT, "install", "get_urdf", "share", "get_urdf", "worlds")

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DRIVE_NODE = os.path.join(SCRIPT_DIR, "drive_node.py")
HEALTH_WATCH = os.path.join(SCRIPT_DIR, "health_watch.py")
FAULT_INJECTOR = os.path.join(SCRIPT_DIR, "lidar_fault_injector.py")
ATTRIBUTION_VALIDATION = os.path.join(SCRIPT_DIR, "attribution_validation.py")


def next_experiment_dir(base=EXP_BASE):
    os.makedirs(base, exist_ok=True)
    nums = []
    for d in os.listdir(base):
        m = re.fullmatch(r"experiment_(\d+)", d)
        if m:
            nums.append(int(m.group(1)))
    n = (max(nums) + 1) if nums else 1
    return os.path.join(base, f"experiment_{n:03d}")


def git_commit():
    try:
        out = subprocess.check_output(
            ["git", "-C", WS_ROOT, "rev-parse", "HEAD"], stderr=subprocess.DEVNULL)
        return out.decode().strip()
    except Exception:
        return "unknown"


def write_fastlio_config(exp_dir, enable_degen):
    """从 fast_lio 默认配置生成实验用配置 (按 --algo 开关退化检测 + health)。

    - --algo degen: 同时打开 degeneracy.enable 和 health.enable
      (P2 health 聚合在 degeneracy 门控内, 二者必须同开才有 /lio/health)。
    - --algo baseline: 两者均为 false (零侵入)。
    """
    dst = os.path.join(exp_dir, "mid360.yaml")
    with open(FASTLIO_CONFIG) as f:
        lines = f.readlines()
    out = []
    section = None  # 当前处于 ros__parameters 下哪个段 (8 空格缩进键)
    for line in lines:
        m_sect = re.match(r"^\s{8}([a-zA-Z_]+):\s*(#.*)?$", line)
        if m_sect:
            section = m_sect.group(1)
        elif not line.startswith(" ") and line.strip():
            section = None  # 回到顶层 (/**:)
        if re.match(r"^\s*enable:\s*(true|false)\s*(#.*)?$", line) and \
                section in ("degeneracy", "health"):
            out.append(f"            enable: {str(enable_degen).lower()}"
                       f"                # 由 run_experiment 生成\n")
        else:
            out.append(line)
    with open(dst, "w") as f:
        f.writelines(out)
    return dst


def write_fault_lidar_config(exp_dir):
    """复制实验配置并把 common.lid_topic 指向 /livox/lidar_faulty,
    返回 mid360_fault.yaml 路径 (fault replay 专用, 不改原始实验配置)。"""
    src = os.path.join(exp_dir, "mid360.yaml")
    dst = os.path.join(exp_dir, "mid360_fault.yaml")
    with open(src) as f:
        content = f.read()
    content = content.replace('lid_topic:  "/livox/lidar"',
                              'lid_topic:  "/livox/lidar_faulty"')
    content = content.replace('lid_topic: "/livox/lidar"',
                              'lid_topic: "/livox/lidar_faulty"')
    with open(dst, "w") as f:
        f.write(content)
    return "mid360_fault.yaml"


def write_fault_annotations(exp_dir, fault_cfg):
    """由 fault 配置自动生成 scenario_annotations.yaml (correspondence_failure 真值)。
    追加写入, 保留手工标注段。"""
    with open(fault_cfg) as f:
        data = yaml.safe_load(f) or {}
    segs = (data.get("fault") or {}).get("segments", [])
    ann_path = os.path.join(exp_dir, "scenario_annotations.yaml")
    existing = []
    if os.path.exists(ann_path):
        with open(ann_path) as f:
            existing = (yaml.safe_load(f) or {}).get("scenarios", [])
    for s in segs:
        existing.append({
            "name": f"fault_{s['type']}_{s['start']}_{s['end']}",
            "start": float(s["start"]),
            "end": float(s["end"]),
            "ground_truth": "correspondence_failure",
        })
    with open(ann_path, "w") as f:
        yaml.dump({"scenarios": existing}, f, default_flow_style=False,
                  allow_unicode=True, sort_keys=False)
    return ann_path


def _degen_enabled(exp_dir):
    """读取实验配置 mid360.yaml 的 degeneracy.enable (决定是否开启 health)。"""
    cfg_path = os.path.join(exp_dir, "mid360.yaml")
    if not os.path.exists(cfg_path):
        return False
    try:
        with open(cfg_path) as f:
            d = yaml.safe_load(f) or {}
        return bool(d.get("/**", {}).get("ros__parameters", {})
                    .get("degeneracy", {}).get("enable", False))
    except Exception:
        return False


def _fastlio_health_args(exp_dir):
    """fastlio launch 附加 health 参数 (P2: health 依赖 degeneracy 门控)。"""
    if not _degen_enabled(exp_dir):
        return []
    return ["health_enable:=true", "health_imu_window_sec:=0.5"]


def _fastlio_adaptive_args(exp_dir, adaptive=False, max_geom=None, max_match=None):
    """fastlio launch 附加 P3 adaptive 参数 (唯一参数源 = launch, 依赖 degeneracy 门控)。

    --adaptive 时返回 adaptive_enable:=true (+ 可选 scale 覆盖)。
    max_geom/max_match 为 None 时使用 launch 默认值 (2.0/50.0)。
    """
    if not adaptive or not _degen_enabled(exp_dir):
        return []
    args = ["adaptive_enable:=true"]
    if max_geom is not None:
        args.append(f"adaptive_max_geom_scale:={max_geom}")
    if max_match is not None:
        args.append(f"adaptive_max_match_scale:={max_match}")
    return args


def _normalize_world(w):
    """world 参数规范化：
    - 绝对路径 → 原样（实验期生成夹具，get_urdf_launch 按绝对路径加载）
    - 包内地图名 → 去掉 .world 后缀的规范名
    """
    if not w:
        return None
    if w.startswith("/"):
        return w
    return w[:-5] if w.endswith(".world") else w


def _check_world_ready(world):
    """启动前校验 world（快速失败，避免 Gazebo 静默回退 empty.world → 无限坠落）：
    - 包内地图必须属于 VALID_WORLDS 白名单
    - 文件必须存在于 install/get_urdf/share/get_urdf/worlds/（launch 实际解析路径）
    - 绝对路径（实验期生成夹具）必须存在
    """
    if world.startswith("/"):
        if not os.path.isfile(world):
            print(f"[FATAL] world 文件不存在: {world}")
            sys.exit(1)
        return world
    if world not in VALID_WORLDS:
        print(f"[FATAL] world '{world}' 不在确认地图白名单: {VALID_WORLDS}")
        print("        确认地图（P0/P0.1 已验证）：test_world / normal_indoor / "
              "bookstore / hospital")
        print("        受控退化场景请用 --scenario "
              "normal_room|corridor|single_plane|open_ground（实验期生成夹具）")
        sys.exit(1)
    install_world = os.path.join(GET_URDF_WORLDS_INSTALL, world + ".world")
    src_world = os.path.join(WS_ROOT, "src", "get_urdf", "worlds", world + ".world")
    if not os.path.isfile(install_world):
        print(f"[FATAL] install 中不存在 world: {install_world}")
        print("        get_urdf_launch 解析的是 install 目录；请先重新构建 get_urdf:")
        print(f"        cd {WS_ROOT} && colcon build --packages-select get_urdf --symlink-install")
        sys.exit(1)
    if not os.path.isfile(src_world):
        print(f"[WARN] src 中不存在 {src_world}（install 有副本，疑似历史残留，请核对）")
    return world


def _topic_field_once(topic, field, timeout=4):
    """echo 话题的指定字段（取首行值），失败返回 None。

    注意: 不用 --once(-1)，因其与 --field 在本版 ros2 (Humble) 组合有 bug
    （报 "The passed message type is invalid" / 参数解析失败）。改为短超时
    连续 echo，取首行字段值。
    """
    cmd = ["timeout", str(timeout), "ros2", "topic", "echo", topic, "--field", field]
    try:
        out = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                             timeout=timeout + 3)
        lines = out.stdout.decode().strip().splitlines()
        return lines[0].strip() if lines else None
    except Exception:
        return None


def _check_sim_healthy(gazebo_log, world):
    """运行时健康门禁：验证世界真正加载、机器人落地且不坠落、雷达有点云。

    关键判据说明（针对"world 文件缺失 → Gazebo 静默回退 empty.world"的无声故障）：
      - 机器人 base_footprint 落地后 z≈0（不是 0.30），故**不能用绝对 z 阈值**判坠落；
      - 真坠落 = 初始落地(0.5s)后 z 仍持续大幅下降（自由落体 2s ≈ 20m）；
      - empty.world 也自带地面，所以"无房间"本身不坠落，真正判定是 gazebo.log
        是否出现 "Could not open file" + "Falling back on worlds/empty.world"。
    返回 (ok, reason)。
    """
    print("[run-online] 健康门禁: 等待 /gt_odom 并检查机器人是否落地 ...")
    # 1) 等待 /gt_odom 出现（世界真正加载 + 机器人已 spawn）。
    #    大场景（bookstore/hospital 等）gzserver 加载可达 30~60s，窗口放宽到 60s。
    z = None
    t0 = time.time()
    while time.time() - t0 < 60:
        val = _topic_field_once("/gt_odom", "pose.pose.position.z", timeout=4)
        if val is not None:
            try:
                z = float(val)
                break
            except ValueError:
                pass
        time.sleep(1)
    if z is None:
        return False, f"/gt_odom 无数据（world={world} 可能未加载成功）"
    # 2) 等待初始落地（机器人从 spawn z=0.30 落向地面，~0.5s），再采样验证 z 不再持续下降
    time.sleep(3)
    val = _topic_field_once("/gt_odom", "pose.pose.position.z", timeout=4)
    if val is None:
        return False, "/gt_odom 数据中断"
    try:
        z2 = float(val)
    except ValueError:
        z2 = z
    time.sleep(2)
    val = _topic_field_once("/gt_odom", "pose.pose.position.z", timeout=4)
    if val is None:
        return False, "/gt_odom 数据中断"
    try:
        z3 = float(val)
    except ValueError:
        z3 = z2
    # 自由落体判据：初始落地已结束，z 仍持续大幅下降 → 疑似无地面/无限坠落
    if z3 - z2 < -0.5:
        return False, f"机器人 z 持续下降 {z2:.3f}->{z3:.3f}（疑似无地面/无限坠落）"

    # 3) 雷达点云非空
    val = _topic_field_once("/livox/lidar", "header.stamp.sec", timeout=8)
    if val is None:
        return False, "/livox/lidar 无数据（雷达未输出点云）"

    # 4) gazebo.log 不应出现回退 empty.world（world 文件缺失 → 无房间结构）
    if os.path.isfile(gazebo_log):
        with open(gazebo_log, errors="ignore") as f:
            content = f.read()
        if "Could not open file" in content and "Falling back" in content:
            return False, "gazebo 回退到 empty.world（world 文件缺失），请重新构建 get_urdf"
    return True, f"仿真健康: 机器人落地(z≈{z3:.3f}), 雷达有点云"


def _spawn_pose(world, x, y, z, yaw):
    """出生位姿：用户显式传参优先，否则用推荐位姿（RECOMMENDED_SPAWN），否则 None(用 launch 默认)。"""
    rec = RECOMMENDED_SPAWN.get(world) if not world.startswith("/") else None
    if x is None:
        x = rec["x"] if rec else None
    if y is None:
        y = rec["y"] if rec else None
    if z is None:
        z = rec["z"] if rec else None
    if yaw is None:
        yaw = rec["yaw"] if rec else None
    return x, y, z, yaw


def generate_scenario(scenario, out_dir):
    """用 generate_worlds 生成受控退化场景到实验目录，返回 .world 绝对路径。"""
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import generate_worlds
    out_path = os.path.join(out_dir, f"{scenario}.world")
    generate_worlds.generate_one(scenario, out_path)
    return out_path


def write_resolved_snapshot(exp_dir, drive=None, fault_cfg=None, adaptive=None):
    """P2.3/P3: 把实际生效的 resolved 参数快照写入实验目录 (config.yaml 追加 + 独立文件)。

    health 参数以 _fastlio_health_args 为准 (degen 门控); P3 adaptive 参数以
    _fastlio_adaptive_args 为准 (唯一参数源 = launch); attribution 阈值表全文复制
    到实验目录; fault config 全文复制; drive 参数记录。保证 <exp_dir> 自包含、可复现
    —— 看任意 experiment 目录即可知那次实验真正运行的阈值/注入/驱动配置。
    """
    import shutil
    resolved = {
        "resolved_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "health": {"enable": _degen_enabled(exp_dir), "imu_window_sec": 0.5,
                   "via": "mapping.launch.py launch 参数 (mid360.yaml health 段已停用)"},
        "adaptive": {"enable": False,
                     "via": "mapping.launch.py launch 参数 (唯一参数源, mid360.yaml 无 adaptive 段)"},
    }
    if adaptive:
        resolved["adaptive"]["enable"] = bool(adaptive.get("enable", False))
        resolved["adaptive"]["max_geom_scale"] = adaptive.get("max_geom_scale")
        resolved["adaptive"]["max_match_scale"] = adaptive.get("max_match_scale")
        # 记录实际会传入 launch 的参数 (与 _fastlio_adaptive_args 一致)
        resolved["adaptive"]["launch_args"] = _fastlio_adaptive_args(
            exp_dir,
            adaptive=bool(adaptive.get("enable", False)),
            max_geom=adaptive.get("max_geom_scale"),
            max_match=adaptive.get("max_match_scale"))
    attr_src = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "..", "config", "attribution_params.yaml")
    if os.path.exists(attr_src):
        shutil.copy(attr_src, os.path.join(exp_dir, "attribution_params.yaml"))
        resolved["attribution_params_file"] = "attribution_params.yaml (实验目录内快照)"
    if drive:
        resolved["drive"] = drive
    if fault_cfg and os.path.exists(fault_cfg):
        shutil.copy(fault_cfg, os.path.join(exp_dir, "fault_config.yaml"))
        resolved["fault_config_file"] = "fault_config.yaml (实验目录内快照)"
    cfg_path = os.path.join(exp_dir, "config.yaml")
    with open(cfg_path, "a") as f:
        f.write("\n# ==== resolved 参数快照 (P2.3 冻结, run-online/replay 写入) ====\n")
        yaml.dump({"resolved": resolved}, f, default_flow_style=False,
                  allow_unicode=True, sort_keys=False)
    print(f"[run-online] resolved 参数快照已写入 {os.path.join(exp_dir, 'config.yaml')}")
    return resolved


def cmd_prepare(args):
    algo = args.algo
    scenario = getattr(args, "scenario", None)
    world = getattr(args, "world", None) or "test_world"
    # 先做静态校验（白名单/文件存在），失败时不残留空实验目录
    if scenario:
        if scenario not in SCENARIOS:
            print(f"[FATAL] 未知场景 '{scenario}'，可选: {list(SCENARIOS)}")
            sys.exit(1)
    else:
        world = _normalize_world(world)
        world = _check_world_ready(world)
    exp_dir = next_experiment_dir()
    os.makedirs(exp_dir, exist_ok=True)
    if scenario:
        world = generate_scenario(scenario, exp_dir)
        world = _check_world_ready(world)
    args.world = world  # 供 cmd_all 直接传递给 run-online
    write_fastlio_config(exp_dir, enable_degen=(algo == "degen"))

    meta = {
        "id": os.path.basename(exp_dir),
        "date": time.strftime("%Y-%m-%d %H:%M:%S"),
        "world": world,
        "scenario": scenario or "",
        "algo_mode": algo,
        "git_commit": git_commit(),
        "fastlio_config": "mid360.yaml",
        "notes": "P1 Degeneracy Detection and Evaluation",
    }
    with open(os.path.join(exp_dir, "config.yaml"), "w") as f:
        f.write("# lio_evaluation 实验元信息 (prepare 生成)\n")
        for k, v in meta.items():
            f.write(f"{k}: {v}\n")
        f.write("\n# liosam.txt 占位 (P1 无 LIO-SAM, 有对应话题时由 recorder 自动覆盖)\n")
    open(os.path.join(exp_dir, "liosam.txt"), "w").close()

    print(f"[prepare] 已创建 {exp_dir} (algo={algo}, world={world})")
    print(f"          运行: python3 run_experiment.py run-online {exp_dir} --duration 60 "
          f"[--world {world}]")
    return exp_dir


def _launch(cmd, logfile):
    f = open(logfile, "w")
    p = subprocess.Popen(cmd, stdout=f, stderr=subprocess.STDOUT,
                         preexec_fn=os.setsid)
    return p, f


def _stop_all(procs):
    for p, f in procs:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGINT)
        except Exception:
            try:
                p.terminate()
            except Exception:
                pass
    time.sleep(3)
    for p, f in procs:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGKILL)
        except Exception:
            pass
        f.close()
    # 兜底：直接清理可能幸存的 gzserver/gzclient（孤儿会占用 11345 端口，破坏下一次实验）
    subprocess.run(["pkill", "-9", "-f", "gzserver"], stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-9", "-f", "gzclient"], stderr=subprocess.DEVNULL)


def cmd_run_online(args):
    exp_dir = os.path.abspath(args.exp_dir)
    logdir = os.path.join(exp_dir, "logs")
    os.makedirs(logdir, exist_ok=True)
    world = args.world or "test_world"
    world = _normalize_world(world)
    world = _check_world_ready(world)
    x, y, z, yaw = _spawn_pose(world, args.x, args.y, args.z, args.yaw)

    procs = []
    print(f"[run-online] 启动仿真栈: world={world}, "
          f"出生位姿=(x={x},y={y},z={z},yaw={yaw}), 输出={exp_dir}")
    try:
        # 0. 清理残留 gzserver/gzclient：孤儿 gzserver 会占用 11345 端口，
        #    新 gazebo 会连到旧世界 → spawn_entity 报 "Entity already exists"。
        subprocess.run(["pkill", "-9", "-f", "gzserver"], stderr=subprocess.DEVNULL)
        subprocess.run(["pkill", "-9", "-f", "gzclient"], stderr=subprocess.DEVNULL)
        time.sleep(1)
        # 1. Gazebo（world 可为包内地图名或实验期生成夹具的绝对路径）
        gazebo_cmd = ["ros2", "launch", "get_urdf", "get_urdf_launch.py", f"world:={world}"]
        if getattr(args, "headless", False):
            gazebo_cmd.append("headless:=true")
        if getattr(args, "no_rviz", False):
            gazebo_cmd.append("rviz:=false")
        for k, v in (("x", x), ("y", y), ("z", z), ("yaw", yaw)):
            if v is not None:
                gazebo_cmd.append(f"{k}:={v}")
        procs.append(_launch(gazebo_cmd, os.path.join(logdir, "gazebo.log")))
        time.sleep(5)

        # 2. 健康门禁（世界加载 + 机器人落地 + 雷达就绪）。
        #    大场景（bookstore/hospital 等）模型加载可达 30~60s，若 FAST-LIO 提前启动，
        #    首帧会注册到"未加载完的局部几何" → 立即发散。故先门禁、后启动 FAST-LIO。
        ok, reason = _check_sim_healthy(os.path.join(logdir, "gazebo.log"), world)
        if not ok:
            print(f"[run-online][HEALTH-FAIL] {reason}")
            print("[run-online] 实验无效, 已中止。请检查 logs/gazebo.log / logs/gt.log")
            sys.exit(1)
        print(f"[run-online][HEALTH-OK] {reason}")

        # 3. fast_lio (使用实验配置, 按 algo 开启退化检测) — 世界就绪后再启动
        #    --fault 时改用 mid360_fault.yaml (订阅 /livox/lidar_faulty) + 启动 injector
        fault_cfg = getattr(args, "fault", None)
        config_file = "mid360.yaml"
        injector_proc = None
        if fault_cfg:
            config_file = write_fault_lidar_config(exp_dir)
            write_fault_annotations(exp_dir, os.path.abspath(fault_cfg))
            print(f"[run-online] fault injection 已启用: {os.path.abspath(fault_cfg)}")
        write_resolved_snapshot(
            exp_dir,
            drive={"mode": getattr(args, "drive_mode", "wander"),
                   "pause_sec": getattr(args, "pause_sec", 4.0)},
            fault_cfg=os.path.abspath(fault_cfg) if fault_cfg else None,
            adaptive={"enable": bool(getattr(args, "adaptive", False)),
                      "max_geom_scale": getattr(args, "adaptive_max_geom_scale", None),
                      "max_match_scale": getattr(args, "adaptive_max_match_scale", None)})
        fastlio_cmd = ["ros2", "launch", "fast_lio", "mapping.launch.py",
                       f"config_path:={exp_dir}", f"config_file:={config_file}",
                       "use_sim_time:=true",
                       *_fastlio_health_args(exp_dir),
                       *_fastlio_adaptive_args(exp_dir,
                                               getattr(args, "adaptive", False),
                                               getattr(args, "adaptive_max_geom_scale", None),
                                               getattr(args, "adaptive_max_match_scale", None))]
        if getattr(args, "no_rviz", False):
            fastlio_cmd.append("rviz:=false")  # fast_lio 自带 rviz2, 软渲染消耗大
        procs.append(_launch(fastlio_cmd, os.path.join(logdir, "fastlio.log")))
        # 4. lio_evaluation 采集 (proxy + recorder + monitor)
        procs.append(_launch(
            ["ros2", "launch", "lio_evaluation", "lio_evaluation.launch.py",
             f"output_dir:={exp_dir}"],
            os.path.join(logdir, "eval.log")))
        # 5. 真值桥接
        procs.append(_launch(
            ["ros2", "launch", "ground_truth_bridge", "ground_truth_bridge_launch.py",
             "use_sim_time:=true"],
            os.path.join(logdir, "gt.log")))
        # 5b. fault injector (需在 lidar 发布后启动, 先于运动驱动)
        if fault_cfg:
            inj = _launch(
                [sys.executable, FAULT_INJECTOR, "--ros-args",
                 "-p", "use_sim_time:=true", "-p", f"config:={os.path.abspath(fault_cfg)}"],
                os.path.join(logdir, "injector.log"))
            procs.append(inj)
        # 5c. canonical bag 录制 (P2.3: 固定输入轨迹供 P3 baseline/adaptive A/B 复放)
        if getattr(args, "record_bag", None):
            bag_dir = os.path.abspath(args.record_bag)
            # ros2 bag record -o 拒绝已存在目录: 空目录先清理, 非空则跳过 (防误覆盖)
            if os.path.exists(bag_dir):
                if os.listdir(bag_dir):
                    print(f"[run-online][WARN] bag 目录非空, 跳过录制: {bag_dir}")
                else:
                    os.rmdir(bag_dir)
            if not os.path.exists(bag_dir):
                procs.append(_launch(
                    ["ros2", "bag", "record", "-o", bag_dir,
                     "/livox/lidar", "/livox/imu", "/gt_odom"],
                    os.path.join(logdir, "bagrecord.log")))
                print(f"[run-online] canonical bag 录制: {bag_dir} "
                      f"(/livox/lidar /livox/imu /gt_odom, 不含 /lio/health — "
                      f"replay 时由新 FAST-LIO 生成, 避免双路混合)")
        time.sleep(3)  # 等 fast_lio/eval/gt 节点完成发现与启动

        # ---- 运动驱动：脚本漫游 / 手动遥控(WASD) / 禁用 ----
        # 脚本漫游 + 避障为默认（静态机器人 ATE/RPE 无意义，实验期必须移动）；
        # --teleop 时改为启动 gui_teleop_node，由操作者在 Teleop 窗口内 WASD 驾驶。
        teleop_proc = None
        if getattr(args, "no_drive", False):
            print("[run-online] --no-drive: 不启动任何运动驱动（机器人静止）")
        elif getattr(args, "teleop", False):
            tp = _launch(
                ["ros2", "run", "gui_teleop", "gui_teleop_node"],
                os.path.join(logdir, "teleop.log"))
            teleop_proc, _ = tp
            procs.append(tp)
            print("[run-online] 手动遥控(Teleop)已启动: 请在打开的窗口内驾驶")
            print("[run-online]   WASD=前后/左右转, Shift=加速, 空格=急停, "
                  "Q/E=横移; 关闭窗口即提前停止采集")
        else:
            drive_args = ["--ros-args", "-p", "use_sim_time:=true",
                          "-p", f"drive_mode:={getattr(args, 'drive_mode', 'wander')}"]
            if getattr(args, "drive_mode", "wander") == "stop_go":
                drive_args += ["-p", f"pause_sec:={getattr(args, 'pause_sec', 4.0)}"]
            procs.append(_launch(
                [sys.executable, DRIVE_NODE] + drive_args,
                os.path.join(logdir, "drive.log")))
            print(f"[run-online] 脚本化驱动已启动 "
                  f"(mode={getattr(args, 'drive_mode', 'wander')}"
                  f"+ 避障: 前进腿→转向, 前向锥见障步进避障)")

        # ---- 运行期健康看门狗（翻车/卡死/RTF） ----
        procs.append(_launch(
            [sys.executable, HEALTH_WATCH,
             "--ros-args",
             "-p", "use_sim_time:=true",
             "-p", f"output_dir:={exp_dir}",
             "-p", f"abort:={getattr(args, 'health_abort', False)}"],
            os.path.join(logdir, "health_watch.log")))
        print("[run-online] 健康看门狗已启动 (health_watch.log -> health.log)")

        print(f"[run-online] 采集 {args.duration}s(仿真时间), Ctrl+C 可提前停止...")
        abort_file = os.path.join(exp_dir, "ABORT")
        stopped_early = False
        sim0 = None
        t_wall0 = time.time()
        while True:
            if os.path.exists(abort_file):
                print("[run-online] 健康看门狗触发中止 "
                      "(FLIP, 见 health.log 与 ABORT 哨兵)")
                stopped_early = True
                break
            # 手动遥控模式：Teleop 窗口被关闭 → 提前停止采集
            if teleop_proc is not None and teleop_proc.poll() is not None:
                print("[run-online] Teleop 窗口已关闭, 提前停止采集")
                break
            # 以仿真时间推进 args.duration 秒为准（RTF<1 时墙钟自动延长），
            # 保证落盘轨迹的真实跨度达到请求时长。
            val = _topic_field_once("/clock", "clock.sec", timeout=3)
            if val is not None:
                try:
                    sim_now = float(val)
                except ValueError:
                    sim_now = None
                if sim_now is not None:
                    if sim0 is None:
                        sim0 = sim_now
                    elif sim_now - sim0 >= args.duration:
                        print(f"[run-online] 采集完成: "
                              f"仿真时长 {sim_now - sim0:.1f}s")
                        break
            # 墙钟超时兜底（RTF 过低时避免无限等待）
            if time.time() - t_wall0 > args.duration * 4:
                print(f"[run-online] 墙钟超时兜底 "
                      f"({args.duration * 2}s), 停止采集")
                break
            time.sleep(1)
        if stopped_early:
            with open(os.path.join(exp_dir, "config.yaml"), "a") as f:
                f.write("aborted: true # 健康看门狗提前中止 (翻车)\n")
    except KeyboardInterrupt:
        print("[run-online] 收到 Ctrl+C, 停止采集")
    finally:
        _stop_all(procs)
        print("[run-online] 已停止全部进程")


def cmd_replay(args):
    exp_dir = os.path.abspath(args.exp_dir)
    logdir = os.path.join(exp_dir, "logs")
    os.makedirs(logdir, exist_ok=True)
    bag = os.path.abspath(args.bag)
    if not os.path.isdir(bag):
        print(f"[replay] bag 不存在: {bag}")
        sys.exit(1)

    # P2: 可选 LiDAR fault injection —— 生成故障版配置 + 自动标注 scenario_annotations
    fault_cfg = getattr(args, "fault", None)
    config_file = "mid360.yaml"
    if fault_cfg:
        config_file = write_fault_lidar_config(exp_dir)
        injector_path = os.path.abspath(fault_cfg)
        write_fault_annotations(exp_dir, injector_path)  # 自动生成对应真值
        print(f"[replay] fault injection 已启用: {injector_path} "
              f"(fastlio 订阅 /livox/lidar_faulty)")
    write_resolved_snapshot(exp_dir,
                            fault_cfg=os.path.abspath(fault_cfg) if fault_cfg else None,
                            adaptive={"enable": bool(getattr(args, "adaptive", False)),
                                      "max_geom_scale": getattr(args, "adaptive_max_geom_scale", None),
                                      "max_match_scale": getattr(args, "adaptive_max_match_scale", None)})

    procs = []
    print(f"[replay] 回放 {bag} -> {exp_dir}")
    try:
        procs.append(_launch(
            ["ros2", "launch", "fast_lio", "mapping.launch.py",
             f"config_path:={exp_dir}", f"config_file:={config_file}",
             "use_sim_time:=true",
             *_fastlio_health_args(exp_dir),
             *_fastlio_adaptive_args(exp_dir,
                                     getattr(args, "adaptive", False),
                                     getattr(args, "adaptive_max_geom_scale", None),
                                     getattr(args, "adaptive_max_match_scale", None)),
             *(["rviz:=false"] if getattr(args, "no_rviz", False) else [])],
            os.path.join(logdir, "fastlio.log")))
        procs.append(_launch(
            ["ros2", "launch", "lio_evaluation", "lio_evaluation.launch.py",
             f"output_dir:={exp_dir}"],
            os.path.join(logdir, "eval.log")))
        if fault_cfg:
            procs.append(_launch(
                [sys.executable, FAULT_INJECTOR, "--ros-args",
                 "-p", "use_sim_time:=true", "-p", f"config:={injector_path}"],
                os.path.join(logdir, "injector.log")))
        time.sleep(3)
        procs.append(_launch(
            ["ros2", "bag", "play", bag, "--clock"],
            os.path.join(logdir, "bagplay.log")))
        print(f"[replay] 回放 {args.duration}s...")
        time.sleep(args.duration)
    except KeyboardInterrupt:
        print("[replay] 收到 Ctrl+C")
    finally:
        _stop_all(procs)


def cmd_eval(args):
    exp_dir = os.path.abspath(args.exp_dir)
    base = os.path.dirname(os.path.abspath(__file__))
    compute = os.path.join(base, "compute_metrics.py")
    directional = os.path.join(base, "directional_analysis.py")
    subprocess.run([sys.executable, compute, exp_dir], check=False)
    subprocess.run([sys.executable, directional, exp_dir], check=False)
    # P2: 误差归因两层验证 (健康 + 归因 CSV 存在时才运行)
    if os.path.exists(os.path.join(exp_dir, "lio_health.csv")):
        subprocess.run([sys.executable, ATTRIBUTION_VALIDATION, exp_dir], check=False)


def cmd_all(args):
    exp_dir = cmd_prepare(args)  # prepare 会把 args.world 解析为最终 world（含 --scenario 生成路径）
    if args.bag:
        cmd_replay(argparse.Namespace(exp_dir=exp_dir, bag=args.bag, duration=args.duration,
                                      fault=getattr(args, "fault", None),
                                      no_rviz=getattr(args, "no_rviz", False),
                                      adaptive=getattr(args, "adaptive", False),
                                      adaptive_max_geom_scale=getattr(
                                          args, "adaptive_max_geom_scale", None),
                                      adaptive_max_match_scale=getattr(
                                          args, "adaptive_max_match_scale", None)))
    else:
        cmd_run_online(argparse.Namespace(
            exp_dir=exp_dir, world=args.world,
            x=args.x, y=args.y, z=args.z, yaw=args.yaw, duration=args.duration,
            no_drive=args.no_drive, teleop=args.teleop, headless=args.headless,
            no_rviz=args.no_rviz, health_abort=args.health_abort,
            drive_mode=getattr(args, "drive_mode", "wander"),
            pause_sec=getattr(args, "pause_sec", 4.0),
            fault=getattr(args, "fault", None),
            record_bag=getattr(args, "record_bag", None),
            adaptive=getattr(args, "adaptive", False),
            adaptive_max_geom_scale=getattr(args, "adaptive_max_geom_scale", None),
            adaptive_max_match_scale=getattr(args, "adaptive_max_match_scale", None)))
    cmd_eval(argparse.Namespace(exp_dir=exp_dir))


def main():
    p = argparse.ArgumentParser(description="lio_evaluation 实验编排")
    sub = p.add_subparsers(dest="cmd", required=True)

    def _add_world_args(sp):
        sp.add_argument("--world", default=None,
                        help="包内确认地图名 (test_world/normal_indoor/bookstore/"
                             "hospital) 或 .world 绝对路径")
        sp.add_argument("--scenario", choices=list(SCENARIOS), default=None,
                        help="受控退化场景 (normal_room/corridor/single_plane/"
                             "open_ground)，实验期生成夹具")

    def _add_spawn_args(sp):
        sp.add_argument("--x", type=float, default=None, help="出生 x (m)")
        sp.add_argument("--y", type=float, default=None, help="出生 y (m)")
        sp.add_argument("--z", type=float, default=None, help="出生 z (m), 默认 0.30")
        sp.add_argument("--yaw", type=float, default=None, help="出生 yaw (rad)")
        sp.add_argument("--no-drive", action="store_true",
                        help="禁用运动驱动（默认开启脚本漫游；静态机器人 ATE/RPE 无意义）")
        sp.add_argument("--teleop", action="store_true",
                        help="用手动遥控(WASD)代替脚本漫游: 启动 gui_teleop_node "
                             "在 Teleop 窗口内驾驶小车录数据")
        sp.add_argument("--headless", action="store_true",
                        help="无 GUI 运行仿真（只起 gzserver，不起 gzclient），降低渲染负载")
        sp.add_argument("--no-rviz", action="store_true",
                        help="不起 rviz2（实验默认建议开启，减少软渲染负载）")
        sp.add_argument("--health-abort", action="store_true",
                        help="健康看门狗检测到翻车时提前中止采集（默认仅记录 health.log）")
        # P2: 驱动模式 (stop_go 制造近静止窗口; const_vel 走廊近匀速直行)
        # 注意: dest 必须是 drive_mode (run-online 内部读 args.drive_mode);
        # 此前误用 --drive 的默认 dest `drive` 导致 drive_mode 恒为 wander (已修复)。
        sp.add_argument("--drive", dest="drive_mode",
                        choices=["wander", "stop_go", "const_vel"],
                        default="wander",
                        help="脚本化驱动模式: wander(漫游) / stop_go(前进-静止) / "
                             "const_vel(低速近匀速直行)")
        sp.add_argument("--pause-sec", type=float, default=4.0,
                        help="stop_go 静止段时长 (s)")

    def _add_fault_arg(sp):
        sp.add_argument("--fault", default=None, metavar="FAULT_CONFIG.yaml",
                        help="LiDAR fault injection 配置 (correspondence_failure 真值): "
                             "回放时启动 injector, fastlio 订阅 /livox/lidar_faulty")

    def _add_adaptive_arg(sp):
        sp.add_argument("--adaptive", action="store_true",
                        help="P3: 启用自适应 LiDAR 权重 (adaptive.enable=true, "
                             "须 --algo degen / degeneracy.enable=true)")
        sp.add_argument("--adaptive-max-geom-scale", type=float, default=None,
                        metavar="X",
                        help="P3: geometry channel 最大 covariance 放大倍数 "
                             "(默认 launch 2.0, mild)")
        sp.add_argument("--adaptive-max-match-scale", type=float, default=None,
                        metavar="X",
                        help="P3: matching channel 最大 covariance 放大倍数 "
                             "(默认 launch 50.0, aggressive)")

    p_prepare = sub.add_parser("prepare")
    _add_world_args(p_prepare)
    p_prepare.add_argument("--algo", choices=["baseline", "degen"], default="degen")
    p_prepare.set_defaults(func=cmd_prepare)

    p_run = sub.add_parser("run-online")
    p_run.add_argument("exp_dir")
    _add_world_args(p_run)
    _add_spawn_args(p_run)
    _add_fault_arg(p_run)
    _add_adaptive_arg(p_run)
    p_run.add_argument("--duration", type=int, default=60)
    p_run.add_argument("--record-bag", default=None, metavar="BAG_DIR",
                       help="P2.3: 实验同时录制 canonical bag 到 BAG_DIR "
                            "(/livox/lidar /livox/imu /gt_odom /lio/health)")
    p_run.set_defaults(func=cmd_run_online)

    p_replay = sub.add_parser("replay")
    p_replay.add_argument("exp_dir")
    p_replay.add_argument("--bag", required=True)
    p_replay.add_argument("--duration", type=int, default=60)
    _add_fault_arg(p_replay)
    _add_adaptive_arg(p_replay)
    p_replay.set_defaults(func=cmd_replay)

    p_eval = sub.add_parser("eval")
    p_eval.add_argument("exp_dir")
    p_eval.set_defaults(func=cmd_eval)

    p_all = sub.add_parser("all")
    _add_world_args(p_all)
    _add_spawn_args(p_all)
    p_all.add_argument("--algo", choices=["baseline", "degen"], default="degen")
    p_all.add_argument("--duration", type=int, default=60)
    p_all.add_argument("--bag", default=None)
    p_all.add_argument("--record-bag", default=None, metavar="BAG_DIR",
                       help="P2.3: run-online 同时录制 canonical bag")
    _add_fault_arg(p_all)
    _add_adaptive_arg(p_all)
    p_all.set_defaults(func=cmd_all)

    args = p.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
