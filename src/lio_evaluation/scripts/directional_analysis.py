#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""weak direction vs GT directional error 对照分析 (M6)

建立"内部观测退化方向"与"GT 实际误差方向"之间的联系:
  1. groundtruth.txt + fastlio.txt -> SE(3) Umeyama 对齐 + 时间插值
  2. 逐帧位置误差 e_t (x/y/z) + 姿态误差 (rotation angle)
  3. 从 degen_exact.csv 取每帧 weak_direction (几何 Hessian 最小特征值特征向量)
  4. 计算误差在 weak direction 上的投影 e_weak = w[0:3] · e_t
  5. 对照: weak 方向误差 vs x/y/z 各轴误差; 识别增长最快方向

用法: python3 directional_analysis.py <experiment_dir>
输出: directional_error.png, result.yaml 追加 directional 字段
"""

import os
import sys
import csv

import numpy as np
import yaml

from evo.core import sync
from evo.tools import file_interface


def slerp(q0, q1, t):
    q0 = np.asarray(q0, dtype=np.float64)
    q1 = np.asarray(q1, dtype=np.float64)
    if q0.dot(q1) < 0.0:
        q1 = -q1
    omega = np.arccos(np.clip(q0.dot(q1), -1.0, 1.0))
    if abs(omega) < 1e-9:
        return q0
    return (np.sin((1.0 - t) * omega) * q0 + np.sin(t * omega) * q1) / np.sin(omega)


def load_tum(path):
    times, pos, quat = [], [], []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            p = line.split()
            if len(p) < 8:
                continue
            times.append(float(p[0]))
            pos.append([float(x) for x in p[1:4]])
            quat.append([float(x) for x in p[4:8]])
    return np.array(times), np.array(pos), np.array(quat)


def quat_to_rotm(q):
    x, y, z, w = q[0], q[1], q[2], q[3]
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def rel_rotation_angle(q_ref, q_est):
    """估计相对参考的姿态误差角 (rad)"""
    R = quat_to_rotm(q_est).T @ quat_to_rotm(q_ref)
    cos_a = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    return np.arccos(cos_a)


def load_degen(path):
    if not os.path.exists(path):
        return None
    rows = {}
    with open(path) as f:
        for r in csv.DictReader(f):
            try:
                rows[float(r["timestamp"])] = {
                    "score": float(r["score"]),
                    "weak": np.array([float(r[k]) for k in ("w0", "w1", "w2", "w3", "w4", "w5")]),
                }
            except (ValueError, KeyError):
                continue
    return rows


def nearest_weak(degen, t, tol=0.2):
    if degen is None:
        return None
    keys = np.array(sorted(degen.keys()))
    i = np.searchsorted(keys, t)
    best = None
    for j in (i - 1, i):
        if 0 <= j < len(keys) and abs(keys[j] - t) <= tol:
            if best is None or abs(keys[j] - t) < abs(best[0] - t):
                best = (keys[j], degen[keys[j]])
    return best[1]["weak"] if best else None

def main():
    if len(sys.argv) < 2:
        print("用法: directional_analysis.py <experiment_dir>")
        sys.exit(1)
    out_dir = sys.argv[1]
    gt_tum = os.path.join(out_dir, "groundtruth.txt")
    est_tum = os.path.join(out_dir, "fastlio.txt")
    degen_path = os.path.join(out_dir, "degen_exact.csv")
    if not (os.path.exists(gt_tum) and os.path.exists(est_tum)):
        print(f"缺少轨迹文件: {gt_tum} 或 {est_tum}")
        sys.exit(1)

    gt_t, gt_p, gt_q = load_tum(gt_tum)
    est_t, est_p, est_q = load_tum(est_tum)

    # 1) GT 按 est 时间戳插值
    gt_map = {t: (p, q) for t, p, q in zip(gt_t, gt_p, gt_q)}
    keys = np.array(sorted(gt_map.keys()))
    gt_pos_i = np.empty((len(est_t), 3))
    gt_quat_i = np.empty((len(est_t), 4))
    for i, t in enumerate(est_t):
        idx = min(max(np.searchsorted(keys, t), 1), len(keys) - 1)
        t0, t1 = keys[idx - 1], keys[idx]
        if t1 - t0 < 1e-12:
            gt_pos_i[i], gt_quat_i[i] = gt_map[t0]
            continue
        s = np.clip((t - t0) / (t1 - t0), 0.0, 1.0)
        p0, q0 = gt_map[t0]
        p1, q1 = gt_map[t1]
        gt_pos_i[i] = p0 + s * (np.asarray(p1) - np.asarray(p0))
        gt_quat_i[i] = slerp(q0, q1, s)

    # 2) SE(3) Umeyama 对齐 (est -> gt 坐标系)
    gt_traj = file_interface.read_tum_trajectory_file(gt_tum)
    est_traj = file_interface.read_tum_trajectory_file(est_tum)
    gt_sync, est_sync = sync.associate_trajectories(gt_traj, est_traj, max_diff=0.05)
    est_sync.align(gt_sync, correct_scale=False)
    est_aligned_p = est_sync.positions_xyz

    # 3) 逐帧误差
    n = len(est_aligned_p)
    err = est_aligned_p - gt_pos_i[:n]
    # 注意: 姿态误差必须用“对齐后”的姿态 (est_sync.orientations_quat_wxyz)。
    # 直接比较原始 est_q 会把“出生 yaw 帧差”(GT 从 spawn yaw 开始, FAST-LIO odom 从 0 开始)
    # 误算成 ~90° 姿态误差: 如 indoor_room 出生 yaw=1.57(90°) 时 rot_err≈90°、bookstore
    # 出生 yaw=-1.72(-98.6°) 时 rot_err≈98.7°，而 yaw=0 的场景 rot_err 都很小。
    est_q_wxyz = est_sync.orientations_quat_wxyz[:n]   # evo 内部为 wxyz 顺序
    est_q_aligned = est_q_wxyz[:, [1, 2, 3, 0]]        # 转 xyzw，与 rel_rotation_angle 一致
    rot_ang = np.array([rel_rotation_angle(gt_quat_i[i], est_q_aligned[i])
                        for i in range(n)])

    # 4) weak direction 投影
    degen = load_degen(degen_path)
    e_weak = np.full(n, np.nan)
    for i, t in enumerate(est_t[:n]):
        w = nearest_weak(degen, t)
        if w is not None:
            wtrans = w[:3]
            nrm = np.linalg.norm(wtrans)
            if nrm > 1e-6:
                wtrans = wtrans / nrm
            e_weak[i] = np.abs(wtrans @ err[i])

    # 5) 统计
    stats = {
        "n_frames": int(n),
        "pos_err_rms_m": {
            "x": float(np.sqrt(np.mean(err[:, 0] ** 2))),
            "y": float(np.sqrt(np.mean(err[:, 1] ** 2))),
            "z": float(np.sqrt(np.mean(err[:, 2] ** 2))),
        },
        "pos_err_total_rms_m": float(np.sqrt(np.mean(np.sum(err ** 2, axis=1)))),
        "rot_err_mean_deg": float(np.mean(rot_ang) * 180.0 / np.pi),
        "rot_err_max_deg": float(np.max(rot_ang) * 180.0 / np.pi),
    }
    valid = ~np.isnan(e_weak)
    if valid.sum() > 3:
        stats["weak_dir_err_rms_m"] = float(np.sqrt(np.mean(e_weak[valid] ** 2)))
        half = n // 2
        growth = {ax: float(np.mean(np.abs(err[half:, i])) - np.mean(np.abs(err[:half, i])))
                  for i, ax in enumerate(("x", "y", "z"))}
        stats["axis_error_growth_m"] = growth
        stats["max_growth_axis"] = max(growth, key=lambda k: growth[k])
    else:
        stats["weak_dir_err_rms_m"] = None


    # 6) 绘图
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax1 = plt.subplots(figsize=(12, 6))
    t_sec = est_t[:n] - est_t[0]
    ax1.plot(t_sec, np.abs(err[:, 0]), label="|err_x|", lw=1.0)
    ax1.plot(t_sec, np.abs(err[:, 1]), label="|err_y|", lw=1.0)
    ax1.plot(t_sec, np.abs(err[:, 2]), label="|err_z|", lw=1.0)
    ax1.plot(t_sec, e_weak, label="|err on weak_dir|", lw=2.0, color="red")
    ax1.set_xlabel("t (s)")
    ax1.set_ylabel("position error (m)")
    ax1.legend(loc="upper left")
    ax2 = ax1.twinx()
    if degen:
        dkeys = np.array(sorted(degen.keys()))
        dscore = np.array([degen[k]["score"] for k in dkeys])
        ax2.plot(dkeys - est_t[0], dscore, label="degeneracy score", color="grey",
                 alpha=0.7, linestyle="--")
    ax2.set_ylabel("score")
    ax2.set_ylim(0, 1)
    fig.tight_layout()
    fig_path = os.path.join(out_dir, "directional_error.png")
    fig.savefig(fig_path, dpi=120)
    plt.close(fig)

    print("=== weak direction vs GT directional error ===")
    print("pos err rms: x=%.3f y=%.3f z=%.3f total=%.3f m" % (
        stats["pos_err_rms_m"]["x"], stats["pos_err_rms_m"]["y"],
        stats["pos_err_rms_m"]["z"], stats["pos_err_total_rms_m"]))
    print("rot err mean=%.2f max=%.2f deg" % (
        stats["rot_err_mean_deg"], stats["rot_err_max_deg"]))
    if stats.get("weak_dir_err_rms_m"):
        print("weak_dir err rms: %.3f m" % stats["weak_dir_err_rms_m"])
        print("axis growth: %s" % stats["axis_error_growth_m"])
        print("max growth axis: %s" % stats["max_growth_axis"])
    print("已生成: %s" % fig_path)

    # 7) 写入 result.yaml
    ry = os.path.join(out_dir, "result.yaml")
    if os.path.exists(ry):
        with open(ry) as f:
            result = yaml.safe_load(f) or {}
        result["directional"] = {k: v for k, v in stats.items() if k != "n_frames"}
        with open(ry, "w") as f:
            yaml.dump(result, f, default_flow_style=False, allow_unicode=True, sort_keys=False)
        print("已更新: %s" % ry)


if __name__ == "__main__":
    main()

