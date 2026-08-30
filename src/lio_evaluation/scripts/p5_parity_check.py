#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P5 parity check: 用 P5.0 采集的 first-iteration 数据验证 gate 判定一致性。

背景: P5 gate 判定在 C++ (laserMapping.cpp robust_gate_reject_check) 里实现,
  本脚本用 Python 复刻相同逻辑 (rolling-baseline relative 通道 + state machine),
  对真实 replay 的 degen_iterations.csv (iteration==1 行) 重放:
    - 统计 clean / V5 / stress / single_plane / normal_indoor 各场景的
      拒绝帧数 / 连续拒绝最大长度 / recovery 状态分布;
    - 验证 P5.0 结论: clean 场景 0 拒绝 (或极低), fault 场景大量拒绝;
    - 验证 single_plane (geometry) 不误杀。

用法:
  python3 p5_parity_check.py <exp1> [<exp2> ...]
"""
import argparse
import csv
import os

from p5_synthetic_test import GateSim


def load_first(exp_dir):
    rows = []
    with open(os.path.join(exp_dir, "degen_iterations.csv")) as f:
        for r in csv.DictReader(f):
            rows.append(r)
    return [r for r in rows if int(r["iteration"]) == 1]


def run(exp_dir):
    first = load_first(exp_dir)
    g = GateSim()
    stats = {
        "frames": len(first), "rejected": 0, "max_consecutive": 0,
        "state_hist": {0: 0, 1: 0, 2: 0, 3: 0},
        "reject_frames": [],  # (timestamp, eff, rel_num, rel_ratio, rel_p90)
    }
    for r in sorted(first, key=lambda x: float(x["timestamp"])):
        t = float(r["timestamp"])
        eff = float(r["effective_feature_num"])
        cand = float(r["candidate_feature_num"])
        p90 = float(r["residual_p90"])
        rej, st, cc, sev = g.step(t, eff, cand, p90)
        stats["state_hist"][st] += 1
        if rej:
            stats["rejected"] += 1
            stats["max_consecutive"] = max(stats["max_consecutive"], cc)
            if len(stats["reject_frames"]) < 10:
                stats["reject_frames"].append({
                    "t": round(t, 2), "eff": eff,
                    "rel_num": round(g.last_rel[0], 3),
                    "rel_ratio": round(g.last_rel[1], 3),
                    "rel_p90": round(g.last_rel[2], 3)})
    return stats


def main():
    ap = argparse.ArgumentParser(description="P5 gate parity check")
    ap.add_argument("exps", nargs="+")
    args = ap.parse_args()

    print(f"{'exp':20s} {'帧数':>5s} {'拒绝':>5s} {'拒绝率':>7s} {'max连续':>6s} "
          f"{'NORM':>5s} {'R_ONCE':>6s} {'DEGRAD':>7s} {'REC_REQ':>7s}")
    for exp in args.exps:
        s = run(exp)
        name = os.path.basename(exp)
        h = s["state_hist"]
        print(f"{name:20s} {s['frames']:5d} {s['rejected']:5d} "
              f"{s['rejected']/max(s['frames'],1)*100:6.1f}% {s['max_consecutive']:6d} "
              f"{h[0]:5d} {h[1]:6d} {h[2]:7d} {h[3]:7d}")
        if s["reject_frames"]:
            print(f"  前 5 个拒绝帧: {s['reject_frames'][:5]}")

    print("\n判定准则 (P5.0 结论):")
    print("  - clean (corridor_clean / normal_indoor / single_plane): 拒绝率应≈0%")
    print("  - fault (V5 / stress): 拒绝率应显著 > 0")
    print("  - single_plane: 0 拒绝 → geometry 与 correspondence 分离")


if __name__ == "__main__":
    main()
