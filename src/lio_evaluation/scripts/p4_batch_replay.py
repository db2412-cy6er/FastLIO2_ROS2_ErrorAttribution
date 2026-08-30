#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P4 批量回放: 对同一 canonical bag 跑 degen / directional(beta) A/B 对比。

用于 P4.5 标定 (single_plane_run1: beta 网格) 与 P4.6 正式 A/B (holdout bag)。
每次 run 流程: prepare --algo degen --scenario S → replay (可选 --directional beta)
→ eval → 收集 ATE mean。输出汇总 CSV。

用法示例:
  # 标定: single_plane_run1 上 degen + beta{0.3,0.5,0.7}, 各 3 reps
  python3 p4_batch_replay.py --bag data/bags/single_plane_run1 --scenario single_plane \
      --tag calib_sp1 --beta 0.3 0.5 0.7 --reps 3 --duration 72
  # 正式 A/B: holdout bag 上 degen + beta0.5, 各 3 reps
  python3 p4_batch_replay.py --bag data/bags/single_plane_run2 --scenario single_plane \
      --tag ab_sp2 --beta 0.5 --reps 3 --duration 72

--include-degen: 同时跑 degen-only (无 --directional) 组 (每组 reps 次)。
"""
import argparse
import csv
import os
import re
import subprocess
import sys
import time

WS_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))
RUN = os.path.join(WS_ROOT, "src/lio_evaluation/scripts/run_experiment.py")
EXP_BASE = os.path.join(WS_ROOT, "data/experiments")


def _sh(cmd, timeout=None):
    return subprocess.run([sys.executable, RUN] + cmd,
                          capture_output=True, text=True, timeout=timeout)


def prepare(world_or_scenario, is_world=False):
    if is_world:
        r = _sh(["prepare", "--algo", "degen", "--world", world_or_scenario])
    else:
        r = _sh(["prepare", "--algo", "degen", "--scenario", world_or_scenario])
    m = re.search(r"experiment_(\d+)", r.stdout + r.stderr)
    if not m:
        print("prepare 失败:\n", r.stdout, r.stderr)
        sys.exit(1)
    exp = os.path.join(EXP_BASE, f"experiment_{m.group(1)}")
    print(f"[batch] prepare -> {os.path.basename(exp)}", flush=True)
    return exp


def replay(exp, bag, duration, beta=None, fault=None):
    cmd = ["replay", exp, "--bag", bag, "--duration", str(duration)]
    if beta is not None:
        cmd += ["--directional", "--directional-beta-t", str(beta)]
    if fault:
        cmd += ["--fault", fault]
    r = _sh(cmd, timeout=duration * 4 + 240)
    if r.returncode != 0:
        print(f"[batch] replay 失败 ({os.path.basename(exp)}): "
              f"{r.stdout[-500:]}{r.stderr[-500:]}", flush=True)
        return False
    return True


def eval_ate(exp):
    _sh(["eval", exp])
    result_yaml = os.path.join(exp, "result.yaml")
    if not os.path.exists(result_yaml):
        return None
    with open(result_yaml) as f:
        for line in f:
            if line.strip().startswith("mean_m:"):
                return float(line.split(":")[1])
    return None


def run_batch(bag, scenario, tag, betas, reps, duration, include_degen, fault=None,
              is_world=False):
    os.makedirs(EXP_BASE, exist_ok=True)
    out = os.path.join(EXP_BASE, f"p4_batch_{tag}.csv")
    results = []
    header = ["tag", "config", "rep", "exp", "ate_mean_m"]
    with open(out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
    print(f"[batch] tag={tag} scenario={scenario} bag={os.path.basename(bag)} "
          f"betas={betas} reps={reps} duration={duration} degen={include_degen}")

    groups = []
    if include_degen:
        groups.append(("degen", None))
    for b in betas:
        groups.append((f"dir_b{b}", b))

    for gname, beta in groups:
        for rep in range(1, reps + 1):
            t0 = time.time()
            exp = prepare(scenario, is_world=is_world)
            if not replay(exp, bag, duration, beta=beta, fault=fault):
                continue
            ate = eval_ate(exp)
            if ate is None:
                print(f"[batch] {gname} rep{rep} 无 ATE (eval 失败)", flush=True)
                continue
            row = [tag, gname, rep, os.path.basename(exp), f"{ate:.6f}"]
            with open(out, "a", newline="") as f:
                csv.writer(f).writerow(row)
            print(f"[batch] {gname} rep{rep}: ATE mean = {ate:.4f} m "
                  f"({time.time()-t0:.0f}s)", flush=True)

    print(f"[batch] 完成, 汇总: {out}")
    # 打印 median 汇总
    import statistics
    with open(out) as f:
        rows = list(csv.DictReader(f))
    for gname in set(r["config"] for r in rows):
        vals = [float(r["ate_mean_m"]) for r in rows if r["config"] == gname]
        if vals:
            print(f"[batch] {gname}: n={len(vals)} median={statistics.median(vals):.4f} "
                  f"mean={statistics.mean(vals):.4f}")


def main():
    ap = argparse.ArgumentParser(description="P4 批量回放 A/B")
    ap.add_argument("--bag", required=True)
    ap.add_argument("--scenario", required=False,
                    choices=["normal_room", "corridor", "single_plane", "open_ground"],
                    help="受控退化场景 (与 --world 二选一)")
    ap.add_argument("--world", default=None,
                    help="包内确认地图名 (test_world/normal_indoor/bookstore/hospital); "
                         "与 --scenario 二选一 (--scenario 优先)")
    ap.add_argument("--tag", required=True)
    ap.add_argument("--beta", type=float, nargs="*", default=[],
                    help="directional beta_t 网格 (可多个)")
    ap.add_argument("--reps", type=int, default=1)
    ap.add_argument("--duration", type=int, default=60)
    ap.add_argument("--include-degen", action="store_true",
                    help="同时跑 degen-only 组 (baseline)")
    ap.add_argument("--fault", default=None, help="fault config yaml (相对 workspace 或绝对路径)")
    args = ap.parse_args()
    bag = os.path.abspath(args.bag)
    if not os.path.isdir(bag):
        print(f"[batch] bag 不存在: {bag}")
        sys.exit(1)
    fault = None
    if args.fault:
        fault = os.path.abspath(args.fault)
    is_world = bool(not args.scenario and args.world)
    scene = args.world if is_world else args.scenario
    run_batch(bag, scene, args.tag, args.beta, args.reps, args.duration,
              args.include_degen, fault=fault, is_world=is_world)


if __name__ == "__main__":
    main()
