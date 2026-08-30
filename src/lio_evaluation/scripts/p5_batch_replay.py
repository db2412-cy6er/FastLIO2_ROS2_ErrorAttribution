#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P5 批量回放: 对 canonical bag 跑 degen / robust_gate A/B 对比 (median-of-3)。

用于 P5.4 正式 A/B:
  - V5 holdout (corridor_run2 + corridor_holdout_faults.yaml)
  - stress (corridor_run2 + corridor_stress_faults.yaml)
  - 无害性: normal_indoor_run1 (无 fault)
  - regression: single_plane_run2 (无 fault)

每次 run: prepare (corridor/single_plane 场景) → replay (--robust-gate 可选) →
eval → 收集 ATE mean。输出汇总 CSV。

用法示例:
  python3 p5_batch_replay.py --bag data/bags/corridor_run2 --scenario corridor \
      --tag ab_v5 --fault data/fault_configs/corridor_holdout_faults.yaml \
      --groups degen robust_gate --reps 3 --duration 100
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
        print(f"prepare 失败:\n{r.stdout[-500:]}{r.stderr[-500:]}")
        sys.exit(1)
    exp = os.path.join(EXP_BASE, f"experiment_{m.group(1)}")
    print(f"[batch] prepare -> {os.path.basename(exp)}", flush=True)
    return exp


def replay(exp, bag, duration, robust_gate=False, fault=None):
    cmd = ["replay", exp, "--bag", bag, "--duration", str(duration), "--no-rviz"]
    if robust_gate:
        cmd.append("--robust-gate")
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


def run_batch(bag, scenario, tag, groups, reps, duration, fault=None, is_world=False):
    os.makedirs(EXP_BASE, exist_ok=True)
    out = os.path.join(EXP_BASE, f"p5_batch_{tag}.csv")
    results = []
    header = ["tag", "config", "rep", "exp", "ate_mean_m"]
    with open(out, "w", newline="") as f:
        csv.writer(f).writerow(header)
    print(f"[batch] tag={tag} scenario={scenario} bag={os.path.basename(bag)} "
          f"groups={groups} reps={reps} duration={duration} fault={fault}")

    for gname in groups:
        for rep in range(1, reps + 1):
            t0 = time.time()
            exp = prepare(scenario, is_world=is_world)
            if not replay(exp, bag, duration, robust_gate=(gname == "robust_gate"),
                          fault=fault):
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
    import statistics
    with open(out) as f:
        rows = list(csv.DictReader(f))
    for gname in set(r["config"] for r in rows):
        vals = [float(r["ate_mean_m"]) for r in rows if r["config"] == gname]
        if vals:
            print(f"[batch] {gname}: n={len(vals)} median={statistics.median(vals):.4f} "
                  f"mean={statistics.mean(vals):.4f}")


def main():
    ap = argparse.ArgumentParser(description="P5 批量回放 A/B")
    ap.add_argument("--bag", required=True)
    ap.add_argument("--scenario", required=False,
                    choices=["normal_room", "corridor", "single_plane", "open_ground"])
    ap.add_argument("--world", default=None,
                    help="包内地图 (normal_indoor 等); 与 --scenario 二选一 (scenario 优先)")
    ap.add_argument("--tag", required=True)
    ap.add_argument("--groups", nargs="+", default=["degen", "robust_gate"],
                    choices=["degen", "robust_gate"])
    ap.add_argument("--reps", type=int, default=3)
    ap.add_argument("--duration", type=int, default=60)
    ap.add_argument("--fault", default=None, help="fault config yaml (绝对路径)")
    args = ap.parse_args()

    bag = os.path.abspath(args.bag)
    if not os.path.isdir(bag):
        print(f"[batch] bag 不存在: {bag}")
        sys.exit(1)
    fault = os.path.abspath(args.fault) if args.fault else None
    is_world = bool(not args.scenario and args.world)
    scene = args.world if is_world else args.scenario
    run_batch(bag, scene, args.tag, args.groups, args.reps, args.duration,
              fault=fault, is_world=is_world)


if __name__ == "__main__":
    main()
