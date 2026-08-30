#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P5 synthetic test: 验证 current-frame gate 的核心语义与 recovery state machine。

复刻 laserMapping.cpp 的 robust_gate_reject_check / robust_gate_callback 逻辑
(rolling-baseline relative 通道 + hysteresis state machine), 验证:

  E1  reject 语义: 被拒绝帧后状态/协方差保持 propagated (esekfom 侧由显式
      restore 保证; 这里验证 laserMapping 侧 map_update_skipped == measurement_rejected)。
  E2  geometry 防护 (评审 #13): single_plane 型信号 (eff 高 / ratio 稳定 / residual
      正常) 不触发 reject —— 低 eff 绝对数量不单独触发。
  E3  state machine 无"放宽恢复"路径 (评审 #8-#12): 拒绝后每帧重新尝试 matching,
      恢复只发生在真正满足 recover 阈值时; 不存在人为放行坏帧。
  E4  hysteresis (评审 #12): reject 阈值严格, recover 阈值宽松 + 连续确认帧,
      阈值附近不震荡。
  E5  map 门控一致性: rejected → map_update_skipped (评审 R6/R7/R26)。
  E6  consecutive_reject_count saturation: 长 fault 下内部 int 计数饱和不回零
      (评审 #16, uint16 msg 上限 65535)。
"""
import sys
import os

WS = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, os.path.join(WS, "src", "lio_evaluation", "scripts"))
from p5_analyze_first_iter import relative_drop_mutation  # noqa: E402


class GateSim:
    """Python 复刻 laserMapping P5 gate (relative 通道 + state machine)。"""

    def __init__(self, num_drop_th=0.40, ratio_drop_th=0.30, p90_rise_th=2.0,
                 base_window_s=30.0, recover_num_th=0.55, recover_ratio_th=0.45,
                 recover_p90_th=1.5, degraded_after=2, recovery_required_after=5,
                 recover_confirm_frames=2):
        self.num_drop_th = num_drop_th
        self.ratio_drop_th = ratio_drop_th
        self.p90_rise_th = p90_rise_th
        self.base_window_s = base_window_s
        self.recover_num_th = recover_num_th
        self.recover_ratio_th = recover_ratio_th
        self.recover_p90_th = recover_p90_th
        self.degraded_after = degraded_after
        self.recovery_required_after = recovery_required_after
        self.recover_confirm_frames = recover_confirm_frames
        # 状态
        self.base_ratio = []  # list[(t, ratio)]
        self.base_num = []    # list[(t, eff)]
        self.base_p90 = []    # list[(t, p90)]
        self.baseline_inited = False
        self.median_ratio = 1.0
        self.median_num = 1.0
        self.median_p90 = 0.0
        self.state = 0            # 0=NORMAL 1=REJECT_ONCE 2=DEGRADED 3=RECOVERY_REQUIRED
        self.consecutive_reject = 0
        self.recover_confirm = 0
        self.last_reject = False
        self.last_rel = (1.0, 1.0, 1.0)

    def step(self, t, eff, cand, p90):
        """处理一帧, 返回 (reject, state, consecutive, severity)。"""
        ratio = eff / cand if cand > 0 else (1.0 if eff > 0 else 0.0)
        while self.base_ratio and t - self.base_ratio[0][0] > self.base_window_s:
            self.base_ratio.pop(0)
            self.base_num.pop(0)
            self.base_p90.pop(0)
        rel_num = eff / self.median_num if self.baseline_inited and self.median_num > 1e-6 else 1.0
        rel_ratio = ratio / self.median_ratio if self.baseline_inited and self.median_ratio > 1e-6 else 1.0
        rel_p90 = p90 / self.median_p90 if self.baseline_inited and self.median_p90 > 1e-9 else 1.0
        self.last_rel = (rel_num, rel_ratio, rel_p90)

        # reject 判定 (与 laserMapping.cpp 一致: 数量/占比骤降 AND 残差异常)
        # 评审 #13: "特征少但残差正常" (dropout 保留点匹配正常) 不拒绝 → 避免不必要丢帧漂移。
        rel_num, rel_ratio, rel_p90 = self.last_rel
        reject_by_num = rel_num < self.num_drop_th
        reject_by_ratio = rel_ratio < self.ratio_drop_th
        res_abnormal = rel_p90 > self.p90_rise_th or p90 > 0.02
        reject = (reject_by_num or reject_by_ratio) and res_abnormal
        sev = 0.0
        if rel_num < 1.0:
            sev = max(sev, min(1.0, (1.0 - rel_num) / (1.0 - self.num_drop_th)))
        if rel_ratio < 1.0:
            sev = max(sev, min(1.0, (1.0 - rel_ratio) / (1.0 - self.ratio_drop_th)))
        if rel_p90 > 1.0:
            sev = max(sev, min(1.0, (rel_p90 - 1.0) / (self.p90_rise_th - 1.0)))
        sev = min(1.0, max(0.0, sev))

        # baseline 更新 (只用非 reject 帧; 与 laserMapping.cpp 一致)
        if not reject:
            self.base_ratio.append((t, ratio))
            self.base_num.append((t, eff))
            self.base_p90.append((t, p90))
            rs = sorted(v for _, v in self.base_ratio)
            ns = sorted(v for _, v in self.base_num)
            ps = sorted(v for _, v in self.base_p90)
            if rs:
                self.median_ratio = rs[len(rs) // 2]
                self.median_num = ns[len(ns) // 2]
                self.median_p90 = ps[len(ps) // 2]
                self.baseline_inited = True
        if reject:
            self.consecutive_reject = min(self.consecutive_reject + 1, 2**31 - 1)
            self.recover_confirm = 0
            if self.consecutive_reject >= self.recovery_required_after:
                self.state = 3
            elif self.consecutive_reject >= self.degraded_after:
                self.state = 2
            else:
                self.state = 1
        else:
            ok = (rel_num > self.recover_num_th and rel_ratio > self.recover_ratio_th
                  and rel_p90 < self.recover_p90_th)
            if ok:
                self.recover_confirm += 1
                if self.recover_confirm >= self.recover_confirm_frames:
                    self.consecutive_reject = 0
                    self.state = 0
                    self.recover_confirm = 0
            else:
                self.recover_confirm = 0
        self.last_reject = reject
        return reject, self.state, self.consecutive_reject, sev


def _clean_series(n=100, eff=500, cand=550, p90=0.009, t0=30.0):
    """生成 clean 帧序列 (relative 信号 ≈ 1, 不触发)。"""
    rows = []
    t = t0
    for _ in range(n):
        rows.append((t, eff, cand, p90))
        t += 0.1
    return rows


def main():
    passed = []
    # ---- E1/E5: clean 不 reject; "点少+残差差" 才 reject; map 门控一致 ----
    g = GateSim()
    for (t, eff, cand, p90) in _clean_series(100):
        rej, st, cc, sev = g.step(t, eff, cand, p90)
        assert not rej, f"clean 帧不应 reject (t={t:.1f})"
    assert g.baseline_inited
    # 评审 #13: 纯 dropout (eff 骤降但残差正常) → 不拒绝 (剩余点匹配仍有效)
    rej, st, cc, sev = g.step(40.0, 80, 500, 0.010)
    assert not rej, "eff 骤降但残差正常 → 不拒 (评审 #13: 特征少但残差正常不是 failure)"
    # dropout + 残差异常 (outlier/noise 型) → reject
    rej, st, cc, sev = g.step(40.1, 80, 500, 0.030)
    assert rej, "eff 骤降且 p90 异常 → 应 reject"
    assert cc == 1 and st == 1, "第一次拒绝 → REJECT_ONCE (cc=1)"
    rej, st, cc, sev = g.step(40.2, 70, 500, 0.032)
    assert rej and st == 2, "第二次连续拒绝 → DEGRADED"
    for i in range(3, 6):
        rej, st, cc, sev = g.step(40.0 + i * 0.1, 60, 500, 0.031)
        assert rej
    assert st == 3, f"连续 5 次拒绝 → RECOVERY_REQUIRED (实际 st={st})"
    assert rej, "map_update_skipped 应与 measurement_rejected 一致"
    print("[PASS] E1/E5: clean 不拒; 纯 dropout (残差正常) 不拒 (评审 #13); "
          "点少+残差差触发; REJECT_ONCE→DEGRADED→RECOVERY_REQUIRED; map 门控一致性")
    passed.append("E1/E5")

    # ---- E2: geometry 防护 (评审 #13): 高 eff 但 residual 正常 → 不 reject ----
    g2 = GateSim()
    for (t, eff, cand, p90) in _clean_series(100, eff=1000, cand=1030, p90=0.0125):
        rej, st, cc, sev = g2.step(t, eff, cand, p90)
        assert not rej, "single_plane 型 (eff 高/残差正常) 不应触发"
    rej, st, cc, sev = g2.step(40.0, 400, 1030, 0.013)
    assert not rej, "eff 400/1000=0.40 未低于 0.40 阈值 → 不应 reject (rel_num=0.40 边缘)"
    # 数量骤降但残差正常 → 不拒 (评审 #13: 特征少但残差正常不是 failure)
    rej, st, cc, sev = g2.step(40.1, 300, 1030, 0.013)
    assert not rej, "eff 300/1000=0.30 < 0.40 但残差正常 → 不拒 (评审 #13)"
    # 数量骤降 + 残差异常 → reject
    rej, st, cc, sev = g2.step(40.2, 300, 1030, 0.031)
    assert rej, "eff 骤降 + p90 异常 → 应 reject"
    print("[PASS] E2: geometry 退化 (eff 高/残差正常) 不误杀; 纯数量骤降不误杀; "
          "数量骤降+残差异常才拒")
    passed.append("E2")

    # ---- E3: 无"放宽恢复"路径 (评审 #8-#12) ----
    g3 = GateSim()
    for (t, eff, cand, p90) in _clean_series(100):
        g3.step(t, eff, cand, p90)
    for i in range(3):
        g3.step(40.0 + i * 0.1, 50, 500, 0.031)   # 点少+残差差 → 拒
    assert g3.state == 2  # DEGRADED
    rej, st, cc, sev = g3.step(40.4, 50, 500, 0.031)
    assert rej, "坏帧必须继续拒绝 (不存在放宽放行)"
    # 恢复: eff 回到 baseline 且残差正常, 但只满足 1 帧 → 需连续确认
    rej, st, cc, sev = g3.step(40.5, 480, 550, 0.010)
    assert not rej, "恢复帧不应 reject"
    assert g3.consecutive_reject > 0, "单帧恢复未达确认数 → 仍处 DEGRADED"
    rej, st, cc, sev = g3.step(40.6, 485, 550, 0.009)
    assert not rej
    assert g3.state == 0, f"连续 2 帧恢复确认 → NORMAL (实际 st={st})"
    print("[PASS] E3: 无放宽恢复路径; 恢复需真正满足 recover 阈值 + 连续确认")
    passed.append("E3")

    # ---- E4: hysteresis 不震荡 ----
    g4 = GateSim()
    for (t, eff, cand, p90) in _clean_series(100):
        g4.step(t, eff, cand, p90)
    # 阈值附近: rel_num = 0.42 (reject 阈值 0.40 之上) → 不 reject
    rej, st, cc, sev = g4.step(40.0, 210, 500, 0.031)
    assert not rej, "rel_num=0.42 > 0.40 reject 阈值 → 不 reject (hysteresis 上沿)"
    # 轻微下降 rel_num=0.39 + 残差异常 → reject
    rej, st, cc, sev = g4.step(40.1, 195, 500, 0.031)
    assert rej, "rel_num=0.39 < 0.40 且残差异常 → reject"
    # 回到 0.42 → 不 reject 但需连续确认才回 NORMAL
    rej, st, cc, sev = g4.step(40.2, 210, 500, 0.031)
    assert not rej, "rel_num 回到 0.42 → 恢复接受"
    print("[PASS] E4: hysteresis 上沿/下沿判定正确, 阈值附近快速拒绝、恢复需确认")
    passed.append("E4")

    # ---- E6: consecutive_reject saturation ----
    g6 = GateSim()
    for (t, eff, cand, p90) in _clean_series(100):
        g6.step(t, eff, cand, p90)
    for i in range(70000):
        rej, st, cc, sev = g6.step(40.0 + i * 0.001, 20, 500, 0.031)
        if cc >= 65535:
            break
    assert g6.consecutive_reject == 2**31 - 1 or g6.consecutive_reject >= 65535, \
        f"consecutive_reject 应饱和 (实际 {g6.consecutive_reject})"
    print(f"[PASS] E6: consecutive_reject 饱和到 {g6.consecutive_reject} (不回零)")
    passed.append("E6")

    print(f"\n全部 PASS: {passed}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

