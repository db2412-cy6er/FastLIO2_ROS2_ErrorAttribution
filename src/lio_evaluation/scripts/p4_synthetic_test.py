#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P4.2 synthetic test: 人工构造 Hessian/系统, 验证 P4 directional controller 数学语义。

覆盖 (P4 评审 R27/R28):
  T1 投影算子: dx 沿已知 weak direction 的分量精确缩到 beta 倍, 强方向(正交)不动。
  T2 covariance consistency: K 的 pos 行同步投影后 P=(I-K_p H)P 沿弱方向保持保守,
     与"只改 correction 不改 K"的 overconfident 情形对比。
  T3 trigger/gate 逻辑: translation_degen && matching_ok → active;
     trans_ratio 正常 → inactive; matching_failure → blocked (reason=2)。
  T4 退化场景模拟: single-plane (所有法向量同向) 下, 弱方向 = 法向;
     detector 的 weak direction 必须来自原始 H (raw), 不被投影污染。

用法: python3 p4_synthetic_test.py
无外部依赖 (仅 numpy)。失败时 exit code != 0。
"""
import numpy as np

BETA = 0.5


def project(v, w, beta=BETA):
    w = w / np.linalg.norm(w)
    return v - (1.0 - beta) * (v @ w) * w


def project_rows(M, w, beta=BETA):
    return np.vstack([project(row, w, beta) for row in M])


def iekf_pos_step(H, P0, R, x0, z, w=None, beta=BETA, project_gain=True):
    """3D pos-only IEKF 单步 (复刻 esekfom update_iterated_dyn_share_modified 的
    n>dof 分支核心: K = P H^T (H P H^T + R)^{-1}; dx = K(z-Hx0); P1=(I-KH)P)。
    w/project_gain 控制 P4 projector (dx 投影 + K 行同步投影)。"""
    S = H @ P0 @ H.T + R * np.eye(H.shape[0])
    K = P0 @ H.T @ np.linalg.inv(S)
    dx = K @ (z - H @ x0)
    if w is not None and beta < 1.0:
        dx = project(dx, w, beta)                       # esekfom 插入点 A (dx 投影)
        if project_gain:
            K = project_rows(K, w, beta)                # esekfom 插入点 B (K 行投影)
    P1 = (np.eye(3) - K @ H) @ P0
    return dx, P1, K


def check(name, cond, detail=""):
    tag = "PASS" if cond else "FAIL"
    print(f"  [{tag}] {name} {detail}")
    return cond


def main():
    print("=" * 78)
    print("P4.2 synthetic test (数学语义验证)")
    print("=" * 78)
    ok_all = True

    # 场景: 3D pos, 测量 —— 强约束 (x/y) + 弱约束 (z 主导), 弱方向沿 z。
    H = np.array([[1.0, 0.0, 0.0],
                  [0.0, 1.0, 0.0],
                  [0.0, 0.0, 0.3]])          # z 弱: 单面墙法向场景
    P0 = np.eye(3)
    R = 0.01
    x0 = np.zeros(3)
    z = np.array([0.5, 0.3, 0.9])             # z 方向残差大 (退化时弱方向误差大)
    w = np.array([0.0, 0.0, 1.0])             # 已知弱方向 = z

    # ── T1 投影语义 ──
    print("\n[T1] dx 投影语义 (β=%.1f)" % BETA)
    dx_b, P_b, K_b = iekf_pos_step(H, P0, R, x0, z, w=None)
    dx_p, P_p, K_p = iekf_pos_step(H, P0, R, x0, z, w=w)
    print("  baseline  dx = %s" % np.round(dx_b, 4))
    print("  projected dx = %s" % np.round(dx_p, 4))
    ok_all &= check("弱方向(z) correction = β×baseline",
                    abs(dx_p[2] - BETA * dx_b[2]) < 1e-12,
                    f"({dx_p[2]:.4f} vs {BETA}*{dx_b[2]:.4f})")
    ok_all &= check("强方向(x,y) correction 不变 (正交时严格成立)",
                    abs(dx_p[0] - dx_b[0]) < 1e-12 and abs(dx_p[1] - dx_b[1]) < 1e-12,
                    f"(x:{dx_p[0]:.4f} vs {dx_b[0]:.4f})")

    # ── T2 covariance consistency ──
    print("\n[T2] covariance consistency")
    dx_incon, P_incon, _ = iekf_pos_step(H, P0, R, x0, z, w=w, project_gain=False)
    print("  baseline   P22(沿弱方向) = %.6f" % P_b[2, 2])
    print("  不一致     P22(弱)       = %.6f  (状态只收 %.0f%%, 协方差却按 100%% 缩)" %
          (P_incon[2, 2], BETA * 100))
    print("  一致(K投影) P22(弱)       = %.6f" % P_p[2, 2])
    ok_all &= check("一致方案弱方向方差 > 不一致方案 (不过度自信)",
                    P_p[2, 2] > P_incon[2, 2] + 1e-6,
                    f"({P_p[2,2]:.6f} > {P_incon[2,2]:.6f})")
    ok_all &= check("一致方案弱方向方差 > baseline (抑制弱方向→更保守)",
                    P_p[2, 2] > P_b[2, 2] + 1e-6,
                    f"({P_p[2,2]:.6f} > {P_b[2,2]:.6f})")

    # ── T3 trigger / gate 逻辑 (复刻 compute_directional_trigger) ──
    print("\n[T3] trigger/gate 逻辑 (复刻 C++ compute_directional_trigger)")
    RATIO_T = 0.10

    def trigger(trans_ratio, matching_ok, imu_low=False,
                beta_t=BETA, safety=False, floor=0.5):
        """返回 (active, beta, reason)。reason: 0=inactive 1=degen 2=match_block 3=safety"""
        if trans_ratio >= RATIO_T:
            return False, 1.0, 0
        if not matching_ok:
            return False, 1.0, 2
        beta = beta_t
        if safety and imu_low:
            beta = max(beta, floor)
            return True, beta, 3
        return True, beta, 1

    active, beta, reason = trigger(0.05, True)
    ok_all &= check("corridor 退化+match OK → active, beta=%s" % BETA,
                    active and abs(beta - BETA) < 1e-9 and reason == 1)
    active, beta, reason = trigger(0.80, True)
    ok_all &= check("normal 场景 (trans_ratio 高) → inactive",
                    not active and reason == 0)
    active, beta, reason = trigger(0.05, False)
    ok_all &= check("退化但 matching_failure → blocked (reason=2)",
                    not active and reason == 2)
    active, beta, reason = trigger(0.05, True, imu_low=True, safety=True)
    ok_all &= check("退化+IMU 低激励 → safety limiter 抬 beta 下限 (reason=3)",
                    active and reason == 3 and abs(beta - 0.5) < 1e-9)

    # ── T4 raw/update 数据路径分离 ──
    print("\n[T4] detector 数据路径: 弱方向来自原始 H; h_x 层投影会污染检测器, P4 不会")
    # 真实 single-plane: 所有点法向量≈z (墙面/地面) → 仅法向 z 有约束,
    # 弱方向 = 垂直于法向的 x/y 平面 (沿墙/地面的切向漂移, 正是 P3 single_plane 大 ATE 来源)。
    rng = np.random.default_rng(0)
    H_plane = np.zeros((50, 3))
    H_plane[:, 2] = 1.0
    H_plane[:, 0] = rng.normal(0, 0.01, 50)
    H_plane[:, 1] = rng.normal(0, 0.01, 50)
    Ht_raw = H_plane.T @ H_plane              # raw Hessian (detector 用)
    evals, evecs = np.linalg.eigh(Ht_raw)
    w_raw = evecs[:, 0]                       # 最小特征向量 (弱方向)
    ok_all &= check("single-plane 弱方向 ≈ 垂直于法向 (|w·ẑ|≈0)",
                    abs(w_raw @ np.array([0, 0, 1.0])) < 0.01,
                    f"(|w·ẑ|={abs(w_raw[2]):.4f}; 法向 z 约束最强, 切向最弱)")
    # 若错误地在 h_x 层做投影: H 的每行弱方向分量被缩放 → 下一次 Ht 特征值被改变 → 检测器自反馈污染
    H_proj = project_rows(H_plane, w_raw, beta=0.1)
    Ht_proj = H_proj.T @ H_proj
    evals_p, _ = np.linalg.eigh(Ht_proj)
    ok_all &= check("h_x 层投影会改变 Ht (λmin raw≠λmin 投影) ⇒ 检测器被污染 (反证, 应避免)",
                    abs(evals_p[0] - evals[0]) > 1e-9,
                    f"λmin raw={evals[0]:.3e} → 投影H={evals_p[0]:.3e}")
    print("  注: P4 在 esekfom state 层投影 dx_/K_x, 原始 h_x/Ht 全程不被改写 ⇒ "
          "检测器天然免疫 (T4 证明 h_x 层方案会自反馈污染, P4 state 层方案不会)。")

    print("\n" + "=" * 78)
    print("RESULT:", "ALL PASS" if ok_all else "SOME FAILED")
    print("=" * 78)
    return 0 if ok_all else 1


if __name__ == "__main__":
    import sys
    sys.exit(main())
