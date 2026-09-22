#!/usr/bin/env python3
"""단일 관절 시스템 식별: τ = J·α + b·ω + c·sign(ω) + g_s·sin(q) + g_c·cos(q) + τ0  최소제곱.

    fit_params.py <dir1> [<dir2> ...] [--no-gravity] [--link-inertia I] [--cutoff 30] [--out fit/]

입력 디렉터리: excite.py 출력(cmd.csv, meta.yaml) 또는 isaac/single_joint_rig.py 출력(sim.csv, meta.yaml).
여러 디렉터리(예: const_vel + chirp)를 주면 합쳐서 한 번에 피팅하고, 개별 피팅도 같이 보고한다.

출력: <out>/fit.yaml  파라미터·표준오차·R²·잔차 RMS, Isaac DCMotorCfg 매핑(armature = J − I_link, dynamic_friction = c, viscous_friction = b)
      <out>/fit_timeseries.png, <out>/fit_friction.png
"""
import argparse, math
from pathlib import Path

import numpy as np
import pandas as pd
import yaml
from scipy.signal import butter, filtfilt

EXCLUDE_PHASES = {"goto", "pre_hold"}


def load(d: Path):
    meta = yaml.safe_load((d / "meta.yaml").read_text()) if (d / "meta.yaml").exists() else {}
    if (d / "sim.csv").exists():
        df = pd.read_csv(d / "sim.csv"); df = df.rename(columns={"tau_applied": "tau"}); hz = meta.get("hz", 500)
        df["phase"] = "sim"
    else:
        df = pd.read_csv(d / "cmd.csv"); hz = meta.get("loop_hz", 500)
        df = df[~df["phase"].isin(EXCLUDE_PHASES)].reset_index(drop=True)
    df["t"] = np.arange(len(df)) / hz
    return df, meta, hz


def preprocess(df, hz, cutoff):
    """필터는 α(수치 미분)에만 적용. q/dq/τ 는 원본 — sgn(ω) 의 스텝을 뭉개지 않기 위해."""
    b, a = butter(4, cutoff / (hz / 2))
    dq_f = filtfilt(b, a, df["dq"].values); alpha = np.gradient(dq_f, 1.0 / hz)
    return df["q"].values, df["dq"].values, alpha, df["tau"].values


GCOS = True  # 좁은 범위(|q|<0.5)에서는 cos(q)≈상수 → tau0 와 공선. --no-gcos 로 끔


def regress(q, dq, alpha, tau, gravity=True, vel_deadband=0.05, huber_delta=None, alpha_max=None):
    """|ω| < deadband 샘플 제외(정지 마찰 모호 구간). alpha_max 지정 시 |α| > alpha_max 도 제외(정속 plateau 만).
    huber_delta 가 주어지면 IRLS 로버스트 회귀."""
    keep = np.abs(dq) >= vel_deadband
    if alpha_max is not None: keep &= np.abs(alpha) <= alpha_max
    q, dq, alpha, tau = q[keep], dq[keep], alpha[keep], tau[keep]
    sgn = np.sign(dq)
    cols = {"J": alpha, "b": dq, "c": sgn}
    if gravity:
        cols["g_sin"] = np.sin(q)
        if GCOS: cols["g_cos"] = np.cos(q)
    cols["tau0"] = np.ones_like(q)
    X = np.column_stack(list(cols.values())); names = list(cols.keys())
    theta, *_ = np.linalg.lstsq(X, tau, rcond=None)
    if huber_delta:  # IRLS: 잔차가 delta 보다 크면 가중치 delta/|r|
        for _ in range(10):
            r = tau - X @ theta; w = np.minimum(1.0, huber_delta / np.maximum(np.abs(r), 1e-12))
            Xw = X * w[:, None]; theta_new = np.linalg.lstsq(Xw.T @ X, Xw.T @ tau, rcond=None)[0]
            if np.allclose(theta, theta_new, rtol=1e-6): theta = theta_new; break
            theta = theta_new
    res = tau - X @ theta; n, p = X.shape
    sigma2 = res @ res / max(n - p, 1)
    cov = sigma2 * np.linalg.pinv(X.T @ X); se = np.sqrt(np.diag(cov))
    r2 = 1 - (res @ res) / max(((tau - tau.mean()) ** 2).sum(), 1e-12)
    return dict(zip(names, theta.tolist())), dict(zip(names, se.tolist())), float(np.sqrt(sigma2)), float(r2), (res, keep)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dirs", nargs="+"); ap.add_argument("--no-gravity", action="store_true")
    ap.add_argument("--link-inertia", type=float, default=None, help="링크 관성 (관절 기준, kg m²). 없으면 meta.link_inertia_joint")
    ap.add_argument("--cutoff", type=float, default=30.0, help="α 계산용 저역 필터 차단 주파수 Hz"); ap.add_argument("--out", default=None)
    ap.add_argument("--deadband", type=float, default=0.05, help="|ω| 이하 샘플 제외 (rad/s)")
    ap.add_argument("--huber", type=float, default=None, help="Huber 델타 (Nm). 지정 시 로버스트 회귀")
    ap.add_argument("--alpha-max", type=float, default=3.0, help="1단계(마찰) 피팅에 쓰는 정속 plateau 판정 |α| 상한 (rad/s²)")
    ap.add_argument("--no-gcos", action="store_true", help="cos(q) 중력항 제외 (좁은 범위 관절: tau0 와 공선)")
    ap.add_argument("--chirp-fmax", type=float, default=4.0, help="2단계(J) 에 쓰는 처프 최고 주파수 Hz. 그 이상은 백래시 영역이라 J 과소추정 (joint7 실측)")
    a = ap.parse_args()
    global GCOS; GCOS = not a.no_gcos
    dirs = [Path(d) for d in a.dirs]; out = Path(a.out or dirs[0].parent / ("fit_" + "+".join(d.name for d in dirs))); out.mkdir(parents=True, exist_ok=True)

    sets, metas = [], []
    for d in dirs:
        df, meta, hz = load(d); q, dq, al, tau = preprocess(df, hz, a.cutoff)
        sets.append((d.name, q, dq, al, tau, df["t"].values)); metas.append(meta)
    gravity = not a.no_gravity and not any(m.get("gravity_axis") for m in metas)

    report = {"inputs": [str(d) for d in dirs], "gravity_terms": gravity, "cutoff_hz": a.cutoff, "deadband": a.deadband, "huber": a.huber, "per_dataset": {}, "joint": {}}
    for name, q, dq, al, tau, _ in sets:
        th, se, rms, r2, _ = regress(q, dq, al, tau, gravity, a.deadband, a.huber)
        report["per_dataset"][name] = {"params": th, "se": se, "rms_resid_Nm": rms, "r2": r2, "n": int(len(q))}
    Q, DQ, AL, TAU = (np.concatenate([s[i] for s in sets]) for i in (1, 2, 3, 4))
    only_cv = all(m.get("mode") in ("const_vel", "hold") for m in metas)
    th, se, rms, r2, (res, keep) = regress(Q, DQ, AL, TAU, gravity, a.deadband, a.huber, a.alpha_max if only_cv else None)
    report["joint"] = {"params": th, "se": se, "rms_resid_Nm": rms, "r2": r2, "n": int(keep.sum())}

    # ---- 2단계: 마찰·중력은 저주파(const_vel) 세트에서, J 는 처프 세트에서 마찰 고정 후 추정.
    # 고주파 속도 반전 구간에서는 Coulomb 마찰이 스틱션에 머물러 과소추정되므로 (합성 검증: 20 Hz 처프 c −16 %).
    modes = [m.get("mode", "") for m in metas]
    lo = [i for i, m in enumerate(modes) if m in ("const_vel", "hold")]; hi = [i for i, m in enumerate(modes) if m == "chirp"]
    if lo and hi:
        cat = lambda idx, k: np.concatenate([sets[i][k] for i in idx])
        th1, se1, rms1, r21, (_, k1) = regress(cat(lo, 1), cat(lo, 2), cat(lo, 3), cat(lo, 4), gravity, a.deadband, a.huber, a.alpha_max)
        # 처프 세트: f ≤ chirp_fmax 구간만 사용 (지수 처프 f(t)=f0·exp(ln(f1/f0)·t/dur))
        def band_mask(i):
            m = metas[i]; ar = m.get("args", {}); n = len(sets[i][1])
            if "f0" not in ar: return np.ones(n, bool)
            f0, f1, dur = ar["f0"], ar["f1"], ar["dur"]; tt = np.arange(n) / hz
            return f0 * np.exp(np.log(f1 / f0) * tt / dur) <= a.chirp_fmax
        bm = np.concatenate([band_mask(i) for i in hi])
        q2, dq2, al2, tau2 = (cat(hi, k)[bm] for k in (1, 2, 3, 4))
        fric = th1["b"] * dq2 + th1["c"] * np.sign(dq2) + (th1["g_sin"] * np.sin(q2) + th1.get("g_cos", 0.0) * np.cos(q2) if gravity else 0)
        k2 = np.abs(dq2) >= a.deadband; X2 = np.column_stack([al2[k2], np.ones(k2.sum())]); y2 = (tau2 - fric)[k2]
        (J2, t02), *_ = np.linalg.lstsq(X2, y2, rcond=None); r2res = y2 - X2 @ [J2, t02]
        seJ = float(np.sqrt(r2res @ r2res / max(len(y2) - 2, 1) * np.linalg.pinv(X2.T @ X2)[0, 0]))
        th = dict(th1); th["J"] = float(J2); th["tau0"] = float(t02); se = dict(se1); se["J"] = seJ
        report["two_stage"] = {"stage1_sets": [sets[i][0] for i in lo], "stage2_sets": [sets[i][0] for i in hi],
                               "params": th, "se": se, "stage1_r2": r21, "stage1_rms_Nm": rms1, "stage1_kept_frac": float(k1.mean()), "alpha_max": a.alpha_max, "chirp_fmax": a.chirp_fmax, "stage2_n": int(k2.sum()),
                               "stage2_rms_Nm": float(np.sqrt(r2res @ r2res / len(y2)))}
        print(f"[two-stage] 마찰·중력 ← {','.join(report['two_stage']['stage1_sets'])} (plateau {k1.mean()*100:.0f}% 사용) / J ← {','.join(report['two_stage']['stage2_sets'])}")

    I_link = a.link_inertia if a.link_inertia is not None else metas[0].get("link_inertia_joint")
    isaac = {"armature": (th["J"] - I_link) if I_link is not None else None, "link_inertia_used": I_link,
             "dynamic_friction": abs(th["c"]), "friction": abs(th["c"]), "viscous_friction": th["b"]}
    if gravity:
        isaac["gravity_check"] = {"m_g_d_fit_Nm": float(math.hypot(th["g_sin"], th.get("g_cos", 0.0))),
                                  "m_g_d_meta_Nm": (metas[0]["mass"] * 9.81 * metas[0]["com_dist"]) if "mass" in metas[0] else None}
    report["isaac_dcmotor_cfg"] = isaac
    truth = metas[0].get("actuator_cfg")
    if truth:
        report["truth_from_meta"] = truth
    (out / "fit.yaml").write_text(yaml.safe_dump(report, sort_keys=False, allow_unicode=True))

    # ---- 플롯
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    fig, axs = plt.subplots(len(sets), 1, figsize=(11, 3 * len(sets)), squeeze=False)
    def model(q, dq, al):
        m = th["J"] * al + th["b"] * dq + th["c"] * np.sign(dq) + th["tau0"]
        return m + (th["g_sin"] * np.sin(q) + th.get("g_cos", 0.0) * np.cos(q) if gravity else 0)
    for ax, (name, q, dq, al, tau, t) in zip(axs[:, 0], sets):
        pred = model(q, dq, al)
        ax.plot(t, tau, lw=0.8, label="τ measured"); ax.plot(t, pred, lw=0.8, ls="--", label="τ model"); ax.set_title(name); ax.set_ylabel("Nm"); ax.legend(loc="upper right")
    axs[-1, 0].set_xlabel("t [s]"); fig.tight_layout(); fig.savefig(out / "fit_timeseries.png", dpi=110)
    fig, ax = plt.subplots(figsize=(7, 5))
    fr = TAU - th["J"] * AL - (th["g_sin"] * np.sin(Q) + th.get("g_cos", 0.0) * np.cos(Q) if gravity else 0) - th["tau0"]
    ax.scatter(DQ[keep], fr[keep], s=2, alpha=0.3, label="τ − Jα − g(q) − τ0"); w = np.linspace(DQ.min(), DQ.max(), 400)
    ax.plot(w, th["b"] * w + th["c"] * np.sign(w), "r", label=f"b·ω + c·sgn(ω)  (b={th['b']:.4f}, c={th['c']:.4f})")
    ax.set_xlabel("ω [rad/s]"); ax.set_ylabel("friction torque [Nm]"); ax.legend(); ax.grid(alpha=0.3); fig.tight_layout(); fig.savefig(out / "fit_friction.png", dpi=110)

    print(f"→ {out}/fit.yaml")
    print(f"joint fit (n={len(Q)}): R²={r2:.4f}  RMS={rms:.4f} Nm")
    for k in th: print(f"  {k:6s} = {th[k]:+.5f} ± {se[k]:.5f}")
    print("isaac cfg:", {k: (round(v, 5) if isinstance(v, float) else v) for k, v in isaac.items() if not isinstance(v, dict)})
    if gravity and isaac.get("gravity_check"): print("gravity  :", isaac["gravity_check"])
    if truth: print("truth    :", {k: truth[k] for k in ("armature", "dynamic_friction", "viscous_friction") if k in truth})


if __name__ == "__main__":
    main()
