#!/usr/bin/env python3
"""보고서 그림 생성 → docs/report/fig/*.png  (.venv/bin/python docs/report/make_figs.py)"""
import sys, math, subprocess
from pathlib import Path
import numpy as np, pandas as pd, yaml
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt; import matplotlib.image as mpimg

ROOT = Path(__file__).resolve().parents[2]; FIG = Path(__file__).resolve().parent / "fig"; FIG.mkdir(exist_ok=True)
sys.path.insert(0, str(ROOT / "isaac")); from bench_metrics import chirp_frf, PALETTE, CHIRP_F1  # noqa: E402
BENCH = ROOT / "data/sim/bench"
plt.rcParams.update({"font.family": ["DejaVu Sans", "Noto Sans CJK KR"], "font.size": 9, "axes.titlesize": 10, "axes.labelsize": 9, "legend.fontsize": 8,
                     "axes.spines.top": False, "axes.spines.right": False, "axes.grid": True, "grid.alpha": 0.25, "grid.linewidth": 0.6})
GREY = "#888"; INK = "#333"
LABEL = {"dm4310": "DM4310 (M)", "dm4340": "DM4340 (M)", "dm8009": "DM8009 (M)", "rs00_wrist": "RS00", "rs02_elbow": "RS02", "rs03_shoulder": "RS03",
         "mit_shoulder": "MIT U10", "mit_elbow": "MIT U10 (belt)", "bh_5013": "Berkeley 5013", "bh_8513": "Berkeley 8513", "bhl_5010_arm": "BH Lite 5010",
         "g1_shoulder": "G1 shoulder", "g1_elbow": "G1 elbow", "g1_wrist_roll": "G1 wrist roll", "g1_wrist_pitch": "G1 wrist pitch", "h1_shoulder": "H1 shoulder", "h1_elbow": "H1 elbow", "h12_wrist": "H1-2 wrist"}
CLASSES = ["wrist", "elbow", "shoulder"]
M = pd.read_csv(BENCH / "metrics.csv")


def style(p, i):
    return dict(color="#111" if p.startswith("dm") else PALETTE[i % len(PALETTE)], lw=2.6 if p.startswith("dm") else 1.3, label=LABEL.get(p, p), zorder=5 if p.startswith("dm") else 3)


# ---- 1. 클래스별 오버레이 (step / bode / multisine err / free decay)
def fig_bench(cls):
    sub = M[M.cls == cls]; rig = sub.iloc[0]
    fig, ax = plt.subplots(2, 2, figsize=(9.2, 6.4)); ax = ax.ravel()
    for i, (_, r) in enumerate(sub.iterrows()):
        p = r.preset; d = BENCH / cls / p; kw = style(p, i)
        meta = yaml.safe_load((d / "meta.yaml").read_text())
        s = pd.read_csv(d / "step.csv"); m = (s.t_s >= 1.8) & (s.t_s <= 4.0); ax[0].plot(s.t_s[m] - 2, np.degrees(s.q[m]), **kw)
        if i == 0: ax[0].plot(s.t_s[m] - 2, np.degrees(s.q_des[m]), color=GREY, lw=1, ls="--", label="q_des", zorder=1)
        c = pd.read_csv(d / "chirp.csv"); fs, mag, ph, _ = chirp_frf(c, meta["proto"]["chirp"]["f0"], meta["rig"]["chirp_f1"], meta["proto"]["chirp"]["dur"])
        ax[1].plot(fs, 20 * np.log10(np.maximum(mag, 1e-3)), **kw)
        ms = pd.read_csv(d / "multisine.csv"); m = (ms.t_s >= 6) & (ms.t_s <= 10); ax[2].plot(ms.t_s[m], np.degrees(ms.q_des[m] - ms.q[m]), **kw)
        fd = pd.read_csv(d / "free_decay.csv"); fr = fd[fd.phase == "free"]; ax[3].plot(fr.t_s - fr.t_s.iloc[0], np.degrees(fr.q), **kw)
    ax[0].set(title="(a) Step +0.3 rad", xlabel="t [s]", ylabel="q [deg]")
    ax[1].set(title="(b) Closed-loop |q/q_des|, position chirp 0.1 rad", xlabel="f [Hz]", ylabel="[dB]", xscale="log"); ax[1].axhline(-3, color=GREY, lw=1, ls="--")
    f1 = CHIRP_F1[cls]; ticks = [t for t in [0.3, 0.5, 1, 2, 3, 5, 10, 20, 40] if 0.25 <= t <= f1]; ax[1].set_xticks(ticks); ax[1].set_xticklabels([f"{t:g}" for t in ticks]); ax[1].minorticks_off()
    ax[2].set(title="(c) Multisine tracking error (6–10 s)", xlabel="t [s]", ylabel="q_des − q [deg]")
    ax[3].set(title="(d) Free decay after release, kp=kd=0" + (" (spin-down 2 rad/s)" if cls == "elbow" else ""), xlabel="t since release [s]", ylabel="q [deg]")
    h, l = ax[0].get_legend_handles_labels(); fig.legend(h, l, loc="lower center", ncol=min(len(l), 5), frameon=False, bbox_to_anchor=(0.5, -0.01))
    fig.suptitle(f"{cls} class rig: I_link {rig.I_link:.4g} kg·m², m·g·d {rig.mgd:.3g} Nm, kp {rig.kp:g} / kd {rig.kd:g}", fontsize=10, color=INK)
    fig.tight_layout(rect=(0, 0.06, 1, 0.97)); fig.savefig(FIG / f"bench_{cls}.png", dpi=200); plt.close(fig)


# ---- 2. 정규화 지표 막대 (클래스 × 지표)
def fig_bars():
    mets = [("bw_3db_hz", "−3 dB bandwidth [Hz]", False), ("ms_track_rms_deg", "Multisine tracking RMS [deg]", True), ("tau_over_Jtot", "τ_pk / (armature + I_link) [rad/s²]", False)]
    fig, axes = plt.subplots(3, 3, figsize=(9.2, 7.2))
    for r_, cls in enumerate(CLASSES):
        sub = M[M.cls == cls].copy()
        for c_, (k, title, lower_better) in enumerate(mets):
            ax = axes[r_, c_]; v = sub[k].copy(); capped = v.isna() & (k == "bw_3db_hz"); v = v.fillna(CHIRP_F1[cls] if k == "bw_3db_hz" else 0)
            order = np.argsort(-v.values if not lower_better else v.values); names = sub.preset.values[order]; vals = v.values[order]; cp = capped.values[order]
            cols = ["#111" if n.startswith("dm") else "#a9b4c2" for n in names]
            ax.barh(range(len(names)), vals, color=cols, height=0.62)
            ax.set_yticks(range(len(names))); ax.set_yticklabels([LABEL.get(n, n) for n in names]); ax.invert_yaxis(); ax.grid(axis="y", alpha=0)
            for i, (val, cap) in enumerate(zip(vals, cp)):
                ax.text(val, i, ("  > " if cap else "  ") + (f"{val:.1f}" if val < 100 else f"{val:.0f}"), va="center", fontsize=7.5, color=INK)
            ax.set_xlim(0, max(vals) * 1.28); ax.tick_params(axis="y", labelsize=7.5)
            if r_ == 0: ax.set_title(title + ("  (lower is better)" if lower_better else ""), fontsize=9)
            if c_ == 0: ax.set_ylabel(cls, fontsize=10, fontweight="bold")
    fig.suptitle("Same rig and same PD within each class. Black: measured DM (M). Grey: spec-based (D/S)", fontsize=10, color=INK)
    fig.tight_layout(rect=(0, 0, 1, 0.97)); fig.savefig(FIG / "bench_bars.png", dpi=200); plt.close(fig)


# ---- 3. 마찰 민감도 (추종 RMS, 3 조건)
def fig_sensitivity():
    conds = [("원래 (DM만 실측 마찰)", BENCH), ("전 프리셋 마찰 0", ROOT / "data/sim/bench_fric_zero"), ("Coulomb 하한 2 %·τpk", ROOT / "data/sim/bench_fric_floor02")]
    data = [(n, pd.read_csv(p / "metrics.csv")) for n, p in conds if (p / "metrics.csv").exists()]
    fig, axes = plt.subplots(1, 3, figsize=(9.2, 3.4))
    mk = ["o", "s", "^"]; cc = ["#111", "#2a78d6", "#eb6834"]
    for ax, cls in zip(axes, CLASSES):
        base = data[0][1]; sub = base[base.cls == cls].sort_values("ms_track_rms_deg"); names = sub.preset.values
        for j, (n, d) in enumerate(data):
            dd = d[d.cls == cls].set_index("preset").reindex(names)
            ax.plot(dd.ms_track_rms_deg.values, range(len(names)), mk[j], color=cc[j], ms=6, label=n, alpha=0.9, zorder=3 + j)
        ax.set_yticks(range(len(names))); ax.set_yticklabels([LABEL.get(n, n) for n in names], fontsize=7.5); ax.invert_yaxis()
        for i, n in enumerate(names):
            if n.startswith("dm"): ax.axhspan(i - 0.4, i + 0.4, color="#eee", zorder=0)
        ax.set_title(cls, fontsize=10); ax.set_xlabel("Multisine tracking RMS [deg]"); ax.grid(axis="y", alpha=0); ax.set_xlim(left=0)
    h, l = axes[0].get_legend_handles_labels(); fig.legend(h, l, loc="lower center", ncol=3, frameon=False, fontsize=8, bbox_to_anchor=(0.5, -0.02))
    fig.suptitle("Friction sensitivity. Shaded rows: measured DM. Bandwidth ranking is unchanged in all three conditions", fontsize=9.5, color=INK)
    fig.tight_layout(rect=(0, 0.07, 1, 0.93)); fig.savefig(FIG / "sensitivity.png", dpi=200); plt.close(fig)


# ---- 4. 마찰 피팅 몽타주 (기존 png)
def fig_fit_montage():
    srcs = [("DM4310 · joint7", ROOT / "data/2026-09-21_j7/fit/fit_friction.png"), ("DM4340 · joint3", ROOT / "data/2026-09-22_j3/fit/fit_friction.png"), ("DM8009 · joint1 (hold)", ROOT / "data/2026-09-22_j1c/fit/fit_friction.png")]
    fig, axes = plt.subplots(1, 3, figsize=(9.6, 2.9))
    for ax, (t, p) in zip(axes, srcs): ax.imshow(mpimg.imread(p)); ax.set_title(t, fontsize=9); ax.axis("off")
    fig.tight_layout(); fig.savefig(FIG / "fit_friction.png", dpi=220); plt.close(fig)


# ---- 5. 파이프라인 도식 (graphviz)
def fig_pipeline():
    dot = r'''digraph G { rankdir=TB; nodesep=0.35; ranksep=0.35; node [shape=box, style="rounded,filled", fillcolor="#f4f4f2", color="#777", fontname="Noto Sans CJK KR", fontsize=10, margin="0.12,0.06"]; edge [color="#555", fontname="Noto Sans CJK KR", fontsize=8.5];
  real [label="실물 OpenArm\n3축 (DM4310/4340/8009)\nMIT 모드 직접 명령 500 Hz", fillcolor="#e3eefb"];
  rec [label="candump + cmd.csv\nq, dq, τ(전류 추정), T_MOS"];
  fit [label="2단계 식별\n① const_vel → Coulomb·점성·중력\n② chirp → J = armature + I_link"];
  cfg [label="DCMotorCfg ×3\n(effort, sat, v_max, armature,\nfriction 3항)", fillcolor="#e3eefb"];
  rig [label="Isaac 단일 관절 리그\n등가 막대 (I_link, m·g·d)\n유령질량 보정 질량행렬"];
  res [label="sim2real 잔차\n홀드아웃 multisine 재생\nΔq ≤ 2°, Δτ ≤ 10 % 정격", fillcolor="#e8f5ec"];
  spec [label="타사 사양 → 프리셋 15개\nRobstride · Unitree · MIT\nBerkeley · BH Lite (D/S)", fillcolor="#fbeee6"];
  bench [label="공통 벤치\n클래스별 동일 리그·동일 PD\nstep · chirp · multisine · free_decay", fillcolor="#e8f5ec"];
  out [label="정규화 지표 표\n+ 마찰 민감도\n+ 잔차 오차 막대"];
  real -> rec -> fit -> cfg -> rig -> res; cfg -> bench; spec -> bench; rig -> bench; bench -> out; res -> out [label=" 오차 막대"];
  {rank=same; cfg; spec;} {rank=same; res; bench;}
}'''
    subprocess.run(["dot", "-Tpng", "-Gdpi=200", "-o", str(FIG / "pipeline.png")], input=dot.encode(), check=True)


if __name__ == "__main__":
    for c in CLASSES: fig_bench(c)
    fig_bars(); fig_sensitivity(); fig_fit_montage(); fig_pipeline()
    print("→", sorted(p.name for p in FIG.glob("*.png")))
