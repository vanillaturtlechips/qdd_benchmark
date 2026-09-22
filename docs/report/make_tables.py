#!/usr/bin/env python3
"""보고서 표 생성 → docs/report/tables/*.md  (숫자는 전부 데이터 파일에서 읽음. 손으로 옮겨 적지 않는다)"""
import math
from pathlib import Path
import numpy as np, pandas as pd, yaml

ROOT = Path(__file__).resolve().parents[2]; T = Path(__file__).resolve().parent / "tables"; T.mkdir(exist_ok=True)
SPECS = yaml.safe_load((ROOT / "isaac/specs.yaml").read_text())
IDENT = {"DM4310": ("data/2026-09-21_j7/identified_dm4310.yaml", "joint7 (손목)", "replay_dm4310_j7_Ifix", "0.3~2.9 Hz"),
         "DM4340": ("data/2026-09-22_j3/identified_dm4340.yaml", "joint3 (팔꿈치 롤)", "replay_dm4340_j3", "0.3~2.9 Hz"),
         "DM8009": ("data/2026-09-22_j1/identified_dm8009.yaml", "joint1 (어깨)", "replay_dm8009_j1_lowf_Ifix", "≤1.5 Hz")}
ARM_SRC = {"DM4310": "펌웨어†", "DM4340": "**처프 실측**", "DM8009": "펌웨어‡"}
LABEL = {"dm4310": "DM4310", "dm4340": "DM4340", "dm8009": "DM8009", "rs00_wrist": "RS00", "rs02_elbow": "RS02", "rs03_shoulder": "RS03", "mit_shoulder": "MIT U10", "mit_elbow": "MIT U10 (belt)",
         "bh_5013": "Berkeley 5013", "bh_8513": "Berkeley 8513", "bhl_5010_arm": "BH Lite 5010", "g1_shoulder": "G1 shoulder", "g1_elbow": "G1 elbow", "g1_wrist_roll": "G1 wrist roll",
         "g1_wrist_pitch": "G1 wrist pitch", "h1_shoulder": "H1 shoulder", "h1_elbow": "H1 elbow", "h12_wrist": "H1-2 wrist"}
GRADE = {"dm": "M", "rs": "D+S", "mit": "M(dyno)", "bh_": "D", "bhl": "S", "g1": "D+S", "h1": "D", "h12": "D"}
CHIRP_F1 = {"wrist": 20, "elbow": 40, "shoulder": 8}


def grade(p):
    for k, v in GRADE.items():
        if p.startswith(k): return v
    return "?"


def md(rows, hdr, left=1):
    """첫 left 개 열은 왼쪽, 나머지는 오른쪽 정렬 (숫자 열)."""
    al = "|" + "".join(":---|" if i < left else "---:|" for i in range(len(hdr)))
    return "\n".join(["| " + " | ".join(hdr) + " |", al] + ["| " + " | ".join(str(c) for c in r) + " |" for r in rows]) + "\n"


def pm(d, k, nd=3):
    v = d["fit"].get(k)
    return "–" if v is None else f"{v['value']:.{nd}g} ± {v['se']:.1g}"


# T1 식별 파라미터. I_link 은 link_inertia.yaml(hand 0.127 kg 보정 후) 기준. DM8009 는 hold 조건(j1c) 피팅을 채택
LI = yaml.safe_load((ROOT / "isaac/link_inertia.yaml").read_text())["M_diag"]; ILINK = {"DM4310": LI["openarm_right_joint7"], "DM4340": LI["openarm_right_joint3"], "DM8009": LI["openarm_right_joint1"]}
rows = []
for m, (p, joint, _, _) in IDENT.items():
    d = yaml.safe_load((ROOT / p).read_text()); dc = d["isaac_dcmotor"]; s = SPECS["damiao"][m]
    if m == "DM8009":
        h = yaml.safe_load((ROOT / "data/2026-09-22_j1c/fit/fit.yaml").read_text())["joint"]; P, SE = h["params"], h["se"]
        c_, b_, g_, r2, rms, J = f"{P['c']:.3g} ± {SE['c']:.1g} (hold)§", "0.10 ± 0.05§", f"{P['g_sin']:.4g} ± {SE['g_sin']:.1g}", f"{h['r2']:.3f}", f"{h['rms_resid_Nm']:.2f}", "–"
    else:
        c_, b_, g_, r2, rms, J = pm(d, "c"), pm(d, "b"), pm(d, "g_sin", 2), f"{d['stage1_r2']:.3f}", f"{d['stage1_rms_Nm']:.3f}", f"{d['fit']['J']['value']:.4f}"
    rows.append([f"**{m}**", joint, f"{s['gear_ratio']}:1", c_, b_, f"{dc['armature']:.5f} ({ARM_SRC[m]})", J, f"{ILINK[m]:.4f}", g_, r2, rms,
                 f"{s['rated_torque_Nm']} / {s['peak_torque_Nm']}", f"{s['noload_speed_rpm']*2*math.pi/60:.1f} (D)"])
(T / "t1_params.md").write_text(md(rows, left=3, hdr=["모터", "식별 축", "감속", "Coulomb c [Nm]", "점성 b [Nm·s/rad]", "armature [kg·m²]", "J_fit (총 관성)", "I_link (Isaac)", "m·g·d [Nm]", "R² (1단계)", "잔차 RMS [Nm]", "정격/피크 [Nm] (D)", "v_max [rad/s]"]))

# T2 잔차
rows = []
for m, (p, joint, rd, band) in IDENT.items():
    r = yaml.safe_load((ROOT / "data/sim" / rd / "residual.yaml").read_text()); v = r["verdict"]
    rows.append([f"**{m}**", r["n"], f"{v['pos_rms_deg']:.2f}°", f"{math.degrees(r['residual_max']['q_rad']):.2f}°", f"{v['tau_rms_Nm']:.2f} Nm ({v['tau_rms_Nm']/SPECS['damiao'][m]['rated_torque_Nm']*100:.0f} %)",
                 f"{v['pos_limit_deg']:.0f}° / {v['tau_limit_Nm']:.1f} Nm", f"{r['tracking_rms_deg']['real']:.2f}° / {r['tracking_rms_deg']['sim']:.2f}°", band, "**PASS**" if v["pos_pass"] and v["tau_pass"] else "FAIL"])
(T / "t2_residual.md").write_text(md(rows, left=1, hdr=["모터", "표본 (500 Hz)", "Δq RMS", "Δq max", "Δτ RMS (정격 대비)", "기준 (Δq / Δτ)", "추종오차 실물 / 시뮬", "유효 대역", "판정"]))

# T3 클래스별 벤치 요약
M = pd.read_csv(ROOT / "data/sim/bench/metrics.csv")
def f(v, fmt): return "–" if (v is None or (isinstance(v, float) and np.isnan(v))) else fmt.format(v)
for cls in ["wrist", "elbow", "shoulder"]:
    sub = M[M.cls == cls].sort_values("bw_3db_hz", ascending=False, na_position="first"); rows = []
    for _, r in sub.iterrows():
        name = f"**{LABEL[r.preset]}**" if r.preset.startswith("dm") else LABEL[r.preset]
        bw = f"> {CHIRP_F1[cls]}" if np.isnan(r.bw_3db_hz) else f"{r.bw_3db_hz:.1f}"
        stop = "한계 접촉" if r.get("fd_hit_limit") else ("미정지" if np.isnan(r.fd_stop_s) else f"{r.fd_stop_s:.2f}")
        rows.append([name, grade(r.preset), f"{r.tau_pk:g}", f"{r.vel_lim:.1f}", (f"{r.armature:.2e}" if r.armature > 0 else "0 (미공개)"), f"{r.coulomb:.3g}" if r.coulomb > 0 else "0 (미공개)", f"{r.arm_over_Ilink:.2f}",
                     bw, f"{r.step_overshoot_pct:.0f}", f"{r.step_ss_err_deg:.2f}", f"{r.ms_track_rms_deg:.2f}", stop])
    (T / f"t3_bench_{cls}.md").write_text(md(rows, left=2, hdr=["액추에이터", "등급", "τpk [Nm]", "v_max [rad/s]", "armature [kg·m²]", "Coulomb [Nm]", "arm/I_link", "−3 dB [Hz]", "오버슈트 [%]", "정상오차 [°]", "추종 RMS [°]", "백드라이브 정지 [s]"]))

# T4 민감도 (추종 RMS · 정상오차, 3 조건)
Z = pd.read_csv(ROOT / "data/sim/bench_fric_zero/metrics.csv"); F = pd.read_csv(ROOT / "data/sim/bench_fric_floor02/metrics.csv")
for cls in ["wrist", "elbow", "shoulder"]:
    sub = M[M.cls == cls].sort_values("ms_track_rms_deg"); rows = []
    for _, r in sub.iterrows():
        z = Z[(Z.cls == cls) & (Z.preset == r.preset)].iloc[0]; fl = F[(F.cls == cls) & (F.preset == r.preset)].iloc[0]
        name = f"**{LABEL[r.preset]}**" if r.preset.startswith("dm") else LABEL[r.preset]
        rows.append([name, f"{r.coulomb:.3g}" if r.coulomb > 0 else "0", f"{fl.coulomb:.2f}", f"{r.ms_track_rms_deg:.2f}", f"{z.ms_track_rms_deg:.2f}", f"{fl.ms_track_rms_deg:.2f}",
                     f"{r.step_ss_err_deg:.2f}", f"{z.step_ss_err_deg:.2f}", f"{fl.step_ss_err_deg:.2f}", f(r.bw_3db_hz, "{:.1f}"), f(z.bw_3db_hz, "{:.1f}"), f(fl.bw_3db_hz, "{:.1f}")])
    (T / f"t4_sens_{cls}.md").write_text(md(rows, left=1, hdr=["액추에이터", "Coulomb 원래", "Coulomb 2 % 하한", "추종 RMS 원래", "마찰 0", "2 % 하한", "정상오차 원래", "마찰 0", "2 % 하한", "−3 dB 원래", "마찰 0", "2 % 하한"]))

# T5 데이터시트 요약 (팔 클래스만, 핵심 열)
import re
lines = (ROOT / "isaac/comparison_specs.md").read_text().splitlines(); tab = [l for l in lines if l.startswith("|")]
hdr = [c.strip() for c in tab[0].strip("|").split("|")]; rows = []
want = ["계열", "액추에이터", "클래스", "피크 Nm", "정격 Nm", "최고속도 rad/s", "감속비", "질량 kg", "피크밀도 Nm/kg", "정격밀도 Nm/kg", "armature kg·m²", "arm 출처", "τpk/armature rad/s²"]
idx = [hdr.index(w) for w in want]
for l in tab[2:]:
    c = [x.strip() for x in l.strip("|").split("|")]
    if c[2] == "leg": continue
    rows.append([c[i] for i in idx])
(T / "t5_specs_arm.md").write_text(md(rows, want, left=3))
print("→", sorted(p.name for p in T.glob("*.md")))
