#!/usr/bin/env python3
"""specs.yaml → 데이터시트 수준 비교표 (isaac/comparison_specs.md). data/sim/bench/metrics.csv 가 있으면 WP6 시뮬 열을 붙인다.

정규화 지표
  peak_density  = 피크 토크 / 질량 (Nm/kg)         — 질량 미공개면 빈칸
  rated_density = 정격(연속) 토크 / 질량 (Nm/kg)
  peak/rated    = 피크/정격 비 (열 여유)
  tau/armature  = 피크 토크 / 반사관성 (rad/s², 가속 능력) — armature 출처를 열에 표기
출처 등급: M=실측(다이노/펌웨어), D=데이터시트, S=시뮬 cfg 값(튜닝/플레이스홀더 가능)
"""
import math
from pathlib import Path
import yaml

S = yaml.safe_load((Path(__file__).resolve().parent / "specs.yaml").read_text())
RPM = 2 * math.pi / 60
rows = []

def add(family, name, cls, peak, rated, speed, ratio, mass, armature, arm_src, rob, note=""):
    rows.append(dict(family=family, name=name, cls=cls, peak=peak, rated=rated, speed=speed, ratio=ratio, mass=mass,
                     armature=armature, arm_src=arm_src, robot=rob, note=note))

# Damiao (OpenArm 실물) — armature 는 WP4 식별 결과 (isaac/actuators.py 와 동일). DM4340 만 처프 실측, 나머지는 펌웨어 로터관성×Gr² (분리 불가 사유는 PROGRESS.md)
DM_IDENT = {"DM4310": (0.00169, "M(fw)", "마찰 실측, armature 펌웨어"), "DM4340": (0.02557, "M(chirp fit)", "마찰·armature 실측"), "DM8009": (0.01669, "M(fw)", "마찰 실측(hold), armature 펌웨어")}
for m, cls in (("DM4310", "wrist"), ("DM4340", "elbow"), ("DM8009", "shoulder")):
    d = S["damiao"][m]; arm, src, note = DM_IDENT[m]
    add("Damiao", m, cls, d["peak_torque_Nm"], d["rated_torque_Nm"], d["noload_speed_rpm"] * RPM, d["gear_ratio"], d["mass_kg"], arm, src, "OpenArm", note)
# Robstride (K-Bot)
for m, cls, mc in (("RS00", "wrist", "motor_00"), ("RS02", "elbow", "motor_02"), ("RS03", "shoulder", "motor_03"), ("RS04", "leg", "motor_04")):
    r = S["robstride"][m]; mj = S["kscale_kbot_v2"]["mjcf_classes"][mc]
    add("Robstride", m, cls, r["peak_torque_Nm"], r["rated_torque_Nm"], r["noload_speed_rpm"] * RPM, r["gear_ratio"], r["mass_kg"], mj["armature_kgm2"], "S(K-Scale MJCF)", "K-Bot")
# MIT
for j, cls in (("shoulder_flexion", "shoulder"), ("elbow", "elbow"), ("knee", "leg")):
    jj = S["mit_humanoid"]["joints"][j]; mod = S["mit_humanoid"]["modules"][jj["module"]]; belt = jj["total_ratio"] / mod["gear_ratio"]
    add("MIT", f"{jj['module']}@{j}", cls, jj["torque_Nm"], None, jj["speed_rads"], jj["total_ratio"], mod["mass_kg"] if belt == 1 else None,
        mod["output_inertia_kgm2"] * belt ** 2, "M(dyno)", "MIT Humanoid", "벨트 증속 포함" if belt != 1 else "다이노 포화 31 Nm" if jj["module"] == "U10" else "")
# Berkeley Humanoid
for m, cls in (("5013", "wrist"), ("8513", "shoulder"), ("8518", "leg"), ("10413", "leg")):
    a = S["berkeley_humanoid"]["actuators"][m]
    add("Berkeley", m, cls, a["peak_torque_Nm"], a["sustained_torque_Nm"], a["max_speed_rads_48V"], a["gear_ratio"], a["mass_kg"], a["rotor_inertia_kgm2"] * a["gear_ratio"] ** 2, "D(rotor×G²)", "Berkeley Humanoid", "토크 클래스 매칭(팔 미장착)")
# Berkeley Lite
for m, cls in (("5010", "wrist"), ("6512", "leg")):
    a = S["berkeley_humanoid_lite"]["actuators"][m]
    add("Berkeley Lite", m, cls, a["isaac_effort_limit_Nm"], None, a["isaac_velocity_limit_rads"], None, None, a["isaac_armature_kgm2"], "S(Isaac cfg)", "BH Lite", "사이클로이드, 값=시뮬 cfg")
# Unitree
for j, cls in (("shoulder_pitch", "shoulder"), ("elbow", "elbow"), ("wrist_roll", "wrist"), ("wrist_pitch", "wrist"), ("knee", "leg")):
    if j == "knee":
        add("Unitree", "G1 knee", cls, 139, None, 20, None, None, None, "-", "G1", "Isaac Lab cfg"); continue
    jj = S["unitree_g1_29dof"]["joints"][j]; mj = S["unitree_g1_29dof"]["mjcf_joint_defaults"].get(jj.get("mjcf_class"), {})
    add("Unitree", f"G1 {j}", cls, jj["effort_Nm"], None, jj["velocity_rads"], None, None, mj.get("armature_kgm2"), "S(MJCF 공통값)", "G1", "URDF effort")
for j, cls in (("shoulder_pitch", "shoulder"), ("elbow", "elbow")):
    jj = S["unitree_h1"]["joints"][j]; add("Unitree", f"H1 {j}", cls, jj["effort_Nm"], None, jj["velocity_rads"], None, None, None, "-", "H1", "URDF effort")
jj = S["unitree_h1_2"]["joints"]["wrist_roll"]; add("Unitree", "H1-2 wrist_roll", "wrist", jj["effort_Nm"], None, jj["velocity_rads"], None, None, None, "-", "H1-2", "URDF effort")

# WP6 시뮬 열 (bench_all.py → bench_metrics.py). 표 행 ↔ 프리셋 이름
PRESET_OF = {("Damiao", "DM4310"): "dm4310", ("Damiao", "DM4340"): "dm4340", ("Damiao", "DM8009"): "dm8009",
             ("Robstride", "RS00"): "rs00_wrist", ("Robstride", "RS02"): "rs02_elbow", ("Robstride", "RS03"): "rs03_shoulder",
             ("MIT", "U10@shoulder_flexion"): "mit_shoulder", ("MIT", "U10@elbow"): "mit_elbow", ("Berkeley", "5013"): "bh_5013", ("Berkeley", "8513"): "bh_8513",
             ("Berkeley Lite", "5010"): "bhl_5010_arm", ("Unitree", "G1 shoulder_pitch"): "g1_shoulder", ("Unitree", "G1 elbow"): "g1_elbow",
             ("Unitree", "G1 wrist_roll"): "g1_wrist_roll", ("Unitree", "G1 wrist_pitch"): "g1_wrist_pitch", ("Unitree", "H1 shoulder_pitch"): "h1_shoulder", ("Unitree", "H1 elbow"): "h1_elbow", ("Unitree", "H1-2 wrist_roll"): "h12_wrist"}
SIM_COLS = [("bw_3db_hz", "시뮬 −3dB Hz", "{:.2f}"), ("ms_track_rms_deg", "시뮬 추종 RMS °", "{:.2f}"), ("step_settle_ms", "시뮬 정착 ms", "{:.0f}"), ("fd_stop_s", "시뮬 백드라이브 정지 s", "{:.2f}")]
mpath = Path(__file__).resolve().parent.parent / "data/sim/bench/metrics.csv"; sim = {}
if mpath.exists():
    import csv
    for r in csv.DictReader(open(mpath)): sim[r["preset"]] = r

def f(v, fmt="{:.3g}"):
    if v is None or v == "": return "-"
    try: return fmt.format(float(v)) if fmt != "{}" else str(v)
    except ValueError: return str(v)
hdr = ["계열", "액추에이터", "클래스", "피크 Nm", "정격 Nm", "최고속도 rad/s", "감속비", "질량 kg", "피크밀도 Nm/kg", "정격밀도 Nm/kg", "피크/정격", "armature kg·m²", "arm 출처", "τpk/armature rad/s²", "로봇", "비고"] + ([h for _, h, _ in SIM_COLS] if sim else [])
order = {"wrist": 0, "elbow": 1, "shoulder": 2, "leg": 3}
rows.sort(key=lambda r: (order[r["cls"]], -(r["peak"] or 0)))
lines = ["| " + " | ".join(hdr) + " |", "|" + "---|" * len(hdr)]
for r in rows:
    pd_ = r["peak"] / r["mass"] if r["mass"] else None; rd = r["rated"] / r["mass"] if (r["mass"] and r["rated"]) else None
    pr = r["peak"] / r["rated"] if r["rated"] else None; ta = r["peak"] / r["armature"] if r["armature"] else None
    cells = [r["family"], r["name"], r["cls"], f(r["peak"]), f(r["rated"]), f(r["speed"]), f(r["ratio"]), f(r["mass"]), f(pd_), f(rd), f(pr, "{:.1f}"), f(r["armature"], "{:.2e}"), r["arm_src"], f(ta, "{:.0f}"), r["robot"], r["note"]]
    if sim:
        sr = sim.get(PRESET_OF.get((r["family"], r["name"])), {})
        if sr and sr.get("bw_3db_hz", "") == "": sr["bw_3db_hz"] = "> " + {"wrist": "20", "elbow": "40", "shoulder": "8"}[sr["cls"]]  # 처프 상한까지 −3 dB 미도달
        if sr and sr.get("fd_hit_limit") == "True": sr["fd_stop_s"] = "한계 접촉"
        cells += [f(sr.get(k), fmt) if sr else "-" for k, _, fmt in SIM_COLS]
    lines.append("| " + " | ".join(cells) + " |")
out = Path(__file__).resolve().parent / "comparison_specs.md"
head = ["# QDD 액추에이터 비교표: 데이터시트 + WP6 시뮬 열 (2026-09-22)", "",
        "출처 등급: **M**=실측(다이노/펌웨어 읽기/처프 피팅), **D**=데이터시트, **S**=시뮬 cfg 값(튜닝/플레이스홀더 가능). 값 출처 URL은 `specs.yaml`.", "",
        "주의: Unitree MJCF armature 0.01 은 전 관절 공통값(범용), K-Scale MJCF 는 클래스별 상이(튜닝 추정). DM armature 는 DM4340 만 처프 실측, DM4310/8009 는 펌웨어 로터관성×Gr² (링크·그리퍼 관성 불확실성 > armature 라 분리 불가). "
        "'피크'는 데이터시트 피크 또는 URDF effort. 정격 미공개는 빈칸.", "",
        "시뮬 열(있을 때): 클래스별 동일 리그·동일 PD 의 Isaac 단일 관절 벤치 (`isaac/bench_metrics.md` 에 전체 지표·정의). 팔 클래스(wrist/elbow/shoulder)만 벤치 대상, leg 행은 빈칸. "
        "DM 열만 실측 검증(WP5 잔차 Δq ≤0.9°, Δτ ≤3 % 정격), 타사 열은 사양 기반 예측.", ""]
out.write_text("\n".join(head + lines) + "\n"); print("\n".join(lines)); print(f"\n→ {out}")
