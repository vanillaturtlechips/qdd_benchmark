"""Isaac Lab 내장 로봇의 액추에이터 cfg 파라미터를 표로 덤프 (시뮬 스폰 없이 cfg 객체만 읽음).

    python isaac/dump_actuator_cfgs.py --headless  →  isaac/actuator_cfgs.md, actuator_cfgs.yaml

비교 대상: OpenArm(bi), G1 29DOF, G1(기본), H1, GR1T2, Digit v4. 파라미터는 joint_names_expr 그룹 단위.
"""
import argparse
from pathlib import Path

from isaaclab.app import AppLauncher

ap = argparse.ArgumentParser(); AppLauncher.add_app_launcher_args(ap); args = ap.parse_args()
app = AppLauncher(args).app

import yaml  # noqa: E402
from isaaclab_assets.robots import openarm, unitree, fourier, agility  # noqa: E402

ROBOTS = {
    "OpenArm_bi": openarm.OPENARM_BI_CFG,
    "OpenArm_bi_highPD": openarm.OPENARM_BI_HIGH_PD_CFG,
    "G1_29dof": unitree.G1_29DOF_CFG,
    "G1": unitree.G1_CFG,
    "H1": unitree.H1_CFG,
    "GR1T2": fourier.GR1T2_CFG,
    "Digit_v4": agility.DIGIT_V4_CFG,
}
FIELDS = ["effort_limit", "effort_limit_sim", "saturation_effort", "velocity_limit", "velocity_limit_sim",
          "stiffness", "damping", "armature", "friction"]


def fmt(v):
    if v is None: return "-"
    if isinstance(v, dict): return "{" + ", ".join(f"{k}:{x:g}" for k, x in v.items()) + "}"
    if isinstance(v, float): return f"{v:g}"
    return str(v)


rows, ydump = [], {}
for rname, cfg in ROBOTS.items():
    ydump[rname] = {"usd": str(getattr(cfg.spawn, "usd_path", "")), "actuators": {}}
    for gname, act in cfg.actuators.items():
        rec = {"class": type(act).__name__, "joints": list(act.joint_names_expr)}
        for f in FIELDS:
            rec[f] = getattr(act, f, None)
        ydump[rname]["actuators"][gname] = rec
        rows.append([rname, gname, rec["class"], ",".join(rec["joints"])[:40]] + [fmt(rec[f]) for f in FIELDS])

out = Path(__file__).resolve().parent
hdr = ["robot", "group", "class", "joints"] + FIELDS
md = ["| " + " | ".join(hdr) + " |", "|" + "---|" * len(hdr)] + ["| " + " | ".join(str(c) for c in r) + " |" for r in rows]
(out / "actuator_cfgs.md").write_text("\n".join(md) + "\n")
(out / "actuator_cfgs.yaml").write_text(yaml.safe_dump(ydump, sort_keys=False, allow_unicode=True))
print("\n".join(md)); print(f"\n→ {out/'actuator_cfgs.md'}, {out/'actuator_cfgs.yaml'}")
app.close()
