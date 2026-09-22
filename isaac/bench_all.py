"""WP6 공통 벤치: CLASS_MATCH 의 전 프리셋을 클래스별로 통일한 단일 관절 리그(관성·중력·PD 게인)에 올려
{step, chirp, multisine, free_decay} 동일 프로토콜을 한 프로세스에서 실행한다 (프리셋마다 Articulation 1개, 동시 시뮬).

    ~/isaac/env_isaaclab/bin/python isaac/bench_all.py --headless                      # 18 프리셋 × 4 모드
    ~/isaac/env_isaaclab/bin/python isaac/bench_all.py --headless --classes wrist --modes step,chirp --presets dm4310,g1_wrist_roll

클래스 리그 = WP5 에서 잔차가 검증된 OpenArm 대표 관절(joint7/3/1)의 등가 막대 + 그 관절의 운용 PD 게인(config/motors.yaml).
따라서 DM 열에는 WP5 잔차(오차 막대)가 그대로 적용되고, 타사 모터는 "그 자리에 꽂았다면" 의 시뮬 결과다.
출력: data/sim/bench/<class>/<preset>/{step,chirp,multisine,free_decay}.csv + meta.yaml  → isaac/bench_metrics.py 가 지표 표를 만든다.
"""
import argparse, csv, math, time
from pathlib import Path

from isaaclab.app import AppLauncher

ap = argparse.ArgumentParser()
ap.add_argument("--classes", default="wrist,elbow,shoulder"); ap.add_argument("--modes", default="step,chirp,multisine,free_decay")
ap.add_argument("--presets", default=None, help="쉼표 구분 부분집합 (기본: 클래스의 CLASS_MATCH 전체)")
ap.add_argument("--out", default=None); ap.add_argument("--hz", type=int, default=500)
ap.add_argument("--friction", default="asis", help="민감도: asis | zero (전 프리셋 Coulomb·점성 0) | floor:<frac> (Coulomb 가 frac×τpk 미만인 프리셋에 그 값을 채움, 예 floor:0.02)")
ap.add_argument("--effort", default="peak", choices=["peak", "rated"], help="peak: effort_limit=saturation_effort(피크)로 통일 — 과도응답 벤치용. rated: 프리셋 값(연속 정격) 그대로")
AppLauncher.add_app_launcher_args(ap); args = ap.parse_args()
app = AppLauncher(args).app

import sys; sys.path.insert(0, str(Path(__file__).resolve().parent)); sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))  # noqa: E402,E702
import numpy as np, torch, yaml  # noqa: E402
import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.assets import Articulation, ArticulationCfg  # noqa: E402
from isaaclab.sim import SimulationContext, SimulationCfg  # noqa: E402
from actuators import PRESETS, CLASS_MATCH  # noqa: E402
from rig_usd import ROOT, build_rig_usd, equivalent_rod  # noqa: E402
import excite as ex  # noqa: E402

# 클래스 리그: I_link(isaac/link_inertia.yaml, 유령질량 보정) · m·g·d(실측 identified_*.yaml) · 운용 kp/kd(config/motors.yaml) — WP5 replay 리그와 동일
CLASS_RIG = {
    "wrist":    dict(joint="right_joint7", inertia=0.00559, mgd=0.72,  kp=10.0, kd=0.5,  chirp_f1=20.0),
    "elbow":    dict(joint="right_joint3", inertia=0.00215, mgd=0.06,  kp=70.0, kd=2.0,  chirp_f1=40.0),   # 롤 축 → 중력 정렬 리그. I_link 이 작아 폐루프 ωn≈28 Hz → f1 40
    "shoulder": dict(joint="right_joint1", inertia=0.434,   mgd=11.74, kp=70.0, kd=2.75, chirp_f1=8.0),
}
# 프로토콜 (클래스 내 동일). 진폭은 소신호 수준으로 통일
PROTO = dict(step=dict(amp=0.3, hold=2.0, reps=2), chirp=dict(amp=0.1, f0=0.2, dur=30.0),
             multisine=dict(freqs=[0.3, 0.7, 1.3, 2.9], amp=0.4, dur=14.0), free_decay=dict(release_from=0.8, hold_s=1.5, dur=10.0, spin_v0=2.0))


class PosChirp(ex.Traj):
    """PD 위치 기준 선형 처프 (폐루프 대역폭용). q_des = amp·sin φ, φ = 2π(f0 t + (f1−f0) t²/2T)."""
    def __init__(self, amp, f0, f1, dur, kp, kd):
        self.amp, self.f0, self.f1, self.duration, self.kp, self.kd = amp, f0, f1, dur, kp, kd
    def freq(self, t): return self.f0 + (self.f1 - self.f0) * t / self.duration
    def __call__(self, t, q, dq):
        ph = 2 * math.pi * (self.f0 * t + (self.f1 - self.f0) * t * t / (2 * self.duration)); w = 2 * math.pi * self.freq(t)
        return dict(q_des=self.amp * math.sin(ph), dq_des=self.amp * w * math.cos(ph), kp=self.kp, kd=self.kd, tau_ff=0.0, phase="chirp")


def make_traj(mode, rig, gravity_axis):
    p, kp, kd = PROTO[mode], rig["kp"], rig["kd"]
    if mode == "step": return ex.Step(p["amp"], p["hold"], p["reps"], 0.0, kp, kd)
    if mode == "chirp": return PosChirp(p["amp"], p["f0"], rig["chirp_f1"], p["dur"], kp, kd)
    if mode == "multisine": return ex.MultiSine(p["freqs"], p["amp"], p["dur"], 0.0, kp, kd)
    if mode == "free_decay":  # 진자 리그: 0.8 rad 에서 놓음. 중력 정렬 리그: 0 rad 에서 spin_v0 로 돌려 보냄(코스트다운)
        return ex.FreeDecay(0.0 if gravity_axis else p["release_from"], p["hold_s"], p["dur"], kp, kd)
    raise KeyError(mode)


def main():
    out_root = Path(args.out or ROOT / "data" / "sim" / "bench"); dt = 1.0 / args.hz
    classes = args.classes.split(","); modes = args.modes.split(","); subset = set(args.presets.split(",")) if args.presets else None
    sim = SimulationContext(SimulationCfg(dt=dt, device=args.device))
    runs = []  # dict(cls, preset, art, usd, gravity_axis, meta)
    for ci, cls in enumerate(classes):
        rig = CLASS_RIG[cls]; mass, length, ga = equivalent_rod(rig["inertia"], rig["mgd"])
        usd = build_rig_usd(mass, length, ga, ROOT / "isaac" / "assets" / f"rig_{cls}.usda")
        for pi, preset in enumerate(CLASS_MATCH[cls]):
            if subset and preset not in subset: continue
            act_cfg = PRESETS[preset](); act_cfg.joint_names_expr = ["joint"]; act_cfg.stiffness, act_cfg.damping = rig["kp"], rig["kd"]
            if args.effort == "peak": act_cfg.effort_limit = act_cfg.saturation_effort
            if args.friction == "zero": act_cfg.friction = act_cfg.dynamic_friction = act_cfg.viscous_friction = 0.0
            elif args.friction.startswith("floor:"):
                fl = float(args.friction.split(":")[1]) * act_cfg.saturation_effort
                if act_cfg.dynamic_friction < fl: act_cfg.friction = act_cfg.dynamic_friction = fl
            spawn = sim_utils.UsdFileCfg(usd_path=usd,
                                         articulation_props=sim_utils.ArticulationRootPropertiesCfg(fix_root_link=True, enabled_self_collisions=False, sleep_threshold=0.0, stabilization_threshold=0.0),
                                         rigid_props=sim_utils.RigidBodyPropertiesCfg(sleep_threshold=0.0, stabilization_threshold=0.0, linear_damping=0.0, angular_damping=0.0))
            cfg = ArticulationCfg(prim_path=f"/World/{cls}_{preset}", spawn=spawn, init_state=ArticulationCfg.InitialStateCfg(pos=(2.0 * pi, 3.0 * ci, 1.0)), actuators={"j": act_cfg})
            meta = dict(source="isaac_bench", cls=cls, preset=preset, hz=args.hz, rig=dict(rig, mass=mass, length=length, gravity_axis=ga, link_inertia_joint=mass * length**2 / 3.0), proto=PROTO, effort_mode=args.effort, friction_mode=args.friction,
                        actuator_cfg={k: float(v) for k, v in vars(act_cfg).items() if k in ("effort_limit", "saturation_effort", "velocity_limit", "armature", "friction", "dynamic_friction", "viscous_friction") and isinstance(v, (int, float))})
            runs.append(dict(cls=cls, preset=preset, art=Articulation(cfg), gravity_axis=ga, rig=rig, meta=meta))
    sim.reset()
    print(f"[bench] {len(runs)} rigs, modes={modes}, device={sim.device}", flush=True)
    dev = sim.device; z = torch.zeros(1, 1, device=dev)

    def reset_all(q0=0.0, dq0=0.0):
        for r in runs:
            r["art"].write_joint_state_to_sim(torch.full((1, 1), q0, device=dev), torch.full((1, 1), dq0, device=dev))
            r["art"].set_joint_effort_target(z); r["art"].set_joint_position_target(torch.full((1, 1), q0, device=dev)); r["art"].set_joint_velocity_target(z)
            r["art"].write_data_to_sim()
        sim.step()
        for r in runs: r["art"].update(dt)

    for mode in modes:
        trajs = [make_traj(mode, r["rig"], r["gravity_axis"]) for r in runs]
        reset_all(0.0)
        if mode == "free_decay":  # 진자 리그는 놓을 각도에서 시작 (pre_hold 로 유지), 중력 정렬 리그는 0
            for r in runs:
                r["art"].write_joint_state_to_sim(torch.full((1, 1), 0.0 if r["gravity_axis"] else PROTO["free_decay"]["release_from"], device=dev), z); r["art"].write_data_to_sim()
            sim.step(); [r["art"].update(dt) for r in runs]
        files, writers = [], []
        for r in runs:
            d = out_root / r["cls"] / r["preset"]; d.mkdir(parents=True, exist_ok=True)
            (d / "meta.yaml").write_text(yaml.safe_dump(r["meta"], sort_keys=False, allow_unicode=True))
            f = open(d / f"{mode}.csv", "w", newline=""); w = csv.writer(f); w.writerow(["t_s", "phase", "q_des", "dq_des", "kp", "kd", "tau_ff", "q", "dq", "tau_applied", "tau_computed"])
            files.append(f); writers.append(w)
        duration = max(tr.duration for tr in trajs); released = [False] * len(runs)
        t, i, t0 = 0.0, 0, time.time()
        while t < duration and app.is_running():
            states = [(float(r["art"].data.joint_pos[0, 0]), float(r["art"].data.joint_vel[0, 0])) for r in runs]
            cmds = []
            for k, (r, tr) in enumerate(zip(runs, trajs)):
                q, dq = states[k]; c = tr(t, q, dq); cmds.append(c); art = r["art"]; act = art.actuators["j"]
                if mode == "free_decay" and c["phase"] == "free" and not released[k]:
                    released[k] = True
                    if r["gravity_axis"]:  # 코스트다운: 놓는 순간 초기 각속도 주입
                        art.write_joint_state_to_sim(torch.full((1, 1), q, device=dev), torch.full((1, 1), PROTO["free_decay"]["spin_v0"], device=dev))
                act.stiffness[:] = c["kp"]; act.damping[:] = c["kd"]
                if act.is_implicit_model:
                    art.write_joint_stiffness_to_sim(torch.full((1, 1), c["kp"], device=dev)); art.write_joint_damping_to_sim(torch.full((1, 1), c["kd"], device=dev))
                art.set_joint_position_target(torch.full((1, 1), c["q_des"], device=dev)); art.set_joint_velocity_target(torch.full((1, 1), c["dq_des"], device=dev))
                art.set_joint_effort_target(torch.full((1, 1), c["tau_ff"], device=dev)); art.write_data_to_sim()
            sim.step()
            for k, r in enumerate(runs):
                r["art"].update(dt); c = cmds[k]; q, dq = states[k]
                writers[k].writerow([f"{t:.6f}", c["phase"], f"{c['q_des']:.6f}", f"{c['dq_des']:.6f}", c["kp"], c["kd"], f"{c['tau_ff']:.6f}", f"{q:.6f}", f"{dq:.6f}",
                                     f"{float(r['art'].data.applied_torque[0, 0]):.6f}", f"{float(r['art'].data.computed_torque[0, 0]):.6f}"])
            t += dt; i += 1
            if i % 2500 == 0: print(f"  [{mode}] t={t:5.1f}/{duration:.1f}s  wall {time.time() - t0:5.1f}s", flush=True)
        for f in files: f.close()
        print(f"[bench] {mode}: {i} steps, wall {time.time() - t0:.1f}s → {out_root}/<class>/<preset>/{mode}.csv", flush=True)
    import os, threading; threading.Timer(10.0, lambda: os._exit(0)).start()
    app.close()


if __name__ == "__main__":
    main()
