"""단일 관절 테스트 리그: 1-링크 진자(고정 베이스 + 1 revolute 조인트)에 임의 액추에이터 cfg 를 주입해 동일 프로토콜 실행.

    python isaac/single_joint_rig.py --headless --actuator dm4310 --mode step
    python isaac/single_joint_rig.py --headless --actuator g1_leg --mode chirp
    python isaac/single_joint_rig.py --headless --actuator dm4310 --mode replay --cmd data/.../cmd.csv   # 실측 홀드아웃 재생 (WP5)

액추에이터 프리셋은 isaac/actuators.py (WP4 결과가 DM 프리셋을 채운다). 진자 링크 질량·길이는 --mass/--length 로 통일.
출력: <out>/sim.csv  (t_s, q_des, dq_des, tau_ff, q, dq, tau_applied, tau_computed)
"""
import argparse, csv, math
from pathlib import Path

from isaaclab.app import AppLauncher

ap = argparse.ArgumentParser()
ap.add_argument("--actuator", required=True); ap.add_argument("--mode", default="step", choices=["hold", "step", "chirp", "multisine", "free_decay", "const_vel", "replay"])
ap.add_argument("--cmd", default=None, help="replay 모드: excite.py 의 cmd.csv")
ap.add_argument("--mass", type=float, default=0.5); ap.add_argument("--length", type=float, default=0.15)
ap.add_argument("--inertia", type=float, default=None, help="하류 링크 관성 I (관절축 기준, kg m²). --mgd 와 함께 주면 막대 질량·길이를 자동 계산")
ap.add_argument("--mgd", type=float, default=None, help="중력 토크 진폭 m·g·d (Nm). 0 이면 --gravity-axis 로 처리")
ap.add_argument("--q0", type=float, default=None, help="초기 관절각 (replay 시 기본: cmd.csv 첫 행 q)")
ap.add_argument("--phase", default=None, help="replay 시 cmd.csv 에서 이 phase 행만 사용 (예: multisine)")
ap.add_argument("--tau0", type=float, default=0.0, help="실물 토크 바이어스 (sim.csv 의 tau_real_unbiased 계산용 메타에 기록)")
ap.add_argument("--gravity-axis", action="store_true", help="관절축을 중력과 정렬(중력 토크 0). 기본은 수평축(진자)")
ap.add_argument("--dur", type=float, default=10.0); ap.add_argument("--hz", type=int, default=500); ap.add_argument("--decimation", type=int, default=1)
ap.add_argument("--out", default=None); ap.add_argument("--tau", type=float, default=0.5, help="chirp 토크 진폭"); ap.add_argument("--f1", type=float, default=20.0, help="chirp 최고 주파수")
ap.add_argument("--levels", default="0.5,1,2,4", help="const_vel 속도 레벨")
AppLauncher.add_app_launcher_args(ap); args = ap.parse_args()
app = AppLauncher(args).app

import torch  # noqa: E402
import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.assets import Articulation, ArticulationCfg  # noqa: E402
from isaaclab.sim import SimulationContext, SimulationCfg  # noqa: E402
import sys; sys.path.insert(0, str(Path(__file__).resolve().parent)); sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))  # noqa: E402,E702
from actuators import PRESETS  # noqa: E402

from rig_usd import ROOT, build_rig_usd, equivalent_rod  # noqa: E402


def main():
    # 실물 관절 등가 막대: I = (4/3) m d², m g d = mgd  →  d = I / ((4/3)(mgd/g)), m = (mgd/g)/d, L = 2d
    if args.inertia is not None:
        args.mass, args.length, ga = equivalent_rod(args.inertia, args.mgd, args.mass); args.gravity_axis = args.gravity_axis or ga
        print(f"[rig] I={args.inertia:.5f} mgd={args.mgd} → mass={args.mass:.4f} kg length={args.length:.4f} m gravity_axis={args.gravity_axis}")
    sim = SimulationContext(SimulationCfg(dt=1.0 / args.hz, device=args.device))
    usd = build_rig_usd(args.mass, args.length, args.gravity_axis)
    act_cfg = PRESETS[args.actuator]()
    act_cfg.joint_names_expr = ["joint"]
    spawn = sim_utils.UsdFileCfg(usd_path=usd,
                                 articulation_props=sim_utils.ArticulationRootPropertiesCfg(fix_root_link=True, enabled_self_collisions=False,
                                                                                            sleep_threshold=0.0, stabilization_threshold=0.0),
                                 rigid_props=sim_utils.RigidBodyPropertiesCfg(sleep_threshold=0.0, stabilization_threshold=0.0, linear_damping=0.0, angular_damping=0.0))
    cfg = ArticulationCfg(prim_path="/World/rig", spawn=spawn,
                          init_state=ArticulationCfg.InitialStateCfg(pos=(0, 0, 1.0)), actuators={"j": act_cfg})
    rig = Articulation(cfg)
    sim.reset()
    dt = 1.0 / args.hz
    if args.q0 is not None:
        rig.write_joint_state_to_sim(torch.tensor([[args.q0]], device=sim.device), torch.zeros(1, 1, device=sim.device))
        rig.write_data_to_sim(); sim.step(); rig.update(dt)

    # 궤적: excite.py 와 같은 정의를 재사용 (tools/excite.py 의 Traj 클래스)
    import importlib; ex = importlib.import_module("excite")
    kp, kd = float(act_cfg.stiffness) if not isinstance(act_cfg.stiffness, dict) else 0.0, float(act_cfg.damping) if not isinstance(act_cfg.damping, dict) else 0.0
    replay = None
    if args.mode == "replay":
        with open(args.cmd) as f: replay = list(csv.DictReader(f))
        if args.phase: replay = [r for r in replay if r["phase"] == args.phase]
        duration = len(replay) * dt
        if args.q0 is None: args.q0 = float(replay[0]["q"])
    else:
        tr = {"hold": lambda: ex.Hold(0.0, kp, kd, args.dur), "step": lambda: ex.Step(0.3, 2.0, 3, 0.0, kp, kd),
              "chirp": lambda: ex.Chirp(args.tau, 0.1, args.f1, args.dur, 0.0, 2.0, 0.0),
              "multisine": lambda: ex.MultiSine([0.3, 0.7, 1.3, 2.9], 0.4, args.dur, 0.0, kp, kd),
              "free_decay": lambda: ex.FreeDecay(0.8, 2.0, args.dur, kp, kd),
              "const_vel": lambda: ex.ConstVel([float(x) for x in args.levels.split(",")], args.dur / len(args.levels.split(",")), -1.05, 1.05, kp, kd, 10.0)}[args.mode]()
        duration = tr.duration

    if args.mode == "replay" and args.q0 is not None:
        rig.write_joint_state_to_sim(torch.tensor([[args.q0]], device=sim.device), torch.zeros(1, 1, device=sim.device))
        rig.write_data_to_sim(); sim.step(); rig.update(dt)
    out = Path(args.out or ROOT / "data" / "sim" / f"{args.actuator}_{args.mode}"); out.mkdir(parents=True, exist_ok=True)
    import yaml
    I_link = args.mass * args.length**2 / 3.0  # 막대, 관절(끝단) 기준
    (out / "meta.yaml").write_text(yaml.safe_dump(dict(
        source="isaac_rig", actuator=args.actuator, mode=args.mode, hz=args.hz, mass=args.mass, length=args.length,
        gravity_axis=args.gravity_axis, link_inertia_joint=I_link, com_dist=args.length / 2, q0=args.q0, replay_cmd=args.cmd, replay_phase=args.phase, tau0_real=args.tau0,
        actuator_cfg={k: (float(v) if isinstance(v, (int, float)) else str(v)) for k, v in vars(act_cfg).items()
                      if k in ("effort_limit", "saturation_effort", "velocity_limit", "armature", "friction", "dynamic_friction", "viscous_friction")}), sort_keys=False))
    with open(out / "sim.csv", "w", newline="") as f:
        w = csv.writer(f); w.writerow(["t_s", "q_des", "dq_des", "kp", "kd", "tau_ff", "q", "dq", "tau_applied", "tau_computed"])
        t, i = 0.0, 0
        while t < duration and app.is_running() and (replay is None or i < len(replay)):
            q = float(rig.data.joint_pos[0, 0]); dq = float(rig.data.joint_vel[0, 0])
            if replay is not None:
                r = replay[i]; c = dict(q_des=float(r["q_des"]), dq_des=float(r["dq_des"]), kp=float(r["kp"]), kd=float(r["kd"]), tau_ff=float(r["tau_ff"]))
            else:
                c = tr(t, q, dq)
            # 액추에이터 모델(DCMotor/ImplicitActuator)의 PD: tau = kp*(q_des-q) + kd*(dq_des-dq) + tau_ff
            # 명시적 액추에이터는 자체 stiffness/damping 텐서를 쓰므로 그것을 갱신한다 (PhysX 드라이브 게인이 아님)
            act = rig.actuators["j"]
            act.stiffness[:] = c["kp"]; act.damping[:] = c["kd"]
            if not act.is_implicit_model:
                pass  # 명시적 모델: PhysX 드라이브 게인은 0 으로 유지되고 계산 토크가 effort 로 들어감
            else:
                rig.write_joint_stiffness_to_sim(torch.tensor([[c["kp"]]], device=sim.device))
                rig.write_joint_damping_to_sim(torch.tensor([[c["kd"]]], device=sim.device))
            rig.set_joint_position_target(torch.tensor([[c["q_des"]]], device=sim.device))
            rig.set_joint_velocity_target(torch.tensor([[c["dq_des"]]], device=sim.device))
            rig.set_joint_effort_target(torch.tensor([[c["tau_ff"]]], device=sim.device))
            rig.write_data_to_sim(); sim.step(); rig.update(dt)
            w.writerow([f"{t:.6f}", c["q_des"], c["dq_des"], c["kp"], c["kd"], c["tau_ff"], q, dq,
                        float(rig.data.applied_torque[0, 0]), float(rig.data.computed_torque[0, 0])])
            t += dt; i += 1
    print(f"→ {out/'sim.csv'}  ({i} steps)", flush=True)
    # headless 에서 app.close() 가 간헐적으로 hang → 데이터는 이미 저장됐으므로 워치독으로 강제 종료
    import os, threading; threading.Timer(10.0, lambda: os._exit(0)).start()
    app.close()


if __name__ == "__main__":
    main()
