"""OpenArm Isaac 모델의 관절공간 질량행렬에서 각 관절 하류 링크 관성(I_link, 관절축 기준)을 읽는다.
armature = J_fit − I_link 계산용. 자세는 매달린 q=0 (실험 자세와 동일). 출력: isaac/link_inertia.yaml"""
import argparse
from isaaclab.app import AppLauncher
ap = argparse.ArgumentParser(); AppLauncher.add_app_launcher_args(ap); a = ap.parse_args(); app = AppLauncher(a).app
import torch, yaml, math
from pathlib import Path
import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation
from isaaclab.sim import SimulationContext, SimulationCfg
from isaaclab_assets.robots.openarm import OPENARM_BI_CFG

sim = SimulationContext(SimulationCfg(dt=0.002, device=a.device))
cfg = OPENARM_BI_CFG.replace(prim_path="/World/openarm"); robot = Articulation(cfg); sim.reset()
names = robot.joint_names
robot.write_joint_state_to_sim(torch.zeros(1, len(names), device=sim.device), torch.zeros(1, len(names), device=sim.device))
sim.step(); robot.update(0.002)
# 유령 질량 제거: hand / ee_tcp 는 가상 프레임인데 USD 에 1.0 kg 이 박혀 있음 (Isaac Lab openarm 에셋 버그)
bodies = robot.body_names; masses = robot.root_physx_view.get_masses().clone()
# URDF 기준 보정: hand 는 실제 링크 0.127 kg (parallel_link/config/inertials.yaml), ee_tcp(=URDF hand_tcp) 는 관성 없는 프레임
ghost = [i for i, b in enumerate(bodies) if b.endswith("_hand") or b.endswith("_ee_tcp")]
for i in ghost: masses[0, i] = 0.127 if bodies[i].endswith("_hand") else 1e-4
robot.root_physx_view.set_masses(masses, torch.arange(1, device="cpu"))
sim.step(); robot.update(0.002)
print("corrected bodies:", {bodies[i]: float(masses[0, i]) for i in ghost})
# 중력 토크 교차검증: joint7 을 90° 로 두고 일반화 중력힘 읽기 → 실측 m·g·d 0.72 Nm 과 비교
import numpy as np
qz = torch.zeros(1, len(names), device=sim.device); i7 = names.index("openarm_right_joint7"); qz[0, i7] = math.pi / 2
robot.write_joint_state_to_sim(qz, torch.zeros_like(qz)); sim.step(); robot.update(0.002)
g = robot.root_physx_view.get_gravity_compensation_forces()[0].cpu().numpy() if hasattr(robot.root_physx_view, "get_gravity_compensation_forces") else None
if g is not None: print(f"gravity torque at joint7 (q7=90°): {g[g.shape[0] - len(names) + i7]:.4f} Nm   (실측 m·g·d = 0.72)")
robot.write_joint_state_to_sim(torch.zeros_like(qz), torch.zeros_like(qz)); sim.step(); robot.update(0.002)
M = robot.root_physx_view.get_generalized_mass_matrices()[0].cpu().numpy()
# 고정 베이스면 M 은 dof×dof. 부동 베이스면 앞 6 자유도 제외
off = M.shape[0] - len(names)
out = {n: float(M[off + i, off + i]) for i, n in enumerate(names)}
# 하류 링크 관성: 질량행렬 대각항은 그 관절 하류 전체(그리퍼 포함)의 관절축 기준 관성 (USD armature 포함 시 빼야 함)
arm = robot.data.joint_armature[0].cpu().numpy() if hasattr(robot.data, "joint_armature") else None
res = {"pose": "hanging q=0", "note": "M_ii = 관절 i 하류 링크 전체의 관절축 기준 관성 (hand=0.127 kg URDF 값, ee_tcp≈0 로 보정 후)", "corrected_bodies": {bodies[i]: float(masses[0, i]) for i in ghost}, "M_diag": out}
if arm is not None: res["usd_armature"] = {n: float(arm[i]) for i, n in enumerate(names)}
masses = robot.root_physx_view.get_masses()[0].cpu().numpy(); coms = robot.root_physx_view.get_coms()[0].cpu().numpy()
res["bodies"] = {b: {"mass": float(masses[i]), "com_local": [float(x) for x in coms[i][:3]]} for i, b in enumerate(bodies)}
for i, b in enumerate(bodies):
    if "right" in b: print(f"  body {b:28s} m={masses[i]:.4f} kg  com={coms[i][:3].round(4)}")
p = Path(__file__).resolve().parent / "link_inertia.yaml"; p.write_text(yaml.safe_dump(res, sort_keys=False))
for n in names:
    if "right" in n: print(f"{n:32s} M_ii={out[n]:.5f}" + (f"  usd_armature={res['usd_armature'][n]:.5f}" if arm is not None else ""))
print("→", p, flush=True)
import os, threading; threading.Timer(8, lambda: os._exit(0)).start(); app.close()
