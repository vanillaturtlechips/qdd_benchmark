"""수정된 로컬 OpenArm USD 를 Isaac 에 로드해 검증: 질량, DOF 목록, 질량행렬 대각, 스폰 오류 여부."""
import argparse
from isaaclab.app import AppLauncher
ap = argparse.ArgumentParser(); ap.add_argument("usd"); AppLauncher.add_app_launcher_args(ap); a = ap.parse_args(); app = AppLauncher(a).app
import torch, math
import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation
from isaaclab.sim import SimulationContext, SimulationCfg
from isaaclab_assets.robots.openarm import OPENARM_BI_CFG
sim = SimulationContext(SimulationCfg(dt=0.002, device=a.device))
cfg = OPENARM_BI_CFG.replace(prim_path="/World/openarm"); cfg.spawn.usd_path = a.usd
robot = Articulation(cfg); sim.reset()
names = robot.joint_names; bodies = robot.body_names
print("num joints:", len(names)); print("joints:", names)
masses = robot.root_physx_view.get_masses()[0].cpu().numpy()
for i, b in enumerate(bodies):
    if b.endswith("_hand") or b.endswith("_ee_tcp") or b.endswith("link7"): print(f"  body {b:28s} m={masses[i]:.4f} kg")
robot.write_joint_state_to_sim(torch.zeros(1, len(names), device=sim.device), torch.zeros(1, len(names), device=sim.device)); sim.step(); robot.update(0.002)
M = robot.root_physx_view.get_generalized_mass_matrices()[0].cpu().numpy(); off = M.shape[0] - len(names)
for n in names:
    if "right_joint" in n: print(f"  M[{n}] = {M[off + names.index(n), off + names.index(n)]:.5f}")
for _ in range(50): sim.step()  # 50 스텝 안정성
print("50 steps OK, joint_pos max |q| =", float(robot.data.joint_pos.abs().max()))
import os, threading; threading.Timer(5, lambda: os._exit(0)).start(); app.close()
