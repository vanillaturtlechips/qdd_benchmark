"""OpenArm bimanual USD 의 hand/ee_tcp/finger prim 을 직접 검사: 계층, 적용 API, physics:mass, 조인트 연결."""
import argparse
from isaaclab.app import AppLauncher
ap = argparse.ArgumentParser(); AppLauncher.add_app_launcher_args(ap); a = ap.parse_args(); app = AppLauncher(a).app
from pxr import Usd, UsdPhysics, UsdGeom
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR, retrieve_file_path
import isaaclab.sim as sim_utils
from isaaclab.sim import SimulationContext, SimulationCfg
from isaaclab_assets.robots.openarm import OPENARM_BI_CFG
sim = SimulationContext(SimulationCfg(dt=0.002, device=a.device))
usd = OPENARM_BI_CFG.spawn.usd_path; print("USD:", usd)
stage = Usd.Stage.Open(usd, Usd.Stage.LoadAll)   # 원격 URL 직접 오픈 → 참조/페이로드 해석
print("root layer:", stage.GetRootLayer().identifier)
for p in stage.Traverse():
    if p.HasAuthoredReferences() or p.HasAuthoredPayloads():
        print("  ref/payload prim:", p.GetPath())
print("prim count:", len(list(stage.Traverse())))
targets = [p for p in stage.Traverse() if any(k in p.GetName() for k in ("hand", "tcp", "finger", "link7"))]
print("\n=== prim 검사 (우측만) ===")
for p in targets:
    if "right" not in p.GetPath().pathString: continue
    apis = [s for s in p.GetAppliedSchemas()]
    mass = p.GetAttribute("physics:mass"); mval = mass.Get() if mass and mass.HasAuthoredValue() else None
    rb = UsdPhysics.RigidBodyAPI(p) if p.HasAPI(UsdPhysics.RigidBodyAPI) else None
    print(f"{p.GetPath()}\n    type={p.GetTypeName()}  RigidBodyAPI={p.HasAPI(UsdPhysics.RigidBodyAPI)}  MassAPI={p.HasAPI(UsdPhysics.MassAPI)}  physics:mass(authored)={mval}  children={[c.GetName() for c in p.GetChildren()][:6]}")
print("\n=== 조인트 (우측 link7 하류) ===")
for p in stage.Traverse():
    if not p.IsA(UsdPhysics.Joint): continue
    j = UsdPhysics.Joint(p); b0 = [t.pathString.split('/')[-1] for t in j.GetBody0Rel().GetTargets()]; b1 = [t.pathString.split('/')[-1] for t in j.GetBody1Rel().GetTargets()]
    if any("right" in x and any(k in x for k in ("link7", "hand", "tcp", "finger")) for x in b0 + b1):
        print(f"  {p.GetName():40s} {p.GetTypeName():22s} {b0} -> {b1}")
import os, threading; threading.Timer(8, lambda: os._exit(0)).start(); app.close()
