"""enactic openarm_isaac_lab 의 physics 레이어에서 hand/ee_tcp 관련 prim 의 authored 속성 전부 덤프."""
import sys, argparse
from isaaclab.app import AppLauncher
ap = argparse.ArgumentParser(); ap.add_argument("path"); AppLauncher.add_app_launcher_args(ap); a = ap.parse_args(); app = AppLauncher(a).app
from pxr import Usd, UsdPhysics, Sdf
path = a.path
layer = Sdf.Layer.FindOrOpen(path); stage = Usd.Stage.Open(path)
print("layer:", path, "| root prims:", [p.name for p in layer.rootPrims][:5])
def dump(prim):
    print(f"\n{prim.GetPath()}  type={prim.GetTypeName()}  schemas={list(prim.GetAppliedSchemas())}")
    for a in prim.GetAttributes():
        if a.HasAuthoredValue(): print(f"    {a.GetName()} = {a.Get()}")
    for r in prim.GetRelationships():
        if r.HasAuthoredTargets(): print(f"    rel {r.GetName()} -> {[t.pathString for t in r.GetTargets()]}")
for p in stage.Traverse():
    n = p.GetPath().pathString
    if "right" in n and any(k in n for k in ("_hand", "ee_tcp", "link7")) and "visuals" not in n and "collisions" not in n:
        dump(p)

import os, threading; threading.Timer(5, lambda: os._exit(0)).start(); app.close()
