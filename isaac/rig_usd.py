"""단일 관절 리그 USD 생성 (single_joint_rig.py / bench_all.py 공용).
고정 베이스 + 1 revolute + 막대 링크. 관절축 Z(중력 정렬) 또는 Y(수평, 진자)."""
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
RIG_USD = ROOT / "isaac" / "assets" / "single_joint_rig.usda"


def equivalent_rod(inertia, mgd, mass=0.5):
    """실물 관절 등가 막대: I = (4/3) m d², m g d = mgd  →  d = I / ((4/3)(mgd/g)), m = (mgd/g)/d, L = 2d.
    mgd 가 0.2 Nm 이하이면 중력 무시 가능 → 축을 중력과 정렬하고 관성만 맞춤. 반환 (mass, length, gravity_axis)."""
    if mgd and mgd > 0.2:
        md = mgd / 9.81; d = inertia / ((4.0 / 3.0) * md)
        return md / d, 2 * d, False
    return mass, math.sqrt(3 * inertia / mass), True


def build_rig_usd(mass, length, gravity_axis, path=RIG_USD):
    from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Gf
    path = Path(path); path.parent.mkdir(parents=True, exist_ok=True)
    stage = Usd.Stage.CreateNew(str(path)); UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z); UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/rig"); stage.SetDefaultPrim(root.GetPrim())

    # ArticulationRootAPI 는 강체(base)에 적용 — Isaac Lab 의 fix_root_link 가 이 링크를 월드에 고정한다
    base = UsdGeom.Xform.Define(stage, "/rig/base")
    UsdPhysics.RigidBodyAPI.Apply(base.GetPrim())
    UsdPhysics.ArticulationRootAPI.Apply(base.GetPrim()); PhysxSchema.PhysxArticulationAPI.Apply(base.GetPrim())
    UsdGeom.Cube.Define(stage, "/rig/base/geom").GetSizeAttr().Set(0.05)  # 시각용, 충돌 없음

    link = UsdGeom.Xform.Define(stage, "/rig/link")
    axis = "Z" if gravity_axis else "Y"
    # 링크 원점 = 관절. 막대는 관절에서 -Z 방향(진자) 또는 +X 방향(중력 정렬 축)으로 뻗음
    rod_offset = Gf.Vec3d(0, 0, -length / 2) if not gravity_axis else Gf.Vec3d(length / 2, 0, 0)
    UsdPhysics.RigidBodyAPI.Apply(link.GetPrim()); mapi = UsdPhysics.MassAPI.Apply(link.GetPrim())
    mapi.GetMassAttr().Set(mass); mapi.GetCenterOfMassAttr().Set(Gf.Vec3f(*rod_offset))
    I = mass * length**2 / 12.0  # 막대, 질량중심 기준 (PhysX 가 CoM 오프셋으로 평행축 처리)
    mapi.GetDiagonalInertiaAttr().Set(Gf.Vec3f(I, I, I * 0.01) if not gravity_axis else Gf.Vec3f(I * 0.01, I, I))
    cyl = UsdGeom.Cylinder.Define(stage, "/rig/link/geom"); cyl.GetRadiusAttr().Set(0.01); cyl.GetHeightAttr().Set(length)
    cyl.GetAxisAttr().Set("Z" if not gravity_axis else "X"); cyl.AddTranslateOp().Set(rod_offset)

    j = UsdPhysics.RevoluteJoint.Define(stage, "/rig/joint")
    j.GetAxisAttr().Set(axis); j.GetBody0Rel().SetTargets(["/rig/base"]); j.GetBody1Rel().SetTargets(["/rig/link"])
    j.GetLocalPos0Attr().Set(Gf.Vec3f(0, 0, 0)); j.GetLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
    j.GetLowerLimitAttr().Set(-170.0); j.GetUpperLimitAttr().Set(170.0)
    drv = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular"); drv.GetStiffnessAttr().Set(0.0); drv.GetDampingAttr().Set(0.0)
    stage.GetRootLayer().Save()
    return str(path)
