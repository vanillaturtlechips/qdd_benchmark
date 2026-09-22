"""비교용 액추에이터 프리셋 (단일 관절 리그용). 값의 출처는 isaac/specs.yaml.

원칙
- 프리셋은 **모터 물리 파라미터만** 담는다 (effort/saturation/velocity/armature/friction).
  PD 게인(stiffness/damping)은 제어기 선택이지 모터 속성이 아니므로 벤치가 궤적과 함께 통일해서 준다
  (single_joint_rig.py 가 매 틱 act.stiffness/damping 을 덮어씀). 여기 값은 초기값일 뿐.
- DM 계열: 데이터시트/펌웨어 초기값 → WP4 식별 결과로 갱신 (`identified=True` 로 표시).
- Unitree 팔: 공개 URDF effort/velocity + 공개 MJCF armature/frictionloss. 모두 사양 기반(실측 아님).

Isaac Lab DCMotorCfg 필드 ↔ 의미
  effort_limit       : 연속 토크 한계 (Nm)   ← 정격 토크
  saturation_effort  : 속도 0 최대 토크      ← 피크 토크
  velocity_limit     : 무부하 최고 속도 (rad/s)
  armature           : 반사 관성 (kg m²)      ← 로터 관성 × Gr²
  friction           : 정지 마찰 (Nm) — _dc 가 dynamic_friction(Coulomb) 에도 같은 값을 넣는다
  effort_limit 은 DCMotor 의 속도-토크 직선을 위에서 자르는 상한이라 정격으로 두면 피크에 절대 못 미친다.
  벤치(bench_all.py --effort peak, 기본)는 수 초 과도응답이므로 effort_limit=saturation_effort 로 통일한다.
"""
import math
from pathlib import Path

import yaml
from isaaclab.actuators import DCMotorCfg

SPECS = yaml.safe_load((Path(__file__).resolve().parent / "specs.yaml").read_text())
RPM = 2 * math.pi / 60


def _dc(effort, saturation, velocity, armature=0.0, friction=0.0, kp=10.0, kd=0.5):
    # Isaac Sim 5.x 관절 마찰은 (static=friction, dynamic=dynamic_friction, viscous) 3항. `friction` 만 주면 정지 마찰뿐이라
    # 움직이는 동안 Coulomb 마찰이 0 이 된다 (2026-09-22 벤치에서 G1 wrist 진자가 감쇠 없이 계속 도는 것으로 발견) → dynamic 도 같은 값으로 채운다
    return DCMotorCfg(joint_names_expr=[".*"], effort_limit=effort, saturation_effort=saturation, velocity_limit=velocity,
                      stiffness=kp, damping=kd, armature=armature, friction=friction, dynamic_friction=friction)


# ---- Damiao (OpenArm 실물). identified=False 인 동안은 데이터시트/펌웨어 값 ----
def dm4310():
    """[identified 2026-09-22, right joint7] data/2026-09-21_j7/identified_dm4310.yaml
    Coulomb 0.122 Nm, viscous 0.0246 실측. armature 는 펌웨어 1.689e-5×10² = 0.00169 — joint7 에서는 그리퍼 관성
    불확실성(URDF 대비 실측 중력 +33 %)이 armature 보다 커서 실측 분리 불가 (총 유효 관성 0.0059 는 홀드아웃으로 검증)."""
    s = SPECS["damiao"]["DM4310"]
    c = _dc(s["rated_torque_Nm"], s["peak_torque_Nm"], s["noload_speed_rpm"] * RPM, armature=0.00169, friction=0.1217)
    c.dynamic_friction = 0.1217; c.viscous_friction = 0.0246
    return c


def dm4340():
    """[identified 2026-09-22, right joint3] data/2026-09-22_j3/identified_dm4340.yaml
    armature 0.0256 (펌웨어 0.0290), Coulomb 0.482 Nm, viscous 0.188. 40:1 이라 백드라이브 사실상 없음."""
    s = SPECS["damiao"]["DM4340"]
    c = _dc(s["rated_torque_Nm"], s["peak_torque_Nm"], s["noload_speed_rpm"] * RPM, armature=0.02557, friction=0.4815)
    c.dynamic_friction = 0.4815; c.viscous_friction = 0.1878
    return c


def dm8009():
    """[identified 2026-09-22, right joint1] data/2026-09-22_j1/identified_dm8009.yaml
    Coulomb 0.95 Nm (11 Nm 중력 부하 하), viscous 0.10±0.05. armature 는 펌웨어 2.06e-4×81 (링크 관성 0.40 대비 4% 라 미실측)."""
    s = SPECS["damiao"]["DM8009"]
    c = _dc(s["rated_torque_Nm"], s["peak_torque_Nm"], s["noload_speed_rpm"] * RPM, armature=0.01669, friction=0.814)
    c.dynamic_friction = 0.814; c.viscous_friction = 0.10  # hold 조건(joint2~7 PD 유지) 재측정값. 비hold 는 0.95 (하류 관절 꿈틀거림 마찰 포함)
    return c


# ---- Unitree 팔 (사양 기반). effort 를 피크로 취급, 연속 토크 미공개 → effort_limit 도 같은 값 ----
def _unitree(robot, joint):
    j = SPECS[robot]["joints"][joint]
    mj = SPECS[robot].get("mjcf_joint_defaults", {}).get(j.get("mjcf_class"), {})
    return _dc(j["effort_Nm"], j["effort_Nm"], j["velocity_rads"],
               armature=mj.get("armature_kgm2", 0.0), friction=mj.get("frictionloss_Nm", 0.0))


# ---- 피팅 검증용 합성 모터: 정답을 아는 파라미터 (Isaac Sim 5.x: friction/dynamic_friction = Nm, viscous = Nm·s/rad) ----
SYNTH_TRUTH = dict(armature=0.003, coulomb=0.15, viscous=0.02)


def synth():
    c = _dc(effort=50.0, saturation=50.0, velocity=50.0, armature=SYNTH_TRUTH["armature"], friction=SYNTH_TRUTH["coulomb"])
    c.dynamic_friction = SYNTH_TRUTH["coulomb"]; c.viscous_friction = SYNTH_TRUTH["viscous"]
    return c


# ---- Robstride (K-Bot). 사양 + K-Scale MJCF armature/frictionloss ----
def _robstride(model, mjcf_class):
    r = SPECS["robstride"][model]; mj = SPECS["kscale_kbot_v2"]["mjcf_classes"][mjcf_class]
    return _dc(r["rated_torque_Nm"], r["peak_torque_Nm"], r["noload_speed_rpm"] * RPM, armature=mj["armature_kgm2"], friction=mj["frictionloss_Nm"])


# ---- MIT Humanoid (Table II, 관절 기준 벨트 증속 포함). 연속 토크 미공개 → effort=peak ----
def _mit(joint):
    j = SPECS["mit_humanoid"]["joints"][joint]; m = SPECS["mit_humanoid"]["modules"][j["module"]]
    ratio = j["total_ratio"] / m["gear_ratio"]  # 벨트 배율
    return _dc(j["torque_Nm"], j["torque_Nm"], j["speed_rads"], armature=m["output_inertia_kgm2"] * ratio**2, friction=0.0)


# ---- Berkeley Humanoid (Table 2). rotor_inertia × 9² = armature ----
def _berkeley(model):
    a = SPECS["berkeley_humanoid"]["actuators"][model]
    return _dc(a["sustained_torque_Nm"], a["peak_torque_Nm"], a["max_speed_rads_48V"], armature=a["rotor_inertia_kgm2"] * a["gear_ratio"] ** 2, friction=0.0)


# ---- Berkeley Humanoid Lite (공개 Isaac Lab cfg 값 그대로) ----
def _bhl(model):
    a = SPECS["berkeley_humanoid_lite"]["actuators"][model]
    return _dc(a["isaac_effort_limit_Nm"], a["isaac_effort_limit_Nm"], a["isaac_velocity_limit_rads"], armature=a["isaac_armature_kgm2"], friction=0.0)


PRESETS = {
    "synth": synth,
    # K-Bot / Robstride
    "rs00_wrist":    lambda: _robstride("RS00", "motor_00"),
    "rs02_elbow":    lambda: _robstride("RS02", "motor_02"),
    "rs03_shoulder": lambda: _robstride("RS03", "motor_03"),
    # MIT Humanoid 팔
    "mit_shoulder":  lambda: _mit("shoulder_flexion"),
    "mit_elbow":     lambda: _mit("elbow"),
    # Berkeley Humanoid (토크 클래스 매칭, 관절 역할 아님)
    "bh_5013":       lambda: _berkeley("5013"),
    "bh_8513":       lambda: _berkeley("8513"),
    # Berkeley Humanoid Lite
    "bhl_5010_arm":  lambda: _bhl("5010"),
    "bhl_6512_leg":  lambda: _bhl("6512"),
    # OpenArm 실물
    "dm4310": dm4310, "dm4340": dm4340, "dm8009": dm8009,
    # 주 비교: G1 29DoF 팔
    "g1_shoulder":    lambda: _unitree("unitree_g1_29dof", "shoulder_pitch"),
    "g1_elbow":       lambda: _unitree("unitree_g1_29dof", "elbow"),
    "g1_wrist_roll":  lambda: _unitree("unitree_g1_29dof", "wrist_roll"),
    "g1_wrist_pitch": lambda: _unitree("unitree_g1_29dof", "wrist_pitch"),
    # 보조: H1 / H1-2 팔
    "h1_shoulder":    lambda: _unitree("unitree_h1", "shoulder_pitch"),
    "h1_elbow":       lambda: _unitree("unitree_h1", "elbow"),
    "h12_wrist":      lambda: _unitree("unitree_h1_2", "wrist_roll"),
}

# 클래스 매칭 (보고서 비교 축): 같은 역할의 관절끼리
CLASS_MATCH = {
    "shoulder": ["dm8009", "rs03_shoulder", "mit_shoulder", "bh_8513", "g1_shoulder", "h1_shoulder"],       # OpenArm joint1-2 (20~60 Nm급)
    "elbow":    ["dm4340", "rs02_elbow", "mit_elbow", "g1_elbow", "h1_elbow"],                             # OpenArm joint3-4
    "wrist":    ["dm4310", "rs00_wrist", "bh_5013", "bhl_5010_arm", "g1_wrist_roll", "g1_wrist_pitch", "h12_wrist"],  # OpenArm joint5-7 (5~15 Nm급)
}
