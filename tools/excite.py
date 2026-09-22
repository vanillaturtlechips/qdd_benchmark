#!/usr/bin/env python3
"""단일 축 MIT 모드 실험 궤적 발생기 (ros2_control 우회, openarm_can 파이썬 바인딩 사용).

  excite.py --arm right --joint joint7 --mode hold       --dur 10
  excite.py --arm right --joint joint7 --mode const_vel  --levels 0.5,1,2,4 --dur 4
  excite.py --arm right --joint joint7 --mode chirp      --tau 0.8 --f0 0.1 --f1 20 --dur 30
  excite.py --arm right --joint joint7 --mode step       --amp 0.3 --hold 2 --reps 3
  excite.py --arm right --joint joint7 --mode multisine  --freqs 0.3,0.7,1.3,2.9 --amp 0.4 --dur 60
  excite.py --arm right --joint joint7 --mode free_decay --release_from 0.8 --dur 6
  공통: --out data/<dir>  --dry-run(플롯만)  --kp/--kd(기본: 운용값)  --center(기본: 시작 위치)

전제: 해당 팔의 controller_manager 가 내려가 있어야 한다 (CAN 소켓 배타). 타깃 모터 1개만 enable 한다.
출력: <out>/cmd.csv  (t_s, phase, q_des, dq_des, kp, kd, tau_ff, q, dq, tau, t_mos, t_rotor, err_code, fb_n)
      <out>/meta.yaml (궤적 파라미터, 모터 펌웨어 파라미터, 시작/종료 온도, abort 사유)
"""
import argparse, csv, math, signal, sys, time
from pathlib import Path

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from safety import Limits, Safety  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent


# ---------------------------------------------------------------- 궤적 정의
class Traj:
    """t(s) -> dict(q_des, dq_des, kp, kd, tau_ff, phase). duration 속성 필요."""
    duration = 0.0
    def __call__(self, t, q_now, dq_now): raise NotImplementedError


class Hold(Traj):
    def __init__(self, q0, kp, kd, dur): self.q0, self.kp, self.kd, self.duration = q0, kp, kd, dur
    def __call__(self, t, q, dq): return dict(q_des=self.q0, dq_des=0.0, kp=self.kp, kd=self.kd, tau_ff=0.0, phase="hold")


class ConstVel(Traj):
    """사다리꼴 속도 프로파일 위치 램프. 속도 레벨마다 dur 초, [lo,hi] 에서 반전하되 가속도를 a_max 로 제한
    (삼각파의 순간 반전은 기어 백래시 충격을 일으킴 — 2026-09-21 joint7 실험). 레벨 전환 시 위치·속도 연속."""
    def __init__(self, levels, dur, lo, hi, kp, kd, a_max=10.0):
        self.levels, self.dur, self.lo, self.hi, self.kp, self.kd, self.a = levels, dur, lo, hi, kp, kd, a_max
        self.duration = dur * len(levels); self.qd, self.vd, self.dir, self.t_prev = lo, 0.0, +1, 0.0
    def __call__(self, t, q, dq):
        i = min(int(t // self.dur), len(self.levels) - 1); v = self.levels[i]; dt = t - self.t_prev; self.t_prev = t
        # 반전 판단: 현재 속도로 감속했을 때 멈추는 거리 d = v²/(2a) 만큼 벽 앞에서 방향 전환 시작
        stop_d = self.vd ** 2 / (2 * self.a)
        if self.dir > 0 and self.qd + stop_d >= self.hi: self.dir = -1
        elif self.dir < 0 and self.qd - stop_d <= self.lo: self.dir = +1
        target = self.dir * v
        dv = max(-self.a * dt, min(self.a * dt, target - self.vd)); self.vd += dv
        self.qd = min(max(self.qd + self.vd * dt, self.lo), self.hi)
        return dict(q_des=self.qd, dq_des=self.vd, kp=self.kp, kd=self.kd, tau_ff=0.0, phase=f"v{v:g}")


class Chirp(Traj):
    """지수 처프 토크 여기. 약한 센터링 kp/kd(기록됨)로 범위 이탈 방지.
    grav=(g_sin, g_cos) 를 주면 중력 토크를 피드포워드로 보상 → 센터링 kp 를 낮춰 공진(√(kp/J))을 대역 밖으로 보낼 수 있다."""
    def __init__(self, amp, f0, f1, dur, center, kp_c, kd_c, grav=(0.0, 0.0)):
        self.amp, self.f0, self.f1, self.duration, self.c, self.kp, self.kd, self.g = amp, f0, f1, dur, center, kp_c, kd_c, grav
    def __call__(self, t, q, dq):
        k = math.log(self.f1 / self.f0)
        phase = 2 * math.pi * self.f0 * self.duration / k * (math.exp(k * t / self.duration) - 1)
        tau_g = self.g[0] * math.sin(q) + self.g[1] * math.cos(q)
        return dict(q_des=self.c, dq_des=0.0, kp=self.kp, kd=self.kd, tau_ff=self.amp * math.sin(phase) + tau_g, phase="chirp")


class Step(Traj):
    """PD 위치 스텝 응답: center ± amp 를 hold 초씩 reps 회."""
    def __init__(self, amp, hold, reps, center, kp, kd):
        self.amp, self.hold, self.c, self.kp, self.kd = amp, hold, center, kp, kd
        self.seq = [0, +1, 0, -1] * reps; self.duration = hold * len(self.seq)
    def __call__(self, t, q, dq):
        i = min(int(t // self.hold), len(self.seq) - 1)
        return dict(q_des=self.c + self.seq[i] * self.amp, dq_des=0.0, kp=self.kp, kd=self.kd, tau_ff=0.0, phase=f"step{self.seq[i]:+d}")


class MultiSine(Traj):
    """홀드아웃 궤적. 위상은 seed 고정 → Isaac 재생 시 동일 궤적 재현."""
    def __init__(self, freqs, amp, dur, center, kp, kd, seed=0):
        rng = np.random.default_rng(seed)
        self.f = np.array(freqs); self.ph = rng.uniform(0, 2 * np.pi, len(freqs))
        self.a = amp / len(freqs); self.duration, self.c, self.kp, self.kd = dur, center, kp, kd
        self.ramp = 2.0  # 시작 2초 진폭 램프
    def __call__(self, t, q, dq):
        w = 2 * np.pi * self.f; r = min(t / self.ramp, 1.0)
        qd = self.c + r * float(np.sum(self.a * np.sin(w * t + self.ph)))
        vd = r * float(np.sum(self.a * w * np.cos(w * t + self.ph)))
        return dict(q_des=qd, dq_des=vd, kp=self.kp, kd=self.kd, tau_ff=0.0, phase="multisine")


class FreeDecay(Traj):
    """release_from 에서 hold_s 초 유지 후 kp=kd=tau=0 로 놓음 (중력 진자 감쇠 / 백드라이브)."""
    def __init__(self, release_from, hold_s, dur, kp, kd):
        self.q0, self.hold_s, self.duration, self.kp, self.kd = release_from, hold_s, hold_s + dur, kp, kd
    def __call__(self, t, q, dq):
        if t < self.hold_s: return dict(q_des=self.q0, dq_des=0.0, kp=self.kp, kd=self.kd, tau_ff=0.0, phase="pre_hold")
        return dict(q_des=0.0, dq_des=0.0, kp=0.0, kd=0.0, tau_ff=0.0, phase="free")


class GoTo(Traj):
    """현재 위치 → 목표까지 min-jerk 램프 (실험 전 준비 단계)."""
    def __init__(self, q_from, q_to, dur, kp, kd):
        self.a, self.b, self.duration, self.kp, self.kd = q_from, q_to, dur, kp, kd
    def __call__(self, t, q, dq):
        s = min(t / self.duration, 1.0); p = 10 * s**3 - 15 * s**4 + 6 * s**5
        dp = (30 * s**2 - 60 * s**3 + 30 * s**4) / self.duration
        return dict(q_des=self.a + (self.b - self.a) * p, dq_des=(self.b - self.a) * dp, kp=self.kp, kd=self.kd, tau_ff=0.0, phase="goto")


# ---------------------------------------------------------------- 하드웨어
class Rig:
    """타깃 모터(+선택적으로 hold 모터들)를 초기화·enable. 명령은 openarm_can 바인딩으로 보내고,
    **상태는 python-can 스니퍼로 직접 받아 자체 디코딩**한다 (라이브러리는 CTRL_MODE 쓰기 응답 등
    파라미터 프레임을 STATE 로 오파싱해 get_position() 이 -12.47 rad 를 돌려준 사고가 있었음, 2026-09-22)."""
    PARAMS = ["KT_Value", "Gr", "Inertia", "Damp", "TMAX", "VMAX", "PMAX", "TIMEOUT", "MAX_SPD", "NPP", "Rs", "LS", "Flux", "sw_ver", "hw_ver"]
    HOLD_DEV_MAX = 0.15   # rad, hold 축 허용 편차
    HOLD_DQ_MAX = 2.0     # rad/s
    HOLD_RAMP_S = 2.0     # hold 게인 램프 시간

    def __init__(self, can_if, canfd, jcfg, motor_limits, hold_joints=None, hz=500):
        import openarm_can as oa
        import can
        from collections import deque
        self.oa, self.hz = oa, hz
        self.arm = oa.OpenArm(can_if, canfd)
        self.hold = hold_joints or []            # [(jcfg, kp, kd)]
        self.joints = [jcfg] + [h[0] for h in self.hold]
        types = [getattr(oa.MotorType, j["motor"]) for j in self.joints]
        self.arm.init_arm_motors(types, [j["send_id"] for j in self.joints], [j["recv_id"] for j in self.joints], [oa.ControlMode.MIT] * len(types))
        self.arm.set_callback_mode_all(oa.CallbackMode.STATE)
        self.lim = {j["recv_id"]: motor_limits[j["motor"]] for j in self.joints}
        self.sid_of = {j["recv_id"]: j["send_id"] for j in self.joints}
        self.recv_id = jcfg["recv_id"]
        self.sniff = can.Bus(channel=can_if, interface="socketcan", fd=canfd,
                             can_filters=[{"can_id": j["recv_id"], "can_mask": 0x7FF} for j in self.joints])
        self.hist = {j["recv_id"]: deque(maxlen=50) for j in self.joints}   # 검증된 상태 프레임 (q, dq, tau, tmos, trotor, err)
        self.fb_n = 0
        self.hold_q, self.ramp_tick, self.hold_fault = None, 0, None

    # ---- 프레임 디코딩 (candump2csv.py 와 동일 규칙) ----
    @staticmethod
    def _u2f(u, lo, hi, bits): return lo + (hi - lo) * u / ((1 << bits) - 1)

    def _decode(self, rid, data):
        """상태 프레임이면 dict, 파라미터/기타 프레임이면 None."""
        if len(data) < 8: return None
        if data[1] == 0 and data[2] in (0x33, 0x55) and data[0] == (self.sid_of[rid] & 0xFF): return None  # 파라미터 응답
        if (data[0] & 0x0F) != (self.sid_of[rid] & 0x0F): return None  # 하위 4bit = 모터 ID
        L = self.lim[rid]
        q = self._u2f((data[1] << 8) | data[2], -L["p_max"], L["p_max"], 16)
        dq = self._u2f((data[3] << 4) | (data[4] >> 4), -L["v_max"], L["v_max"], 12)
        tau = self._u2f(((data[4] & 0xF) << 8) | data[5], -L["t_max"], L["t_max"], 12)
        return dict(q=q, dq=dq, tau=tau, t_mos=data[6], t_rotor=data[7], err_code=data[0] >> 4)

    def _drain(self):
        n_target = 0
        while True:
            m = self.sniff.recv(timeout=0)
            if m is None: break
            st = self._decode(m.arbitration_id, bytes(m.data))
            if st is None: continue
            self.hist[m.arbitration_id].append(st)
            if m.arbitration_id == self.recv_id: n_target += 1
        return n_target

    def latest(self, rid):
        h = self.hist[rid]; return h[-1] if h else None

    # ---- 파라미터 (hold 없을 때만) ----
    def query_params(self):
        assert not self.hold, "query_params() 는 hold 와 함께 쓰지 말 것 (query_param_all 이 전체 모터에 브로드캐스트)"
        out = {}
        self.arm.set_callback_mode_all(self.oa.CallbackMode.PARAM)
        for name in self.PARAMS:
            rid = getattr(self.oa.MotorVariable, name).value
            try:
                for _ in range(3):
                    self.arm.query_param_all(rid); self.arm.recv_all(5000)
                    val = float(self.arm.get_arm().get_motors()[0].get_param(rid))
                    if val != -1.0: break
                out[name] = val
            except Exception as e:
                out[name] = f"n/a ({e})"
        self.arm.set_callback_mode_all(self.oa.CallbackMode.STATE)
        self._drain(); [h.clear() for h in self.hist.values()]  # 파라미터 질의 중 프레임은 버림
        return out

    # ---- enable / 정착 ----
    def zero_cmd(self):
        for i in range(len(self.joints)):
            self.arm.get_arm().mit_control_one(i, self.oa.MITParam(0.0, 0.0, 0.0, 0.0, 0.0))

    def enable(self, settle_ticks=250):
        """enable 후 kp=kd=τ=0 으로 settle_ticks 동안 대기하며 모든 모터의 검증된 상태 프레임을 모은다.
        각 모터 ≥5 프레임, q 편차 < 0.005 rad, |q| ≤ π 를 만족해야 통과. 실패 시 disable 후 예외."""
        self._drain(); [h.clear() for h in self.hist.values()]
        self.arm.enable_all(); self.arm.recv_all(2000 * len(self.joints))
        for _ in range(settle_ticks):
            self.zero_cmd(); self.arm.recv_all(500); self._drain(); time.sleep(1.0 / self.hz)
        table, bad = [], []
        for j in self.joints:
            h = list(self.hist[j["recv_id"]])[-10:]
            if len(h) < 5: bad.append(f"{j.get('name', j['recv_id'])}: 상태 프레임 {len(h)}개 (<5)"); continue
            qs = [x["q"] for x in h]; spread = max(qs) - min(qs); q = sum(qs) / len(qs)
            ok = spread < 0.005 and abs(q) <= math.pi and all(x["err_code"] in (0x0, 0x1) for x in h)
            table.append((j, q, spread, h[-1]["t_mos"], h[-1]["err_code"], ok))
            if not ok: bad.append(f"{j.get('name', j['recv_id'])}: q={q:+.3f} spread={spread:.4f} err=0x{h[-1]['err_code']:x}")
        if bad:
            self.disable(); raise SystemExit("[excite] enable 정착 실패 → disable:\n  " + "\n  ".join(bad))
        self.hold_q = [q for (j, q, *_ ) in table[1:]]
        return table

    # ---- 명령 ----
    def send(self, kp, kd, q, dq, tau):
        self.arm.get_arm().mit_control_one(0, self.oa.MITParam(kp, kd, q, dq, tau))
        if self.hold:
            r = min(self.ramp_tick / (self.HOLD_RAMP_S * self.hz), 1.0); self.ramp_tick += 1
            for i, (hj, hkp, hkd) in enumerate(self.hold):
                self.arm.get_arm().mit_control_one(1 + i, self.oa.MITParam(hkp * r, hkd * r, self.hold_q[i], 0.0, 0.0))

    def recv(self, timeout_us):
        self.arm.recv_all(timeout_us)
        self.fb_n = self._drain()
        st = self.latest(self.recv_id) or dict(q=float("nan"), dq=0.0, tau=0.0, t_mos=0, t_rotor=0, err_code=0x0)
        # hold 축 감시
        if self.hold and self.hold_fault is None:
            for i, (hj, *_ ) in enumerate(self.hold):
                h = self.latest(hj["recv_id"])
                if h is None: continue
                if abs(h["q"] - self.hold_q[i]) > self.HOLD_DEV_MAX or abs(h["dq"]) > self.HOLD_DQ_MAX or h["err_code"] not in (0x0, 0x1):
                    self.hold_fault = f"hold {hj.get('name', hj['recv_id'])}: q={h['q']:+.3f} (target {self.hold_q[i]:+.3f}) dq={h['dq']:+.2f} err=0x{h['err_code']:x}"
        return dict(q=st["q"], dq=st["dq"], tau=st["tau"], t_mos=st["t_mos"], t_rotor=st["t_rotor"], err_code=st["err_code"], fb_n=self.fb_n)

    def disable(self):
        try:
            self.zero_cmd(); self.arm.recv_all()
            self.arm.disable_all(); self.arm.recv_all()
        finally:
            self.sniff.shutdown()


# ---------------------------------------------------------------- 메인
def build_traj(a, sf, q_now):
    kp = a.kp if a.kp is not None else sf["kp"]; kd = a.kd if a.kd is not None else sf["kd"]
    c = a.center if a.center is not None else q_now
    lo, hi = sf["q_min"] + sf["margin"], sf["q_max"] - sf["margin"]
    if a.mode == "hold":       return Hold(c, kp, kd, a.dur)
    if a.mode == "const_vel":  return ConstVel([float(x) for x in a.levels.split(",")], a.dur, lo, hi, kp, kd, a.a_max)
    if a.mode == "chirp":      return Chirp(a.tau, a.f0, a.f1, a.dur, c, a.kp_center, a.kd_center, tuple(float(x) for x in a.grav.split(",")))
    if a.mode == "step":       return Step(a.amp, a.hold, a.reps, c, kp, kd)
    if a.mode == "multisine":  return MultiSine([float(x) for x in a.freqs.split(",")], a.amp, a.dur, c, kp, kd, a.seed)
    if a.mode == "free_decay": return FreeDecay(a.release_from, 2.0, a.dur, kp, kd)
    raise SystemExit(f"unknown mode {a.mode}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--arm", required=True, choices=["left", "right"]); ap.add_argument("--joint", required=True)
    ap.add_argument("--mode", required=True, choices=["hold", "const_vel", "chirp", "step", "multisine", "free_decay"])
    ap.add_argument("--out", default=None); ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--motors", default=str(ROOT / "config/motors.yaml"))
    ap.add_argument("--dur", type=float, default=10.0); ap.add_argument("--kp", type=float); ap.add_argument("--kd", type=float)
    ap.add_argument("--center", type=float, default=None, help="궤적 중심 위치 (기본: 시작 위치)")
    ap.add_argument("--levels", default="0.25,0.5,1,2,3"); ap.add_argument("--a-max", type=float, default=10.0, help="const_vel 반전 가속도 제한 rad/s²"); ap.add_argument("--tau", type=float, default=0.5)
    ap.add_argument("--f0", type=float, default=0.1); ap.add_argument("--f1", type=float, default=20.0)
    ap.add_argument("--kp-center", type=float, default=1.0); ap.add_argument("--kd-center", type=float, default=0.15)
    ap.add_argument("--grav", default="0,0", help="chirp 중력 보상 g_sin,g_cos (Nm), fit.yaml 의 값")
    ap.add_argument("--amp", type=float, default=0.3); ap.add_argument("--hold", type=float, default=2.0); ap.add_argument("--reps", type=int, default=3)
    ap.add_argument("--freqs", default="0.3,0.7,1.3,2.9"); ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--release-from", type=float, default=0.8)
    ap.add_argument("--goto-dur", type=float, default=2.0)
    ap.add_argument("--hold-others", action="store_true", help="타깃 외 관절(joint2~7)을 운용 kp/kd 로 정착 위치에 유지 (어깨 실험용). 시작 전 표 확인 필요")
    ap.add_argument("--hold-check", action="store_true", help="실기 dry-check: enable→정착→표 출력→disable 만 (게인 인가 없음)")
    ap.add_argument("--yes", action="store_true", help="hold 확인 프롬프트 생략")
    a = ap.parse_args()

    cfg = yaml.safe_load(Path(a.motors).read_text())
    jcfg = cfg["joints"][a.joint]; arm = cfg["arms"][a.arm]; sj = cfg["safety"]["per_joint"][a.joint]
    sf = dict(kp=jcfg["kp"], kd=jcfg["kd"], q_min=sj["q_min"], q_max=sj["q_max"], margin=cfg["safety"]["soft_margin_rad"])
    limits = Limits(sj["q_min"], sj["q_max"], sj["dq_max"], sj["tau_max"], cfg["safety"]["t_mos_max_C"], cfg["safety"]["t_rotor_max_C"],
                    cfg["safety"]["soft_margin_rad"], cfg["safety"]["feedback_miss_ticks"])
    hz = cfg["loop_hz"]; dt = 1.0 / hz

    if a.dry_run:
        q0 = a.center if a.center is not None else 0.0
        tr = build_traj(a, sf, q0)
        ts = np.arange(0, tr.duration, dt)
        rows = [tr(t, q0, 0.0) for t in ts]
        import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
        fig, ax = plt.subplots(3, 1, sharex=True, figsize=(10, 7))
        ax[0].plot(ts, [r["q_des"] for r in rows]); ax[0].axhline(sj["q_min"], c="r", ls="--"); ax[0].axhline(sj["q_max"], c="r", ls="--"); ax[0].set_ylabel("q_des [rad]")
        ax[1].plot(ts, [r["dq_des"] for r in rows]); ax[1].axhline(sj["dq_max"], c="r", ls="--"); ax[1].axhline(-sj["dq_max"], c="r", ls="--"); ax[1].set_ylabel("dq_des [rad/s]")
        ax[2].plot(ts, [r["tau_ff"] for r in rows]); ax[2].axhline(sj["tau_max"], c="r", ls="--"); ax[2].axhline(-sj["tau_max"], c="r", ls="--"); ax[2].set_ylabel("tau_ff [Nm]"); ax[2].set_xlabel("t [s]")
        fig.suptitle(f"{a.arm} {a.joint} {a.mode}  dur={tr.duration:.1f}s  kp={rows[0]['kp']} kd={rows[0]['kd']}")
        out = Path(a.out or ROOT / "data" / "dry_run"); out.mkdir(parents=True, exist_ok=True)
        p = out / f"dryrun_{a.arm}_{a.joint}_{a.mode}.png"; fig.savefig(p, dpi=110); print(f"dry-run plot → {p}")
        return

    out = Path(a.out or ROOT / "data" / f"{time.strftime('%Y-%m-%d_%H%M%S')}_{a.arm}_{a.joint}_{a.mode}")
    out.mkdir(parents=True, exist_ok=True)
    meta = dict(arm=a.arm, joint=a.joint, motor=jcfg["motor"], mode=a.mode, args=vars(a), loop_hz=hz, safety=sj, hold_others=a.hold_others,
                start_utc=time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()), start_epoch=time.time())

    hold = None
    if a.hold_others or a.hold_check:
        hold = [(dict(cfg["joints"][j], name=j), cfg["joints"][j]["kp"], cfg["joints"][j]["kd"]) for j in cfg["joints"] if j != a.joint and j != "gripper"]
    rig = Rig(arm["can_if"], arm["canfd"], dict(jcfg, name=a.joint), cfg["motor_limits"], hold, hz)
    if hold: print(f"[excite] hold-others: {len(hold)} joints will be held at their settled pose (gains ramp {Rig.HOLD_RAMP_S}s, dev>{Rig.HOLD_DEV_MAX} rad → abort)")
    print(f"[excite] {a.arm}/{a.joint} ({jcfg['motor']}) on {arm['can_if']}  mode={a.mode}")
    try:
        if hold:
            meta["motor_params"] = "skipped (--hold-others: query_param_all() would broadcast to all held motors and stall STATE feedback)"
            print("[excite] motor params: skipped (hold-others active)")
        else:
            meta["motor_params"] = rig.query_params(); print("[excite] motor params:", meta["motor_params"])
        table = rig.enable(); fb = rig.recv(2000)
    except SystemExit:
        raise
    except Exception:
        rig.disable(); raise
    print("[excite] settled pose (검증된 상태 프레임, kp=kd=0 상태):")
    for (j, q, spread, tmos, err, ok) in table:
        print(f"   {j['name']:8s} q={q:+.4f} rad ({math.degrees(q):+6.1f}°)  spread={spread:.4f}  T_MOS={tmos}°C  err=0x{err:x}  {'OK' if ok else 'BAD'}" + ("   ← target" if j is table[0][0] else "   ← HOLD"))
    if a.hold_check:
        print("[excite] --hold-check: 표만 확인하고 종료 (게인 인가 없음)"); rig.disable(); return
    if hold and not a.yes:
        try:
            ans = input("[excite] 위 hold 목표로 진행할까요? (y/N) ").strip().lower()
        except (KeyboardInterrupt, EOFError):
            ans = "n"; print()
        if ans != "y": rig.disable(); raise SystemExit("[excite] 사용자 취소 → disable 완료")
    q_start = fb["q"]
    if q_start != q_start: rig.disable(); raise SystemExit("[excite] 타깃 상태 프레임 미수신")
    meta["q_start"] = q_start; meta["t_mos_start"] = fb["t_mos"]; meta["t_rotor_start"] = fb["t_rotor"]
    print(f"[excite] q_start={q_start:+.3f} rad  T_MOS={fb['t_mos']}°C  err=0x{fb['err_code']:x}")
    if not (sj["q_min"] <= q_start <= sj["q_max"]):
        rig.disable(); raise SystemExit(f"시작 위치 {q_start:.3f} 가 안전 범위 [{sj['q_min']},{sj['q_max']}] 밖. 손으로 옮긴 뒤 재시도.")

    main_tr = build_traj(a, sf, q_start)
    q_first = main_tr(0.0, q_start, 0.0)["q_des"]
    phases = []
    if abs(q_first - q_start) > 0.01 and a.mode != "chirp":
        phases.append(GoTo(q_start, q_first, a.goto_dur, sf["kp"], sf["kd"]))
    phases.append(main_tr)
    meta["phases"] = [type(p).__name__ for p in phases]; meta["duration_s"] = sum(p.duration for p in phases)

    safety = Safety(limits); stop = {"flag": False}
    signal.signal(signal.SIGINT, lambda *_: stop.__setitem__("flag", True))
    cols = ["t_s", "phase", "q_des", "dq_des", "kp", "kd", "tau_ff", "q", "dq", "tau", "t_mos", "t_rotor", "err_code", "fb_n", "loop_late_us"]
    n_late = 0
    try:
        with open(out / "cmd.csv", "w", newline="") as f:
            w = csv.DictWriter(f, cols); w.writeheader()
            t0 = time.perf_counter(); tick = 0; q, dq = q_start, 0.0
            for tr in phases:
                t_ph0 = time.perf_counter()
                while True:
                    t_target = t0 + tick * dt; now = time.perf_counter()
                    if now < t_target: time.sleep(t_target - now)
                    late_us = (time.perf_counter() - t_target) * 1e6
                    if late_us > dt * 1e6: n_late += 1
                    t_ph = time.perf_counter() - t_ph0
                    if t_ph >= tr.duration or stop["flag"]: break
                    c = tr(t_ph, q, dq)
                    qd, vd, kp, kd, tf = safety.clamp_cmd(c["q_des"], c["dq_des"], c["kp"], c["kd"], c["tau_ff"], q)
                    rig.send(kp, kd, qd, vd, tf)
                    fb = rig.recv(cfg["recv_timeout_us"])
                    t_s = time.time(); q, dq = fb["q"], fb["dq"]
                    ok = safety.check(fb["fb_n"] > 0, q, dq, fb["tau"], fb["t_mos"], fb["t_rotor"], fb["err_code"], t_s)
                    if ok and rig.hold_fault:
                        safety.st.abort_reason = rig.hold_fault; ok = False
                    w.writerow(dict(t_s=f"{t_s:.6f}", phase=c["phase"], q_des=f"{qd:.5f}", dq_des=f"{vd:.5f}", kp=kp, kd=kd, tau_ff=f"{tf:.4f}",
                                    q=f"{q:.5f}", dq=f"{dq:.5f}", tau=f"{fb['tau']:.4f}", t_mos=fb["t_mos"], t_rotor=fb["t_rotor"],
                                    err_code=fb["err_code"], fb_n=fb["fb_n"], loop_late_us=f"{late_us:.0f}"))
                    tick += 1
                    if not ok:
                        print(f"\n[excite] ABORT: {safety.st.abort_reason}"); stop["flag"] = True; break
                    if tick % hz == 0:
                        print(f"\r  t={tick/hz:6.1f}s  {c['phase']:10s} q={q:+.3f} dq={dq:+.3f} tau={fb['tau']:+.3f} T={fb['t_mos']}°C late={n_late}", end="", flush=True)
                if stop["flag"]: break
    finally:
        print("\n[excite] disabling motor"); rig.disable()
        fb_end = dict(t_mos=fb["t_mos"], t_rotor=fb["t_rotor"]) if "fb" in dir() else {}
        meta.update(end_epoch=time.time(), ticks=tick, loop_late_ticks=n_late, abort_reason=safety.st.abort_reason,
                    interrupted=stop["flag"] and safety.st.abort_reason is None, t_end=fb_end, safety_events=safety.st.events)
        (out / "meta.yaml").write_text(yaml.safe_dump(meta, allow_unicode=True, sort_keys=False))
        print(f"[excite] → {out}  ticks={tick} late={n_late} abort={safety.st.abort_reason}")


if __name__ == "__main__":
    main()
