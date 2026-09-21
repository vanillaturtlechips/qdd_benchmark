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
    """삼각파 위치 램프. 속도 레벨마다 dur 초, [lo,hi] 에서 방향 반전. 레벨 전환 시 위치 연속 (적분식)."""
    def __init__(self, levels, dur, lo, hi, kp, kd):
        self.levels, self.dur, self.lo, self.hi, self.kp, self.kd = levels, dur, lo, hi, kp, kd
        self.duration = dur * len(levels); self.qd, self.dir, self.t_prev = lo, +1, 0.0
    def __call__(self, t, q, dq):
        i = min(int(t // self.dur), len(self.levels) - 1); v = self.levels[i]
        self.qd += self.dir * v * (t - self.t_prev); self.t_prev = t
        if self.qd >= self.hi: self.qd, self.dir = self.hi, -1
        elif self.qd <= self.lo: self.qd, self.dir = self.lo, +1
        return dict(q_des=self.qd, dq_des=self.dir * v, kp=self.kp, kd=self.kd, tau_ff=0.0, phase=f"v{v:g}")


class Chirp(Traj):
    """지수 처프 토크 여기. 약한 센터링 kp(기록됨)로 범위 이탈 방지."""
    def __init__(self, amp, f0, f1, dur, center, kp_c, kd_c):
        self.amp, self.f0, self.f1, self.duration, self.c, self.kp, self.kd = amp, f0, f1, dur, center, kp_c, kd_c
    def __call__(self, t, q, dq):
        k = math.log(self.f1 / self.f0)
        phase = 2 * math.pi * self.f0 * self.duration / k * (math.exp(k * t / self.duration) - 1)
        return dict(q_des=self.c, dq_des=0.0, kp=self.kp, kd=self.kd, tau_ff=self.amp * math.sin(phase), phase="chirp")


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
    """타깃 모터 1개만 초기화·enable. err_code/피드백 카운트는 python-can 스니퍼로 병행 수신."""
    PARAMS = ["KT_Value", "Gr", "Inertia", "Damp", "TMAX", "VMAX", "PMAX", "TIMEOUT", "MAX_SPD", "NPP", "Rs", "LS", "Flux", "sw_ver", "hw_ver"]

    def __init__(self, can_if, canfd, jcfg):
        import openarm_can as oa
        import can
        self.oa = oa
        self.arm = oa.OpenArm(can_if, canfd)
        self.arm.init_arm_motors([getattr(oa.MotorType, jcfg["motor"])], [jcfg["send_id"]], [jcfg["recv_id"]], [oa.ControlMode.MIT])
        self.arm.set_callback_mode_all(oa.CallbackMode.STATE)
        # get_motors() 는 복사본을 돌려주므로(std::vector<Motor> by value) 매번 새로 받아야 한다
        self.recv_id = jcfg["recv_id"]
        self.sniff = can.Bus(channel=can_if, interface="socketcan", fd=canfd, can_filters=[{"can_id": self.recv_id, "can_mask": 0x7FF}])
        self.err_code, self.fb_n = 0x0, 0

    def query_params(self):
        out = {}
        for name in self.PARAMS:
            rid = getattr(self.oa.MotorVariable, name).value
            try:
                self.arm.query_param_all(rid); self.arm.recv_all(2000)
                out[name] = float(self.arm.get_arm().get_motors()[0].get_param(rid))
            except Exception as e:  # 펌웨어에 없는 파라미터
                out[name] = f"n/a ({e})"
        return out

    def enable(self):
        self.arm.enable_all(); self.arm.recv_all()
        self.zero_cmd(); self.arm.recv_all()

    def zero_cmd(self):
        self.arm.get_arm().mit_control_one(0, self.oa.MITParam(0.0, 0.0, 0.0, 0.0, 0.0))

    def send(self, kp, kd, q, dq, tau):
        self.arm.get_arm().mit_control_one(0, self.oa.MITParam(kp, kd, q, dq, tau))

    def recv(self, timeout_us):
        self.arm.recv_all(timeout_us)
        self.fb_n = 0
        while True:
            m = self.sniff.recv(timeout=0)
            if m is None: break
            if m.arbitration_id == self.recv_id and len(m.data) >= 8:
                self.err_code = m.data[0] >> 4; self.fb_n += 1
        mt = self.arm.get_arm().get_motors()[0]
        return dict(q=mt.get_position(), dq=mt.get_velocity(), tau=mt.get_torque(), t_mos=mt.get_state_tmos(), t_rotor=mt.get_state_trotor(),
                    err_code=self.err_code, fb_n=self.fb_n)

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
    if a.mode == "const_vel":  return ConstVel([float(x) for x in a.levels.split(",")], a.dur, lo, hi, kp, kd)
    if a.mode == "chirp":      return Chirp(a.tau, a.f0, a.f1, a.dur, c, a.kp_center, a.kd_center)
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
    ap.add_argument("--levels", default="0.5,1,2,4"); ap.add_argument("--tau", type=float, default=0.5)
    ap.add_argument("--f0", type=float, default=0.1); ap.add_argument("--f1", type=float, default=20.0)
    ap.add_argument("--kp-center", type=float, default=2.0); ap.add_argument("--kd-center", type=float, default=0.0)
    ap.add_argument("--amp", type=float, default=0.3); ap.add_argument("--hold", type=float, default=2.0); ap.add_argument("--reps", type=int, default=3)
    ap.add_argument("--freqs", default="0.3,0.7,1.3,2.9"); ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--release-from", type=float, default=0.8)
    ap.add_argument("--goto-dur", type=float, default=2.0)
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
    meta = dict(arm=a.arm, joint=a.joint, motor=jcfg["motor"], mode=a.mode, args=vars(a), loop_hz=hz, safety=sj,
                start_utc=time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()), start_epoch=time.time())

    rig = Rig(arm["can_if"], arm["canfd"], jcfg)
    print(f"[excite] {a.arm}/{a.joint} ({jcfg['motor']}) on {arm['can_if']}  mode={a.mode}")
    try:
        meta["motor_params"] = rig.query_params(); print("[excite] motor params:", meta["motor_params"])
        rig.enable(); fb = rig.recv(2000)
    except Exception:
        rig.disable(); raise
    q_start = fb["q"]; meta["q_start"] = q_start; meta["t_mos_start"] = fb["t_mos"]; meta["t_rotor_start"] = fb["t_rotor"]
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
