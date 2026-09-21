"""excite.py 용 안전 감시. 매 틱 check() 를 호출하고, 반환된 MITParam 스케일/정지 지시를 따른다.

정지 규칙 (하나라도 걸리면 abort):
  - 피드백 err_code 가 enabled(0x1) 가 아님
  - 피드백 연속 미수신 > feedback_miss_ticks
  - |q| 가 hard 한계(q_min/q_max) 초과, |dq| > dq_max, T_MOS/T_Rotor 초과
소프트 벽: q 가 한계에서 soft_margin 이내로 들어오면 그 방향 토크를 선형 감쇠시킨다.
"""
from dataclasses import dataclass, field


@dataclass
class Limits:
    q_min: float
    q_max: float
    dq_max: float
    tau_max: float
    t_mos_max: float
    t_rotor_max: float
    soft_margin: float
    miss_ticks: int


@dataclass
class SafetyState:
    miss: int = 0
    abort_reason: str | None = None
    events: list = field(default_factory=list)


class Safety:
    def __init__(self, lim: Limits):
        self.lim = lim
        self.st = SafetyState()

    def clamp_cmd(self, q_des, dq_des, kp, kd, tau_ff, q_now):
        """명령을 한계 안으로 자르고, 소프트 벽 근처에서 tau_ff 를 감쇠한다."""
        L = self.lim
        q_des = min(max(q_des, L.q_min), L.q_max)
        dq_des = min(max(dq_des, -L.dq_max), L.dq_max)
        tau_ff = min(max(tau_ff, -L.tau_max), L.tau_max)
        # 소프트 벽: 벽 쪽으로 미는 토크만 감쇠
        d_lo, d_hi = q_now - L.q_min, L.q_max - q_now
        if tau_ff < 0 and d_lo < L.soft_margin:
            tau_ff *= max(d_lo, 0.0) / L.soft_margin
        if tau_ff > 0 and d_hi < L.soft_margin:
            tau_ff *= max(d_hi, 0.0) / L.soft_margin
        return q_des, dq_des, kp, kd, tau_ff

    def check(self, fb_ok: bool, q, dq, tau, t_mos, t_rotor, err_code, t_s) -> bool:
        """False 를 반환하면 즉시 정지해야 한다."""
        L, st = self.lim, self.st
        if not fb_ok:
            st.miss += 1
            if st.miss > L.miss_ticks:
                return self._abort(f"feedback missing {st.miss} ticks", t_s)
            return True
        st.miss = 0
        if err_code != 0x1:
            return self._abort(f"motor err_code=0x{err_code:x}", t_s)
        if q < L.q_min or q > L.q_max:
            return self._abort(f"q={q:.3f} out of [{L.q_min},{L.q_max}]", t_s)
        if abs(dq) > L.dq_max:
            return self._abort(f"|dq|={abs(dq):.2f} > {L.dq_max}", t_s)
        if t_mos > L.t_mos_max:
            return self._abort(f"T_MOS={t_mos} > {L.t_mos_max}", t_s)
        if t_rotor > L.t_rotor_max:
            return self._abort(f"T_Rotor={t_rotor} > {L.t_rotor_max}", t_s)
        return True

    def _abort(self, reason, t_s):
        self.st.abort_reason = reason
        self.st.events.append({"t_s": t_s, "abort": reason})
        return False
