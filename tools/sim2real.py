#!/usr/bin/env python3
"""WP5: 실물 홀드아웃(excite.py cmd.csv) vs Isaac 재생(single_joint_rig.py sim.csv) 잔차.

    sim2real.py <real_dir> <sim_dir> --motor DM4310 [--phase multisine] [--tau0 -0.76] [--pos-deg 2 --tau-frac 0.10]

합격 기준(기본): 위치 RMS ≤ 2°, 토크 RMS ≤ 10 % × 정격 토크(specs.yaml rated_torque_Nm).
출력: <sim_dir>/residual.yaml, residual.png
"""
import argparse, math
from pathlib import Path
import numpy as np, pandas as pd, yaml

ROOT = Path(__file__).resolve().parent.parent

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("real"); ap.add_argument("sim"); ap.add_argument("--motor", required=True)
    ap.add_argument("--phase", default="multisine"); ap.add_argument("--tau0", type=float, default=0.0, help="실물 토크 바이어스 (실측 τ 에서 뺌)")
    ap.add_argument("--pos-deg", type=float, default=2.0); ap.add_argument("--tau-frac", type=float, default=0.10)
    a = ap.parse_args()
    specs = yaml.safe_load((ROOT / "isaac/specs.yaml").read_text())["damiao"][a.motor]
    real = pd.read_csv(Path(a.real) / "cmd.csv"); real = real[real.phase == a.phase].reset_index(drop=True)
    sim = pd.read_csv(Path(a.sim) / "sim.csv")
    n = min(len(real), len(sim)); real, sim = real.iloc[:n], sim.iloc[:n]; t = np.arange(n) / 500.0
    # 명령 일치 확인 (재생이 같은 명령을 받았는지)
    cmd_err = float(np.abs(real.q_des.values - sim.q_des.values).max())
    tau_real = real.tau.values - a.tau0
    res = {"q_rad": real.q.values - sim.q.values, "dq_rads": real.dq.values - sim.dq.values, "tau_Nm": tau_real - sim.tau_applied.values}
    rms = {k: float(np.sqrt(np.mean(v ** 2))) for k, v in res.items()}; mx = {k: float(np.abs(v).max()) for k, v in res.items()}
    # 신호 크기 대비 (정규화 RMS)
    sig = {"q_rad": float(np.std(real.q.values)), "dq_rads": float(np.std(real.dq.values)), "tau_Nm": float(np.std(tau_real))}
    track_real = float(np.sqrt(np.mean((real.q_des.values - real.q.values) ** 2))); track_sim = float(np.sqrt(np.mean((sim.q_des.values - sim.q.values) ** 2)))
    tau_lim = a.tau_frac * specs["rated_torque_Nm"]
    verdict = {"pos_rms_deg": math.degrees(rms["q_rad"]), "pos_limit_deg": a.pos_deg, "pos_pass": math.degrees(rms["q_rad"]) <= a.pos_deg,
               "tau_rms_Nm": rms["tau_Nm"], "tau_limit_Nm": tau_lim, "tau_pass": rms["tau_Nm"] <= tau_lim,
               "tau_limit_note": f"{a.tau_frac*100:.0f}% × rated {specs['rated_torque_Nm']} Nm"}
    out = {"real": str(a.real), "sim": str(a.sim), "motor": a.motor, "phase": a.phase, "n": int(n), "cmd_match_max_rad": cmd_err, "tau0_subtracted": a.tau0,
           "residual_rms": rms, "residual_max": mx, "signal_std": sig, "nrms": {k: rms[k] / sig[k] if sig[k] else None for k in rms},
           "tracking_rms_deg": {"real": math.degrees(track_real), "sim": math.degrees(track_sim)}, "verdict": verdict}
    Path(a.sim, "residual.yaml").write_text(yaml.safe_dump(out, sort_keys=False, allow_unicode=True))
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    fig, ax = plt.subplots(4, 1, figsize=(12, 11), sharex=True)
    ax[0].plot(t, real.q_des, "k--", lw=0.7, label="q_des"); ax[0].plot(t, real.q, lw=0.9, label="real"); ax[0].plot(t, sim.q, lw=0.9, label="sim"); ax[0].set_ylabel("q [rad]"); ax[0].legend(loc="upper right")
    ax[1].plot(t, real.dq, lw=0.8, label="real"); ax[1].plot(t, sim.dq, lw=0.8, label="sim"); ax[1].set_ylabel("dq [rad/s]"); ax[1].legend(loc="upper right")
    ax[2].plot(t, tau_real, lw=0.8, label=f"real (−tau0 {a.tau0:+.2f})"); ax[2].plot(t, sim.tau_applied, lw=0.8, label="sim"); ax[2].set_ylabel("τ [Nm]"); ax[2].legend(loc="upper right")
    ax[3].plot(t, np.degrees(res["q_rad"]), lw=0.8, label=f"Δq  rms {math.degrees(rms['q_rad']):.2f}°"); ax[3].plot(t, res["tau_Nm"], lw=0.8, label=f"Δτ  rms {rms['tau_Nm']:.3f} Nm"); ax[3].axhline(0, c="k", lw=0.5); ax[3].set_ylabel("residual"); ax[3].legend(loc="upper right"); ax[3].set_xlabel("t [s]")
    fig.suptitle(f"{a.motor} sim2real  pos {'PASS' if verdict['pos_pass'] else 'FAIL'} ({verdict['pos_rms_deg']:.2f}°≤{a.pos_deg}°)  tau {'PASS' if verdict['tau_pass'] else 'FAIL'} ({rms['tau_Nm']:.3f}≤{tau_lim:.2f} Nm)")
    fig.tight_layout(); fig.savefig(Path(a.sim, "residual.png"), dpi=110)
    print(f"{a.motor}: n={n} cmd_match={cmd_err:.1e}")
    print(f"  Δq  rms {math.degrees(rms['q_rad']):.3f}° (max {math.degrees(mx['q_rad']):.2f}°)  nRMS {out['nrms']['q_rad']:.3f}  → {'PASS' if verdict['pos_pass'] else 'FAIL'} (≤{a.pos_deg}°)")
    print(f"  Δdq rms {rms['dq_rads']:.3f} rad/s  nRMS {out['nrms']['dq_rads']:.3f}")
    print(f"  Δτ  rms {rms['tau_Nm']:.4f} Nm (max {mx['tau_Nm']:.3f})  nRMS {out['nrms']['tau_Nm']:.3f}  → {'PASS' if verdict['tau_pass'] else 'FAIL'} (≤{tau_lim:.3f} Nm)")
    print(f"  추종오차 rms: real {math.degrees(track_real):.2f}°  sim {math.degrees(track_sim):.2f}°")

if __name__ == "__main__":
    main()
