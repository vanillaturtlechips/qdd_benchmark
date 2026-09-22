#!/usr/bin/env python3
"""WP6 지표: bench_all.py 출력(data/sim/bench/<class>/<preset>/*.csv) → 지표 표 + 클래스별 오버레이 그림.

    .venv/bin/python isaac/bench_metrics.py [--root data/sim/bench]

지표 정의 (클래스 내 동일 리그·동일 PD 이므로 모터 파라미터 차이만 반영)
  step       : 10–90 % 상승시간, 오버슈트 %, 5 % 정착시간, 정상상태 오차(마지막 0.2 s), 피크 토크, 포화 비율 — 전 전이 평균
  chirp      : 위치 기준 선형 처프의 폐루프 FRF H=q/q_des (알려진 처프 위상으로 락인 복조, 3주기 창) → −3 dB 대역폭, −90° 주파수, 공진 피크 dB
  multisine  : 램프 후(t≥3 s) 추종 RMS/최대 오차(deg), 토크 RMS/피크, 포화 비율
  free_decay : 놓은 뒤 이동 여부·이동량·정지시간·스윙 수·정지각(진자 리그: Coulomb 스톨 각). 중력 정렬 리그(elbow)는 2 rad/s 코스트다운
  파생       : τpk/armature (모터 고유), τpk/(armature+I_link) (그 관절에서의 가속 능력), armature/I_link
출력: <root>/metrics.csv, isaac/bench_metrics.md, <root>/plots/<class>.png
"""
import argparse, math
from pathlib import Path
import numpy as np, pandas as pd, yaml

ROOT = Path(__file__).resolve().parent.parent
CHIRP_F1 = {"wrist": 20, "elbow": 40, "shoulder": 8}
SENS_ROOTS = [("zero", "전 프리셋 마찰 0", ROOT / "data/sim/bench_fric_zero"), ("floor02", "Coulomb 하한 2 %·τpk", ROOT / "data/sim/bench_fric_floor02")]


def FRIC_SRC(preset, coulomb):
    if preset.startswith("dm"): return "M 실측"
    if preset.startswith("g1") or preset.startswith("rs"): return "S 범용값"
    return "미공개(0)" if coulomb == 0 else "S"
PALETTE = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#4a3aa7", "#e34948"]  # dataviz 기본 범주 팔레트, 고정 순서


def step_metrics(d, tau_lim):
    t, q, qd, tau = d.t_s.values, d.q.values, d.q_des.values, d.tau_applied.values
    edges = np.flatnonzero(np.diff(qd) != 0) + 1
    rows = []
    for k, e in enumerate(edges):
        e2 = edges[k + 1] if k + 1 < len(edges) else len(t)
        q0, q1 = q[e - 1], qd[e]; dlt = q1 - q0
        if abs(dlt) < 1e-6: continue
        seg = (q[e:e2] - q0) / dlt; ts = t[e:e2] - t[e]
        i10 = np.argmax(seg >= 0.1); i90 = np.argmax(seg >= 0.9)
        rise = ts[i90] - ts[i10] if seg.max() >= 0.9 else np.nan
        over = max(seg.max() - 1.0, 0.0) * 100
        out = np.flatnonzero(np.abs(seg - seg[-1]) > 0.05); settle = ts[out[-1] + 1] if len(out) and out[-1] + 1 < len(ts) else (np.nan if len(out) else 0.0)
        ss = abs(q1 - q[e2 - 100:e2].mean())
        tp = np.abs(tau[e:e2]).max(); sat = np.mean(np.abs(tau[e:e2]) >= 0.98 * tau_lim)
        rows.append((rise, over, settle, ss, tp, sat))
    m = np.nanmean(np.array(rows), axis=0)
    return dict(step_rise_ms=m[0] * 1e3, step_overshoot_pct=m[1], step_settle_ms=m[2] * 1e3, step_ss_err_deg=math.degrees(m[3]), step_tau_peak=m[4], step_sat_frac=m[5])


def chirp_frf(d, f0, f1, T, cycles=5.0):
    t, q, qd = d.t_s.values, d.q.values, d.q_des.values
    dt = t[1] - t[0]; ph = 2 * np.pi * (f0 * t + (f1 - f0) * t * t / (2 * T)); ref = np.exp(-1j * ph)
    fs, H = [], []
    for tc in np.arange(0.5, T - 0.5, 0.1):
        f = f0 + (f1 - f0) * tc / T; L = max(cycles / f, 0.5) / 2
        m = (t >= tc - L) & (t <= tc + L)
        if m.sum() < 10: continue
        num = np.sum(q[m] * ref[m]); den = np.sum(qd[m] * ref[m])
        if abs(den) < 1e-9: continue
        fs.append(f); H.append(num / den)
    fs, H = np.array(fs), np.array(H); mag = np.abs(H); phase = np.degrees(np.unwrap(np.angle(H)))
    def cross(y, thr, below=True):
        idx = np.flatnonzero((y < thr) if below else (y > thr))
        if not len(idx) or idx[0] == 0: return np.nan
        i = idx[0]; return float(fs[i - 1] + (fs[i] - fs[i - 1]) * (thr - y[i - 1]) / (y[i] - y[i - 1]))
    ipk = int(np.argmax(mag))
    return fs, mag, phase, dict(bw_3db_hz=cross(mag, 1 / math.sqrt(2)), f_90deg_hz=cross(phase, -90.0), res_peak_db=20 * math.log10(mag[ipk]), res_freq_hz=float(fs[ipk]) if mag[ipk] > 1.05 else np.nan)


def multisine_metrics(d, tau_lim, t_min=3.0):
    s = d[d.t_s >= t_min]; e = s.q_des.values - s.q.values; tau = s.tau_applied.values
    return dict(ms_track_rms_deg=math.degrees(np.sqrt(np.mean(e ** 2))), ms_track_max_deg=math.degrees(np.abs(e).max()),
                ms_tau_rms=float(np.sqrt(np.mean(tau ** 2))), ms_tau_peak=float(np.abs(tau).max()), ms_sat_frac=float(np.mean(np.abs(tau) >= 0.98 * tau_lim)))


def free_decay_metrics(d, gravity_axis, thr=0.02):
    fr = d[d.phase == "free"]; t, q, dq = fr.t_s.values - fr.t_s.values[0], fr.q.values, fr.dq.values
    travel = float(np.abs(np.diff(q)).sum()); moving = np.flatnonzero(np.abs(dq) > thr)
    at_rest = len(moving) == 0 or moving[-1] < len(dq) - 25  # 마지막 50 ms 정지
    stop = float(t[moving[-1]]) if (len(moving) and at_rest) else (0.0 if not len(moving) else np.nan)
    swings = int(np.sum(np.diff(np.sign(dq[np.abs(dq) > thr])) != 0)) / 2 if len(moving) else 0
    hit_limit = bool(np.abs(q).max() >= math.radians(165))  # 리그 관절 한계 ±170° 에 닿음 → 정지는 마찰이 아니라 한계 접촉
    return dict(fd_moved=bool(travel > math.radians(1)), fd_travel_rad=travel, fd_stop_s=np.nan if hit_limit else stop, fd_swings=swings, fd_hit_limit=hit_limit,
                fd_rest_deg=math.degrees(abs(q[-1])) if (at_rest and not gravity_axis and not hit_limit) else np.nan)


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--root", default=str(ROOT / "data/sim/bench")); ap.add_argument("--no-plots", action="store_true"); a = ap.parse_args()
    root = Path(a.root); rows = []; series = {}
    for cls_dir in sorted(root.iterdir()):
        if not cls_dir.is_dir() or cls_dir.name == "plots": continue
        for pdir in sorted(cls_dir.iterdir()):
            if not (pdir / "meta.yaml").exists(): continue
            meta = yaml.safe_load((pdir / "meta.yaml").read_text()); rig, cfg, proto = meta["rig"], meta["actuator_cfg"], meta["proto"]
            tau_lim = cfg["effort_limit"]; r = dict(cls=cls_dir.name, preset=pdir.name, kp=rig["kp"], kd=rig["kd"], I_link=rig["inertia"], mgd=rig["mgd"],
                                                  tau_pk=cfg["saturation_effort"], vel_lim=cfg["velocity_limit"], armature=cfg["armature"], coulomb=cfg.get("dynamic_friction", cfg["friction"]), viscous=cfg.get("viscous_friction", 0.0))
            r["fric_src"] = FRIC_SRC(pdir.name, r["coulomb"]); r["arm_over_Ilink"] = r["armature"] / r["I_link"]; r["tau_over_arm"] = r["tau_pk"] / r["armature"] if r["armature"] else np.nan; r["tau_over_Jtot"] = r["tau_pk"] / (r["armature"] + r["I_link"])
            s = series.setdefault(cls_dir.name, {}); s[pdir.name] = {}
            if (pdir / "step.csv").exists():
                d = pd.read_csv(pdir / "step.csv"); r.update(step_metrics(d, tau_lim)); s[pdir.name]["step"] = d
            if (pdir / "chirp.csv").exists():
                d = pd.read_csv(pdir / "chirp.csv"); fs, mag, ph, m = chirp_frf(d, proto["chirp"]["f0"], rig["chirp_f1"], proto["chirp"]["dur"]); r.update(m)
                r["chirp_sat_frac"] = float(np.mean(np.abs(d.tau_applied.values) >= 0.98 * tau_lim)); s[pdir.name]["frf"] = (fs, mag, ph)
            if (pdir / "multisine.csv").exists():
                d = pd.read_csv(pdir / "multisine.csv"); r.update(multisine_metrics(d, tau_lim)); s[pdir.name]["multisine"] = d
            if (pdir / "free_decay.csv").exists():
                d = pd.read_csv(pdir / "free_decay.csv"); r.update(free_decay_metrics(d, rig["gravity_axis"])); s[pdir.name]["free_decay"] = d
            rows.append(r)
    df = pd.DataFrame(rows); df.to_csv(root / "metrics.csv", index=False); print(f"→ {root/'metrics.csv'}")
    if root.resolve() == (ROOT / "data/sim/bench").resolve():
        write_md(df, root); print("→ isaac/bench_metrics.md")
        if not a.no_plots: plots(series, df, root)


def write_md(df, root):
    def f(v, fmt="{:.3g}"):
        if v is None or (isinstance(v, float) and np.isnan(v)): return "–"
        if isinstance(v, str): return v
        if isinstance(v, (bool, np.bool_)): return "예" if v else "아니오"
        return fmt.format(v)
    cols = [("preset", "프리셋", "{}"), ("armature", "armature kg·m²", "{:.2e}"), ("coulomb", "Coulomb Nm", "{:.3g}"), ("fric_src", "마찰 출처", "{}"), ("tau_pk", "τpk Nm", "{:.3g}"), ("vel_lim", "v_max rad/s", "{:.3g}"),
            ("arm_over_Ilink", "arm/I_link", "{:.2f}"), ("tau_over_Jtot", "τpk/(arm+I) rad/s²", "{:.0f}"),
            ("step_rise_ms", "상승 ms", "{:.0f}"), ("step_overshoot_pct", "오버슈트 %", "{:.1f}"), ("step_settle_ms", "정착 ms", "{:.0f}"), ("step_ss_err_deg", "정상오차 °", "{:.2f}"), ("step_sat_frac", "스텝 포화", "{:.0%}"),
            ("bw_3db_hz", "−3 dB Hz", "{:.2f}"), ("f_90deg_hz", "−90° Hz", "{:.2f}"), ("res_peak_db", "공진 dB", "{:.1f}"),
            ("ms_track_rms_deg", "추종 RMS °", "{:.2f}"), ("ms_tau_rms", "τ RMS Nm", "{:.2f}"), ("ms_sat_frac", "MS 포화", "{:.0%}"),
            ("fd_moved", "백드라이브 이동", "{}"), ("fd_travel_rad", "이동 rad", "{:.2f}"), ("fd_stop_s", "정지 s", "{:.2f}"), ("fd_swings", "스윙", "{:.1f}"), ("fd_rest_deg", "정지각 °", "{:.1f}")]
    cls_note = {"wrist": "OpenArm joint7 등가 리그 (I 0.00559, m·g·d 0.72 Nm, kp 10 / kd 0.5). 진자, 0.8 rad 에서 놓음.",
                "elbow": "OpenArm joint3 등가 리그 (I 0.00215, 롤 축 → 중력 정렬, kp 70 / kd 2.0, 처프 0.2→40 Hz). 백드라이브 = 0 rad 에서 2 rad/s 코스트다운 (마찰 0 모델은 ±170° 한계까지 굴러감). armature 가 I_link 를 지배하는 조건.",
                "shoulder": "OpenArm joint1 등가 리그 (I 0.434, m·g·d 11.74 Nm, kp 70 / kd 2.75). 진자, 0.8 rad 에서 놓음."}
    L = ["# WP6 공통 벤치 지표 (Isaac 단일 관절 리그, 2026-09-22)", "",
         "클래스별로 **동일 리그(I_link·m·g·d)·동일 PD 게인·동일 궤적**. 모터마다 다른 것은 DCMotorCfg 파라미터(τpk, v_max, armature, Coulomb/점성)뿐이다. "
         "effort_limit 은 전 프리셋 피크(saturation_effort)로 통일(`bench_all.py --effort peak`). 값 출처 등급은 `comparison_specs.md` 참조. DM 3종만 실측(M), 나머지는 D/S.", "",
         "프로토콜: step ±0.3 rad (2 s 유지 ×2회) · chirp q_des 0.1 rad, 0.2→20 Hz(어깨 8 Hz) 30 s 선형 · multisine {0.3,0.7,1.3,2.9} Hz 합 0.4 rad 14 s · free_decay 1.5 s 유지 후 kp=kd=0.", "",
         "주의: DM 열의 오차 막대는 WP5 잔차(Δq 0.4~0.8°, Δτ 정격 2~3 %)를 그대로 적용. 타사 열은 사양 기반 모델의 예측치이며 실측 검증 없음(비대칭 비교). "
         "Unitree(S) armature 0.01 / friction 0.2 는 전 관절 공통 범용값이라 클래스 내 순위 해석에 주의.", ""]
    for cls in ["wrist", "elbow", "shoulder"]:
        sub = df[df.cls == cls]
        if sub.empty: continue
        L += [f"## {cls}", "", cls_note[cls], "", "| " + " | ".join(h for _, h, _ in cols) + " |", "|" + "---|" * len(cols)]
        for _, r in sub.iterrows():
            r = r.copy()
            if np.isnan(r.get("bw_3db_hz", np.nan)) and "bw_3db_hz" in r: r["bw_3db_hz"] = f"> {CHIRP_F1.get(cls, 20):g}"
            if r.get("fd_hit_limit"): r["fd_stop_s"] = "한계 접촉"
            L.append("| " + " | ".join(f(r.get(k), fmt) if k != "preset" else (f"**{r[k]}**" if r[k].startswith("dm") else r[k]) for k, _, fmt in cols) + " |")
        L.append("")
    # 마찰 민감도 (bench_all.py --friction zero / floor:0.02 → 별도 루트). 격차가 마찰 데이터 유무 때문임을 보이는 표
    sens = [(k, label, pd.read_csv(p / "metrics.csv")) for k, label, p in SENS_ROOTS if (p / "metrics.csv").exists()]
    if sens:
        L += ["## 마찰 민감도", "",
              "타사 모델 대부분은 마찰이 **미공개 → 0** 이라 정상오차·추종·백드라이브에서 DM(실측 마찰)이 불리해 보인다. 같은 벤치를 (a) 전 프리셋 마찰 0, (b) Coulomb 이 피크의 2 % 미만인 프리셋에 2 %·τpk(DM 실측 비율 1.7~2.0 %) 를 채워 재실행한 결과. "
              "대역폭 순위는 마찰과 무관(armature 순)하고, 정상오차·추종 격차는 마찰 가정에 따라 사라지거나 역전된다.", ""]
        sc = [("bw_3db_hz", "−3 dB Hz", "{:.1f}"), ("step_ss_err_deg", "정상오차 °", "{:.2f}"), ("ms_track_rms_deg", "추종 RMS °", "{:.2f}"), ("fd_stop_s", "정지 s", "{:.2f}")]
        for cls in ["wrist", "elbow", "shoulder"]:
            base = df[df.cls == cls]
            if base.empty: continue
            hdr = ["프리셋", "Coulomb Nm"] + [f"{h} · 원래" for _, h, _ in sc]
            for _, label, _ in sens: hdr += [f"{h} · {label}" for _, h, _ in sc]
            L += [f"### {cls}", "", "| " + " | ".join(hdr) + " |", "|" + "---|" * len(hdr)]
            for _, r in base.iterrows():
                cells = [f"**{r.preset}**" if r.preset.startswith("dm") else r.preset, f(r.coulomb)] + [f(r.get(k), fmt) for k, _, fmt in sc]
                for _, _, sd in sens:
                    sr = sd[(sd.cls == cls) & (sd.preset == r.preset)]
                    cells += [f(sr.iloc[0].get(k), fmt) if len(sr) else "–" for k, _, fmt in sc]
                L.append("| " + " | ".join(cells) + " |")
            L.append("")
    (ROOT / "isaac" / "bench_metrics.md").write_text("\n".join(L))


def plots(series, df, root):
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    (root / "plots").mkdir(exist_ok=True)
    for cls, S in series.items():
        names = list(S); fig, ax = plt.subplots(2, 2, figsize=(13, 8.5)); ax = ax.ravel()
        for i, n in enumerate(names):
            c = PALETTE[i % len(PALETTE)]; lw = 2.4 if n.startswith("dm") else 1.4; kw = dict(color=c, lw=lw, label=n, alpha=0.95)
            if "step" in S[n]:
                d = S[n]["step"]; m = (d.t_s >= 1.8) & (d.t_s <= 4.0); ax[0].plot(d.t_s[m] - 2.0, np.degrees(d.q[m]), **kw)
                if i == 0: ax[0].plot(d.t_s[m] - 2.0, np.degrees(d.q_des[m]), color="#888", lw=1, ls="--", label="q_des")
            if "frf" in S[n]:
                fs, mag, ph = S[n]["frf"]; ax[1].semilogx(fs, 20 * np.log10(np.maximum(mag, 1e-3)), **kw)
            if "multisine" in S[n]:
                d = S[n]["multisine"]; m = d.t_s >= 3.0; ax[2].plot(d.t_s[m], np.degrees(d.q_des[m] - d.q[m]), **kw)
            if "free_decay" in S[n]:
                d = S[n]["free_decay"]; fr = d[d.phase == "free"]; ax[3].plot(fr.t_s - fr.t_s.iloc[0], np.degrees(fr.q), **kw)
        ax[0].set(title="Step ±0.3 rad (first +step)", xlabel="t [s]", ylabel="q [deg]"); ax[1].set(title="Closed-loop |q/q_des| (position chirp)", xlabel="f [Hz]", ylabel="dB"); ax[1].axhline(-3, color="#888", lw=1, ls="--")
        ax[2].set(title="Multisine tracking error (t ≥ 3 s)", xlabel="t [s]", ylabel="q_des − q [deg]"); ax[3].set(title="Free decay after release (kp=kd=0)", xlabel="t since release [s]", ylabel="q [deg]")
        for a_ in ax: a_.grid(alpha=0.25, lw=0.6); a_.spines[["top", "right"]].set_visible(False)
        rig = df[df.cls == cls].iloc[0]; fig.suptitle(f"{cls} class, same rig (I_link {rig.I_link:.4g} kg·m², m·g·d {rig.mgd:.3g} Nm), same PD (kp {rig.kp:g} / kd {rig.kd:g})", fontsize=12)
        ax[0].legend(fontsize=8, ncol=2, frameon=False); fig.tight_layout(); fig.savefig(root / "plots" / f"{cls}.png", dpi=130); plt.close(fig)
    print(f"→ {root/'plots'}/<class>.png")


if __name__ == "__main__":
    main()
