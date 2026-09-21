#!/usr/bin/env python3
"""candump 로그(-L 포맷) → CSV.

    candump2csv.py can0.log --arm right [--motors config/motors.yaml] [-o can0.csv]

출력 1) <stem>.csv      : 모터 피드백 (recv_id 0x11~0x18)
       t_s, joint, motor_type, q_rad, dq_rads, tau_Nm, t_mos_C, t_rotor_C, err_code, err_name, dt_ms
출력 2) <stem>_cmd.csv  : MIT 명령 (send_id 0x01~0x08)
       t_s, joint, q_des, dq_des, kp, kd, tau_ff
출력 3) <stem>_param.csv: 파라미터 응답 (07 00 33 <rid> <f32 LE>)  t_s, joint, rid, name, value
출력 4) stdout 요약    : 축별 샘플 수·평균 dt·에러 발생 횟수, 파라미터 값

프레임 구분 (recv_id 로 오는 프레임): byte1==0 이고 byte2 가 0x33(read)/0x55(write) 이고 byte0==send_id 이면 파라미터 응답,
아니면 상태 피드백. send_id 로 가는 FF..FC/FD/FE 는 enable/disable/set_zero 명령으로 기록만 한다.

디코딩은 dm_motor_control.cpp (parse_motor_state / pack_mit_control_data) 와 동일.
"""
import argparse, csv, re, struct, sys
from pathlib import Path
import yaml

LINE = re.compile(r"^\((?P<ts>[\d.]+)\)\s+(?P<if>\S+)\s+(?P<id>[0-9A-Fa-f]+)#(?P<fd>#[0-9A-Fa-f])?(?P<data>[0-9A-Fa-f]*)")


PARAM_NAMES = {0: "UV_Value", 1: "KT_Value", 2: "OT_Value", 3: "OC_Value", 4: "ACC", 5: "DEC", 6: "MAX_SPD", 7: "MST_ID", 8: "ESC_ID",
               9: "TIMEOUT", 10: "CTRL_MODE", 11: "Damp", 12: "Inertia", 13: "hw_ver", 14: "sw_ver", 15: "SN", 16: "NPP", 17: "Rs", 18: "LS",
               19: "Flux", 20: "Gr", 21: "PMAX", 22: "VMAX", 23: "TMAX", 24: "I_BW", 25: "KP_ASR", 26: "KI_ASR", 27: "KP_APR", 28: "KI_APR",
               29: "OV_Value", 30: "GREF", 31: "Deta", 32: "V_BW", 33: "IQ_c1", 34: "VL_c1", 35: "can_br", 36: "sub_ver"}
INT_PARAMS = {7, 8, 9, 10, 13, 14, 15, 16, 35, 36}  # Damiao 펌웨어에서 int32 로 저장되는 RID (ID·모드·버전·NPP 등)
SPECIAL_CMD = {b"\xff" * 7 + b"\xfc": "enable", b"\xff" * 7 + b"\xfd": "disable", b"\xff" * 7 + b"\xfe": "set_zero"}


def is_param_frame(data, send_id):
    return len(data) >= 8 and data[1] == 0 and data[2] in (0x33, 0x55) and data[0] == (send_id & 0xFF)


def u2f(u, lo, hi, bits):
    return lo + (hi - lo) * u / ((1 << bits) - 1)


def load_cfg(path):
    cfg = yaml.safe_load(Path(path).read_text())
    recv = {int(j["recv_id"]): (name, j["motor"]) for name, j in cfg["joints"].items()}
    send = {int(j["send_id"]): (name, j["motor"]) for name, j in cfg["joints"].items()}
    return cfg, recv, send


def decode_feedback(data, lim):
    q = (data[1] << 8) | data[2]
    dq = (data[3] << 4) | (data[4] >> 4)
    tau = ((data[4] & 0xF) << 8) | data[5]
    return dict(
        q_rad=u2f(q, -lim["p_max"], lim["p_max"], 16),
        dq_rads=u2f(dq, -lim["v_max"], lim["v_max"], 12),
        tau_Nm=u2f(tau, -lim["t_max"], lim["t_max"], 12),
        t_mos_C=data[6], t_rotor_C=data[7], err_code=data[0] >> 4,
    )


def decode_mit(data, lim):
    q = (data[0] << 8) | data[1]
    dq = (data[2] << 4) | (data[3] >> 4)
    kp = ((data[3] & 0xF) << 8) | data[4]
    kd = (data[5] << 4) | (data[6] >> 4)
    tau = ((data[6] & 0xF) << 8) | data[7]
    return dict(
        q_des=u2f(q, -lim["p_max"], lim["p_max"], 16),
        dq_des=u2f(dq, -lim["v_max"], lim["v_max"], 12),
        kp=u2f(kp, 0, 500, 12), kd=u2f(kd, 0, 5, 12),
        tau_ff=u2f(tau, -lim["t_max"], lim["t_max"], 12),
    )


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log")
    ap.add_argument("--motors", default=str(Path(__file__).resolve().parent.parent / "config/motors.yaml"))
    ap.add_argument("--arm", default=None, help="joint 이름 접두어로 붙일 팔 이름 (left/right)")
    ap.add_argument("-o", "--out", default=None)
    a = ap.parse_args()

    cfg, recv_map, send_map = load_cfg(a.motors)
    lims = cfg["motor_limits"]
    err_names = {int(k): v for k, v in cfg["error_codes"].items()}
    prefix = f"{a.arm}_" if a.arm else ""

    out = Path(a.out) if a.out else Path(a.log).with_suffix(".csv")
    out_cmd = out.with_name(out.stem + "_cmd.csv"); out_par = out.with_name(out.stem + "_param.csv")
    fb_cols = ["t_s", "joint", "motor_type", "q_rad", "dq_rads", "tau_Nm", "t_mos_C", "t_rotor_C", "err_code", "err_name", "dt_ms"]
    cmd_cols = ["t_s", "joint", "q_des", "dq_des", "kp", "kd", "tau_ff", "special"]
    par_cols = ["t_s", "joint", "rid", "name", "value"]

    last_t, stats, params, n_other, n_bad = {}, {}, {}, 0, 0
    joint_send = {name: int(j["send_id"]) for name, j in cfg["joints"].items()}
    with open(a.log) as f, open(out, "w", newline="") as fo, open(out_cmd, "w", newline="") as fc, open(out_par, "w", newline="") as fp:
        wf, wc, wp = csv.DictWriter(fo, fb_cols), csv.DictWriter(fc, cmd_cols), csv.DictWriter(fp, par_cols)
        wf.writeheader(); wc.writeheader(); wp.writeheader()
        for line in f:
            m = LINE.match(line)
            if not m:
                n_bad += 1; continue
            cid, ts = int(m["id"], 16), float(m["ts"])
            data = bytes.fromhex(m["data"])
            if cid in recv_map and len(data) >= 8:
                joint, mtype = recv_map[cid]
                if mtype not in lims:
                    n_other += 1; continue
                if is_param_frame(data, joint_send[joint]):
                    rid = data[3]; val = struct.unpack("<i" if rid in INT_PARAMS else "<f", data[4:8])[0]
                    wp.writerow(dict(t_s=f"{ts:.6f}", joint=prefix + joint, rid=rid, name=PARAM_NAMES.get(rid, f"rid{rid}"), value=f"{val:.6g}"))
                    params.setdefault(joint, {})[PARAM_NAMES.get(rid, f"rid{rid}")] = val
                    continue
                rec = decode_feedback(data, lims[mtype])
                rec["err_name"] = err_names.get(rec["err_code"], f"unknown_{rec['err_code']:x}")
                dt = (ts - last_t[cid]) * 1e3 if cid in last_t else ""
                last_t[cid] = ts
                rec.update(t_s=f"{ts:.6f}", joint=prefix + joint, motor_type=mtype, dt_ms=f"{dt:.3f}" if dt != "" else "")
                wf.writerow(rec)
                s = stats.setdefault(joint, dict(n=0, dt_sum=0.0, dt_n=0, err=0, tmos_max=0))
                s["n"] += 1
                if dt != "": s["dt_sum"] += dt; s["dt_n"] += 1
                if rec["err_code"] not in (0x0, 0x1): s["err"] += 1
                s["tmos_max"] = max(s["tmos_max"], rec["t_mos_C"])
            elif cid in send_map and len(data) >= 8:
                joint, mtype = send_map[cid]
                if mtype not in lims:
                    n_other += 1; continue
                sp = SPECIAL_CMD.get(bytes(data[:8]))
                if sp:
                    wc.writerow(dict(t_s=f"{ts:.6f}", joint=prefix + joint, special=sp)); continue
                rec = decode_mit(data, lims[mtype])
                rec.update(t_s=f"{ts:.6f}", joint=prefix + joint, special="")
                wc.writerow({k: (f"{v:.5f}" if isinstance(v, float) else v) for k, v in rec.items()})
            else:
                n_other += 1  # 0x7FF 브로드캐스트, 파라미터 프레임 등

    print(f"{a.log}: feedback → {out}, cmd → {out_cmd}  (기타 프레임 {n_other}, 파싱 실패 {n_bad})")
    print(f"{'joint':8s} {'n':>7s} {'mean_dt_ms':>11s} {'errors':>7s} {'tmos_max':>9s}")
    for j, s in sorted(stats.items()):
        mdt = s["dt_sum"] / s["dt_n"] if s["dt_n"] else float("nan")
        print(f"{j:8s} {s['n']:7d} {mdt:11.3f} {s['err']:7d} {s['tmos_max']:9d}")
    for j, pr in sorted(params.items()):
        print(f"params {j}: " + ", ".join(f"{k}={v:.6g}" for k, v in pr.items()))


if __name__ == "__main__":
    main()
