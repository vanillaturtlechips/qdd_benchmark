# tools/ — WP1 실물 계측 인프라

환경: `source ~/qdd_benchmark/.venv/bin/activate` (system-site-packages 포함 → ROS 파이썬도 사용 가능).
`openarm_can` 파이썬 바인딩은 이 venv 에 설치되어 있음 (`~/openarm_ws/src/openarm_can/python`, 1.3.4).

## 두 가지 실행 모드

| 모드 | 언제 | CAN 소유자 | 녹화 |
|---|---|---|---|
| **A. ros2_control 위** | PD 스텝·sine 응답, 홀드아웃 재생, 정지 베이스라인 | `controller_manager` | `record.sh <out> --ros` |
| **B. 직접 MIT (excite.py)** | 마찰·관성·지연 식별 (kp/kd/τ_ff 를 임의로 줘야 함) | `excite.py` (해당 팔 controller_manager **정지 필수**) | `record.sh <out> --if can0` 병렬 + `excite.py` 자체 `cmd.csv` |

B 모드에서 excite.py 는 **타깃 모터 1개만 enable** 한다. 나머지 관절은 disable(백드라이브 자유) 상태이므로 팔이 매달린(q≈0) 자세에서만 실행한다.

## 파일

- `record.sh <out_dir> [--ros] [--if can0,can1] [--topics "..."]`
  - `candump -L` → `<out>/canX.log` (커널 rx 타임스탬프, epoch 초)
  - `--ros` → `<out>/bag/` (mcap; `/joint_states`, `*_controller/controller_state`, `/tf`)
  - Ctrl+C 로 종료 → `meta.yaml` 에 시작/종료 시각, `ip -s link` 통계(에러·드롭·bus-off) 기록
- `candump2csv.py <canX.log> --arm right [-o out.csv]`
  - 피드백 CSV + 명령 CSV 분리, stdout 요약(축별 n, 평균 dt, 에러 수, T_MOS 최대)
- `excite.py --arm --joint --mode ... [--dry-run]`
  - `--dry-run` : 하드웨어 없이 궤적 플롯만 (`data/dry_run/*.png`) — **실기 전 항상 먼저 실행**
- `safety.py` : excite.py 가 사용하는 클램프·감시. 한계값은 `config/motors.yaml: safety`

## CSV 스키마

### `canX.csv` (candump2csv.py, 피드백)
| 열 | 단위 | 비고 |
|---|---|---|
| t_s | s (epoch) | 커널 rx 타임스탬프 |
| joint | | `right_joint7` 등 |
| motor_type | | DM4310 / DM4340 / DM8009 |
| q_rad, dq_rads, tau_Nm | rad, rad/s, Nm | 타입별 pMax/vMax/tMax 로 복원. **tau 는 전류 추정치** |
| t_mos_C, t_rotor_C | °C | 정수 |
| err_code / err_name | | byte0 상위 4bit. 0x1=enabled 정상 |
| dt_ms | ms | 같은 축 직전 피드백과의 간격 |

### `canX_cmd.csv` (candump2csv.py, MIT 명령)
`t_s, joint, q_des, dq_des, kp, kd, tau_ff` — 명령 프레임 디코딩. `t_s(cmd)` ↔ 다음 `t_s(feedback)` 차이가 **명령→응답 지연**.

### `cmd.csv` (excite.py)
| 열 | 비고 |
|---|---|
| t_s | epoch, 피드백 수신 직후 |
| phase | goto / hold / v0.5 / chirp / step+1 / multisine / pre_hold / free |
| q_des, dq_des, kp, kd, tau_ff | 실제 송신한 MIT 명령 (safety 클램프 후) |
| q, dq, tau, t_mos, t_rotor | 바인딩에서 읽은 상태 |
| err_code, fb_n | 스니퍼(python-can)로 받은 상태 코드, 이 틱에 수신한 피드백 프레임 수 (0이면 미수신) |
| loop_late_us | 루프 지연 (>2000 이면 500 Hz 미달 틱) |

### `meta.yaml` (excite.py)
궤적 인자, `motor_params`(펌웨어에서 읽은 KT_Value·Gr·Inertia·Damp·TMAX·VMAX·TIMEOUT 등), q_start, 시작/종료 온도, ticks, late 틱 수, abort_reason, safety_events.

## 실험 모드 ↔ WP4 식별 파라미터

| mode | MIT 구성 | 식별 대상 |
|---|---|---|
| const_vel | PD 삼각파 램프, 속도 레벨별 | 마찰 τ(ω): Coulomb + 점성 (+ 중력항, joint1) |
| chirp | kp≈2 센터링 + τ_ff 지수 처프 | armature(반사 관성) J, 대역폭 |
| step | PD 위치 스텝 | PD 폐루프 응답 → Isaac stiffness/damping 검증 |
| multisine | PD, seed 고정 | **홀드아웃** — Isaac 재생 후 잔차 (WP5) |
| free_decay | 유지 후 kp=kd=τ=0 | 백드라이브 저항, (중력축) 진자 감쇠 |
| hold | PD 정지 유지 | effort 노이즈 플로어, 위치 드리프트, 온도 베이스라인 |

## 스모크 테스트 절차 (실기 첫 구동)
1. 우팔 `controller_manager` 정지 (`Ctrl+C`), `ip -br link | grep can` 으로 can0 UP 확인
2. `excite.py --arm right --joint joint7 --mode hold --dur 5` — 움직임 없음, err=0x1, fb_n=1/틱, late 0 확인
3. `excite.py --arm right --joint joint7 --mode const_vel --levels 0.3 --dur 4` — 저속으로 범위·방향 눈으로 확인
4. `candump2csv.py` 로 병렬 캡처 파싱, `cmd.csv` 와 축·부호·스케일 일치 확인
5. 이상 없으면 `config/motors.yaml: id_axes` 확정

## `fit_params.py` — WP4 시스템 식별 (합성 데이터로 검증 완료, 2026-09-21)

```
fit_params.py <const_vel_dir> <chirp_dir> [--alpha-max 3] [--deadband 0.05] [--cutoff 30] [--huber 0.1] [--link-inertia I]
```
모델: τ = J·α + b·ω + c·sign(ω) + g_s·sin q + g_c·cos q + τ0. 입력은 `excite.py` 출력(`cmd.csv`) 또는 Isaac 리그 출력(`sim.csv`) 디렉터리.

**2단계 절차 (기본 동작, meta.mode 로 자동 분기)**
1. `const_vel` 세트에서 정속 plateau 샘플(|α| ≤ alpha_max, |ω| ≥ deadband)만으로 b, c, 중력항 추정
2. `chirp` 세트에서 1단계 값을 고정하고 J 만 추정
3. Isaac 매핑: `armature = J − I_link`, `dynamic_friction = friction = c`, `viscous_friction = b`

**합성 검증 결과** (`synth` 프리셋: armature 0.003, Coulomb 0.15, viscous 0.02; 4레벨 const_vel + 0.1→20 Hz 처프)

| | 진리값 | 중력 정렬 축 | 진자(중력 있음) |
|---|---|---|---|
| armature | 0.003 | 0.00309 (+3 %) | 0.00309 (+3 %) |
| Coulomb c | 0.15 | 0.1505 (+0.3 %) | 0.1505 |
| viscous b | 0.02 | 0.0196 (−2 %) | 0.0196 |
| m·g·d | 0.3679 | — | 0.3685 (+0.2 %) |

검증 중 확인된 함정 (실물 실험 설계에 반영):
- **고주파 처프(20 Hz)로 마찰을 추정하면 c 가 −16 %** — 빠른 속도 반전에서 마찰이 스틱션에 머묾. 그래서 마찰은 const_vel 에서만.
- **속도 레벨이 너무 높으면(범위 ±1.05 rad 에서 ≥ 8 rad/s) plateau 가 사라져** 반전 과도가 마찰 추정을 오염. alpha_max 마스크로 걸러지지만, 실물에서는 레벨 0.25~4 rad/s 위주로 잡고 8 rad/s 는 무부하 최고속도 확인용으로만.
- 4개 속도 레벨이면 b 와 c 분리에 충분 (합성 기준). 실물은 노이즈가 있으므로 6레벨 권장: 0.25, 0.5, 1, 2, 3, 4.
