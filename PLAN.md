# QDD 액추에이터 벤치마크 작업 계획

> **현황 요약은 [PROGRESS.md](PROGRESS.md)** (9/22 13:30 갱신, 27/30 완료, WP1~6 종료, 로봇 실험 끝)

- 기간: 2026-09-21(월) ~ 2026-09-24(목), 실질 3.5일 (≈28h)
- 목적: OpenArm 실물 Damiao QDD 모터를 시스템 식별하여 Isaac Lab 액추에이터 모델로 옮기고, 상용 휴머노이드 오픈소스 모터 모델과 **동일한 시뮬 벤치**에서 정규화 지표로 비교한다.
- 원칙: 실물 측정값과 시뮬 수치를 직접 비교하지 않는다. 양쪽을 같은 표현(액추에이터 파라미터)으로 옮긴 뒤 비교하고, 실물→모델 변환의 잔차를 명시적으로 측정해 오차 막대로 보고한다.

---

## 0. 현재 환경 (확인 완료, 2026-09-21)

| 항목 | 상태 |
|---|---|
| 실물 로봇 | OpenArm 양팔, 좌=can1 / 우=can0, CAN-FD 1M/5M, 모터 응답 확인 |
| 모터 구성 | DM4310 ×4, DM4340 ×2, DM8009 ×2 (그리퍼 제외) |
| ROS 2 | Jazzy, `~/openarm_ws`, `controller_manager`·양팔 hardware_interface 실행 중 |
| ROS 노출 지표 | `/joint_states` position / velocity / effort (`openarm_simple_hardware.cpp:286-291`) |
| ROS 미노출 지표 | T_MOS, T_Rotor, 에러코드 — 드라이버가 파싱은 함(`dm_motor.hpp:37-38`), candump로 직접 획득 필요 |
| effort 의미 | 토크 센서 아님. 전류 기반 추정치(Kt·Iq) |
| can-utils | candump / cansend / canbusload 설치됨 |
| Isaac | Isaac Sim 5.x + Isaac Lab 2.3.2 (`~/isaac/env_isaaclab`), RTX 5080 Laptop 16GB |
| Isaac Lab 내장 모델 | `openarm.py`(ImplicitActuator kp80/kd4 임의값), `unitree.py` G1 29DOF(saturation_effort·armature 정의됨)/H1, `fourier.py` GR1T2, `agility.py` Digit v4 |
| 미설치 | PlotJuggler, python-can |

---

## 1. 범위 확정

| 항목 | 결정 | 이유 |
|---|---|---|
| 비교 대상 | **오픈소스 휴머노이드 QDD 전 계열**: 상용 모듈(Robstride/K-Bot, Unitree G1·H1) + 자체설계 공개(MIT Humanoid, Berkeley Humanoid, Berkeley Humanoid Lite). 사양은 `isaac/specs.yaml`, 프리셋 `isaac/actuators.py`, 데이터시트 비교표 `isaac/comparison_specs.md` | Isaac Lab 내장 cfg 는 팔 파라미터가 플레이스홀더(WP2 덤프로 확인). GR1T2/Digit 은 공개 사양 없음, 외부 레포 USD 변환은 1일 이상 소모 |
| DM 모터 모델 클래스 | **`DCMotorCfg`** (+ 필요시 `DelayedPDActuator`) | G1 29DOF와 동일 클래스 → 파라미터 1:1 비교 가능 |
| 식별 대상 모터 | DM4310 / DM4340 / DM8009 각 **대표 1축** (총 3축) | 8축 전부는 시간 초과. 동일 모델은 동일 파라미터 가정 |
| 식별 파라미터 | 마찰(Coulomb+점성), armature, 무부하 최고속도, 지연, effort 노이즈 플로어, PD 추종 | 추가 하드웨어 불필요 |
| 비교 지표 | 토크밀도(Nm/kg, 데이터시트), τ_max/armature, −3dB 대역폭, 고정부하 sine 추종 RMS, 백드라이브 저항, 지연 | 절대값이 아닌 정규화 지표 |

### 제외 (명시적 절단)

- 토크상수(Kt) 절대 캘리브레이션 (진자/로드셀) → 데이터시트 Kt 사용, "가정"으로 보고
- 부하 걸린 속도-토크 곡선 → 데이터시트 값 사용
- 열 모델 피팅 → T_MOS 베이스라인 기록만
- ActuatorNet 학습
- 그리퍼 모터
- Isaac Lab 외부 휴머노이드 모델 추가

---

## 2. 작업 패키지 (WBS)

### WP1. 실물 계측 인프라 — 월 (3h)
> 산출물: `config/motors.yaml`, `tools/{record.sh,candump2csv.py,excite.py,safety.py,README.md}`, venv `.venv`(openarm_can 바인딩 포함). 파서는 실버스 캡처로 검증(200 Hz, kp70/kd2.75, 대기 T_MOS 27~30 °C). **스모크 테스트(실기 구동) 미완.**
- [x] bag + candump 동시 녹화 스크립트 (`/joint_states`, `/*/controller_state`, `candump -t d -l can0 can1`)
- [x] candump 피드백 파서: 위치·속도·토크·T_MOS·T_Rotor·에러코드 → CSV
- [x] 실험 궤적 발생기 (MIT 모드 직접 명령, ros2_control 우회): 정속 스윕 / 토크 처프 / 스텝 / 다중 사인 홀드아웃
- [x] 안전 한계 내장: 각도·속도·토크 클램프, 에러코드 감지 시 즉시 정지

### WP2. Isaac 벤치 기반 검증 — 월 (2h)
- [x] headless 기동 확인 — 기동·종료 3 s (캐시 완료). `app.close()` hang 은 워치독으로 우회
- [x] cfg 로드·파라미터 덤프 → `isaac/actuator_cfgs.md`. **G1 29DOF legs/feet 만 DCMotorCfg 로 물리 파라미터(effort·saturation·velocity·armature) 보유.** H1/GR1T2/Digit 는 cfg 에 값이 없거나 플레이스홀더(300 Nm 등) → 파라미터 비교 불가, USD 드라이브 값만 존재
- [x] 단일 관절 리그 `isaac/single_joint_rig.py` + 프리셋 `isaac/actuators.py` — dm4310 step/free_decay 로 물리 검증 완료 (중력 droop·진자 주기 이론값 일치)

### WP3. 실물 실험 실행 — 화 종일 (7h)
- [x] **joint7 (DM4310) 완료 9/22** — const_vel 0.25~3, chirp ×2, free_decay, multisine. 식별: `data/2026-09-21_j7/identified_dm4310.yaml`. 교훈: 삼각파 반전 충격 → 사다리꼴(a_max 10) 로 교체 / 센터링 kp 공진 → 중력보상+kd / 3.5 Hz 이상은 백래시 영역
- [x] **joint3 (DM4340) 완료 9/22** — Coulomb 0.48 / viscous 0.19 / armature 0.0256 (펌웨어 0.029). J 대역 안정(5 %). 자중 백드라이브 불가. `data/2026-09-22_j3/identified_dm4340.yaml`
- [x] **joint1 (DM8009) 완료 9/22** — Coulomb 0.95 / viscous 0.10±0.05 / 중력 11.0 Nm / 바이어스 −0.76. armature 는 펌웨어 0.0167 (링크 0.40 의 4 %, 분리 불가). `data/2026-09-22_j1/identified_dm8009.yaml`
- [ ] 오후: 전체 팔 PD 스텝·sine 응답 (현재 kp/kd), 홀드아웃 다중사인 궤적 3회 반복, 명령→피드백 지연 측정
- [x] T_MOS 베이스라인: 대기 27~31 °C (3축, 3일간 동일). 실험 중 상승 ≤1 °C. ~~연속 토크 가열 램프~~ 는 **절단** (30 분 소요, 열 모델은 범위 외)
- [x] 데이터 QA: 전 실험 피드백 수신 100 %(누락 ≤1 틱/런), 에러코드 0, late 0. joint7 삼각파 런·공진 처프는 원인 규명 후 재실험 완료

### WP4. 시스템 식별 — 수 오전 (4h)
- [x] 오프라인 피팅 `tools/fit_params.py` — 2단계(마찰·중력 ← const_vel plateau, J ← chirp). **Isaac 합성 데이터로 검증: armature +3 %, Coulomb +0.3 %, viscous −2 %, 중력 +0.2 %** (tools/README.md 참조). 실측 CSV 입력만 남음
- [x] velocity_limit 은 데이터시트(20.9 / 5.4 / 16.8 rad/s, 미실측 — 팔 범위에서 무부하 최고속 도달 불가). 지연 150 µs → DelayedPD 0 스텝
- [x] `isaac/actuators.py` dm4310/dm4340/dm8009 식별값 반영. 신뢰구간은 각 `identified_*.yaml`

### WP5. sim2real 잔차 측정 — 수 오후 (4h)
- [x] 단일 관절 리그에 식별 cfg + Isaac 질량행렬(유령질량 제거)의 I_link·m·g·d 를 등가 막대로 주입 (`single_joint_rig.py --inertia --mgd`). ~~openarm.py 전체 팔 교체~~ 는 단일 축 검증으로 대체
- [x] 홀드아웃 재생 (`--mode replay`) + `tools/sim2real.py` 잔차 (1차, 9/22):

| 모터 | Δq RMS | ≤2° | Δτ RMS | 기준(10 % 정격) | 판정 | 추종오차 real/sim |
|---|---|---|---|---|---|---|
| DM4310 | 0.38° | PASS | 0.097 Nm | 0.30 | PASS | 1.09° / 1.03° |
| DM4340 | 0.57° | PASS | 0.245 Nm | 0.90 | PASS | 1.22° / 0.70° |
| DM8009 | 1.34° | PASS | 2.564 Nm | 2.00 | FAIL | 3.00° / 3.58° |

  DM8009 토크 FAIL 원인 진단: 홀드아웃에서 J=0.400 (Isaac 0.398 ✓) 이나 b=1.6 (정속 0.10 의 15배) → 액추에이터가 아니라 **하류 관절(joint2~7) disable 상태의 비강체 꿈틀거림**. R² 0.73
- [x] 1회 보강 시도 → **사고로 중단 (9/22 10:58)**. `--hold-others` 첫 실행: joint5~7 hold 목표가 −12.47 rad(쓰레기)로 송신되어 손목이 급격히 꺾임, 비상정지. 원인: openarm_can 이 CTRL_MODE 쓰기 응답을 STATE 프레임으로 파싱 → `get_position()` 오염, 코드가 검증 없이 사용 + 상태 응답 도착 전 레이스(0.3 ms). 증거 `data/2026-09-22_j1b/can0.log` t+153.87. **기능 사용 금지 처리.** 로봇 손상 여부 점검 필요
- [x] `--hold-others` 재설계 후 재측정 (9/22 11:30): 스니퍼 검증 상태프레임 + 정착 확인(≥5프레임, 편차<0.005) + |q|≤π 검사 + 표 확인 프롬프트 + 게인 2 s 램프 + 편차 0.15 rad 감시. 실기 dry-check → hold 정지(6축 ≤0.4 mrad) → 본 실험, 사고 없음
- [x] **DM8009 최종: PASS** — ≤1.3 Hz 홀드아웃(하류 PD 유지) Δq 0.92° / Δτ 0.63 Nm (기준 2° / 2.0). 2.9 Hz 홀드아웃은 hold 로도 FAIL (joint4 74 mrad 흔들림) → **팔꿈치 PD 스프링 공진(~4 Hz) = 다관절 구조 동역학**, 단일 관절 리그 범위 밖. 유효 대역 ≤1.5 Hz 명시. 마찰은 hold 조건 0.81 Nm 채택(비hold 0.95)
- [x] 최종 잔차 (보고서 오차 막대, hand=0.127 kg 보정 링크관성): DM4310 0.39°/0.11 Nm, DM4340 0.57°/0.25 Nm, DM8009 0.84°/0.48 Nm (≤1.5 Hz)
- [x] **9/22 정정**: 유령질량 제거 시 hand 를 0 으로 놓은 오류 → URDF 0.127 kg 으로 보정 (joint7 I_link 0.0043→0.0056, joint1 0.398→0.434). DM4310 armature '실측 5 % 일치' 철회 → 펌웨어값 사용. 이슈 초안도 정정 (hand 는 실제 링크, ee_tcp=hand_tcp, 고정관절→revolute 추가 버그)

### WP6. 공통 벤치 비교 — 화 오후 (2h, 앞당김)
- [x] `isaac/bench_all.py` — CLASS_MATCH 18개 (DM 3 + Robstride 3 + MIT 2 + Berkeley 2 + BH Lite 1 + G1 4 + H1 2 + H1-2 1; synth·bhl_6512_leg 제외) 를 클래스별 리그(joint7/3/1 등가 막대 = WP5 검증 리그) + 운용 PD 로 {step, chirp(위치 0.1 rad), multisine, free_decay} 동시 실행. 4.3 분
- [x] `isaac/bench_metrics.py` → `bench_metrics.md`(대역폭·추종 RMS·정착·백드라이브·τ/(arm+I)) + `plots/<class>.png`, `compare_specs.py` 가 시뮬 4열을 `comparison_specs.md` 에 병합
- [x] 부수 수정: 프리셋 `friction` → `dynamic_friction` 누락 버그 (발견 6), `--effort peak` 통일 (발견 7). 결과 요약은 PROGRESS.md §2.4

### WP7. 보고 — 목 오후 (3h + 버퍼)
- [ ] 파라미터 비교표, 정규화 지표 그래프, 잔차 표
- [ ] 가정·제외 목록 명시 ("실측 DM vs 사양 기반 타사" 비대칭 비교임을 명기)

---

## 3. 일정

```
월 9/21  ▇▇▇ WP1 인프라 ▇▇ WP2 Isaac 확인 ▇ 범위 확정·추 유무 확인
화 9/22  ▇▇▇▇▇▇▇ WP3 실물 실험 (+ 데이터 QA)
수 9/23  ▇▇▇▇ WP4 식별 ▇▇▇▇ WP5 잔차 검증
목 9/24  ▇▇▇ WP6 비교 ▇▇▇ WP7 보고 ▇▇ 버퍼
```

핵심 경로: **WP1 → WP3 → WP4 → WP5**. 화요일 실험이 밀리면 전체가 밀리므로 월요일에 궤적 발생기까지 반드시 완료.

---

## 4. 리스크 및 대응

| 리스크 | 확률 | 대응 |
|---|---|---|
| Isaac Sim 첫 headless 기동 실패 (드라이버/셰이더 캐시) | 중 | 월요일 즉시 확인. 실패 시 화요일 실험과 병렬로 해결 |
| effort 피드백(전류 추정)의 Kt 오차 | 높 | 데이터시트 Kt 가정 명시. 여유 시 DM4310 1축만 진자 캘리브 |
| 실물 실험 중 모터 에러/과열로 재실험 | 중 | 축당 시간 예산 1h 고정, 재실험 슬롯 화 오후 마지막 1h |
| 타사 파라미터 출처 등급 혼재 (M 실측: MIT 다이노·DM 펌웨어 / D 데이터시트: Robstride·Berkeley / S 시뮬 cfg: Unitree·K-Scale MJCF·BH Lite) | 확정 | 비대칭 비교임을 보고서에 명기. Unitree MJCF 의 armature 0.01/friction 0.2 는 모든 관절 클래스에 같은 값 → 실측 아닌 범용값으로 취급. Unitree 모터 단품 질량 미공개 → 토크 밀도는 DM 만 계산 |
| 잔차가 모터 간 차이보다 큼 | 중 | WP5 1회 보강 후에도 크면 해당 지표를 비교표에서 제외하고 사유 기록 |

---

## 5. 오늘(9/21) 결정 사항

- [x] 추 없음 → Kt 캘리브 미실시. Kt 는 펌웨어 Flux·NPP 유도값(DM4310 ≈0.97 Nm/A) 참고로만
- [x] 비교 대상 → **팔 관절끼리 클래스 매칭**으로 확정 (`isaac/actuators.py: CLASS_MATCH`): 어깨 DM8009↔G1 shoulder(25 Nm)↔H1 shoulder(40), 팔꿈치 DM4340↔G1 elbow(25)↔H1 elbow(18), 손목 DM4310↔G1 wrist(25/5)↔H1-2 wrist(19). Isaac Lab 내장 cfg 는 팔 파라미터가 플레이스홀더라 **Unitree 공개 URDF/MJCF 값을 직접 입력** (`isaac/specs.yaml`, 출처 URL 포함). G1 다리는 비교표 제외
- [x] 잔차 합격 기준: 위치 RMS 2°, 토크 RMS 10 % × **정격** 토크 (제안값 적용, 3축 통과)

---

## 6. 운용 규칙 (기존)

- 종료는 Ctrl+C만 사용
- 실험 전 비상정지 해제 확인
- CAN 인터페이스 다운 시 `ip link set canX up` 재기동
