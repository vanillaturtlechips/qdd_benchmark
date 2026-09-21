# QDD 액추에이터 벤치마크 작업 계획

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
| 비교 대상 | **G1 29DOF(주)**, H1, GR1T2 — Isaac Lab 내장 cfg만 | 외부 레포(Berkeley Humanoid, ToddlerBot 등) USD 변환은 1일 이상 소모 |
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
- [ ] headless 기동 확인 (첫 기동 리스크)
- [ ] `OPENARM_BI_CFG`, `G1_29DOF_CFG`, `H1_CFG`, `GR1T2_CFG` 로드 확인
- [ ] 단일 관절 테스트 리그(1-링크 진자) USD + 액추에이터 cfg 주입 스크립트 골격

### WP3. 실물 실험 실행 — 화 종일 (7h)
- [ ] 오전: 3축 × { 정속 스윕 ±(0.5 ~ max) rad/s, 토크 처프 0.1~20 Hz, 자유 감쇠 } — 축당 1h 고정
- [ ] 오후: 전체 팔 PD 스텝·sine 응답 (현재 kp/kd), 홀드아웃 다중사인 궤적 3회 반복, 명령→피드백 지연 측정
- [ ] 병렬: 1축 연속 토크 유지로 T_MOS 상승 기록 (베이스라인용)
- [ ] **당일 데이터 QA** — 결측·포화·에러 있으면 즉시 재실험 (재실험 슬롯: 오후 마지막 1h)

### WP4. 시스템 식별 — 수 오전 (4h)
- [ ] 오프라인 피팅 (numpy/scipy): τ = J·α + b·ω + c·sign(ω) 최소제곱
- [ ] 무부하 최고속도 → `velocity_limit`, 지연 → `DelayedPD` 스텝 수
- [ ] 산출물: `DM4310 / DM4340 / DM8009 DCMotorCfg` 3개 + 파라미터 신뢰구간

### WP5. sim2real 잔차 측정 — 수 오후 (4h)
- [ ] `openarm.py` 액추에이터를 식별 cfg로 교체
- [ ] Isaac에서 홀드아웃 궤적 재생 → 실물 대비 위치/속도/토크 잔차 RMS
- [ ] 잔차 > 합격 기준이면 마찰 모델·지연 **1회만** 보강
- [ ] 최종 잔차 = 보고서 오차 막대

### WP6. 공통 벤치 비교 — 목 오전 (3h)
- [ ] 단일 관절 리그에 모터 cfg 7~8개 (DM 3종 + G1 다리/팔/허리 + H1 + GR1T2) 동일 프로토콜 실행
- [ ] 지표 테이블 자동 생성

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
| G1/H1 파라미터가 데이터시트 기반 | 확정 | 비대칭 비교임을 보고서에 명기 |
| 잔차가 모터 간 차이보다 큼 | 중 | WP5 1회 보강 후에도 크면 해당 지표를 비교표에서 제외하고 사유 기록 |

---

## 5. 오늘(9/21) 결정 사항

- [ ] 질량·팔길이 아는 추가 있는가? → 있으면 Kt 캘리브를 화 오후 옵션으로 편입
- [ ] H1 / GR1T2까지 포함할지, G1 하나만 깊게 갈지 → 시간 부족 시 첫 번째 절단 대상
- [ ] 잔차 합격 기준 수치 → 제안: 위치 RMS 2°, 토크 RMS 10 % of τ_max

---

## 6. 운용 규칙 (기존)

- 종료는 Ctrl+C만 사용
- 실험 전 비상정지 해제 확인
- CAN 인터페이스 다운 시 `ip link set canX up` 재기동
