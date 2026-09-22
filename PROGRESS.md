# 진행 현황 — QDD 액추에이터 벤치마크

**갱신:** 2026-09-22 (화) 13:30 · **마감:** 2026-09-24 (목) · **진행률:** 27/30 항목 (90 %), WP1~WP6 완료 · **로봇 실험 종료**

---

## 1. 한눈에 보기

| WP | 내용 | 상태 |
|---|---|---|
| WP1 | 실물 계측 인프라 (녹화·파서·궤적 발생기·안전) | ✅ 9/21 |
| WP2 | Isaac Lab 벤치 (기동, cfg 덤프, 단일 관절 리그) | ✅ 9/21 |
| WP3 | 실물 실험 3축 (joint7/3/1) | ✅ 9/22 |
| WP4 | 시스템 식별 → `DCMotorCfg` 3종 | ✅ 9/22 |
| WP5 | sim2real 잔차 검증 | ✅ 9/22 — **3축 모두 PASS** |
| WP6 | 18개 프리셋 공통 벤치 + 지표 표 | ✅ 9/22 — `isaac/bench_metrics.md`, `comparison_specs.md` 시뮬 열 |
| WP7 | 보고서 | 🟡 9/22 초안 PDF 18쪽 `docs/report/report.pdf` (md 정본 → pandoc → weasyprint). 검토 대기 |

---

## 2. 핵심 결과

### 2.1 OpenArm 실물 QDD 식별값 (Isaac `DCMotorCfg`)

| | DM4310 (손목 j7) | DM4340 (팔꿈치 j3) | DM8009 (어깨 j1) |
|---|---|---|---|
| armature kg·m² | 0.00169 펌웨어 (실측 분리 불가 — 그리퍼 관성 불확실성 > armature) | **0.0256** 실측 (펌웨어 0.0290, −12 %) | 0.0167 펌웨어 (링크 관성 0.43의 4 %라 분리 불가) |
| Coulomb 마찰 Nm | 0.122 | 0.481 | 0.814 (하류 관절 PD 유지 조건; 비유지 0.95) |
| 점성 마찰 Nm·s/rad | 0.025 | 0.188 | 0.10 ± 0.05 |
| 중력 부하 m·g·d Nm | 0.72 | 0.06 | 11.7 |
| CAN 명령→피드백 지연 | 150 µs | 150 µs | 150 µs |
| 감속비 / 데이터시트 피크 | 10 / 7 Nm | 40 / 27 Nm | 9 / 40 Nm |
| 백드라이브 (자중) | 진자 운동 | **0.0 mrad — 불가** | 미측정(안전) |

**방법론 검증:** DM4340(joint3, 롤 축이라 하류 질량 무관)에서 실측 armature가 펌웨어 로터관성×Gr² 와 12 % 안에서 일치. DM4310 은 9/22 오후 정정 — hand 링크 질량(URDF 0.127 kg)을 0 으로 놓은 계산 오류로 '5 % 일치'가 나왔던 것이며, 보정 후에는 그리퍼 관절 불확실성(시뮬 중력 0.54 vs 실측 0.72 Nm)이 armature 보다 커서 분리 불가. 총 유효 관성 0.0059 는 홀드아웃으로 검증됨.

### 2.2 sim2real 잔차 (보고서 오차 막대)

| 모터 | Δq RMS | Δτ RMS (정격 대비) | 기준 | 유효 대역 |
|---|---|---|---|---|
| DM4310 | 0.38° | 0.10 Nm (3 %) | 2° / 0.30 | 0.3~2.9 Hz |
| DM4340 | 0.57° | 0.25 Nm (3 %) | 2° / 0.90 | 0.3~2.9 Hz |
| DM8009 | 0.84° | 0.48 Nm (2 %) | 2° / 2.0 | **≤1.5 Hz** |

DM8009 2.9 Hz 홀드아웃은 FAIL(Δτ 2.56) — 원인은 팔꿈치 PD 스프링(kp 60, I 0.085 → 공진 ~4 Hz)과의 다관절 커플링. 액추에이터가 아닌 구조 동역학이라 단일 관절 리그 범위 밖. 전체 팔 Isaac 재생으로 확인 가능(선택 과제).

### 2.3 비교 대상 (오픈소스 휴머노이드 QDD 6계열, 23개)

`isaac/comparison_specs.md` — Damiao(실측) · Robstride/K-Bot · Unitree G1/H1 · MIT Humanoid · Berkeley Humanoid · Berkeley Humanoid Lite. 출처 등급 M(실측)/D(데이터시트)/S(시뮬 cfg) 명시. 팔 관절 클래스(손목/팔꿈치/어깨)로 매칭.

잠정 관찰(실측 전 데이터시트 기준): DM4340은 그룹 내 유일한 40:1 → 피크밀도 최고(72 Nm/kg)지만 반사관성 DM4310의 16배, 백드라이브 불가. DM4310/DM8009 피크밀도는 Robstride·Berkeley 대비 60~65 %, 정격 기준은 근접.

---

### 2.4 WP6 공통 벤치 (Isaac 단일 관절 리그, 18 프리셋 × {step, chirp, multisine, free_decay})

`isaac/bench_all.py` (한 프로세스에 18 Articulation 동시 시뮬, 4.3 분) → `isaac/bench_metrics.py` → `isaac/bench_metrics.md` + `data/sim/bench/plots/<class>.png`, `comparison_specs.md` 에 시뮬 4열 병합.
클래스 리그 = WP5 잔차 검증 리그 그대로 (joint7 / joint3 / joint1 의 I_link·m·g·d + 운용 kp/kd). effort_limit 은 전 프리셋 피크로 통일.

| 클래스 (리그) | −3 dB 대역폭 Hz | 추종 RMS ° (multisine 0.4 rad) | 백드라이브 (놓은 뒤) |
|---|---|---|---|
| wrist (I 0.0056, mgd 0.72, kp 10) | H1-2 18.5 · BH5013 17.4 · **DM4310 13.6** · BHL 13.4 · RS00 10.7 · G1 7.7~8.0 | H1-2 0.50 · BH 0.53 · BHL 0.65 · **DM 0.88** · RS00 1.02 · G1 1.6~1.7 | DM 0.33 s 정지, 정지각 9° (Coulomb 스톨 asin(0.122/0.72)=9.8° ✓) · G1 0.5~1.4 s · 마찰 0 모델은 무한 진동 |
| elbow (I 0.0022, 롤축, kp 70) | RS02/H1/MIT > 40 · G1 29 · **DM4340 12.2** | H1 0.00 · RS02 0.07 · MIT 0.10 · G1 0.23 · **DM 0.72** | 2 rad/s 코스트다운: DM 0.08 s / 0.076 rad 정지 · 마찰 0 모델은 ±170° 한계까지 굴러감 |
| shoulder (I 0.434, mgd 11.7, kp 70) | 전부 3.3~3.4 (**DM8009 3.26**) | 전부 6.3~6.8° (**DM 6.27**) | **DM 2.5 s 정지**, 나머지 10 s 내 미정지 (G1 감쇠만) |

- 어깨 클래스는 **부하 지배** (I_link 0.434 ≫ armature ≤0.017): PD 지표는 모터 무관하게 3 % 내 동일, 마찰만 차이(정착 606 vs 790~860 ms). 어깨급 변별은 토크밀도(데이터시트)뿐
- 손목 클래스 대역폭은 armature/I_link 순 (G1 0.01 = 1.8×I_link → 7.7 Hz). DM4310 은 마찰 때문에 정상오차 1.3°(타사 0.65°) — Coulomb/kp 데드밴드
- 팔꿈치 클래스는 DM4340 의 armature(12×I_link)+v_max 5.4 rad/s 가 그대로 드러남 (처프 9.5 % 속도-토크 클리핑)
- 마찰 미공개(0) 모델 — H1/H1-2/MIT/Berkeley/BHL — 의 백드라이브·정상오차 열은 "데이터 없음"이지 우수함이 아님 (보고서에 명기)
- **마찰 민감도 (9/22 15:00, `--friction zero | floor:0.02`, 3클래스 전부, `bench_metrics.md` §마찰 민감도)**: 전 프리셋 마찰 0 → DM4310 정상오차 1.30→0.65°(타사와 동일), 추종 0.88→0.62°(BH 0.53 ~ BHL 0.65 사이), DM4340 추종 0.72→0.50°. 타사에 DM 실측 비율(피크의 2 %) 마찰을 주면 BH5013 0.65→1.69°, RS00 0.80→1.78° 로 **DM과 같거나 더 나쁨**. 대역폭 순위는 세 조건 모두 불변(armature 순). → DM 실측이 틀린 게 아니라 "실측 있는 쪽만 정직하게 불리한" 비대칭. 부수 관찰: 19~25 Nm급(H1-2, G1 roll)에 2 % 마찰(0.4~0.5 Nm)을 주면 kp 10 손목 리그에서 대역폭 붕괴(18→0.5 Hz) — 저게인 관절에 큰 모터를 꽂으면 마찰이 소신호 응답을 잡아먹음

## 3. 발견 사항 (보고서 감)

1. **Isaac Lab OpenArm 에셋 버그** (USD 직접 검사로 확정) — `hand`(URDF 실제 링크 0.127 kg)·`ee_tcp`(URDF `hand_tcp` 무관성 프레임)에 `physics:mass` 미기입 → PhysX 기본 1 kg → joint7 관성 9×, joint1 2.5× 과대. 추가로 두 고정 관절이 **revolute 로 export** 되어 DOF 로 잡힘. 이슈 초안 정정 완료 `docs/isaaclab_issue_openarm_ghost_mass.md`. 대상: `isaac-sim/IsaacLab`
1-b. **URDF 그리퍼 질량 과소** — 시뮬 중력 토크 0.54 vs 실측 0.72 Nm (+33 %). 그리퍼 모터(DM4310 0.3 kg) 미포함 추정
2. **백래시 영역** — DM4310, 3.5 Hz 이상·출력 변위 <1°에서 J 추정이 0.006→0.001로 감소. 로터-링크 분리(비강체 전달). DCMotor 모델 한계이자 별도 비교 지표 후보
3. **J의 여기 진폭 의존성** — 0.6 Nm 처프 0.0060 vs 0.3 Nm 0.0043 → armature 계통 불확실성 ±0.0015
4. **고주파 처프로 마찰 추정 시 −16 %** (합성 검증) → 2단계 피팅(마찰은 정속, J는 처프) 확정
5. **DM4340은 QDD로 분류하기 어려움** — 40:1, 최고속 5.4 rad/s, 자중 백드라이브 0
6. **Isaac Lab `friction` 은 정지 마찰만** (Isaac Sim 5.x 3항 마찰 모델) — 사양 기반 프리셋(Unitree/Robstride) 이 `friction` 만 채워 움직이는 동안 Coulomb 0 이었음. 벤치 첫 실행에서 G1 wrist 진자가 31 rad 를 감쇠 없이 돌아 발견 → `_dc` 가 `dynamic_friction` 도 채우도록 수정. DM/synth 프리셋은 원래 명시해서 WP4·5 결과 영향 없음
7. **`DCMotorCfg.effort_limit` 은 속도-토크 직선의 상한** — 정격으로 두면 피크 토크에 절대 못 미침. 벤치는 `--effort peak` 로 통일, 실물 정격 열은 데이터시트 표에만

---

## 4. 사고 및 교훈 (9/22 10:58)

`--hold-others` 첫 실기 실행에서 joint5~7에 **q_des = −12.47 rad** 명령이 나가 손목이 급격히 꺾임 → 비상정지. **손상 없음 확인.**
- 원인: openarm_can 라이브러리가 CTRL_MODE 쓰기 응답(`xx 00 55 0A 01`)을 STATE 프레임으로 파싱 → `get_position()` 오염 + 진짜 상태 응답 도착 전 0.3 ms 레이스 + 코드가 검증 없이 사용. 증거 `data/2026-09-22_j1b/can0.log` t+153.87
- 재설계: 상태는 스니퍼에서 파라미터 프레임 걸러 자체 디코딩 → 정착 확인(≥5프레임, 편차<0.005, |q|≤π) → 표 출력·`y` 확인 → 게인 2 s 램프 → 편차 0.15 rad 감시. 실기 dry-check(`--hold-check`) → hold 정지 → 본 실험 순으로 재검증, 사고 없음
- 규칙(메모리 저장): 새 관절 enable 시 목표 범위검사·검증된 상태프레임·사용자 확인 필수. 새 제어 기능은 저게인/dry 검증 없이 운용 게인으로 실기 투입 금지

---

## 5. 절단·변경한 계획

| 원안 | 변경 | 사유 |
|---|---|---|
| 비교 대상 G1 다리 | 팔 관절 클래스 매칭, 6계열로 확대 | Isaac Lab 내장 cfg는 팔이 플레이스홀더. 사용자 지적 |
| 전체 팔 ros2_control 스텝·sine | 단일 축 multisine 홀드아웃 3개 | 액추에이터 모델 검증에 충분 |
| T_MOS 가열 램프 | 대기 온도(27~31 °C)만 | 30분 소요, 열 모델 범위 외 |
| 무부하 최고속도 실측 | 데이터시트 | 팔 가동범위에서 도달 불가 |
| Kt 진자 캘리브 | 미실시 (추 없음) | Kt는 펌웨어 Flux·NPP 유도값 참고 |
| DM8009 armature 실측 | 펌웨어 값 | 링크 관성의 4 %, 처프로 분리 불가 |

---

## 6. 산출물 지도

```
qdd_benchmark/
├── PLAN.md / PROGRESS.md
├── config/motors.yaml            축↔CAN·안전한계·식별 대표축
├── tools/
│   ├── record.sh                 candump(+bag) 녹화
│   ├── candump2csv.py            피드백/명령/파라미터 프레임 파서
│   ├── excite.py                 MIT 궤적 발생기 (6모드, 사다리꼴 반전, 중력보상 처프, hold-others)
│   ├── safety.py                 클램프·감시
│   ├── fit_params.py             2단계 시스템 식별 (합성 검증 ±3 %)
│   ├── sim2real.py               홀드아웃 잔차 판정
│   └── README.md                 스키마·절차·함정
├── isaac/
│   ├── specs.yaml                6계열 사양 + 출처 URL + 펌웨어 읽기값
│   ├── actuators.py              프리셋 19개 (DM 3종 식별값 반영), CLASS_MATCH
│   ├── single_joint_rig.py       1-링크 진자 리그 (등가 관성·중력, replay)
│   ├── link_inertia.py           OpenArm 질량행렬 → I_link (유령질량 제거)
│   ├── rig_usd.py                리그 USD 생성 + 등가 막대 계산 (rig/bench 공용)
│   ├── bench_all.py              WP6 벤치 러너 (18 프리셋 동시, 4 프로토콜)
│   ├── bench_metrics.py → bench_metrics.md   지표 표 + plots/<class>.png
│   ├── compare_specs.py → comparison_specs.md   데이터시트 비교표 (+시뮬 열)
│   └── dump_actuator_cfgs.py → actuator_cfgs.md Isaac Lab 내장 cfg 덤프
├── docs/isaaclab_issue_openarm_ghost_mass.md
└── data/                          (149 MB, git 제외)
    ├── 2026-09-21_j7/   identified_dm4310.yaml + fit/
    ├── 2026-09-22_j3/   identified_dm4340.yaml + fit/
    ├── 2026-09-22_j1/   identified_dm8009.yaml (j1b 사고 로그, j1c hold 재측정)
    ├── sim/replay_*     residual.yaml / residual.png
    ├── sim/bench/       <class>/<preset>/{step,chirp,multisine,free_decay}.csv, metrics.csv, plots/
    └── sim/bench_fric_{zero,floor02}/   마찰 민감도 재실행 (같은 구조)
```

---

## 7. 남은 작업

| | 작업 | 예상 |
|---|---|---|
| WP7 | 보고서 초안 완료 (`docs/report/`: report.md 정본, build.py 한 번에 표·그림·PDF 재생성). 남은 것: 사용자 검토·수정 반영, 저자·소속 기입 | 1h |
| 선택 | 전체 팔 Isaac 재생 (7축 명령 candump에 있음) → DM8009 2.9 Hz 커플링 확인 | 2h |
| 선택 | Isaac Lab 이슈 제출 | 10분 |
| 정리 | git 커밋 (WP1 이후 전부 미커밋), 데이터 149 MB 는 HF Dataset 후보 | 20분 |

결정 대기 없음 (잔차 합격 기준은 제안값 2° / 10 % 정격으로 적용했고 3축 모두 통과).
