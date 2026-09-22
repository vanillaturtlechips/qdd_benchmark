# QDD 액추에이터 비교표: 데이터시트 + WP6 시뮬 열 (2026-09-22)

출처 등급: **M**=실측(다이노/펌웨어 읽기/처프 피팅), **D**=데이터시트, **S**=시뮬 cfg 값(튜닝/플레이스홀더 가능). 값 출처 URL은 `specs.yaml`.

주의: Unitree MJCF armature 0.01 은 전 관절 공통값(범용), K-Scale MJCF 는 클래스별 상이(튜닝 추정). DM armature 는 DM4340 만 처프 실측, DM4310/8009 는 펌웨어 로터관성×Gr² (링크·그리퍼 관성 불확실성 > armature 라 분리 불가). '피크'는 데이터시트 피크 또는 URDF effort. 정격 미공개는 빈칸.

시뮬 열(있을 때): 클래스별 동일 리그·동일 PD 의 Isaac 단일 관절 벤치 (`isaac/bench_metrics.md` 에 전체 지표·정의). 팔 클래스(wrist/elbow/shoulder)만 벤치 대상, leg 행은 빈칸. DM 열만 실측 검증(WP5 잔차 Δq ≤0.9°, Δτ ≤3 % 정격), 타사 열은 사양 기반 예측.

| 계열 | 액추에이터 | 클래스 | 피크 Nm | 정격 Nm | 최고속도 rad/s | 감속비 | 질량 kg | 피크밀도 Nm/kg | 정격밀도 Nm/kg | 피크/정격 | armature kg·m² | arm 출처 | τpk/armature rad/s² | 로봇 | 비고 | 시뮬 −3dB Hz | 시뮬 추종 RMS ° | 시뮬 정착 ms | 시뮬 백드라이브 정지 s |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| Unitree | G1 wrist_roll | wrist | 25 | - | 37 | - | - | - | - | - | 1.00e-02 | S(MJCF 공통값) | 2500 | G1 | URDF effort | 7.70 | 1.70 | 114 | 0.48 |
| Unitree | H1-2 wrist_roll | wrist | 19 | - | 31.4 | - | - | - | - | - | - | - | - | H1-2 | URDF effort | 18.48 | 0.50 | 113 | - |
| Robstride | RS00 | wrist | 14 | 5 | 33 | 10 | 0.31 | 45.2 | 16.1 | 2.8 | 5.00e-03 | S(K-Scale MJCF) | 2800 | K-Bot |  | 10.67 | 1.02 | 104 | 1.15 |
| Berkeley | 5013 | wrist | 9.7 | 4.59 | 83.7 | 9 | 0.251 | 38.6 | 18.3 | 2.1 | 4.94e-04 | D(rotor×G²) | 19632 | Berkeley Humanoid | 토크 클래스 매칭(팔 미장착) | 17.41 | 0.53 | 111 | - |
| Damiao | DM4310 | wrist | 7 | 3 | 20.9 | 10 | 0.3 | 23.3 | 10 | 2.3 | 1.69e-03 | M(fw) | 4142 | OpenArm | 마찰 실측, armature 펌웨어 | 13.63 | 0.88 | 112 | 0.33 |
| Unitree | G1 wrist_pitch | wrist | 5 | - | 22 | - | - | - | - | - | 1.00e-02 | S(MJCF 공통값) | 500 | G1 | URDF effort | 7.99 | 1.61 | 111 | 1.40 |
| Berkeley Lite | 5010 | wrist | 4 | - | 10 | - | - | - | - | - | 2.00e-03 | S(Isaac cfg) | 2000 | BH Lite | 사이클로이드, 값=시뮬 cfg | 13.38 | 0.65 | 104 | - |
| MIT | U10@elbow | elbow | 52.2 | - | 35 | 9.33 | - | - | - | - | 5.56e-03 | M(dyno) | 9386 | MIT Humanoid | 벨트 증속 포함 | > 40 | 0.10 | 80 | 한계 접촉 |
| Damiao | DM4340 | elbow | 27 | 9 | 5.45 | 40 | 0.375 | 72 | 24 | 3.0 | 2.56e-02 | M(chirp fit) | 1056 | OpenArm | 마찰·armature 실측 | 12.24 | 0.72 | 76 | 0.08 |
| Unitree | G1 elbow | elbow | 25 | - | 37 | - | - | - | - | - | 1.00e-02 | S(MJCF 공통값) | 2500 | G1 | URDF effort | 28.85 | 0.23 | 74 | 0.12 |
| Unitree | H1 elbow | elbow | 18 | - | 20 | - | - | - | - | - | - | - | - | H1 | URDF effort | > 40 | 0.00 | 86 | 한계 접촉 |
| Robstride | RS02 | elbow | 17 | 7 | 42.9 | 7.75 | 0.38 | 44.7 | 18.4 | 2.4 | 1.50e-03 | S(K-Scale MJCF) | 11333 | K-Bot |  | > 40 | 0.07 | 84 | 0.07 |
| Robstride | RS03 | shoulder | 60 | 21 | 20.4 | 9 | 0.9 | 66.7 | 23.3 | 2.9 | 5.00e-03 | S(K-Scale MJCF) | 12000 | K-Bot |  | 3.40 | 6.81 | 859 | - |
| Berkeley | 8513 | shoulder | 45.3 | 18.9 | 40.7 | 9 | 0.756 | 59.9 | 25 | 2.4 | 5.59e-03 | D(rotor×G²) | 8105 | Berkeley Humanoid | 토크 클래스 매칭(팔 미장착) | 3.39 | 6.81 | 861 | - |
| Damiao | DM8009 | shoulder | 40 | 20 | 16.8 | 9 | 0.93 | 43 | 21.5 | 2.0 | 1.67e-02 | M(fw) | 2397 | OpenArm | 마찰 실측(hold), armature 펌웨어 | 3.26 | 6.27 | 606 | 2.48 |
| Unitree | H1 shoulder_pitch | shoulder | 40 | - | 9 | - | - | - | - | - | - | - | - | H1 | URDF effort | 3.43 | 6.83 | 787 | - |
| MIT | U10@shoulder_flexion | shoulder | 33.6 | - | 55 | 6 | 0.619 | 54.3 | - | - | 2.30e-03 | M(dyno) | 14609 | MIT Humanoid | 다이노 포화 31 Nm | 3.41 | 6.83 | 855 | - |
| Unitree | G1 shoulder_pitch | shoulder | 25 | - | 37 | - | - | - | - | - | 1.00e-02 | S(MJCF 공통값) | 2500 | G1 | URDF effort | 3.37 | 6.69 | 788 | - |
| Unitree | G1 knee | leg | 139 | - | 20 | - | - | - | - | - | - | - | - | G1 | Isaac Lab cfg | - | - | - | - |
| MIT | U12@knee | leg | 136 | - | 22.5 | 12 | - | - | - | - | 8.00e-02 | M(dyno) | 1700 | MIT Humanoid | 벨트 증속 포함 | - | - | - | - |
| Robstride | RS04 | leg | 120 | 40 | 20.9 | 9 | 1.42 | 84.5 | 28.2 | 3.0 | 7.00e-03 | S(K-Scale MJCF) | 17143 | K-Bot |  | - | - | - | - |
| Berkeley | 10413 | leg | 81.1 | 34.2 | 27.9 | 9 | 1.01 | 80.2 | 33.8 | 2.4 | 1.21e-02 | D(rotor×G²) | 6675 | Berkeley Humanoid | 토크 클래스 매칭(팔 미장착) | - | - | - | - |
| Berkeley | 8518 | leg | 62.6 | 26.1 | 29 | 9 | 0.856 | 73.1 | 30.5 | 2.4 | 7.61e-03 | D(rotor×G²) | 8222 | Berkeley Humanoid | 토크 클래스 매칭(팔 미장착) | - | - | - | - |
| Berkeley Lite | 6512 | leg | 6 | - | 10 | - | - | - | - | - | 7.00e-03 | S(Isaac cfg) | 857 | BH Lite | 사이클로이드, 값=시뮬 cfg | - | - | - | - |
