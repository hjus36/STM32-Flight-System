# 제어 모델과 생성 코드 대응

[README](../README.md) · [모델](../control/simulink/) · [모델 workspace 파라미터](../control/matlab/model_parameters.json)

21개 모델은 원본 workspace의 활성 모델에서 선별했습니다. 과거 before_build 백업을 최종 모델로 쓰지 않았습니다. 공개 사본의 creator/lastModifiedBy 개인 메타데이터와 ARM 모델의 기기별 CubeMX 절대경로만 제거했습니다. 모델 연산·선·블록 파라미터·workspace 행렬은 유지했고 MATLAB에서 21개를 실제 로딩하여 참조 및 0.01s fixed-step 설정을 확인했습니다.

| 단계 | 모델 / 관계 | 목적 |
|---|---|---|
| 기본 루프 | Roll_Single_P / PI / PID_Full / PID_2DOF | 단일 축 이상화 플랜트 응답 비교 |
| cascade | Roll_Double_PID, Pitch_Double_PID, Roll_Pitch_Double_PID | 외부 각도·내부 rate, 정/부 step 비교 |
| Normal/SIL | Roll_Pitch_SIL_Harness → Roll_Pitch_Controller_SIL | single Normal과 생성 코드 결과 비교 |
| 포화·tracking | Roll_Pitch_Controller_AW + Motor_Mixing_Bounded | achieved torque anti-windup |
| 시험 harness | Mixer_AW_Integration, Reversal_AW/Baseline, Disturbance_AW/Baseline | 포화·반전·외란 가상 조건 비교 |
| 통합 모델 | LKM_Roll_Pitch_AW_Core_Harness → LKM_Roll_Pitch_AW_Core → AW controller / bounded mixer | 지연된 achieved torque feedback |
| ARM 최종 | LKM_AW_Core_F405 → LKM_AW_Ctrl_F405 / LKM_AW_Mixer_F405 | STM32 통합 및 host fixture에 사용한 생성 코드 |

모든 참조 모델은 같은 `control/simulink/`에 있으며 setup_portfolio가 MATLAB path에 등록합니다. 모델 workspace에 저장된 행렬은 JSON으로 추출했으며 JSON은 **관측 목록이지 모델을 덮어쓰는 설정 파일이 아닙니다**. 블록 내 직접 지정 gain은 아래 생성 코드에서 교차 확인했습니다.

## 최종 연산 계약

- Ts=0.01s, single. 각도 rad, rate rad/s, 토크 N·m, collective 및 motor model thrust N.
- 각도 P=2.0; 내부 rate PI Kp=0.05, Ki=0.2; tracking gain=2.0; derivative 연산 없음.
- 적분 업데이트는 `((achieved - requested)*2 + rate_error*0.2)*0.01`이며 achieved torque에는 Core의 한 step 지연이 있습니다.
- Mixer workspace: 시험용 D=0.21m, L=0.105m, d≈0.0742462128m, 모터당 제한 0~2N. K_mix의 yaw 열은 0. 실제 추력 계측·기체 식별 결과가 아닙니다.
- 통합 C host fixture의 플랜트 J=0.025, D=0.11은 별도 합성 가정입니다. Mixer workspace D(거리)와 plant damping D(감쇠)는 이름이 같아도 다른 물리량입니다.

## 재현 범위

기존 Normal/SIL 보고서는 6개 신호, 각 1001 sample 비교이며 실제 측정이 아닙니다. 공개 경로에서 Roll/Pitch 모델과 Normal/SIL을 재실행한 결과는 [재현 기록](reproduction-results.md)을 따릅니다.

ARM 최종 모델의 CubeMX project 연결은 각 PC에서 설정해야 합니다. 공개 정리 중에는 현재 펌웨어 `.ioc`를 과거 codegen `.ioc`로 임의 대체하지 않았고 ARM 모델의 코드를 재생성하여 기존 검증 C를 바꾸지 않았습니다. 생성 코드와 해당 모델 파일을 모두 보존하되 **새 PC에서 bit-identical ARM codegen 재현은 미검증**입니다. 코드 생성 시 original model configuration·ST support package·CubeMX target을 점검한 별도 작업 사본을 사용합니다.
