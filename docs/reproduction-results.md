# 공개 복사본 재현 결과 — 2026-11-06

[실행 방법](reproduction.md) · [소스 SHA-256](source-manifest.json) · [검증 범위](validation.md)

아래 작업은 **저장소 루트**에서 실행했습니다. 개발 원본은 읽기 전용이고 새 실행 파일·로그·MATLAB cache는 Git 제외 `.local/portfolio_prepare_20261106/` 또는 `.local/matlab-analysis/`, `.local/normal-sil/`에 저장했습니다. ARM compile은 flash/MCU 실행을 하지 않습니다. raw MATLAB log에는 기기·라이선스 정보가 있어 공개하지 않고 관측 결과만 정리합니다.

| 실행 | 종료코드 | 관측 결과 |
|---|---|---|
| `python testing/offline-sil/run_integrated_sil.py --out .local/portfolio_prepare_20261106/host_with_plots` | 0 | 26,860/26,860 반복 조건, 15 scenario, baseline 1,000 virtual steps, production lock 13/13, target macro 거부 2종; PNG 6개 생성 |
| `python testing/offline-sil/tests/test_runner_safety.py` | 0 | 10/10 runner fault/safety test |
| 아래 scheduler GCC compile 후 실행 | 0 / 0 | scheduler + ring buffer 139/139 |
| `python testing/offline-sil/tools/build_arm.py --gcc <ARM_GCC> --drivers <FW_F4_V1.28.3/Drivers> --out .local/portfolio_prepare_20261106/arm_rerun` | 0 | 55 translation units compile/link, ELF32 little-endian ARM; required service symbols 10개와 generated object 함수 3개 확인 |
| MATLAB `load_system` 21모델 / model workspace 조회 | 0 | 참조 모델 모두 존재, fixed-step 0.01s; 저장하지 않음 |
| MATLAB `analyze_roll_double_pid; analyze_pitch_double_pid; analyze_roll_pitch_double_pid; run_normal_sil` | 0 | Normal/SIL 6신호 × 1,001 samples; 최대 시간차/값 차이 모두 0 |
| MATLAB `analyze_roll_pid_models; render_roll_pid_figures` | 0 | P/PI/1DOF PID/2DOF PID 4개 분석 및 derivative-kick 그래프 생성 |

Python은 Matplotlib 3.10.6가 설치된 환경을 사용했습니다. Host GCC 14.2.0, ARM GCC 14.3.1, MATLAB 26.2.0.3433840 R2026b Update 1입니다. 정확한 compiler 경로는 PC 의존 값이므로 위 명령에서는 placeholder로 표시합니다.

Scheduler 독립 명령 (GCC와 저장소 루트를 기준으로 실행):

```text
gcc -std=c11 -Wall -Wextra -Werror -Ifirmware/stm32/LKM_BNO086_Bringup/Core/Inc testing/offline-sil/tests/test_scheduler_buffer.c firmware/stm32/LKM_BNO086_Bringup/Core/Src/lkm_fc100hz_core.c -lm -o .local/portfolio_prepare_20261106/scheduler_regression.exe
.local/portfolio_prepare_20261106/scheduler_regression.exe
```

MATLAB batch에서 `addpath('control/matlab'); setup_portfolio();` 후 표의 함수를 실행했습니다. [재실행 수치 요약](../control/simulations/reproduction_20261106/)을 함께 보존합니다. Roll 5°/Pitch -3° cascade 모의 응답의 rise time 약 0.95s, settling 약 1.55s, overshoot 약 0.10243%는 이상화 모델 결과입니다. 단일 PID 비교에서 2DOF 설정은 초기 max-abs control을 0.523599→0.087266으로 약 83.33% 줄였으며 모든 기체나 PID 설정에 일반화하지 않습니다. 단일 P의 정상상태 오차 2.5°와 NaN 지표도 그대로 남깁니다.

## 데이터 무결성과 그림

실제 과거 run `sil_v1_20261106_040855`의 CSV와 이번 host 재실행 CSV의 SHA-256은 모두 `e326a9e2616627b3c9c04381fd790ff7476d128b04b94dc9c59434451621a432`입니다. 5,472행, 15 scenario이며 모든 행에서 esc_enabled/controller_ready=0, virtual_valid=0인 행의 네 motor model thrust=0을 별도 확인했습니다. 26,860은 독립 시험 개수가 아닙니다.

README/validation의 6 PNG는 보존한 실제 CSV를 `plot_sil.py`로 다시 그렸습니다. CSV 선택·단위 변환·first-fault 집계는 plotter에 있으며 **Synthetic Host SIL / Not Hardware or Flight Validation** 문구를 그림마다 표시했습니다. 그래프 title/축/단위/legend가 잘리지 않는지 육안 검토했습니다. 수치는 새로 만들거나 실측으로 재표시하지 않았습니다.

ARM ELF SHA-256: `9f128746091cc478a9899d6107e4f68d2dd283870b4ccf3ee25cf286cb550249`. 이 해시는 이번 빌드 산출물 식별용이며 debug path/toolchain에 따라 달라질 수 있습니다. ELF는 공개 대상에서 제외했습니다.

## 실패·경고 및 미검증 범위

- 최초 host 실행 환경에 Matplotlib이 없어 그림 생성만 미완료였습니다. C/CSV 성공과 그림 성공을 구분하고 Matplotlib 환경에서 전체를 재실행했습니다.
- 최초 내부 MATLAB inventory helper는 `run`의 working-directory 변경으로 잘못된 상대경로를 해석했습니다. helper의 script-location 기준 경로를 고친 뒤 21모델 조회를 다시 수행했습니다. 공개 모델 알고리즘은 바꾸지 않았습니다.
- Normal/SIL 중 single precision parameter quantization 경고와 optimized-away 내부 SIL signal 4개의 logging 경고가 있었습니다. 비교 대상으로 지정한 출력 6개는 두 실행에서 모두 존재하고 일치했습니다. 경고 없는 빌드라고 주장하지 않습니다.
- ARM helper는 기존 Debug flags/source 목록을 이용한 **새 공개용 compile/link 도구**이며 기존 독립 ELF 감사 전체를 재실행한 것은 아닙니다. CubeIDE GUI import, 새 PC의 ARM target code generation 재현, bit-identical codegen은 미검증입니다.
- 수신기·배터리는 합성 입력입니다. 실물 MCU/SPI/축/주기 jitter/RC/VBAT/ESC/모터/비행 검증은 수행하지 않았습니다. 생산 제어/출력 잠금은 그대로 유지했습니다.
