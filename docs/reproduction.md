# 재현 환경과 실행

[결과](reproduction-results.md) · [출처](source-and-license.md)

## 확인한 도구

MATLAB **26.2.0.3433840 (R2026b) Update 1**, Simulink/Simulink Coder/Embedded Coder/STM32 Microcontroller Blockset 26.2; STM32CubeIDE **2.1.0**, CubeMX **6.16.0**, STM32Cube FW_F4 **V1.28.3**. Host GCC는 MinGW-w64 **14.2.0**, ARM은 GNU Tools for STM32 **14.3.rel1 / GCC 14.3.1**, 그림 재현 Matplotlib **3.10.6**입니다. Python은 3.10 이상을 사용합니다.

## Host SIL

저장소 루트에서:

```text
python -m pip install -r testing/offline-sil/requirements.txt
python testing/offline-sil/run_integrated_sil.py
python testing/offline-sil/tests/test_runner_safety.py
```

GCC/Clang이 PATH에 없으면 `--compiler <compiler executable>`를 지정합니다. Windows에서는 설치된 MATLAB MinGW도 탐색합니다. 새 결과 기본 위치는 `.local/sil-runs/sil_v1_<timestamp>/`이며 기존 결과 디렉터리 덮어쓰기를 거부합니다. Matplotlib이 없으면 C/CSV 시험은 실행되지만 그림을 만들지 못했다고 출력합니다. 이 경우 전체 그림 재현 완료로 주장하지 않습니다.

원본 그래프 재현:

```text
python testing/offline-sil/tools/plot_sil.py testing/offline-sil/results/sil_v1_20261106_040855/trace_all_scenarios.csv --out .local/figures
```

## MATLAB

저장소 루트를 current folder로 두고:

```matlab
addpath('control/matlab');
setup_portfolio();
analyze_roll_double_pid();
analyze_pitch_double_pid();
analyze_roll_pitch_double_pid();
run_normal_sil();
```

단일 루프 비교는 `analyze_roll_pid_models(); render_roll_pid_figures();`를 별도로 실행할 수 있습니다. 분석은 원본 학습 모델·코드를 기반으로 하며 출력/캐시는 `.local`에 둡니다. 모델을 저장하지 않습니다. Normal/SIL 비교에는 Simulink Coder/Embedded Coder와 지원 host C compiler가 필요합니다. 원본 스크립트와 공개 사본의 경로 변경은 manifest로 추적합니다.

공개 ARM 모델의 기기별 CubeMX 경로는 비워 두었습니다. 모델 열기/시뮬레이션과 ARM target 재생성은 다른 작업입니다. 새 코드 생성을 위해 실제 생성 당시의 target project 설정을 복원하는 단계는 별도 검증이 필요합니다.

## ARM 컴파일 (flash 없음)

1. STM32Cube FW_F4 V1.28.3를 별도 설치합니다. `Drivers/STM32F4xx_HAL_Driver`와 `Drivers/CMSIS`를 사용합니다. vendor 원본은 공개 폴더에 중복 넣지 않습니다.
2. CubeIDE 2.1.0에 최소 프로젝트를 import하고 Drivers를 작업 사본에서 공급하거나, 다음 portable compile helper를 사용합니다.

```text
python testing/offline-sil/tools/build_arm.py --gcc <arm-none-eabi-gcc> --drivers <FW_F4_V1.28.3/Drivers> --out .local/arm-build-01
```

이 helper는 기존 Debug 빌드의 Cortex-M4/hard-float/FPv4, O0, section GC, nano/nosys, HAL 소스 목록을 옮긴 **새 공개용 재현 도구**입니다. 기존 ARM 감사 스크립트를 그대로 실행한 것으로 주장하지 않습니다. output은 `.local`의 빈 새 경로만 허용하고 소스/원본/하드웨어를 수정하지 않습니다. ELF의 ARM 형식·필수 linked symbols와 생성 함수의 object symbols를 확인합니다. 생산 잠금으로 사용되지 않는 함수가 ELF에서 제거될 수 있으므로 object 존재와 실행 연결을 구분합니다.

CubeMX Generate Code는 반드시 별도 작업 사본에서 수행하고 Core/Inc·Core/Src 사용자 소스를 보존·대조합니다. 이 저장소의 `.ioc`는 센서 bring-up용이며 full flight controller 설정이 아닙니다. 빌드 성공은 실기 실행/전기적 안전/비행 승인과 무관합니다.
