# Firmware Architecture — 구현 계획

[프로젝트 개요](../README.md) · [코드 관리](../firmware/README.md)

이 문서는 앞으로 구현할 구조를 정리한 초안입니다. LKM_FC_V1용 실행 바이너리, 검증된 센서 드라이버, 제어 루프 또는 비행 펌웨어는 현재 이 저장소에 없습니다.

## STM32 펌웨어 구조

| 영역 | 예정 책임 | 저장 위치 |
|---|---|---|
| Board / HAL | clock, GPIO, SPI/I²C/UART, Timer, ADC, SWD bring-up | `firmware/stm32/` |
| Drivers | 센서 초기화·readout, iBUS, EEPROM, 통신 오류 보고 | `firmware/drivers/` |
| Control | 입력 정규화, 제어 상태, PID, Motor Mixing, 출력 제한 | 모델과 통합 결과에 따라 배치 |
| Generated | 모델에서 생성한 C 코드와 생성 버전·조건 | `firmware/generated/` |
| Application / Diagnostics | 초기화 순서, arming/failsafe, timestamp, 로깅 | `firmware/stm32/` |

Bare-metal/RTOS 선택, task 주기, DMA·interrupt 사용 방식은 아직 확정 구현이 아닙니다. 실제 센서 데이터 주기와 실행시간을 측정한 뒤 결정합니다.

## HAL 및 센서 드라이버

현재 설계의 핀맵·전압·SPI clock 조건에 맞춰 HAL 초기화를 작성하고, 장치별 ID 확인 및 단독 통신 시험부터 진행합니다. BNO086은 SH-2/SHTP, reset/wake startup, 기능별 report와 축 방향을 확인합니다. 기존 참고 펌웨어의 BNO080 드라이버가 그대로 검증됐다고 가정하지 않습니다.

ICM-20602와 LPS22HH는 센서 설정값, timestamp 및 raw data의 단위를 기록합니다. AT24C08C는 주소·WP·read/write와 전원 재인가 후 데이터 유지 시험이 필요합니다.

## MATLAB/Simulink 모델 기반 제어

센서 좌표계·단위·sample time을 먼저 정의합니다. 제어 모델의 입력·출력 계약을 정한 후 자세/각속도 제어, 포화, anti-windup 및 안전 상태 처리를 모델에서 검토할 계획입니다. 아직 모델 파일이나 튜닝된 게인은 없습니다.

## C 코드 생성 및 통합 계획

1. 모델과 MATLAB/Simulink 버전, solver, sample time, parameter set을 기록합니다.
2. 시뮬레이션 입력과 기대 응답을 저장하고 재현성을 확인합니다.
3. C 코드 생성 설정과 생성 산출물의 출처를 기록합니다.
4. 센서 입력·PWM 출력과 연결하되 생성 파일과 수동 HAL/driver 코드를 구분합니다.
5. 보드에서 실행시간, 지연, 단위 변환과 출력 제한을 측정합니다.

## PID 및 Motor Mixing 구조

RC 명령 → 목표 자세/각속도 → 제어 출력 → Motor Mixing → 범위 제한 → PWM 순서를 계획합니다. 모터 번호, CW/CCW, 기체 좌표계 및 센서 축은 실제 장착 방향과 일치시켜야 합니다. 6개 물리 PWM 채널이 있다는 사실과 6개 모터용 mixer 구현 완료는 별개입니다.

센서 오류·RC timeout·arming 전 출력·비정상 reset에서 안전한 출력 상태를 검증한 뒤 모터 시험 범위를 확대합니다. 실제 시험 결과는 [Validation](validation.md)에 연결합니다.
