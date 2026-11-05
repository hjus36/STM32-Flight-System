# Firmware Architecture — 구현과 검증 경계

[프로젝트](../README.md) · [재현](reproduction.md) · [검증](validation.md)

현재 기준은 `firmware/stm32/LKM_BNO086_Bringup/`의 실제 설치본입니다. 초기 패키지의 드라이버를 덮어 가져오지 않았습니다. 아래의 구현/ARM 컴파일은 MCU 실행 증거가 아닙니다.

| 계층 | 주요 파일 | 구현 / 현재 제한 |
|---|---|---|
| 보드 초기화 | main.c, gpio.c, spi.c, usart.c, .ioc | STM32F405RGT6, HSE, SPI1/2, USART6; full FC 주변장치 초기화가 아님 |
| BNO086 | lkm_bno086, link, shtp, stream, attitude, diag | SHTP 전송·startup 진단·일부 SH-2 rotation vector(0x05/0x08) 파서. 전체 CEVA SDK 아님 |
| ICM-20602 | gyro, hal_spi, board_spi1, diag_core | raw gyro·상태·SPI HAL adapter, 실물 readback 미검증 |
| Dual IMU | dual_imu_diag, imu_time_pair | receipt timestamp, freshness/skew/중복 검사, 실제 취득 동기화 아님 |
| 100Hz | fc100hz_core, fc100hz_bridge | 10ms scheduling, ring buffer, 누락/overrun 감시; 실시간 jitter 미측정 |
| Frame / Feedback | rp_frame_contract, frame_monitor, feedback_gate | 단위·축/부호·accuracy·freshness 조건, 실제 장착 축 승인 전 잠금 |
| Generated core | LKM_AW_Core_F405, Ctrl, Mixer | single 10ms, rad/rad/s/N/N·m, 모델 결과만 취급 |
| Shadow | aw_core_preview, aw_shadow | host에서 통합, 실제 firmware enable=false |
| Guard / Mixer | fc_actuator_guard, virtual_mixer, actuator_monitor | 합성 arming/failsafe, production hard lock; ESC/PWM backend 없음 |

`main.c`는 진단·UART·Dual IMU·100Hz·Shadow·frame·actuator service를 순서대로 호출합니다. Dual IMU는 false로 초기화, AW Shadow는 false, frame/actuator는 잠금 상태입니다. 센서/제어 모듈이 포함됐다는 사실을 실제 센서 취득 또는 모터 구동 완료로 해석하지 않습니다.

## 설정과 단위

`.ioc`와 `spi.c`에서 SPI1 5.25MHz, SPI2 Mode3 2.625MHz를 확인했습니다. STM32CubeMX 6.16.0, FW_F4 V1.28.3 프로젝트입니다. 이는 코드 설정이며 실제 클록 파형 측정값이 아닙니다. UART 진단은 USART6입니다.

모델 경계는 각도 rad, 각속도 rad/s, 토크 N·m, 추력 N입니다. BNO 파서의 Euler deg 표현과 모델 rad 변환은 feedback/frame 경계에서 관리합니다. ICM와 BNO의 host receipt timestamp를 비교하며 SH-2 sensor timestamp와 동일하다고 간주하지 않습니다.

## 잠금과 남은 구현

`LKM_SIL_HOST_ONLY`, `LKM_RP_FRAME_HOST_FIXTURE`, `LKM_FC_ACTUATOR_HOST_FIXTURE`는 host 시험 전용입니다. 두 fixture macro를 STM32 정의와 함께 컴파일할 때 거부되는 검사를 유지합니다. 호스트에서 `virtual_valid=1`이어도 `esc_enabled=0`, `controller_ready=0`입니다.

실제 RC/iBUS, VBAT ADC, GPS, LPS22HH, EEPROM, Telemetry, ESC 타이머 출력과 비행 제어 완성은 별도 후속 작업입니다. R10/R11 하드웨어 핀 설계를 펌웨어 완료로 승격하지 않습니다.
