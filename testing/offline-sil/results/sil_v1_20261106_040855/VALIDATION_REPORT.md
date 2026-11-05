# LKM_FC_V1 통합 SIL·Fail-safe 검증 보고서

> **HOST SYNTHETIC SIL ONLY. 실제 비행·MCU 실행·센서 동기화·모터 시험 결과가 아닙니다.**

- 컴파일러: `MATLAB R2026b MinGW-w64 GCC (local executable path omitted)`
- 실제 기존 생성 코드 및 안전 모듈: `13`개 C 원본 직접 컴파일
- C 내부 검증: **26860/26860 PASS**
- 100Hz 합성 정상 시나리오: 1,000개 virtual thrust 프레임
- Fail-safe 합성 시나리오: 14개
- 생산용 잠금 테스트 및 호스트 매크로 ARM 컴파일 차단: PASS
- UART/ESC/PWM/모터 하드웨어 호출: 없음
- 시간 정합: 호스트 수신 시각 비교일 뿐 물리적 측정시각 동기화가 아님
- 기체 모의 플랜트 J=0.025 kg·m², D=0.11 N·m·s/rad: **임의의 합성 가정**, 실제 QAV210 파라미터 아님

## 시나리오 결과

| 시나리오 | 가상 출력 프레임 | 첫 Fail-safe (ms) | 원인 | 종료 상태 |
|---|---:|---:|---|---:|
| `baseline` | 1000 | — | — | 2 |
| `receiver_loss` | 118 | 1210 | RECEIVER | 3 |
| `receiver_recovery` | 205 | 1210 | RECEIVER | 2 |
| `battery_low` | 118 | 1210 | BATTERY | 3 |
| `imu_stale` | 118 | 1210 | SENSOR_OR_AXES | 3 |
| `timestamp_skew` | 118 | 1210 | SENSOR_OR_AXES | 3 |
| `duplicate_sample` | 118 | 1210 | SENSOR_OR_AXES | 3 |
| `scheduler_overrun` | 118 | 1210 | SCHEDULER | 3 |
| `axis_unverified` | 118 | 1210 | SENSOR_OR_AXES | 3 |
| `low_bno_accuracy` | 118 | 1210 | SENSOR_OR_AXES | 3 |
| `nan_target` | 118 | 1210 | CONTROLLER | 3 |
| `nan_mixer` | 118 | 1210 | COMMAND | 3 |
| `emergency_stop` | 118 | 1210 | ESTOP | 3 |
| `receiver_heartbeat_stale` | 118 | 1210 | HEARTBEAT | 3 |
| `battery_heartbeat_stale` | 118 | 1210 | HEARTBEAT | 3 |

## 그래프

그래프는 C 프로그램의 `trace_all_scenarios.csv`를 읽어 자동 생성했습니다.

- `01_attitude_synthetic.png`
- `02_virtual_mixer_thrust.png`
- `03_failsafe_recovery_states.png`
- `04_failsafe_output_zero.png`
- `05_host_timestamp_skew.png`
- `06_fault_injection_matrix.png`

## 핵심 한계
- Simulink Academic License 생성 코드의 배포 조건은 사용자가 직접 확인해야 합니다.
- `virtual_valid`는 실제 모터 아밍 권한이 아닙니다. `esc_enabled` 및 `controller_ready`는 전 구간 0입니다.
- 센서 바이트, 타임스탬프, RC, 배터리, 타이밍, 모의 플랜트 모두 합성 입력입니다.
- MCU의 ISR 지터·SPI 소요시간·실제 장착축·실제 ESC·비행 안정성은 검증되지 않았습니다.
- 실물 시험은 별도 안전 절차로 진행해야 하며 이 자료만으로 비행 승인할 수 없습니다.
