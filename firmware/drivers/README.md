# 사용자 센서·진단 모듈

중복 소스 복사를 피하기 위해 실제 코드는 [Core/Src](../stm32/LKM_BNO086_Bringup/Core/Src/)와 [Core/Inc](../stm32/LKM_BNO086_Bringup/Core/Inc/) 한 곳에 둡니다.

- `lkm_bno086*`: SPI2, SHTP, 부분 SH-2 stream/attitude, startup 및 UART 진단
- `lkm_icm20602*`: SPI1 gyro core, HAL adapter, board binding
- `lkm_dual_imu_diag`, `lkm_imu_time_pair`: 진단과 host receipt timestamp 비교
- `lkm_fc100hz*`, `lkm_rp_*`, `lkm_aw_*`, `lkm_fc_actuator*`: 제어 입력·주기·frame·안전 경계

기존 강의 코드 또는 CEVA SH-2 SDK를 별도 복사하지 않았습니다. 이 프로젝트의 부분 SH-2 구현을 CEVA 전체 공식 드라이버라고 표현하지 않습니다. [상태·제한](../../docs/firmware-architecture.md).
