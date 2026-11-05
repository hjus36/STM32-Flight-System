# Firmware

실제 설치·검증 경로에서 선별한 최소 프로젝트는 [stm32/LKM_BNO086_Bringup](stm32/LKM_BNO086_Bringup/)입니다. 기존 C/H 로직과 생산 출력 잠금을 유지했습니다. `MW_target_hardware_resources.h`의 사용되지 않는 로컬 CubeMX 경로 macro만 공개용 이름으로 바꿨습니다.

- [구조와 구현 범위](../docs/firmware-architecture.md)
- [빌드·재현](../docs/reproduction.md)
- [의존성과 출처](../docs/source-and-license.md)
- [생성 코드 대응](generated/README.md)
- [사용자 드라이버 안내](drivers/README.md)

HAL/CMSIS는 FW_F4 V1.28.3 설치본을 별도 공급합니다. `.ioc`를 이용한 재생성은 **작업 사본에서만** 수행하고 생성된 Core를 검토 없이 덮어쓰지 않습니다. 현재 저장소는 실행 가능한 전체 비행제어 제품이 아니며 실제 MCU/센서/ESC 검증은 아직 없습니다.
