# Hardware Design — LKM_FC_V1

[회로도](../hardware/LKM_FC_V1/schematic/LKM_FC_V1_Schematic.pdf) · [PCB Layout](../hardware/LKM_FC_V1/pcb-layout/LKM_FC_V1_PCB_Layout.pdf) · [Assembly BOM](../hardware/LKM_FC_V1/bom/LKM_FC_V1_BOM.pdf)

## 하드웨어 설계 목표

36×36 mm 보드에 MCU, IMU·기압센서, 전원 보호, 6채널 PWM 및 외부 통신을 통합했습니다. 신호 연결뿐 아니라 센서 주변 디커플링, 클록 배선, GND 귀환 경로, 전원 기동 및 커넥터·장착홀 간격을 함께 검토했습니다. Altium Designer 26.3에서 회로·라이브러리·PCB를 작성했습니다.

## 주요 부품 선정

| 역할 | 선정 부품 |
|---|---|
| MCU | STM32F405RGT6 |
| Fusion IMU | BNO086 |
| 6-axis IMU | ICM-20602 |
| Barometer | LPS22HHTR |
| EEPROM | AT24C08C-SSHM-T |
| eFuse | TPS259531DSGR |
| 3.3 V LDO | TPS7A9001DSKR |
| 외부 RX 버퍼 | SN74LVC1G17DBVR |
| 외부 신호 ESD | TPD2E2U06DRLR |
| VBAT 측정 분리 | TMUX1511PWR |

부품 표와 회로 설계 완료는 실제 조달 완료 또는 동작 검증을 뜻하지 않습니다. BNO080은 초기 참고 설계의 부품이며 현행 보드는 BNO086을 사용합니다.

## 전원 회로

5 V 입력에 TVS, P-channel MOSFET 역극성 보호 및 TPS259531 eFuse를 적용하고, 보호된 전원에서 TPS7A9001로 3.3 V를 생성합니다. EN/UVLO 분압은 상단 240 kΩ / 하단 100 kΩ로, 공칭 상승 4.08 V / 하강 3.74 V를 설계 계산에 사용했습니다. 이 값은 BEC 출력 완료나 전체 시스템 기동을 보장하는 실측 결과가 아닙니다.

VBAT 측정은 분압·보호·TMUX1511과 ADC 입력 필터/부하를 포함합니다. 실제 ADC 환산은 VDDA, 저항 오차, 스위치 및 전압 측정으로 교정합니다. 부저는 MOSFET 구동과 환류 보호 경로를 포함합니다.

## MCU 및 센서 회로

MCU의 VDD/VDDA/VBAT, 독립 VCAP 커패시터, 8 MHz HSE, NRST, BOOT, SWD 연결을 설계했습니다. 센서별 SPI를 분리하고 EEPROM은 I²C에 연결했습니다. BNO086은 32.768 kHz 크리스털, SPI strap, wake/reset 및 interrupt 신호를 포함합니다.

회로도는 Power / MCU / Sensors / UART / PWM/User I/O의 5개 시트입니다. 시트 간 net 명칭과 핀 연결을 제조 데이터 검토와 연결해 확인했습니다. 센서 ID·통신·전원 시퀀스 및 축 방향은 실물 bring-up 대상입니다.

## PCB Stackup

| 순서 | 레이어 | 설계 용도 |
|---|---|---|
| L1 | Top | 부품, 신호·전원 배선 |
| L2 | Inner 1 | GND polygon 및 일부 배선 |
| L3 | Inner 2 | 전원·신호 배선 및 GND polygon |
| L4 | Bottom | 양면 부품과 신호·전원 배선 |

4-Layer, 두께 1.2 mm, 외층 1 oz / 내층 0.5 oz를 제조 조건으로 정했습니다. 내층을 완전히 끊김 없는 전용 GND plane으로 표현하지 않습니다. 실제 배선 주변 GND 연속성과 리턴 경로를 구간별로 검토했으며, 제작용 적층·공정은 제조사 최종 승인 대상입니다.

## PCB Layout

외형은 36×36 mm, 모서리 R2.5, 장착홀 Ø3.1 mm 4개, 홀 중심 패턴 30.5×30.5 mm입니다. 내·외층 신호 전환, 센서 아래 전원 배선, 클록 주변 배선, RX 버퍼 디커플링 및 부저 환류 루프를 단계적으로 검토·개선했습니다.

PWM 6채널은 18핀 J8 한 개에 통합되어 있습니다. 보드 장착과 커넥터 방향은 실제 케이블·기체 조립에서 추가 확인합니다. 3D View 이미지는 이번 공개 자료에서 제외했으며, PCB 실물 사진도 아직 포함하지 않았습니다.

## 제조 검증 결과와 한계

2026-10-30 R10 및 2026-10-31 R11의 로컬 검토에서 저장 PCB와 Gerber, NC Drill, BOM/CPL을 비교했습니다. R11 기록의 범위는 96 Net / 448개 연결 핀, PTH 322 / NPTH 4, 공장 실장 52 MPN 그룹 / 139개 위치입니다. CAM 연결 비교에서 누락 pad center·분리 net·혼합 net 그룹이 검출되지 않았습니다.

이는 출력 정합성 검사입니다. 실제 회로의 전기 동작, 잡음, 열 성능 또는 비행 안정성 PASS가 아닙니다. R10의 DRC 238건은 실크 관련 230+8건으로 남아 있으며 R11은 동일 저장본의 결과를 승계했습니다. 전체 DRC 0건이라고 표시하지 않습니다.

Bottom 7개 via의 Filled & Capped 처리, 스텐실 미세 개구와 노출 패드 솔더량, 실크 clipping, 커넥터의 1.2 mm 보드 실장 조건 및 제조사 수락이 남아 있습니다. [제조 과정 요약](../hardware/LKM_FC_V1/manufacturing/README.md)과 [검증 구분](validation.md)을 참고하세요.
