# 출처·도구 의존성·공개 범위

이 저장소 전체에 새 LICENSE를 부여하지 않았습니다. 소스의 기존 고지와 아래 사용 조건을 삭제하거나 공개 도메인으로 재해석하지 않습니다.

| 분류 | 출처 / 처리 |
|---|---|
| 모델·lkm_*·시험 코드 | 사용자 LKM 개발 workspace의 활성 모델 및 실제 설치 source. 선별 manifest로 원본 상대경로/해시를 추적 |
| STM32 초기화·startup·linker | CubeMX/CubeIDE 생성·ST template, 저작권/AS-IS 고지 유지 |
| HAL/CMSIS | FW_F4 V1.28.3 의존성만 명시; 전체 source는 미포함 |
| 생성 C / 최소 runtime 지원 | 사용자 모델에서 Simulink Coder R2026b 생성; Academic License 고지 유지 |
| 강의/GitHub SDK | 기존 MH-FC 강의 firmware, CEVA 전체 SDK, 외부 라이브러리 무단 복사 없음 |
| CSV·PNG | 실제 Windows host synthetic run; 원본 CSV를 읽는 plotter로 재생성, 실측 아님 |

MathWorks 설치본 R2026b `license_agreement.txt`, Program Offering Guide Part Two §3.2(Coder Programs)는 Generated Forms의 외부 사용과, 생성 코드가 포함된 standalone application에 필요한 지원 소스의 배포 범위를 설명합니다. 이에 따라 이 프로젝트의 생성 C와 빌드에 필요한 지원 파일만 함께 보존하고 MathWorks 도구/범용 라이브러리는 포함하지 않았습니다. [공식 License Center 안내](https://www.mathworks.com/company/aboutus/policies_statements.html)를 통해 적용 약관을 확인할 수 있습니다.

생성 파일의 Academic License는 교육·학술 연구·학위 기관 수업 요건 용도 제한을 명시합니다. 학생 학습 포트폴리오로 정리했으며 상업/정부/다른 조직 용도까지 허용한다고 주장하지 않습니다. 원본 라이선스 범위를 벗어난 사용은 별도 확인 대상입니다.

ST source의 개별 고지는 그대로 유지합니다. 설치한 FW_F4 V1.28.3 `Package_license.md`의 SBOM은 **CMSIS Device = Apache-2.0**, **HAL = BSD-3-Clause**로 구분합니다. system/startup은 CMSIS Device 출처이며 HAL 전체는 별도 의존성으로 남깁니다. CubeMX 생성 파일의 ST 고지 및 적용 package 조건도 보존합니다.

동반 조건은 [third-party notices](third-party-notices.md)에 수록했습니다. 이는 기존 third-party 조건을 전달하는 문서이며 저장소 전체의 새 LICENSE가 아닙니다. [ST CMSIS Device 공식 고지](https://github.com/STMicroelectronics/cmsis-device-f4/blob/master/LICENSE.md)와 [ST HAL 공식 고지](https://github.com/STMicroelectronics/stm32f4xx-hal-driver/blob/master/LICENSE.md)를 교차 확인했습니다.

출처 또는 공개 권한이 불명확한 추가 third-party 파일은 이번 공개 선별에 포함하지 않았습니다. 설치기·검증 로그의 로컬 사용자 경로는 공개 사본에서만 치환하며 원본 고지를 제거하지 않습니다.
