# 출처·도구 의존성·공개 범위

이 저장소 전체에 새 LICENSE를 부여하지 않았습니다. 소스의 기존 고지와 아래 사용 조건을 삭제하거나 공개 도메인으로 재해석하지 않습니다.

| 분류 | 출처 / 처리 |
|---|---|
| 모델·lkm_*·시험 코드 | 사용자 LKM 개발 workspace의 활성 모델 및 실제 설치 source. 선별 manifest로 원본 상대경로/해시를 추적 |
| STM32 초기화·startup·linker | CubeMX/CubeIDE 생성·ST template, 저작권/AS-IS 고지 유지 |
| HAL/CMSIS | FW_F4 V1.28.3 의존성만 명시; 전체 source는 미포함 |
| 생성 C / 최소 runtime 지원 | 사용자 모델에서 Simulink Coder R2026b 생성; Academic License 고지 유지 |
| 강의/GitHub SDK | 기존 MH-FC 강의 firmware, CEVA 전체 SDK, 외부 라이브러리 무단 복사 없음 |
| SIL CSV·그래프 PNG | 실제 Windows host synthetic run; 원본 CSV를 읽는 plotter로 재생성, 실측 아님 |

## MathWorks 생성 소스의 공개 범위 (2026-11-06 검토)

**판정: 학생 포트폴리오로 편집 가능한 생성 C/H와 지원 파일을 공개하는 것은 추가 확인 필요.** GitHub에서 파일이 보인다는 사실, 생성기의 Academic 고지, 비상업적 의도만으로 제3자의 자유로운 복사·재배포 권한이 생기지는 않습니다.

직접 확인한 기준은 R2026b 설치본 `matlabroot/license_agreement.txt`의 Program Offering Guide(Part One §§2.2–2.3, Part Two 서문·§3·§3.2)와 Glossary의 *Academic Internal Operations*, *Application*입니다. [MathWorks Support의 약관 위치 안내](https://www.mathworks.com/matlabcentral/answers/1979884-where-can-i-find-the-mathworks-software-license-agreement) · [공개된 이전 판의 §3.2](https://www.mathworks.com/help/releases/r2023b/pdf_doc/offering/offering.pdf) (문구 대조용) · [Embedded Coder 배포 절차](https://www.mathworks.com/help/ecoder/ug/deploy-component-model.html) (기술 절차). 권리 판단에는 사용 당시의 실제 계약·교육기관 약정이 우선합니다.

| 무엇을 배포하는가 | 확인된 조건과 이 저장소에서의 판단 |
|---|---|
| 사용자 모델을 변환한 C/H (Generated Forms) | §3.2는 Coder로 생성한 결과를 도구 밖에서 사용하기 위해 복사·배포할 수 있게 합니다. Part Two 서문에 따라 사용한 라이선스의 목적 제한도 적용됩니다. 개인 포트폴리오가 해당 학생의 허용된 수업·학업 용도인지는 계약 확인이 필요합니다. |
| MathWorks 소유 지원 파일을 포함한 편집 가능 소스 | §3.2는 해당 파일을 소스 또는 링크된 목적 코드로 제공할 때, 더 큰 독립 응용프로그램에 **필요한 범위에서 그 일부로** 사용하는 조건을 둡니다. 파일이 생성 폴더에 있거나 컴파일된다는 사실만으로 이 조건을 충족하지는 않습니다. 개별 파일을 내려받을 수 있는 현재 공개 구성의 적합성은 미확정입니다. |
| STM32 실행 바이너리 | 링크된 지원 코드도 같은 조건을 따릅니다. Glossary의 Application은 도구 밖에서 독립 실행되며 작성자의 고유 코드와 실질적 기능을 포함해야 합니다. 호스트 SIL·ARM 컴파일 기록은 바이너리 배포 승인이나 실기 실행 증거가 아닙니다. |
| 상업·정부·기타 조직 용도 | 확인된 생성 헤더는 해당 용도를 제한합니다. §3의 추가 로열티 면제는 **이미 허용된 배포**에 한합니다. 이 사본에 상업적 재사용을 허가하지 않으며, 별도 라이선스·재생성 필요 여부는 MathWorks에 확인해야 합니다. |

Part One §2.2와 Glossary는 기관용 라이선스에서 재학생의 수업·학업 요건을 다룹니다. §2.3은 **별도 Student License**의 사용자·용도 제한입니다. 학생 신분이나 생성 헤더만으로 어느 계약을 사용했는지 판별할 수 없습니다. 소스에는 Simulink Coder 26.2 및 ERT 타깃이 기록되어 있지만 이는 라이선스 증명서가 아닙니다.

**파일별 검토.** 모델 C 3개와 모델별 API/private/types 헤더, `const_params.c`, `rt_nonfinite.c/.h`, `complex_types.h` 등 16개 C/H에서 Academic 고지를 확인했습니다. 검토한 18개 C/H에서 별도 Copyright 문구는 발견되지 않았으며, 이 사실이 권리 부재를 뜻하지 않습니다. `rtwtypes.h`에는 호환용 생성 타입이라는 설명이 있고, `MW_target_hardware_resources.h`에는 조건부 타깃 설정·include가 있습니다. 이 두 파일에는 같은 Academic 머리말이 없습니다. 후자가 현재 빌드에 필수라고 단정하지 않습니다. 헤더 유무만으로 모델 출력·독점 지원 파일·자유 재배포 파일을 구분할 수 없습니다.

**확인할 질문.** 실제 사용한 기관/개인 라이선스로 이 학습 결과를 공개 저장소에 상시 제공할 수 있는가? 이 소스 구성과 독립 실행 대상이 §3.2 조건을 충족하는가? 파일별 필수 지원 범위, 유지할 고지 및 수신자 이용 조건은 무엇인가? 소스 공개와 컴파일된 STM32 바이너리에 답이 다른가? 확인 전에는 재사용 라이선스를 붙이거나 배포 범위를 넓히지 않습니다. 권한이 확인되지 않으면 직접 작성한 설명·인터페이스·시험 결과와 적법한 도구에서의 재생성 절차 위주로 제공하고, 원본 모델도 별도로 권리를 확인하는 방법이 있습니다. 이번 검토에서 파일을 철회하거나 MathWorks에 문의하지 않았습니다.

ST source의 개별 고지는 그대로 유지합니다. 설치한 FW_F4 V1.28.3 `Package_license.md`의 SBOM은 **CMSIS Device = Apache-2.0**, **HAL = BSD-3-Clause**로 구분합니다. system/startup은 CMSIS Device 출처이며 HAL 전체는 별도 의존성으로 남깁니다. CubeMX 생성 파일의 ST 고지 및 적용 package 조건도 보존합니다.

동반 조건은 [third-party notices](third-party-notices.md)에 수록했습니다. 이는 기존 third-party 조건을 전달하는 문서이며 저장소 전체의 새 LICENSE가 아닙니다. [ST CMSIS Device 공식 고지](https://github.com/STMicroelectronics/cmsis-device-f4/blob/master/LICENSE.md)와 [ST HAL 공식 고지](https://github.com/STMicroelectronics/stm32f4xx-hal-driver/blob/master/LICENSE.md)를 교차 확인했습니다.

출처 또는 공개 권한이 불명확한 추가 third-party 파일은 이번 공개 선별에 포함하지 않았습니다. 설치기·검증 로그의 로컬 사용자 경로는 공개 사본에서만 치환하며 원본 고지를 제거하지 않습니다.
