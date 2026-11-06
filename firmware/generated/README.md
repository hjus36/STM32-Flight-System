# MATLAB 생성 코드

최종 사용 코드는 [Core/Src](../stm32/LKM_BNO086_Bringup/Core/Src/)와 [Core/Inc](../stm32/LKM_BNO086_Bringup/Core/Inc/)에 한 번만 보관합니다. 별도의 중복 generated C 디렉터리는 만들지 않았습니다.

| 모델 | C 파일 | 원본 생성 정보 |
|---|---|---|
| LKM_AW_Core_F405 | LKM_AW_Core_F405.c/.h | model v1.5, 2026-11-05 16:45:34 |
| LKM_AW_Ctrl_F405 | LKM_AW_Ctrl_F405.c/.h | model v1.26, 2026-11-05 16:35:37 |
| LKM_AW_Mixer_F405 | LKM_AW_Mixer_F405.c/.h | 모델/생성 버전은 원본 header 유지 |

Simulink Coder 26.2 (R2026b), ert.tlc, ARM Cortex-M, float/single, Ts=10ms. `const_params.c`, `rt_nonfinite.*`, `rtwtypes.h`, `complex_types.h`와 private/types 헤더는 모델 데이터·비유한수 처리·타입 정의를 함께 제공합니다. 타깃 자원 설정 헤더까지 모든 빌드에 필요하다는 뜻은 아닙니다. 범용 MathWorks 라이브러리 전체를 배포하지 않습니다.

Academic License와 원본 copyright를 유지합니다. `Validation result: Not run`은 생성 시 메타데이터로 보존했고 별도 수행한 host/ARM 검증은 [검증 문서](../../docs/validation.md)에 기록합니다. [모델·단위·변환](../../docs/control-models.md) · [무결성 manifest](../../docs/source-manifest.json).

생성 도구 사용과 결과물 공개 권한은 별개입니다. 학생 포트폴리오에서 편집 가능한 생성 C/H와 지원 파일을 함께 제공하는 것은 **추가 확인 필요**입니다. [파일별 헤더 조사와 MathWorks §3.2 적용 범위](../../docs/source-and-license.md)를 참고하세요.
