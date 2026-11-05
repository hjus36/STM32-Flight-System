# 통합 Host SIL

원본 `LKM_FC_Integrated_SIL_Visualization_V1`의 테스트/실행기를 선별했습니다. 상대경로, 테스트 fixture CSV 위치, 그래프 한계 표기만 공개 구조에 맞췄습니다. C 테스트와 13개 실제 source의 expected SHA gate는 유지합니다.

[실행 방법](../../docs/reproduction.md) · [결과/제한](../../docs/validation.md) · [원본 실제 실행](results/sil_v1_20261106_040855/)

`results`는 원본 Windows 실행 증거입니다. 과거 배포 패키지의 `examples/` 데이터를 대신 쓰지 않았습니다. `evidence`의 6단계 로그는 경로 개인정보만 치환한 과거 실행 기록입니다. 새 실행 결과는 `.local`에 분리됩니다. 모든 데이터는 합성이며 `sensor-data/`에 넣지 않습니다.
