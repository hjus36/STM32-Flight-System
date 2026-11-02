# Manufacturing — 설계 출력 및 로컬 검토

[하드웨어 설명](../../../docs/hardware-design.md) · [Assembly BOM PDF](../bom/LKM_FC_V1_BOM.pdf) · [검증 범위](../../../docs/validation.md)

## 진행 단계

회로·PCB 설계 후 Altium Designer 26.3에서 Gerber X2, PTH/NPTH NC Drill, Pick & Place를 출력하고 승인된 BOM과 대조했습니다. 2026-10-30 R10, 2026-10-31 R11 제조 데이터에 대해 저장본·출력·부품배치의 정합성을 검토했습니다. R11은 현재 내부 제조 출력 세트이며 이 공개 폴더에 제조 ZIP 자체를 배포하지 않습니다.

2026-11-02 정리 기준 PCB 제조·SMT는 Global Sourcing 부품 입고 및 제조사 수락·공정 승인 대기입니다. 이번 포트폴리오 작성에서 재고나 제조사 주문 상태를 새로 조회하지 않았습니다.

## 검토 과정

1. PCB·회로도 저장본의 hash와 검증 대상 revision을 고정했습니다.
2. native Gerber/Drill/Pick & Place를 생성하고 출력 전후 설계 내용의 동일성을 확인했습니다.
3. 구리·드릴·외형·paste 및 BOM/CPL의 Designator·좌표·회전·Bottom 방향을 대조했습니다.
4. ZIP 내부 파일과 SHA-256을 검증하고, 공정 승인·실물 시험이 남은 항목을 분리했습니다.

R11 기록: 96 Net, 연결 핀 448개, 공장 Assembly BOM 52 MPN 그룹 / 139개 실장 위치, Top 70 / Bottom 69. 이 숫자는 조립 범위와 파일 검토의 결과이며 생산 완료 수량이 아닙니다.

J1·J4·J5·J8은 수동 납땜 대상이고 Molex J2·J3는 공장 실장에 포함합니다. J6는 직접 배선용 패드, TP1은 테스트 포인트, R58은 DNP이므로 공장 실장 목록에서 제외했습니다. BOM 139개와 전체 회로도 부품 수를 혼동하지 않습니다.

## 남은 공정 확인

4층·1.2 mm 적층, Bottom 7개 via의 Epoxy Filled & Capped, 미세 패드와 U8/U9 노출 패드의 paste 조건, 실크 clipping, 커넥터 실장 및 부품 모델 방향은 제조사 최종 승인 범위입니다. 0.10 mm 스텐실은 분석에 사용한 조건이며 공장 승인값으로 표시하지 않습니다.

기존 DRC의 실크 관련 238건을 전체 0건으로 기록하지 않습니다. 로컬 CAM 결과는 제조사 수락·실물 전기 동작·EMI·비행 성능에 대한 보증이 아닙니다.

## 공개 자료 관리

여기에는 과정 설명과 공개용 PDF만 포함합니다. Altium 원본, 전체 Symbol/Footprint Library, Gerber/NC Drill ZIP, BOM/CPL 원본, 내부 Stage 로그, 구매 내역·계정 정보·MCP 설정은 기존 비공개 프로젝트에 유지합니다. 실제 발주는 별도 승인된 제조 패키지를 사용합니다.
