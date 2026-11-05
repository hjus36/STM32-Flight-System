# 2026-11-06 공개 복사본 재실행

실제 MATLAB R2026b 실행에서 생성한 수치 요약입니다. Synthetic simulation / Not Hardware or Flight Validation.
원본 workspace의 과거 보고서는 상위 폴더에 보존했습니다. 이 폴더의 4개 CSV는 공개 폴더 모델로 이번에 다시 계산한 것입니다.

- Roll/Pitch 각각 실행한 double PID 요약은 byte-identical이므로 `roll_double_pid_summary.csv` 한 사본만 보존합니다.
- 이번 Normal/SIL 보고서도 [기존 보고서](../normal_sil_comparison_report.txt)와 byte-identical이므로 중복 복사하지 않습니다. 새 실행의 종료코드와 시각은 별도 로컬 기록에 보존했습니다.
- 모든 명령·종료코드·경고·범위는 [재현 결과](../../../docs/reproduction-results.md)를 따릅니다.
- CSV의 NaN은 단일 P 응답이 목표 추종 기준에 도달하지 못해 rise/settling time을 정의하지 못한 결과입니다. PASS 값으로 치환하지 않았습니다.
