# 검증 기록

2026-09-09에 Isaac Sim 5.1.0을 직접 실행하여 caster 침하와 카메라 수집 문제를 수정하고 검증했다. 환경, 재현, 변경 내역, 수치와 이미지 증거는 [실행 보고서](runtime_fix_20260909/REPORT_KO.md)에 기록했다.

| 검사 | 결과 | 증거 |
|---|---|---|
| 제어·카메라 API·침하 검사 단위 테스트 | 17/17 통과 | `runtime_fix_20260909/unit_tests.log` |
| Isaac 포함 USD 구조 검사 | 103/103 통과 | `runtime_fix_20260909/usd_validation_isaac.json` |
| 빈 차체·caster 90° | 65초, 센서/물리 검사 통과 | `runtime_fix_20260909/final_empty_yaw90.json` |
| 250 kg 적재·caster 270° GUI | 85초, 센서/물리 검사 통과 | `runtime_fix_20260909/final_payload_yaw270.json` |
| 외부 ROS 15개 토픽·TF·전진/후진/좌우 회전/정지 | 빈 차체·적재 모두 통과 | `runtime_fix_20260909/smoke_*yaw*.json` |
| 기존 월드의 침하 검출 | 오류 보고서 저장, 종료 코드 1 | `runtime_fix_20260909/physics_guard_negative.json` |
| 준비 시간 초과 검출 | 오류 보고서 저장, 종료 코드 1 | `runtime_fix_20260909/final_startup_negative.json` |

기존 `usd_validation.json`, `control_tests.txt`, `static_checks.json`은 수정 이전 오프라인 검증 기록이다. 최신 실행/검증 결과는 위 경로를 사용한다. `manifest_sha256.json`과 실행 보고서 폴더의 `final_source_sha256.json`은 현재 소스를 식별한다.

현재 로봇은 11 rigid bodies, 10 revolute joints, 25 colliders를 사용한다. 외관·질량/관성·마찰은 사진과 추정값 기반이며 실측 교정 대상이다. 제공 평면 월드 밖의 지형, 최대속도·제동거리·실물 성능과 바퀴별 지지력 분포는 이번 검증 범위에 포함되지 않는다.
