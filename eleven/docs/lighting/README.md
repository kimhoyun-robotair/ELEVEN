# 실내 조명 검증 — 2026-09-08

Isaac Sim 5.1.0, RTX 5070 Ti, headless `RaytracedLighting`, 1000 × 700 렌더링.
Blender에서 내보낸 정규화된 DiskLight의 광량이 실내를 비추기에 부족했다.
같은 Office 카메라에서 Exposure 3, 8, 11을 비교했으며 3에서는 사용자 스크린샷과 같은 어두운 상태를 재현했다.

천장등과 카 다운라이트를 Exposure 11로 설정했다. 기존 3 대비 광량은 256배다.
외부 조명, 카메라 노출, 조명 위치와 재질은 이 수정에서 변경하지 않았다.
물리 조도(lux)를 교정한 값이 아니라 기본 RTX 화면에서 가시성을 확보한 값이다.

- [수정 전 Office](office-before.png)
- [수정 후 Office](office-after.png)
- [수정 후 Office E1 카](office-cabin.png)
- [수정 후 House 로비](house-lobby.png)

Office 비교 카메라: 위치 (-11, -4, 1.65), 목표 (-10, 1, 1.6), 초점거리 20.
로비·카 카메라는 `isaac/runtime.py`의 `view_lobby`·`view_cabin` 위치를 사용했다.
House의 주방 표면과 E1 카도 확인했다. 모든 층·카메라·렌더 모드를 확인한 것은 아니다.

최종 USD를 다시 열어 Office 156개, House 80개의 조명이 Exposure 11인지 확인했다.
그 외 광원 속성은 이전 파일과 같고 준비 함수를 반복 적용해도 결과가 변하지 않는다.
파일을 재생성해도 `tools/prepare_isaac.py`에서 같은 값을 적용한다.

이미 열려 있는 Isaac 장면은 USD를 다시 열거나 Reload해야 새 값이 반영된다.
