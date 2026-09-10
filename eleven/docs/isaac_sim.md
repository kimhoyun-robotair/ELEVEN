# Isaac Sim 5.1.0에서 환경 열기

이 저장소의 `scenes/office.usda`, `scenes/house.usda`는 미터 단위, Z-up인 OpenUSD 월드입니다. Blender 원본과 내보낸 메시를 유지하면서 PhysX 충돌 및 엘리베이터 제어용 메타데이터를 추가합니다. 경로가 상대 참조이므로 `scenes/`만 따로 복사하지 말고 저장소 전체를 유지하세요.

## 실행

Isaac Sim **5.1.0** 설치 폴더의 Python 실행기를 사용합니다. 일반 시스템 Python으로는 Kit/RTX 런타임이 시작되지 않습니다.

```bash
cd /path/to/testbed_for_sim
/path/to/isaac-sim-5.1.0/python.sh isaac/run.py --scene office
/path/to/isaac-sim-5.1.0/python.sh isaac/run.py --scene house
```

Windows에서는 동일한 명령에 `python.bat`을 사용합니다. 실행기는 `SimulationApp`을 먼저 만든 뒤 Isaac/Omniverse 모듈을 불러오고, USD를 연 뒤 물리 초기화를 수행합니다. 이는 [Isaac Sim 5.1 Python 환경 문서](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/python_scripting/manual_standalone_python.html)의 실행 순서를 따릅니다.

실시간으로 이동할 때는 기본 `RaytracedLighting`을 사용합니다. 정지 상태의 재질·반사·유리를 더 자세히 확인하려면 다음과 같이 Path Tracing으로 시작하고 타임라인을 일시 정지합니다.

```bash
/path/to/isaac-sim-5.1.0/python.sh isaac/run.py --scene office --renderer PathTracing
```

렌더러와 샘플 수 설정은 [5.1 SimulationApp API](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/py/source/extensions/isaacsim.simulation_app/docs/index.html)를 따릅니다. 브라우저의 실시간 PBR 결과와 Isaac RTX 결과는 같은 메시를 사용하지만 조명·유리 표현은 렌더러에 따라 달라집니다.

## 조작

- **Multi-floor testbed** 창에서 각 엘리베이터의 목적층, 열림, 닫힘, 알람, 층별 호출 버튼을 누릅니다. UI의 층 번호는 **1부터**, Python API와 USD의 인덱스는 **0부터** 시작합니다.
- 뷰포트에서 모델의 버튼 또는 그 버튼의 자식 메시를 선택해도 해당 버튼을 누른 것으로 처리합니다. 같은 버튼을 반복해서 누를 때는 다른 곳을 선택했다가 다시 선택하거나 제어 창을 사용합니다. Stage 패널에서 버튼을 선택해도 동일합니다. 편집할 때는 **Viewport selection presses modeled buttons**를 끕니다.
- **Ride / inspect cabin**은 카 내부에 연결된 관찰 카메라로 전환합니다. **View lobby**는 선택한 층의 승강장으로 카메라를 옮깁니다. 이 카메라는 시각적 탐색용이며 동적 로봇을 대체하지 않습니다.
- 목적층 버튼은 해당 요청이 남아 있을 때만 발광합니다. 층별 호출의 위/아래 버튼도 각각 요청 상태를 가집니다. 문 열림 완료로 서비스된 요청은 꺼집니다. 열림/닫힘은 실제 입력 후 0.8초, 알람은 실제 입력 후 1.5초 동안만 켜집니다. 알람은 로컬 표시 기능입니다.
- 승강장 문은 카가 해당 층에 정지하고 정렬되었을 때만 카 문과 함께 열립니다. 이동 중 문 열기 요청은 거절됩니다. 동작 큐, 가속·감속, 문 열기/닫기, 대기는 `simulation/elevator.py`에서 처리합니다.
- 출입구의 PhysX 충돌 질의가 장애물을 감지하면 문을 다시 열고 유지합니다. **Hold door beam occupied**는 같은 경로로 입력되는 수동 테스트 신호입니다. 센서 범위는 문 중앙 기준 폭 1.28 m, 깊이 0.50 m, 높이 1.90 m이며 바닥에서 0.15–2.05 m입니다. 충돌 메시가 없는 물체는 검출되지 않습니다.
- Pause는 물리 시간과 제어기를 멈추고, Stop은 카와 요청을 초기화합니다. 런타임 편집은 기본적으로 USD 세션 레이어에 작성됩니다.

## 기존 Isaac GUI에서 연결

1. Isaac Sim에서 `File > Open`으로 `scenes/office.usda` 또는 `scenes/house.usda`를 엽니다. 이 상태에서 메시·재질·조명과 충돌 설정을 확인할 수 있습니다.
2. Script Editor에서 아래를 실행합니다. 경로를 실제 저장소 경로로 바꿉니다.
3. 타임라인의 Play를 누릅니다. 단순히 USD만 열면 Python 엘리베이터 제어기는 실행되지 않습니다.

```python
import sys
sys.path.insert(0, "/path/to/testbed_for_sim")
from isaac.runtime import attach
testbed = attach()

# 선택 사항: API로 1번 엘리베이터의 3층 요청 (0-based index 2)
testbed.press("E1", "floor", 2)
```

스크립트를 다시 실행하면 이전 런타임 구독과 제어 창을 해제하고 재연결합니다. 다른 월드를 연 경우에도 새 월드에서 `attach()`를 다시 실행합니다.

## 로봇 제어와 물리 탑승

카 바닥과 문은 `UsdPhysics.RigidBodyAPI`의 `kinematicEnabled = true`인 충돌체입니다. 제어기는 PhysX **pre-step**에 카/문 목표 변환을 적용하므로 동적 로봇은 카 바닥과의 접촉으로 운반됩니다. 로봇을 카의 자식 prim으로 재부모화하거나 매 프레임 로봇 높이를 강제로 덮어쓰지 않습니다. 로봇은 `/World/Robot`처럼 카 밖의 경로에 배치하고 정상적인 동적 강체/아티큘레이션, 충돌 메시와 접촉 마찰을 설정하세요.

로봇의 접촉·레이캐스트·UI가 버튼 prim 경로를 확인했을 때 다음 API에 전달합니다. 모든 접촉을 자동으로 버튼 입력으로 해석하지는 않습니다.

```python
testbed.dispatch_press("/World/Elevators/E1/Cabin/Panel/Floor_2")
testbed.press("E1", "hall", 1, "up")
state = testbed.scene.rigs["E1"].controller.snapshot()
print(state["position"], state["doorOpen"], state["cabinLights"])
```

동적 승객 큐브를 **모든 카**에 올린 뒤 최상층까지 운반하고, 실제 큐브 높이 오차 6 cm 이하·요청 서비스 완료·버튼 소등을 검증하는 명령입니다. 실패하면 0이 아닌 종료 코드를 반환합니다.

```bash
/path/to/isaac-sim-5.1.0/python.sh isaac/run.py --scene office --headless --verify-ride --state-output /tmp/office-state.json
/path/to/isaac-sim-5.1.0/python.sh isaac/run.py --scene house --headless --verify-ride --state-output /tmp/house-state.json
```

일반 배치 실행에서 목적층은 `--request E1:3 --request E2:2`처럼 지정할 수 있습니다. `--steps 1800`은 물리 시간 30초이며 렌더링 시간과 다릅니다.

## USD 인터페이스

| 경로 또는 속성 | 의미 |
|---|---|
| `/World/Elevators/E1` | 엘리베이터 기준 프레임. 건물 내 X/Y 위치를 가짐 |
| `testbed:floorHeights` | 로컬 Z의 층 바닥 높이 배열, 미터 |
| `testbed:doorTravel` | 각 문짝의 X 방향 이동 거리, 0.72 m |
| `testbed:label` | 사람이 읽는 엘리베이터 이름 |
| `Cabin` | 카의 시각적 루트. 바닥 표면이 로컬 Z=0 |
| `Cabin/Collision` | 카 바닥의 독립된 kinematic 강체/충돌 메시 |
| `Cabin/Doors/Left`, `Right` | 카 문짝. 각각 독립된 kinematic 강체 |
| `LandingDoors/Floor_N/Left`, `Right` | 층별 문짝. N은 0-based |
| `xformOp:translate:testbedMotion` | 런타임의 가산 이동. 원본 메시 변환을 보존 |
| 버튼의 `testbed:button` | `floor`, `hall`, `open`, `close`, `alarm` |
| 버튼의 `testbed:floor`, `testbed:direction` | 목적/호출층 인덱스와 `up` 또는 `down` |
| 버튼의 `testbed:lightMaterial` 관계 | 버튼마다 다른 `UsdPreviewSurface` 재질을 가리킴 |
| `testbed:currentFloor`, `testbed:state` | 런타임이 기록하는 현재 층/동작 상태 |
| 숫자 메시의 `testbed:displayFloor` | 표시할 층 인덱스. 현재 도달한 층의 숫자만 표시 |

Office는 4층, E1–E4이며 층 높이는 `[0, 3.6, 7.2, 10.8]` m입니다. House는 3층, E1–E2이며 `[0, 3.4, 6.8]` m입니다. 문이 향하는 방향은 엘리베이터 기준 프레임의 −Y이고 카 외곽은 2.4 × 2.4 m입니다. 강체 중첩을 피하기 위해 `Cabin` 자체에는 RigidBodyAPI를 붙이지 않습니다. Kinematic 바디와 계층 규칙은 [OpenUSD 물리 설계 문서](https://openusd.org/release/wp_rigid_body_physics.html#kinematic-bodies)에 설명되어 있습니다.

## 검증 범위와 문제 진단

초기 구현 당시에는 Isaac 런타임 검증을 수행하지 않았습니다. 2026-09-08에는 Isaac Sim 5.1.0 / RTX 5070 Ti에서 headless `RaytracedLighting`으로 Office·House의 실내, 로비, E1 카 내부를 렌더링해 조명을 확인했습니다. [조명 비교 및 검증 기록](lighting/README.md)을 참고하세요. GUI 조작·실제 PhysX 탑승 검증은 이번 조명 확인에 포함하지 않았습니다. Python 구문, 공통 엘리베이터 상태 머신, 생성된 USD 구조/재질/충돌 검증과 실제 Isaac 런타임 검증은 별개입니다. 위의 `--verify-ride` 명령이 대상 워크스테이션에서 수행할 물리 검증 절차입니다.

- `No module named isaacsim/omni`: Isaac 설치의 `python.sh`/`python.bat`인지 확인합니다.
- `Invalid moving collision bodies`: 누락된 prim 경로가 오류에 표시됩니다. 저장소의 준비된 `scenes/*.usda`를 열었는지 확인합니다.
- 문이 계속 열림: 물체가 출입구 충돌 영역에 있거나 수동 beam 입력이 켜졌는지 확인합니다. 질의 자체가 실패하면 오류를 기록하고 문을 열린 상태로 유지합니다.
- 큐브/로봇이 떨어짐: Physics Debug에서 카 바닥 충돌을 확인하고 `--verify-ride`를 실행합니다. 자신이 추가한 로봇의 충돌체와 단위도 확인합니다.
- UI 버튼을 누르는데 움직이지 않음: 타임라인이 Play인지 확인합니다. `File > Open`만으로는 제어기가 시작되지 않습니다.

PhysX pre-step 구독은 [Omni Physics Python API](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/107.0/extensions/runtime/source/omni.physx/docs/api/python.html)를, 출입구 overlap 질의는 [Isaac Sim 5.1 객체 기반 SDG 예제](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/replicator_tutorials/tutorial_replicator_object_based_sdg.html)를 참고합니다. 버튼 선택 이벤트는 [NVIDIA의 USD 선택 이벤트 예제](https://docs.omniverse.nvidia.com/workflows/latest/extensions/object_info.html)의 공식 이벤트 스트림을 사용합니다.
