# APRL Navigation

Isaac Sim Office **1층 / AMR·locomanipulator / ROS 2 Jazzy**의 Nav2 구성입니다.
Nav2 노드·launch·YAML·PGM·속도 중재기는 이 패키지에 두고, Captain Dalgu는
OccupancyGrid/Path/TF 표시와 초기 위치·NavigateToPose action·취소만 처리합니다.

## 실행

```sh
cd ~/aprl_robot_sim
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup
./scripts/ros build
ROS_DOMAIN_ID=73 ROS_LOCALHOST_ONLY=1 ./scripts/sim --scene office --robot locomanipulator
# 위 터미널에서 AMR_READY를 확인한 뒤 새 터미널:
cd ~/aprl_robot_sim
ROS_DOMAIN_ID=73 ROS_LOCALHOST_ONLY=1 ./scripts/ros nav2
# 새 터미널:
cd ~/captain-dalgu
./scripts/run-aprl-sim.sh --nav2
./scripts/connect-tablet.sh
```

`--headless`를 sim 명령에 추가하면 시뮬레이터 창 없이 실제 PhysX·센서가 실행됩니다.
관제 화면은 태블릿 Chrome의 `http://localhost:8080`입니다. Nav2 모드의 GUI 연결 설정은
[config/dalgu.yaml](config/dalgu.yaml)에 있습니다. 기존 Captain 기본 프로필은 그대로 사용할 수 있습니다.
다른 ROS domain을 쓰면 sim/Nav2의 `ROS_DOMAIN_ID`와 GUI의 `DALGU_SIM_DOMAIN`을 맞추세요.

Captain의 action client는 Humble/Jazzy에서 모두 검사했습니다. 이 시뮬레이터 실행기와
[config/nav2.yaml](config/nav2.yaml)의 Nav2 플러그인 구성은 **Jazzy용**입니다.
Humble 로봇에서는 해당 로봇 워크스페이스의 Humble Nav2 bringup과 GUI를 연결하세요.

## 태블릿 조작

1. 지도가 표시되고 내비게이션이 `연결됨`인지 확인합니다. 기본 시작 위치 `(-11, 5, 0°)`는 AMCL에 설정되어 있습니다.
2. 위치가 어긋나면 **Pose Estimate** → 지도에서 로봇 위치를 누르고 바라보는 방향으로 드래그 → **초기 위치 적용**을 누릅니다.
3. **Nav2 Goal** → 목표 위치와 방향을 같은 방식으로 지정 → **목표 전송**을 누릅니다. X/Y/방향을 숫자로 수정할 수도 있습니다.
4. 파란 계획 경로, 남은 거리·예상 시간과 결과를 확인합니다. **목표 취소** 또는 상단 정지 버튼으로 중단합니다.
5. **수동 전환 · 목표 취소**를 누르면 기존 8방향 teleop 버튼을 사용할 수 있습니다.

선택한 화살표는 전송 전 미리보기입니다. 지도 드래그만으로 로봇이 움직이지 않습니다.
Pose Estimate는 위치 추정 메시지이며 로봇의 PhysX 위치를 옮기지 않습니다.
초기 위치 발행 후 로봇 모델·센서가 지도와 맞는지 확인하세요.
화면 숨김·초점 이탈·연결 해제 시 목표를 취소하며, 다시 연결해도 목표가 재개되지 않습니다.

## ROS 계약과 소유권

| 데이터 | 토픽 / 프레임 |
|---|---|
| PGM 지도 | map_server → `/map` (OccupancyGrid) |
| 전역 / 지역 costmap | `/global_costmap/costmap`, `/local_costmap/costmap` (OccupancyGrid) |
| 계획 경로 | `/plan` (Path), 기존 `/trajectory`와 별도 표시 |
| AMCL 추정 / 초기 위치 | `/amcl_pose` / `/initialpose` (PoseWithCovarianceStamped) |
| 목표 | `/navigate_to_pose` (NavigateToPose action) |
| 속도 | GUI `/cmd_vel_teleop` + Nav2 `/cmd_vel_nav` → command_mux → `/cmd_vel` |
| 자율주행 허용 갱신 | `/navigation/inhibit` (Bool, true = 차단) |
| TF | AMCL `map → sim_world`, 기존 시뮬레이터 `sim_world → base_footprint → base_link` |
| 센서 / odom | `/front_right_lidar/scan`, `/rear_left_lidar/scan`, `/ground_truth/odom` |

AMCL의 base frame은 `base_footprint`입니다. 지도 Z=0과 지면을 맞추고, 기존 base_link 높이·센서 TF를 보존합니다.
현재 시뮬레이터의 world TF와 PhysX odometry를 사용하므로 encoder 드리프트가 있는 실제 로봇의 위치 추정 성능 검증은 아닙니다.
IMU의 기준은 `sim_world`, 화면과 목표의 기준은 `map`입니다.
Nav2는 `use_sim_time: true`, GUI는 `/clock`을 사용합니다. 속도 중재기의 watchdog은 **wall clock**으로 동작합니다.

수동 명령은 Nav2보다 우선합니다. 중재기는 마지막 명령이 0.35초 이내이고,
자율주행의 경우 허용 갱신도 0.4초 이내일 때만 속도를 통과시킵니다. GUI는 활성 목표를 가진
화면의 heartbeat가 1초 이상 끊기면 취소합니다. GUI 서버가 종료되어도 허용 갱신이 만료됩니다.
새 Nav2 목표의 기본 속도는 0.2 m/s, 출력 상한은 0.3 m/s·0.6 rad/s입니다.
다른 `/cmd_vel` 발행자가 직접 명령을 보내면 이 중재를 우회하므로 함께 실행하지 않습니다.
외부 RViz/action client가 시작한 목표는 이 GUI의 허용 갱신이 없어 기본적으로 주행하지 않습니다.

`always_send_full_costmap: true`로 설정했습니다. GUI는 전체 OccupancyGrid를 표시하며
`costmap_updates` 증분 메시지를 병합하지 않습니다. Global은 주황/빨강, local은 보라색,
계획 경로는 파랑, 기존 trajectory는 주황색입니다. 레이어별로 숨길 수 있습니다.
각 LaserScan observation source에 `max_obstacle_height: 2.0`을 명시했습니다.
[Jazzy의 source별 기본 상한](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_costmap_2d/plugins/obstacle_layer.cpp#L139)
0.0을 그대로 쓰면 지면 위 라이다 관측이 모두 제거됩니다.
`expected_update_rate: 0.5`는 관측 갱신을 기다리는 최대 간격(ROS 시간, 초)이며,
센서가 끊긴 costmap을 계속 유효하다고 취급하지 않도록 설정했습니다.

## 지도·설정 교체

[maps/office_floor1.yaml](maps/office_floor1.yaml)은 620×500 PGM, 0.05 m/cell,
origin `[-15.5, -12.5, 0]`입니다. 시뮬레이터 USD 충돌 mesh를 지면 위 0.192 m에서
절단해 만든 정적 지도이며 SLAM으로 수집한 지도는 아닙니다. 벽·기둥·가구 다리와
초기 엘리베이터 문을 반영합니다. 이동 장애물은 두 LaserScan의 costmap으로 반영합니다.
현재 팔 자세를 동적으로 투영하는 footprint는 아니며, 기본 접힌 팔 상태를 전제로 합니다.
다층/엘리베이터 이동, House/Research, Scout에는 별도 지도·센서·footprint 설정이 필요합니다.

```sh
# 지도 YAML이 가리키는 PGM과 origin/resolution을 함께 준비
./scripts/ros nav2 map:=/absolute/path/floor.yaml params_file:=/absolute/path/nav2.yaml

# Office USD를 수정한 뒤 지도를 다시 생성할 때 (Isaac 실행이 끝난 상태에서)
APRL_ISAAC_SCRIPT="$PWD/scripts/export-nav-map.py" ./scripts/sim
./scripts/ros build
```

생성 방법·충돌체 경로·SHA256은 `maps/office_floor1.source.json`에 기록됩니다.
관련 Nav2 계약은 [NavigateToPose API](https://api.nav2.org/jazzy/actions/navigation/)와
[Nav2 플러그인 설정](https://docs.nav2.org/jazzy/configuration_and_development/navigation_plugins/)을 참고하세요.

## 종료와 녹화

목표를 취소해 정지한 뒤 녹화 중이면 **중단하고 저장**을 누릅니다. GUI 서버 → Nav2 →
시뮬레이터 터미널에서 각각 Ctrl+C로 종료합니다. 이후 USB 케이블만 분리해도 됩니다.
연결 설정까지 즉시 지우려면 `adb reverse --remove tcp:8080`을 실행합니다.
GUI 연결 해제는 시뮬레이터나 Nav2 프로세스를 종료하지 않습니다.
녹화는 Captain 실행 디렉터리의 `bags/aprl-nav2/`에 원본 지도·costmap·경로·action 상태/feedback과 센서를 저장합니다.

실행 검증 및 캡처는 `~/captain-dalgu/docs/verification/nav2/`에 있습니다.
호스트 Jazzy에서 실제 Isaac/Nav2를 검증했으며 Dockerfile의 Nav2 의존성도 추가했습니다.
Isaac 전체 Docker 이미지 재빌드는 이번 검증 범위에 포함하지 않았습니다.
