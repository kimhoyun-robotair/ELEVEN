# 사진 기반 AMR 모델 · Isaac Sim 5.1.0 · ROS 2 Jazzy

800 × 580 × 245 mm 로봇의 **사진을 참고한 파라미터 기반 재구성 모델**과 테스트 월드, 실행 코드입니다. Ubuntu 24.04 LTS의 Isaac Sim 5.1.0을 대상으로 작성했습니다. 8장 사진에서 보이는 검은 상판, 회색 패널, 모서리 버튼, 카메라 창, 서비스 패널과 상판 장착부를 반영했습니다.

**정밀 사진측량/스캔 복원본은 아닙니다.** 사진에 가려진 휠과 내부 구조, 정확한 센서 외부 파라미터, 부품별 질량·관성·마찰은 추정값입니다. 제공된 치수와 질량 등은 모델에 반영했지만 실제 로봇과 동역학적으로 일치하려면 실측 보정이 필요합니다. USD 구조·치수·질량과 Python 코드는 제작 환경에서 검사했습니다. **Isaac Sim 5.1/RTX/ROS 2 Jazzy를 실제 실행하여 카메라·접지·기초 주행을 검증했습니다.** 재현 조건과 결과는 `verification/runtime_fix_20260909/REPORT_KO.md`에 기록합니다. 결과 범위는 `verification/VALIDATION.md`에 기록했습니다.

## 1. 포함 파일

| 파일 / 폴더 | 용도 |
|---|---|
| `assets/robot.usd` | 독립 로봇 USD: visual, collision, 11 rigid bodies, 10 revolute joints, 카메라와 센서 좌표계 |
| `assets/robot.usda` | 동일 로봇의 명시적인 ASCII 확장자 사본 |
| `assets/test_world.usd` | 빈 차체 + 바닥 + 벽 + 컬러 장애물 + 조명 + 관찰 카메라 |
| `assets/test_world_payload250.usd` | 중앙 고정 payload 250 kg, 총질량 350 kg 월드 |
| `config/robot.json` | 휠, 제어, 센서, 물성, payload 설정 |
| `model/build_assets.py` | 외부 메시/텍스처 없이 USD를 다시 만드는 코드 |
| `sim/run_sim.py`, `sim/control.py` | 물리 주행, RGBD/2D LiDAR 계산, ROS 2 연결, 속도 제한과 timeout |
| `ros2_ws/src/amr_tools/` | teleop, 스트림 모니터, 라이브 검사, RViz 설정 |
| `scripts/` | 실행·의존성·재생성 도우미 |
| `tests/`, `verification/` | 오프라인 검증 코드와 실제 검증 기록 |
| `docs/` | 모델 미리보기, 가정·공식 출처, 토픽/프레임 설명 |
| `reference/` | 사용자가 제공한 원본 사진 8장 |

USD는 ASCII 형식으로 저장하여 최신 바이너리 crate 포맷 의존성을 피했습니다. 월드→로봇 참조는 상대경로이므로 폴더 구조를 유지하세요. 사용자 로봇/월드 geometry는 원격 Isaac Asset 서버에 의존하지 않습니다. Isaac Sim 및 NVIDIA 드라이버, ROS 2 런타임 자체는 압축파일에 포함되지 않습니다.

## 2. 준비

1. Ubuntu **24.04 LTS x86_64**에서 Isaac Sim **5.1.0 워크스테이션 배포판**을 설치합니다. 이 패키지의 실행기는 설치 폴더 안의 `python.sh`를 사용합니다.
2. RTX 지원 GPU/드라이버는 [NVIDIA 5.1 시스템 요구사항](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/requirements.html)을 확인하세요. 카메라를 사용하므로 `--headless`도 RTX 렌더링이 필요합니다.
3. [ROS 2 Jazzy Ubuntu 설치 문서](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)에 따라 `ros-jazzy-desktop`을 설치합니다.
4. 압축을 풀고 다음 명령을 프로젝트 폴더에서 실행합니다.

```bash
cd amr_isaac51
chmod +x scripts/*.sh
./scripts/install_ros_deps.sh --install
./scripts/run_ros_tools.sh build
```

의존성 스크립트는 Jazzy apt 저장소가 이미 구성되어 있다고 가정합니다. 인자 없이 실행하면 설치할 명령을 보여줍니다. ROS 도구에는 사용자 정의 메시지가 없어서 별도의 Python 3.11 ROS workspace 빌드가 필요하지 않습니다.

**터미널을 분리하세요.** Isaac Sim 5.1은 Python 3.11, Ubuntu Jazzy는 Python 3.12입니다. Isaac Sim 터미널에서는 `/opt/ros/jazzy/setup.bash`나 외부 ROS workspace를 source하지 않습니다. 실행기는 Isaac Sim에 포함된 Jazzy 라이브러리를 사용합니다. 외부 ROS 도구 실행기는 시스템 Jazzy를 source합니다. 이는 [Isaac Sim 5.1 공식 ROS 설치 지침](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_ros.html)에 따른 구성입니다.

## 3. 가장 빠른 실행

**터미널 A — ROS를 source하지 않은 새 터미널**

```bash
cd amr_isaac51
export ISAAC_SIM_PATH="$HOME/isaacsim"   # 실제 5.1.0 설치 경로로 변경
export ROS_DOMAIN_ID=0
env -u PYTHONPATH -u PYTHONHOME \
    -u LD_LIBRARY_PATH -u AMENT_PREFIX_PATH \
    -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH \
    -u ROS_DISTRO -u ROS_VERSION -u ROS_PYTHON_VERSION \
    -u BASH_ENV \
    PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
    ./scripts/run_sim.sh
```

`monitor`를 먼저 켤 필요는 없습니다. 처음에는 shader/extension 초기화로 시간이 걸릴 수 있습니다. `AMR_READY`가 출력되면 RGB, depth, LiDAR 유효 데이터와 차체의 안정적인 접지가 확인되고 `/cmd_vel` 입력이 활성화됩니다. GUI에서 로봇 외형과 테스트 월드를 볼 수 있습니다. `/cmd_vel`이 없으면 0 속도 목표를 유지합니다.

**터미널 B — ROS 모니터**

```bash
cd amr_isaac51
export ROS_DOMAIN_ID=0
./scripts/run_ros_tools.sh monitor
```

**터미널 C — 키보드 조작**

```bash
cd amr_isaac51
export ROS_DOMAIN_ID=0
./scripts/run_ros_tools.sh teleop
```

`w/s`: 전후진, `a/d`: 제자리 회전, `u/o`: 전진 회전, `Space/x`: 정지, `q`: 종료. 기본 0.25 m/s로 시작합니다. 키 입력이 끊기면 teleop가 0을 보내고, 시뮬레이터도 독립적으로 0.5초 명령 timeout을 적용합니다. 정지는 설정된 감속과 구동 토크를 따라 이루어집니다.

**터미널 D — 센서 시각화**

```bash
cd amr_isaac51
export ROS_DOMAIN_ID=0
./scripts/run_ros_tools.sh rviz
```

RViz에는 앞뒤 RGB/depth 이미지, LiDAR 두 개, TF, odometry 표시가 구성되어 있습니다. 로봇의 상세 3D 외형은 Isaac Sim에서 봅니다. 자세한 토픽/QoS/단위와 수동 CLI 명령은 `docs/ROS_TOPICS.md`에 있습니다.

## 4. 적재와 자동 검증

ROS가 자동 source되는 터미널에서는 아래 `run_sim.sh` 명령에도 3장의 `env -u ...` 접두 명령을 붙이세요.

```bash
# 적재된 로봇: 중앙에 고정한 250 kg payload
./scripts/run_sim.sh --payload-kg 250

# 창 없이 센서 렌더링, 60 simulation seconds 뒤 종료 및 결과 저장
./scripts/run_sim.sh --headless --duration 60 --report logs/runtime_empty.json
```

`--duration`은 wall time이 아니라 simulation time입니다. 실제 실행 시간이 60초보다 길 수 있습니다. 종료 전에 결과 JSON에 startup 통과 여부, 카메라/스캔 프레임 수, 관절 이름, 바퀴 최저 높이, 차체 기울기, 받은 명령과 오류가 기록됩니다. `--physics-trace`를 붙이면 0.5초 간격의 자세 기록도 저장합니다. `--caster-yaw-deg 90` 등으로 시작 시 caster 선회 각도를 지정해 재현 검사를 할 수 있습니다. 초기 센서 유효 데이터가 없으면 성공으로 처리하지 않습니다.

시뮬레이터를 실행한 상태에서 다른 터미널에서:

```bash
# 토픽/TF/이미지와 scan 유효성 검사
./scripts/run_ros_tools.sh smoke --report logs/smoke_streams.json

# 제공된 월드에서 전진 명령과 정지 응답까지 검사
# 검증 중에는 teleop 등 다른 cmd_vel publisher를 종료하세요.
./scripts/run_ros_tools.sh smoke --drive --report logs/smoke_drive.json

# 후진과 좌/우 제자리 회전까지 검사
./scripts/run_ros_tools.sh smoke --maneuvers --report logs/smoke_maneuvers.json
```

smoke의 성공 종료 코드는 0입니다. 두 검사를 빈 차체와 `--payload-kg 250`에서 각각 수행하면 됩니다. 이는 통신·기초 주행 점검이며 2 m/s 최고속도 달성, 250 kg 운반 성능, 제동거리나 전복 한계를 입증하는 시험은 아닙니다.

## 5. 모델과 물리 정의

| 항목 | 구현 |
|---|---|
| 전체 길이 × 폭 × 높이 | 0.800 × 0.580 × 0.245 m, 빈 차체 초기 자세 기준 |
| 빈 차체 질량 | 100 kg = chassis 82 + drive wheels 10 + caster assemblies 8 |
| 적재 | 0~250 kg; 중앙 고정 상자 0.50 × 0.38 × 0.20 m를 기본 가정 |
| 최대 속도 | 차체 명령 및 각 휠 원주속도 상한 2 m/s; 반지름 0.10 m에서 20 rad/s |
| 구동휠 | 중앙 x=0, y=±0.24 m; 반지름 0.10 m; Y축 속도 구동 2개 |
| 캐스터 | pivot x=±0.30, y=±0.22 m; 반지름 0.04 m, trail 0.02 m |
| 캐스터 관절 | 각 Z축 자유 선회 + Y축 자유 회전; 모터/속도 drive 없음 |
| 접촉 | 원통 휠, caster 포크/베어링, 분할 차체 collider; 평면 접지면; TGS 32/1 iterations, 120 Hz |
| visual / collision | 외형 장식과 충돌 geometry를 분리; 작은 구멍·렌즈·버튼은 충돌에 단순화 |

차체 기준 +X가 전방, +Y가 좌측, +Z가 위입니다. 초기 `base_link` 원점은 지면에서 0.15 m 높이에 있습니다. 무게중심과 대각 관성은 추정된 부품 형상에서 계산합니다. Payload는 동일 rigid body에 고정한 것으로 합성 질량·무게중심·관성을 평행축 정리로 갱신합니다. 차체 collision은 휠과 caster의 전체 회전 공간을 비워 두며, 포크/베어링도 외부 collision을 갖습니다. 조립된 링크 사이의 자기충돌은 비활성화했습니다. 지면·외부 장애물과의 충돌은 활성화되어 있습니다. 물리용 바닥 평면은 시각적 바닥(12×10 m) 밖으로도 이어지며, 제공된 벽 안쪽을 검증 영역으로 사용합니다.

전체 envelope 외형은 이 사진 로봇의 고정 템플릿입니다. `dimensions`만 바꿔 다른 크기의 로봇을 만드는 기능은 제공하지 않으며, 생성기가 이를 검사합니다. 구동/캐스터 치수는 설정에서 바꾼 뒤 자산을 재생성할 수 있습니다. 휠을 크게 바꾸면 외형 간섭 및 접지 높이를 다시 검토하세요. 물성값은 실측값으로 바꿀 수 있습니다.

## 6. 센서 정의

장착 좌표는 로봇 초기 지면 원점 기준이며 실측값이 아닙니다.

| 센서 | x, y, z (m) | yaw | 출력 |
|---|---|---|---|
| 앞 RGBD | +0.400, 0, 0.172 | 0° | RGB + optical-Z depth + CameraInfo |
| 뒤 RGBD | −0.400, 0, 0.172 | 180° | 동일 |
| 앞 우측 LiDAR | +0.377, −0.267, 0.192 | −45° | 270° / 541 rays / 10 Hz LaserScan |
| 뒤 좌측 LiDAR | −0.377, +0.267, 0.192 | 135° | 동일 |

카메라 하우징은 D455 치수 124 × 29 × 26 mm를 반영한 단순화 형상입니다. 기본 센서는 640 × 360, 15 Hz, 수평 87°의 **이상적 정렬 RGBD**로, 실제 D455 RGB와 stereo depth의 서로 다른 intrinsics나 baseline을 재현하지 않습니다. Depth는 0.4~6 m를 유효 범위로 설정하며 나머지는 NaN입니다. 주어진 사진으로 D455 공장 보정값을 복원할 수 없으므로 실제 센서 calibration으로 바꿔야 정합도가 높아집니다.

LiDAR는 모델명이 없어 2D 이상적 raycast로 구현했습니다. PhysX collision scene에서 거리값을 계산하며 자기 로봇은 필터링합니다. 각 센서의 270° 시야는 대각선 바깥쪽을 향합니다. 범위 0.05~20 m, 반사 미검출은 +inf, 최소범위 이하는 NaN입니다. intensity/반사율·회전 중 motion distortion·드롭아웃·노이즈는 모델링하지 않았습니다. 작은 visual-only 장식은 LiDAR 충돌체에 포함되지 않습니다.

`assets/robot.usd`를 GUI에서 단독으로 열면 모델과 물리를 확인할 수 있습니다. ROS publisher/subscriber와 LiDAR 계산은 Python 런타임이 생성하므로 **전체 기능에는 `scripts/run_sim.sh`를 사용하세요.** `realsense2_camera` 드라이버를 따로 실행할 필요가 없습니다.

## 7. 수정과 재생성

```bash
# ROS 자동 source 설정이 있어도 이번 실행만 분리합니다.
isaac_clean() {
  env -u PYTHONPATH -u PYTHONHOME -u LD_LIBRARY_PATH \
      -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH \
      -u ROS_DISTRO -u ROS_VERSION -u ROS_PYTHON_VERSION -u BASH_ENV \
      PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin "$@"
}
export ISAAC_SIM_PATH="$HOME/isaacsim"
isaac_clean ./scripts/build_assets.sh

# USD 구조와 치수, 질량, 관절, 광학축 검사
isaac_clean "$ISAAC_SIM_PATH/python.sh" scripts/run_usd_tool.py tests/validate_usd.py

# 제어기·카메라 프레임 형식·침하 검사 (카메라 테스트에는 numpy 필요)
python3 -m unittest discover -s tests -p 'test_*.py' -v
```

재생성/검증도 위의 ROS 환경 분리 명령을 적용한 터미널에서 실행합니다. `run_usd_tool.py`는 Isaac 앱을 초기화해 포함된 USD 플러그인을 등록합니다. 별도의 `usd-core`가 설치된 Python에서는 `python3 tests/validate_usd.py`도 가능합니다.

USD 재생성은 `assets/`의 생성 파일을 덮어씁니다. 해당 USD를 직접 편집했다면 먼저 별도 사본을 보관하세요. 재현용 설정 해시는 `verification/`의 기록을 참고하고, 설정 변경 후 자산도 같이 갱신하세요. 렌더 해상도/센서 빈도/물성 변경의 적용 범위는 설정·생성 코드에서 확인할 수 있습니다.

## 8. 문제 해결

| 현상 | 확인 |
|---|---|
| `rclpy` / Python ABI 오류 | Isaac 터미널에 system ROS가 source되었는지 확인. 새 터미널에서 run_sim 사용 |
| `python.sh not found` | `ISAAC_SIM_PATH`가 5.1.0 설치 폴더를 가리키는지 확인 |
| 토픽 없음 | `AMR_READY`, 양쪽 ROS_DOMAIN_ID, Fast DDS, 방화벽/DDS 발견 상태 확인 |
| 이미지가 늦거나 Hz가 낮음 | RTX 초기화 완료 대기, GPU VRAM/렌더 부하 확인. Hz는 wall time과 sim time을 구분 |
| startup 센서 실패 | `logs/runtime_report.json`과 Isaac 로그 확인. 제공 월드에서 검증 후 사용자 월드 적용 |
| 물리 흔들림·미끄러짐 | `physics_checks`와 `physics_latest`의 침하량/기울기 확인. 제공 월드는 큰 박스 대신 평면 collider로 접지합니다. 초기 2초 후 2 mm 초과 침하 또는 5° 초과 기울기가 0.5초 지속되면 오류 종료합니다. |
| GUI Stop/Reset 후 제어 이상 | 이 standalone 세션을 종료하고 실행기를 다시 시작. Pause/Resume만 지원 |
| 다른 ROS navigation stack 연결 | 표준 Twist/odom/TF/scan 입력에 연결 가능. map, localization, Nav2 launch는 이 기초 테스트 패키지 범위 밖 |

정밀도를 높이는 데 필요한 값: 구동/캐스터 휠 직경·폭·접지 위치, 캐스터 trail, drive suspension 유무·스프링/감쇠, 실제 무게중심과 관성, 모터 토크 곡선·기어비, D455 calibration, LiDAR 모델/FOV/주기 및 센서 6DoF 장착 좌표. 원본 사진과 현재 추정값 대응은 `docs/SOURCES_AND_ASSUMPTIONS.md`에 정리했습니다.
