# ROS 2 연결 · 조작 · 데이터 확인

대상은 Ubuntu 24.04의 ROS 2 Jazzy입니다. 외부 ROS 노드는 시스템 Python 3.12로 실행합니다. Isaac Sim 프로세스와 이 ROS 터미널은 별도 실행 환경입니다. 외부 ROS 작업공간의 `install/setup.bash`를 Isaac Sim 터미널에 source하지 마세요. Jazzy가 Ubuntu Noble 24.04를 대상으로 한다는 점은 [공식 설치 문서](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)를 기준으로 합니다.

## 설치와 실행

먼저 위 공식 문서로 `ros-jazzy-desktop`을 설치합니다. 프로젝트 최상위 폴더에서:

```bash
# 설치할 의존성 명령을 표시합니다.
./scripts/install_ros_deps.sh
# 실제 설치가 필요할 때만 실행합니다.
./scripts/install_ros_deps.sh --install

export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
./scripts/run_ros_tools.sh build
./scripts/run_ros_tools.sh monitor
```

별도 터미널에서 동일한 `ROS_DOMAIN_ID`로 시뮬레이터를 실행합니다. DDS 통신을 사용하므로 두 프로세스의 domain ID가 같아야 합니다. `run_ros_tools.sh`는 기본값으로 domain 0과 Fast DDS를 사용하고, 사용자가 설정한 값은 유지합니다. Conda나 virtualenv가 활성화되어 있으면 안내 후 중단합니다.

```bash
# 각각 별도 시스템 터미널
./scripts/run_ros_tools.sh teleop
./scripts/run_ros_tools.sh rviz
```

RViz에는 TF, 두 LiDAR, odometry, 앞뒤 RGB와 depth 표시가 저장되어 있습니다. 차체의 USD 렌더링은 Isaac Sim 창에서 확인합니다. RGB와 depth 이미지는 RViz의 해당 이미지 패널에서 볼 수 있습니다. Depth는 자동 정규화하여 표시합니다.

## 토픽 계약

`/cmd_vel`을 제외한 토픽은 시뮬레이터가 발행합니다. 시뮬레이션 시간, 실제 위치에서 계산한 odometry 및 센서 프레임이 서로 같은 시간축을 사용합니다. 모든 길이 단위는 m, 속도는 m/s, 각도는 rad입니다.

| 토픽 | 메시지 | 의미 / frame_id |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | `linear.x`, `angular.z`; base 축 기준 |
| `/clock` | `rosgraph_msgs/msg/Clock` | 시뮬레이션 시간 |
| `/odom` | `nav_msgs/msg/Odometry` | `odom` → `base_footprint`; PhysX ground truth |
| `/joint_states` | `sensor_msgs/msg/JointState` | 구동 2축 및 캐스터 수동 관절 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 움직이는 프레임 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 고정 센서 외부 파라미터 |
| `/front_camera/color/image_raw` | `sensor_msgs/msg/Image` | `rgb8`; `front_camera_optical_frame` |
| `/front_camera/depth/image_raw` | `sensor_msgs/msg/Image` | `32FC1`, m; `front_camera_optical_frame` |
| `/front_camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | 앞쪽 컬러 pinhole calibration |
| `/front_camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | 앞쪽 depth calibration |
| `/rear_camera/color/image_raw` | `sensor_msgs/msg/Image` | `rgb8`; `rear_camera_optical_frame` |
| `/rear_camera/depth/image_raw` | `sensor_msgs/msg/Image` | `32FC1`, m; `rear_camera_optical_frame` |
| `/rear_camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | 뒤쪽 컬러 pinhole calibration |
| `/rear_camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | 뒤쪽 depth calibration |
| `/front_right_lidar/scan` | `sensor_msgs/msg/LaserScan` | `front_right_lidar_link` |
| `/rear_left_lidar/scan` | `sensor_msgs/msg/LaserScan` | `rear_left_lidar_link` |

카메라는 컬러와 depth가 같은 광학 프레임으로 정렬된 합성 RGBD입니다. D455의 스테레오 매칭 오차, IR projector, USB 장치 열거, 펌웨어 및 `realsense2_camera` driver를 에뮬레이션하지는 않습니다. `CameraInfo`는 시뮬레이션에 설정한 해상도와 pinhole intrinsics를 제공합니다. `32FC1`의 유효 depth는 광학축 방향 거리(m)입니다. 실제 D455 calibration과 동일하다고 해석하지 마세요.

`LaserScan`은 2D 수평 거리이며 각도 0은 LiDAR frame의 +X, 양의 각도는 +Z축 주위 반시계 방향입니다. 측정되지 않은 ray는 `inf`입니다. 앞오른쪽과 뒤왼쪽의 위치 및 방위는 모델 설정과 TF를 따릅니다.

구동 관절 이름은 `left_drive_joint`, `right_drive_joint`입니다. 캐스터 관절은 `caster_{fl,fr,rl,rr}_{swivel,roll}_joint` 형식의 8개입니다. 캐스터는 ROS에서 직접 명령하지 않습니다.

## 프레임과 QoS

| 부모 | 자식 | 의미 |
|---|---|---|
| `odom` | `base_footprint` | 지면에 투영한 이동 위치와 yaw |
| `base_footprint` | `base_link` | 차체 실제 높이 및 roll/pitch |
| `base_link` | `front_camera_link` | 앞 카메라 장착 위치 |
| `front_camera_link` | `front_camera_optical_frame` | +Z 전방, +X 오른쪽, +Y 아래 |
| `base_link` | `rear_camera_link` | 뒤 카메라 장착 위치/뒤쪽 방위 |
| `rear_camera_link` | `rear_camera_optical_frame` | 뒤 카메라 광학축 |
| `base_link` | `front_right_lidar_link` | 앞오른쪽 LiDAR |
| `base_link` | `rear_left_lidar_link` | 뒤왼쪽 LiDAR |

차체 축은 +X 전방, +Y 왼쪽, +Z 위입니다. `/cmd_vel`은 reliable / volatile이며, `/tf_static` 구독은 reliable / transient-local입니다. 모니터와 RViz의 센서 구독은 best-effort / volatile을 사용하므로 센서 publisher가 reliable 또는 best-effort인 경우 모두 수신할 수 있습니다. ROS의 [QoS 호환성 및 sensor-data 설명](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Quality-of-Service-Settings.html)을 참고하세요.

## 키보드 제어

`w/s` 전후진, `a/d` 제자리 회전, `u/o` 전진하면서 회전, `Space/x` 정지, `q` 종료입니다. 기본 직진 0.25 m/s, 회전 0.5 rad/s입니다. `+/-`로 직진 속도, `[/]`로 회전 속도를 변경합니다. 키 반복 이벤트가 끊기면 기본 0.25초 wall time 후 0 명령을 발행합니다. 운영체제의 초기 키 반복 대기시간 동안 잠깐 멈출 수 있습니다. 터미널에 포커스를 둔 상태로 사용하세요.

```bash
./scripts/run_ros_tools.sh teleop --speed 0.15 --turn 0.4
```

시뮬레이터는 독립적인 명령 timeout을 적용합니다. 2 m/s는 모델의 명령 상한이며 payload 250 kg에서 그 속도를 달성함을 검증한 값은 아닙니다.

## 실행 후 검증

이 패키지의 라이브 검증은 실행 중인 Isaac Sim + ROS 2 시스템에서 수행해야 합니다. 제작 환경에서 GPU/Isaac Sim 실행 검증을 수행했다는 의미가 아닙니다.

```bash
# 구동 명령을 발행하지 않는 스트림 검사
./scripts/run_ros_tools.sh smoke --timeout 90 --observe 5 --report logs/smoke_streams.json

# 제공된 Test world에서 전방이 빈 상태일 때 명시적으로 선택
# 0.15 m/s × 2 simulation seconds, 이후 0 명령과 정지 응답 확인
./scripts/run_ros_tools.sh smoke --drive --report logs/smoke_drive.json
```

성공 exit code는 0, 검증 실패는 1, 잘못된 명령행은 2, Ctrl-C 중단은 130입니다. 검사는 15개 토픽 수신, 계속 증가하는 시간 stamp, TF 필수 연결, 이미지 stride/배열 길이/encoding, CameraInfo 크기와 focal length, LiDAR 범위와 유한 반사값, 관절 정보, odometry pose를 확인합니다. 선택적 구동 검사는 넓은 허용 범위의 전진 변위와 정지 응답을 검사합니다. 이는 성능·제동거리·안전 인증 시험이 아닙니다.

모니터의 Hz는 wall-clock 수신 빈도입니다. GPU 부하 때문에 시뮬레이션이 느리면 설정한 simulation Hz보다 낮아질 수 있습니다. 화면이 멈추거나 타임라인이 pause 상태이면 smoke는 시간 초과로 실패합니다. 시뮬레이션 reset 후에는 모니터/검사를 재시작합니다. 검증 중 다른 `/cmd_vel` publisher를 동시에 실행하지 마세요.

```bash
# 수동 CLI 확인: 먼저 source /opt/ros/jazzy/setup.bash
ros2 topic list -t
ros2 topic echo /odom --once --qos-reliability best_effort
ros2 topic echo /front_right_lidar/scan --once --qos-reliability best_effort
ros2 topic hz /front_camera/color/image_raw
ros2 topic info /front_camera/depth/image_raw --verbose
```
