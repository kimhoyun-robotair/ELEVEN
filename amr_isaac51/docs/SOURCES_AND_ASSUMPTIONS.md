# 출처, 사진 관찰 및 모델 가정

이 자산은 첨부된 8장의 외관 사진을 참고해 치수 제약에 맞춰 만든 **파라미터 기반 근사 재구성**이다. SfM/MVS 사진측량, 계측 스캔, CAD 역설계 또는 내부 기구의 실측 복원을 수행한 결과는 아니다. 제공된 사진에는 바퀴와 하부 기구가 거의 보이지 않으므로 이 부분은 사용자의 구조 설명과 명시적인 설계 가정으로 구성했다.

## 1. 근거 수준

| 항목 | 근거 | 모델에서의 의미 |
|---|---|---|
| 길이 800 mm, 폭 580 mm, 높이 245 mm | 사용자 제공 사양; 별도 실측하지 않음 | SI 단위 변환 후 기본 외형 제약으로 사용 |
| 공차중량 100 kg, 최대 페이로드 250 kg | 사용자 제공 사양 | 기본 로봇 질량 100 kg; 최대 적재 시 합계 350 kg. 적재 가능하다는 구조 강도 인증은 아님 |
| 최대 속도 2 m/s | 사용자 제공 사양 | 명령 및 휠 접선 목표속도 제한. 경사·미끄러짐·충돌 상황의 실제 속도까지 보장하는 기계식 제한은 아님 |
| 차체 중앙 양옆 구동륜 2개, 전후 좌우 수동 캐스터 4개 | 사용자 제공 설명 | 차동 구동 2축과 캐스터마다 자유 스위블/롤 축을 갖는 기구로 모델링 |
| 전후 중심 D455 각 1개 | 사용자 제공 설명 | 전후를 향하는 RGB 및 depth 센서와 D455 크기의 외관 모델 |
| 전방 우측/후방 좌측 2D LiDAR | 사용자 제공 설명 | 해당 두 위치에 수평 스캔 센서 배치; 제조사·모델명은 특정하지 않음 |
| 회색 낮은 외함, 검정 상판, 모서리 형상, 상판의 평행한 장착판 2개 | 사진에서 관찰 | 외관 실루엣과 표면 장식의 근거; 세부 치수는 비율 추정 |
| 빨간 비상정지 버튼, 측면 버튼/키/표시창, 포트 패널, 전후 센서 창 | 사진에서 관찰 | 시각적 형상 참고. 실제 제어 입출력이나 버튼 기능을 사진에서 추정하지 않음 |
| 바퀴 직경·폭·트레드, 캐스터 트레일/스위블 위치, 최저지상고 | 사진/사용자 사양으로 확정 불가 | 구성 파일의 초기값은 설계 추정이며 실측값으로 교체해야 함 |
| 질량 분배·무게중심·관성텐서, 타이어/바닥 마찰, 모터 토크·가속도 | 제공되지 않음 | 시뮬레이션을 위한 초기값. 질량/형상에 따른 양의 관성 추정 및 튜닝값 |
| 카메라 정확한 extrinsic/intrinsic, 렌즈 왜곡, LiDAR 사양 | 제공되지 않음 | 선언된 명목값 및 설치 좌표 사용. 실제 보정 결과가 아님 |

좌표는 전방 +X, 좌측 +Y, 위쪽 +Z를 사용한다. 전후 사진 판별만으로 센서 좌표를 계측했다고 해석하면 안 된다. 센서 위치는 사용자가 지정한 기능 배치를 우선한다.

## 2. 사진별로 확인한 외관

| 첨부 파일 | 관찰한 요소 |
|---|---|
| `1000025951.jpg` | 한쪽 끝면의 중앙 센서 창, 아래 가로로 긴 센서 창, 코너의 빨간 버튼 |
| `1000025952.jpg` | 끝면/측면 연결, 둥근 하부 코너와 모따기 상판, 표시창과 버튼 배치 |
| `1000025953.jpg` | 측면 전체, 제어 버튼/키/표시창, 검정 상판과 장착부 |
| `1000025954.jpg` | 상판 아래 코너의 검정 센서 영역, 측면 제어 패널 |
| `1000025955.jpg` | 반대쪽 끝면, 가로 센서 창과 접점 모양 장식, 상판 장착부 |
| `1000025956.jpg` | 측면 recessed 포트 패널과 빨간 버튼 |
| `1000025957.jpg` | 포트 패널 쪽 측면 전체와 상판 |
| `1000025958.jpg` | 상판의 전체 윤곽, 모따기 코너, 2개의 긴 평행 장착판 |

사진의 작은 원형 개구나 렌즈처럼 보이는 부분을 추가 초음파·거리 센서로 단정하지 않았다. 명시적으로 요청된 두 RGBD와 두 LiDAR 외의 외관 장식은 센서 데이터 소스로 간주하지 않는다.

## 3. 충돌 및 기구 모델의 적용 범위

Visual은 외관용이고 collision은 주행 접촉을 계산하는 단순화 형상이다. 외함의 세부 버튼·나사·포트, 좁은 홈을 모두 동적 삼각형 충돌체로 만들지 않는다. 구동륜과 캐스터 휠은 매끄러운 원통 충돌체를 사용한다. NVIDIA의 5.1 모바일 로봇 튜토리얼도 휠의 메시 요철로 인한 주행 진동을 피하기 위해 원통 충돌체를 설명한다. 각 rigid link는 형제 prim으로 구성하고 joint 관계로 연결한다. [NVIDIA: Rig a Mobile Robot, Isaac Sim 5.1.0](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/robot_setup_tutorials/rig_mobile_robot.html)

접촉 offset·마찰·solver 값은 기하 치수를 대체하는 실측값이 아니다. 너무 작은 contact offset은 접촉 누락/진동을 만들 수 있고, 너무 크면 불필요한 접촉 제약이 늘어난다. 복합 convex/primitive collision은 외관과의 오차 및 계산량을 함께 고려한 선택이다. USD joint limit/drive의 각도 단위는 도(degree)라는 점도 주의한다. [NVIDIA: Physics Simulation Fundamentals, Isaac Sim 5.1.0](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/physics/simulation_fundamentals.html)

캐스터는 지면 접촉에 의해 돌아가는 자유 스위블과 휠 회전으로 근사한다. 실제 서스펜션, 탄성 타이어, 베어링 저항과 캐스터 shimmy를 계측하지 않았다. 6개 휠이 있는 강체 구조의 접지/하중 분배는 특히 바닥 평탄도와 접촉 파라미터에 민감하다. 실물 제어기 튜닝이나 정량적인 제동거리·경사등판·전복 한계 판단에는 추가 계측과 검증이 필요하다.

Python Articulation Controller에 전달하는 관절 각도/각속도는 rad/rad·s⁻¹이며, 직접 작성하는 USD 각도 속성과 혼동하지 않는다. [NVIDIA: Articulation Controller, Isaac Sim 5.1.0](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/robot_simulation/articulation_controller.html)

## 4. D455 및 LiDAR 센서 근사

Intel의 D455 사양에는 본체 124 × 29 × 26 mm, 명목 depth FOV 86 × 57°, 동작 범위 0.6–6 m가 기재되어 있다. [Intel: RealSense Depth Camera D455 specifications](https://www.intel.com/content/www/us/en/products/sku/205847/intel-realsense-depth-camera-d455/specifications.html)

RealSense의 현재 D455 페이지는 depth FOV 87 × 58°, RGB FOV 90 × 65°를 표기한다. 작은 명목 FOV 차이는 실측 보정값을 제공하지 않으며, 실제 FOV는 해상도/종횡비와 모듈 허용오차에도 영향을 받는다. 런타임의 실제 명목 파라미터는 패키지 구성 파일을 기준으로 한다. [RealSense: D455](https://realsenseai.com/products/real-sense-depth-camera-d455f/), [RealSense: D400 Series Datasheet, October 2025](https://realsenseai.com/wp-content/uploads/2025/09/Intel-RealSense-D400-Series-Datasheet-October-2025.pdf)

이 패키지의 카메라는 이상적인 pinhole RGB/depth 영상 생성기다. depth는 렌더러의 기하학적 깊이를 사용하며, 실제 D455의 좌우 IR 영상 매칭, IR projector, stereo baseline에 따른 가림 영역, 재질별 실패, 레이저 speckle, 노출/동기 오차, rolling/USB 지연, 온도 변화 및 RealSense 펌웨어를 재현하지 않는다. 해상도를 낮추면 동일 종횡비의 명목 FOV를 유지하도록 CameraInfo를 일치시켜야 한다. 실제 정합 RGBD의 보정 절차와 동일하다고 해석하면 안 된다.

LiDAR는 지정 모델이 없으므로 구성 파일에 선언된 시야각/해상도/거리의 일반적인 2D 거리 센서다. 실제 광학 소재 응답, 반사 강도, 다중 반사, 회전 중 운동 왜곡과 장치 통신 프로토콜을 보장하지 않는다. 현재 백엔드와 ROS 메시지 동작은 README 및 실행코드의 설명을 따른다.

## 5. 목표 환경과 검증 범위

Isaac Sim 5.1.0 공식 요구사항에 Ubuntu 24.04가 포함된다. 하드웨어 요구사항은 NVIDIA Compatibility Checker와 버전별 공식 표를 확인한다. [NVIDIA: Isaac Sim 5.1.0 Requirements](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/requirements.html)

Ubuntu 24.04에서는 ROS 2 Jazzy가 권장된다. Isaac Sim 5.1은 Python 3.11을 사용하지만 배포판 Jazzy는 Python 3.12를 사용한다. 표준 ROS 메시지는 별도 프로세스에서 DDS로 통신할 수 있다. 따라서 simulator 프로세스에는 Isaac 내부 Jazzy 라이브러리를 사용하고, 외부 ROS 터미널에서만 `/opt/ros/jazzy/setup.bash`를 source한다. [NVIDIA: ROS 2 Installation, Isaac Sim 5.1.0](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_ros.html)

패키지의 정적 검사는 USD 구성·참조·질량·joint 구조와 코드 구문을 검사한다. 이것만으로 RTX 렌더링, PhysX 접지, 실시간률, DDS 발견 또는 RViz 실제 스트림 수신이 확인된 것은 아니다. 실행 환경에서 수행해야 하는 검증과 이 패키지 제작 환경에서 수행한 검증은 README/검증 결과에서 구분한다.

## 6. 실측으로 갱신할 우선 항목

1. 구동륜 반지름/폭/양륜 중심간 거리와 캐스터 위치/반지름/트레일.
2. 지상고 및 하부 간섭 형상, 휠 하중 분배와 서스펜션 유무.
3. 비적재/적재 상태 무게중심과 적재물 위치·크기·관성.
4. 모터 연속/피크 토크, 기어비, 가속/감속 및 정지 특성.
5. 앞뒤 D455 calibration과 base_link 기준 extrinsics.
6. LiDAR 제조사·모델·수평 시야각·최소/최대 거리·스캔 주기와 extrinsics.

공식 문서 확인일: 2026-09-09. 위 링크는 문서/API 근거이며 첨부 사진의 세부 치수가 정확하다는 근거로 사용하지 않는다.
