# ROS 2 Jazzy interfaces

`../../scripts/ros` uses the system Python 3.12 and Jazzy. The Isaac runtime uses
its own Python 3.11 bridge. Default DDS domain: **73**, RMW: **Fast DDS**.

```sh
./scripts/ros build
./scripts/ros teleop
./scripts/ros arm_teleop       # locomanipulator only, in a separate terminal
./scripts/ros rviz
./scripts/ros verify --robot locomanipulator --report .runtime/streams.json
./scripts/ros rviz --robot scout
./scripts/ros verify --robot scout --report .runtime/scout-streams.json
```

Run these commands from the repository root. Installed equivalents are
`ros2 run aprl_robot_sim teleop`, `arm_teleop`, `route`, and `verify` after sourcing
`install/setup.zsh` or `install/setup.bash`. Set the same `ROS_DOMAIN_ID` for both
processes. `use_sim_time:=true` should be used by downstream ROS nodes.

## Topics

| Topic | ROS message | Meaning / frame |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Differential drive: linear.x, angular.z |
| `/arm/joint_command` | `sensor_msgs/msg/JointState` | Position targets: nero_joint1..7 (rad), gripper (0..0.1 m aperture) |
| `/arm/press_button` | `std_msgs/msg/String` | Contact-confirmed button operation; JSON below |
| `/arm/state` | `std_msgs/msg/String` | Phase, error, request_id, measured tip position and physical contact events |
| `/clock` | `rosgraph_msgs/msg/Clock` | Simulation time |
| `/joint_states` | `sensor_msgs/msg/JointState` | Actual 10 AMR, 19 combined, or 4 Scout DOFs |
| `/robot_description` | `std_msgs/msg/String` | Complete URDF for the selected `amr`, `locomanipulator`, or `scout`, including sensors |
| `/odom` | `nav_msgs/msg/Odometry` | Wheel encoder integration, wheel_odom → wheel_base |
| `/ground_truth/odom` | `nav_msgs/msg/Odometry` | Actual PhysX pose, sim_world → base_link |
| `/mlx/pointcloud` | `sensor_msgs/msg/PointCloud2` | RTX FLASH; flash_lidar_link; x/y/z/intensity float32, t uint32 ns (0) |
| `/imu/data` | `sensor_msgs/msg/Imu` | PhysX specific force including gravity, angular velocity; imu_link |
| `/{front,rear,wrist}_camera/{color,depth}/image_raw` | `sensor_msgs/msg/Image` | rgb8 / 32FC1 optical Z depth in meters |
| `/{front,rear,wrist}_camera/{color,depth}/camera_info` | `sensor_msgs/msg/CameraInfo` | Actual render intrinsics |
| `/{front,rear,wrist}_camera/depth/points` | `sensor_msgs/msg/PointCloud2` | Organized XYZ + packed RGB; corresponding camera_optical_frame |
| `/{front_right,rear_left}_lidar/scan` | `sensor_msgs/msg/LaserScan` | 270° horizontal PhysX raycasts |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | Physical joint poses and fixed sensor extrinsics |
| `/elevator/command` | `std_msgs/msg/String` | Elevator requests; JSON below |
| `/elevator/state` | `std_msgs/msg/String` | ready, scene, robot, sim_time, elevator states |

Wrist and arm topics exist only for `locomanipulator`. Sensor QoS is best effort /
volatile. `/tf_static` and `/robot_description` are reliable / transient local (depth 1). Commands and state use reliable
QoS. Camera point clouds share the corresponding image timestamp and optical
frame (+Z forward, +X right, +Y down). Every second pixel is included in each axis;
invalid depth stays NaN. The packed `rgb` field uses PCL's FLOAT32 bit layout.
An empty nearby scene can validly produce only NaNs for the wrist camera.
RGB rendering uses 0.01–20 m clipping independently of the valid depth range.

TF: `sim_world → base_footprint → base_link`, then the physical wheel, riser,
NERO and sensor frames. `nero_gripper_base → wrist_camera_link →
wrist_camera_optical_frame` follows the moving wrist. There is no second publisher
of arm TF; starting robot_state_publisher with the standalone NERO URDF is unnecessary.
The virtual massless URDF `gripper` link is represented by the aperture command;
the two actual finger joints follow the URDF's ±0.5 mimic ratios.

The simulator publishes the selected URDF once and retains it for late subscribers.
RViz's Robot model reads `/robot_description` and the existing simulator TF; another
`robot_state_publisher` is unnecessary. Run `./scripts/ros build` before RViz so its
`package://aprl_robot_sim/meshes/` and `package://nero_description/meshes/` resources resolve.
The AMR description has 20 links; the combined description has 35. Both include
the 13° upward FLASH mounting and ROS optical frames. Generated files are in `urdf/`;
the NERO visual/collision meshes reuse the supplied description package.

Scout reuses `scout_twin_description/urdf/scout_twin.urdf` and its
`package://scout_twin_description/meshes/` resources. Its URDF name remains
`scout_mini_photo_twin`; the CLI selection and elevator state use `scout`.
The simulator owns its retained description and full TF, so a second state publisher
is unnecessary. The imported launch defaults to viewing the simulator; standalone
preview requires `publish_state:=true preview_joints:=true use_sim_time:=false fixed_frame:=base_link`.

Scout replaces the AMR's FLASH, 2D scans and camera topics with:

| Topic | ROS message | Meaning / frame |
|---|---|---|
| `/mid360/points` | `sensor_msgs/msg/PointCloud2` | RTX rotary approximation; mid360_link; x/y/z/intensity float32; acquisition-end sim stamp |
| `/mid360/imu` | `sensor_msgs/msg/Imu` | Native 200 Hz PhysX IMU; imu_link, mounted within mid360_link |
| `/camera_{front,left,right}/{color,depth}/image_raw` | `sensor_msgs/msg/Image` | 640×400 / 15 Hz rgb8 or 32FC1, camera_{side}_{stream}_optical_frame |
| `/camera_{front,left,right}/{color,depth}/camera_info` | `sensor_msgs/msg/CameraInfo` | Separate RGB 94° and depth 90° horizontal-FOV intrinsics |

The common `/cmd_vel`, clock, odometry, TF, description and elevator topics above
also apply to Scout. Encoder odometry averages the two wheels on each side; skid
slip can differ from `/ground_truth/odom`. No aligned color/depth points are
published for Scout's different camera intrinsics. `verify --robot scout` checks
the MID-360 layout, four joints, complete TF and each image/intrinsics pair.
All scripts respect `ROS_DOMAIN_ID` (default 73).

```sh
./scripts/ros topic echo /robot_description --qos-durability transient_local --once
```

## Controls

Base teleop: **w/s** forward/back, **a/d** rotate, **u/o** forward turn,
**Space/x** stop, **q** exit. **+/-** sets translation speed; **[/]** sets turn rate.
The teleop sends stop after 0.25 s without input; the simulator has a 0.5 s watchdog.

Arm teleop: **1..7** selects a joint, **j/k** jogs −/+ 0.05 rad, **o/c** opens/closes,
**q** exits. Joint targets hold on exit. Motion is speed limited to 0.65 rad/s and
validated against the supplied URDF limits. Base commands are stopped during an
automatic button operation. Run one command publisher per controller at a time.

```sh
./scripts/ros topic pub --once /arm/joint_command sensor_msgs/msg/JointState \
  '{name: [nero_joint1], position: [0.2]}'
./scripts/ros topic pub --once /arm/press_button std_msgs/msg/String \
  '{data: "{\"id\":\"E2\",\"action\":\"hall\",\"floor\":1,\"direction\":\"up\",\"request_id\":\"demo-1\"}"}'
```

The base must first be positioned facing and within reach of the selected panel;
`route` performs this positioning. A button request approaches, presses, retracts,
and returns to the folded home pose. `/arm/state` reports completion only after
PhysX reports contact between the modeled stylus and the selected switch collider.
Unreachable requests or missing contact fail explicitly. This is a local IK/button
controller, not a collision-planning or arbitrary manipulation stack.

Elevator JSON has `id`, `action`, optional `floor`, and optional `direction`.
Actions: `hall`, `floor`, `open`, `close`, `alarm`. Command floor numbers are 1-based;
`floor` and `targetFloor` in elevator state snapshots are 0-based. For example:

```sh
./scripts/ros topic pub --once /elevator/command std_msgs/msg/String \
  '{data: "{\"id\":\"E2\",\"action\":\"hall\",\"floor\":1,\"direction\":\"up\"}"}'
./scripts/ros bag record -o .runtime/run01 /clock /joint_states /tf /tf_static \
  /ground_truth/odom /mlx/pointcloud /imu/data /front_camera/depth/points \
  /rear_camera/depth/points /wrist_camera/depth/points /arm/state /elevator/state
```

`verify` subscribes to live streams, checks timestamps and frames, and compares
simultaneous RGBD images and clouds by depth equality, reprojection and RGB bytes.
The route report records actual distance, height, contact impulses and trajectory.
No generated messages or mock simulator are used for these checks.

The third world, `research`, uses level names B1, 1F, 2F, 3F, 4F, RF at
Z = 0, 4, 8, 12, 16, 20 m. Commands still use 1-based level indices:
`floor: 2` selects 1F, and `floor: 3` selects 2F. State indices are 1 and 2,
respectively. The GUI and modeled buttons show the authored level names.

```sh
./scripts/sim --scene research --robot locomanipulator
# In a second terminal, after AMR_READY:
./scripts/ros route --scene research --robot locomanipulator --report .runtime/research-route.json
```

The example drives 12 m along the B1 corridor, calls and boards E1, rides to 1F,
then exits and drives another 12 m. Use `--robot amr` or `--robot scout` in both
commands to run the same route with ROS elevator requests. The locomanipulator
uses physical hall/cabin button contacts, as in the office and house examples.
The 1F destination button is within the NERO arm's reach; higher destinations
remain available through the GUI and ROS elevator commands.
