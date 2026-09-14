# Scout Twin in APRL

This package contains the supplied Scout Mini sensor rig's URDF, OBJ/MTL meshes,
RViz configuration, description launch and ROS 2 Jazzy inspection tools. Its
model name is `scout_mini_photo_twin`; select it in APRL with `--robot scout`.
The original package declares Apache-2.0. See [SOURCES.md](SOURCES.md) for the
photo reconstruction and sensor assumptions.

From the APRL repository root:

```sh
./scripts/ros build
./scripts/sim --scene office --robot scout
# Separate terminals, after SCOUT_READY:
./scripts/ros rviz --robot scout
./scripts/ros teleop
./scripts/ros verify --robot scout
./scripts/ros run scout_twin_description check_streams.py --duration 20
```

All commands respect `ROS_DOMAIN_ID`, default 73. The simulator publishes
`/robot_description`, `/joint_states` and the full TF tree rooted at `sim_world`.
Launching `description.launch.py` with its defaults opens RViz using these
streams. It does not start another state publisher.

For a standalone URDF preview without Isaac:

```sh
./scripts/ros launch scout_twin_description description.launch.py \
  publish_state:=true preview_joints:=true use_sim_time:=false fixed_frame:=base_link
```

To use only the standalone state publisher, also set `rviz:=false`. Keep the
preview publishers separate from a running simulator on the same DDS domain.

Scout's optional Creeper face is selected with
`./scripts/sim --robot scout --scout-face creeper`; omitting `--scout-face` keeps
the original smile. The simulator publishes the selected URDF on
`/robot_description`, so the usual RViz command needs no additional option.
For a standalone Creeper preview, add `face:=creeper` to the launch command
above. `face:=original` is the default; an explicit `urdf:=...` still overrides it.
The face variants share all joints, inertias, collisions, and sensor frames.
See the [asset notes](../../docs/ASSETS.md#scout-creeper-face) for the Blender source.

Sensor topics preserve the supplied Scout interface:

- `/mid360/points` and `/mid360/imu` use `mid360_link` and `imu_link`.
- `/camera_{front,left,right}/{color,depth}/{image_raw,camera_info}` use separate
  `camera_{front,left,right}_{color,depth}_optical_frame` frames and intrinsics.
- APRL `/odom` is wheel odometry; `/ground_truth/odom` is the actual PhysX pose.
  The imported inspection and pivot tools use ground truth for motion checks.

Run the pivot tool in a clear area with teleop and route stopped:

```sh
./scripts/ros run scout_twin_description pivot_test.py \
  --angle-deg 90 --output .runtime/scout-pivot.json
```

The original `keyboard_teleop.py` is also installed (`i`, `,`, `j`, `l` keys).
APRL's common `./scripts/ros teleop` uses `w`, `s`, `a`, `d` for every robot.
