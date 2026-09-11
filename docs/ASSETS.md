# Assets and source references

`assets/robot/amr_body.usda` is the original AMR USD from `amr_isaac51`,
retained byte for byte. It is a photo-guided parametric model, not a calibrated
scan of the real machine. The original chassis, two drive joints, eight passive
caster joints and diagonal 2D lidar mounts are retained.
`assets/robot/amr.usda` overrides the front RGBD mount into the lower recess and
places a flush FLASH window in the larger upper recess, pitched up 13 degrees. Runtime adds its RTX emitter grid and the central PhysX IMU.
The estimated FLASH/IMU mass is included in the original 100 kg total; no
additional rigid body is attached to the chassis.

`assets/scenes/` contains eleven's original office/house USD layers and every
texture referenced by them. Landing sills receive a -7 mm session-layer offset for caster passage.
Geometry, furnishings, glass and opaque cars,
landing doors, lights, and buttons are reused. Scene source assets were
originally generated for the project in Blender and dedicated to **CC0 1.0**.
The original Python elevator controller and USD rig code are retained under
MIT (see the root LICENSE).

Earlier package consolidation removed: the entire browser demo and vendored Three.js, glTF/buffers and Blender
files unused by Isaac, duplicate render galleries, obsolete test worlds,
old build/install/log trees, cached bytecode, historical verification snapshots,
and the redundant ROS package. The AMR photos are not redistributed.

References inspected for this implementation:

- [SOSLAB ML-X SDK and v2.3.2 guide](https://github.com/SOSLAB-github/ML-X_SDK)
  — read only; none of its source or SDK binaries are included or modified.
- [SOSLAB product brochure](https://oemfile.informamarkets-info.com/FileUpload/2024SEC_35954/2023101012494075.pdf)
  — ML-X(120) 120° × 35° field of view. The SDK guide documents normal 192-column
  packets and a selectable 10 Hz mode. We choose 56 rows at uniform angular
  spacing; the exact factory beam calibration is not available in this package.
- [Isaac Sim 5.1 RTX lidar](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/sensors/isaacsim_sensors_rtx_lidar.html)
  — native OmniLidar and emitter-state schema; local installed 5.1 code checked.
- [Isaac Sim 5.1 ROS installation](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_ros.html)
  — bundled Python 3.11 Jazzy libraries for the simulator, native Ubuntu 24.04
  Jazzy in a separate Python 3.12 process.
- [Isaac Sim 5.1 container installation](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_container.html)
  — official 5.1.0 image used as the binary source for the Ubuntu 24.04 image.

Isaac Sim/NVIDIA software retains its own license. The original AMR's provenance
does not establish a new license for source photographs or real robot designs.

## NERO and Blender additions

The user-supplied gripper URDF and its meshes are distributed in `src/nero_description/`.
Mesh URIs resolve through `nero_description`.
The requested gripper URDF is imported with Isaac 5.1's native URDF importer;
DAE visuals, STL collision meshes, origins, limits and inertial tensors are used.
`assets/robot/nero.usdc` is a flattened, portable result with no temporary-path
references. Unrequested hand variants and xacro sources are retained locally and
excluded from the runtime distribution. Their redistribution license is not inferred
from the repository's MIT license.

The runtime import omits the URDF's world anchor and massless virtual aperture
joint, prefixes link/joint names with `nero_`, and commands the two physical
finger joints with the supplied ±0.5 mimic relationship. The source link7 mass
of 0.000001 kg is a CAD placeholder: the runtime uses an estimated 0.030 kg
wrist adapter. Other original link masses and tensors are retained. The camera,
mount and stylus add 0.088 kg to the gripper rigid body with an estimated inertia
increment of 0.0005 kg·m² per principal axis. Drive limits come from the URDF;
position stiffness 900 and damping 70 are simulator controller settings.

The riser and Gemini 305 were modeled by the installed **Blender 4.0.2**.
Runtime USD meshes/materials are in `assets/robot/components/`. Editable `.blend`
files and generation tools are retained locally and excluded from distribution.
Ubuntu's Blender build lacks the USD export operator; the modeling script exported
evaluated mesh vertices, faces and materials directly to USDA. It did not substitute
a primitive box for the assembled riser or camera.

The riser has four 30 mm profiles, perimeter rails, two 8 mm plates and fasteners.
Its nominal 550 mm height puts the NERO base 795 mm above the floor. Mass uses
2700 kg/m³ aluminum plates, 0.9 kg/m profiles and 0.17 kg fasteners: **8.10008 kg**.
The corresponding center of mass is `(0, 0, 0.256500)` m in `riser_link` and diagonal
inertia is `(0.537006, 0.559879, 0.171462)` kg·m². Profile grooves are visual features;
convex colliders retain separate columns, plates and rails with an open center.
Two fixed joints attach the riser to the chassis and the arm to its top plate.
The chassis rigid body explicitly owns the combined articulation root, so changing
arm posture cannot relocate the base through automatic root selection.

The wrist camera follows the [linked product image](https://www.devicemart.co.kr/goods/view?no=16015588)
and [Orbbec specifications](https://store.orbbec.com/products/gemini-305):
42 × 42 × 23 mm body, 68 g, 18 mm stereo baseline, silver rounded housing, black
front glass, two lenses and side USB-C. Rendered depth is ideal optical Z depth,
not a reconstruction of the device's stereo algorithm. Simulation output is
640 × 400 at 15 Hz and 0.04–1 m. Near-range clipping can yield all-NaN clouds when
no surface is in view. The reference photographs were consulted, not redistributed.

A short shaft with an 8 mm radius elastomer tip is held by the closed gripper for
button operation. Each existing visual lamp has an aligned, kinematic PhysX switch
collider. The elevator receives a request only upon a real tip/switch contact report;
contact position, impulse, separation and simulation time are recorded.
Zero-impulse proximity reports are ignored; activation requires at least 0.001 N·s. This is an
ideal contact switch, not a compliant spring/button or force-controlled manipulation
model. The existing robot disables articulation self-collision; arbitrary arm motions
therefore require a separate collision planner. The supplied folded posture and
button paths are evaluated by running the complete navigation examples.
