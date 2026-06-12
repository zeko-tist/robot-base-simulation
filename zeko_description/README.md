# zeko_description

ROS2 robot description package for **ZEKO**, a differential-drive mobile
robot. Provides a modular Xacro/URDF model, `ros2_control` configuration,
launch files, and meshes, ready for use with `robot_state_publisher`,
`joint_state_publisher`, `ros2_control`, `Nav2`, and **Isaac Sim 6.0**.

```
zeko_description/
├── package.xml
├── CMakeLists.txt
├── README.md
├── urdf/
│   ├── materials.xacro       # shared visual materials/colors
│   ├── base.xacro             # axis remap + global properties + inertia macros + base_link
│   ├── wheels.xacro            # left_wheel_link / right_wheel_link (continuous)
│   ├── casters.xacro           # front_caster_link / rear_caster_link (fixed)
│   ├── support_frame.xacro     # support_frame_link (fixed)
│   ├── pelvis_mount.xacro       # pelvis_mount_link (fixed)
│   ├── zeko.xacro                # top-level: includes all of the above + ros2_control
│   └── zeko.urdf                 # pre-expanded URDF (generated from zeko.xacro)
├── config/
│   └── ros2_control.yaml        # controller_manager: joint_state_broadcaster + diff_drive_controller
├── launch/
│   └── display.launch.py        # RViz + robot_state_publisher + joint_state_publisher_gui
└── meshes/
    ├── base_link.stl
    ├── left_wheel_link.stl
    ├── right_wheel_link.stl
    ├── front_caster_fork.stl
    ├── front_caster_wheel.stl
    ├── rear_caster_fork.stl
    ├── rear_caster_wheel.stl
    ├── support_frame_link.stl
    └── pelvis_mount_link.stl
```

---

## 1. Source data: real Fusion 360 geometry

This package is built from a Fusion 360 export of `zeko_base.f3d` containing
the real STL meshes for every body **and** an auto-generated `zeko.urdf`
with mass/inertia properties computed by Fusion from the actual solid
bodies + assigned materials. Every dimension, mass, and inertia value in
`urdf/base.xacro` is taken directly from that export (or, for the three
bodies Fusion did not export mass properties for, estimated from their
mesh volumes — see Section 3).

### 1.1 The CAD → ROS axis remap

Fusion's export uses its own world frame, call it `(CAD_X, CAD_Y, CAD_Z)`.
Inspecting the real geometry shows:

- The two drive wheels are centered at `CAD_X = ±0.300 m` and rotate about
  axis `(±1, 0, 0)` → **`CAD_X` is the wheel-separation (left/right) axis.**
- The front/rear casters are centered at `CAD_Y = ±0.450 m` →
  **`CAD_Y` is the forward/backward axis.**
- `CAD_Z` is already "up", and the whole assembly's `(0,0,0)` origin sits
  at ground level, on the robot's centerline.

To satisfy the required **X = forward, Y = left, Z = up** convention
(REP-103), every link applies a fixed **-90° rotation about Z**
(`cad_yaw = -pi/2`, defined once in `base.xacro`) to its visual/collision
mesh:

```
ROS_X =  CAD_Y   (forward)
ROS_Y = -CAD_X   (left)
ROS_Z =  CAD_Z   (up)
```

For each link, a CAD-frame anchor point `P_cad` is chosen as that link's
origin. Its mesh `<origin>` is then `rpy="0 0 cad_yaw"`,
`xyz = (-P_cad.y, P_cad.x, -P_cad.z)` — this places the mesh (whose raw
vertices are absolute CAD-frame coordinates) at its correct real position
in the link's local ROS frame. Joint origins to parent links are
`R(P_cad_child - P_cad_parent)` using the same rotation. The exact `P_cad`
used for every link is documented in the corresponding `*.xacro` file.

### 1.2 Front/rear and left/right convention

The CAD export has no explicit "front"/"left" labels, so this package
defines:

- **Front caster** = the caster at `CAD_Y = +0.450` → `ROS_X = +0.45` (+X = forward).
- **Rear caster** = the caster at `CAD_Y = -0.450` → `ROS_X = -0.45`.
- **Left wheel** = the wheel at `CAD_X = -0.300` → `ROS_Y = +0.30` (+Y = left).
- **Right wheel** = the wheel at `CAD_X = +0.300` → `ROS_Y = -0.30`.

If the real robot's "front" turns out to be the opposite end, swap the
`front_caster` / `rear_caster` instantiation arguments in `casters.xacro`
(and the `left`/`right` wheel arguments in `wheels.xacro`) — everything
else (dimensions, masses, joint structure) is unaffected.

---

## 2. Robot structure

```
base_link
├── left_wheel_link    (left_wheel_joint,   continuous, axis = Y)
├── right_wheel_link   (right_wheel_joint,  continuous, axis = Y)
├── front_caster_link  (front_caster_joint, fixed)
├── rear_caster_link   (rear_caster_joint,  fixed)
└── support_frame_link (support_frame_joint, fixed)
      └── pelvis_mount_link (pelvis_mount_joint, fixed)
```

Coordinate convention: **X = forward, Y = left, Z = up** (REP-103).
`base_link` is the root link. Its origin coincides with the original CAD
assembly origin, which sits at ground level on the robot's centerline
(it effectively doubles as `base_footprint`).

### Key dimensions / properties (`urdf/base.xacro`)

All values below come directly from the real CAD geometry (mesh bounding
boxes / Fusion mass properties), with masses for `base_link` and the
wheels rescaled to realistic materials, and the support frame, pelvis
mount, and casters estimated (see Section 3).

| Property                 | Value         | Notes                                                    |
|---------------------------|----------------|------------------------------------------------------------|
| `base_length`             | 1.00 m         | chassis X (forward) extent — from CAD bbox (CAD_Y span)     |
| `base_width`              | 0.65 m         | chassis Y (left/right) extent — from CAD bbox (CAD_X span)  |
| `base_height`             | 0.124 m        | chassis Z extent — from CAD bbox                            |
| `base_bottom_z`           | 0.097 m        | height of chassis bottom face above ground                  |
| `base_mass`               | 59.3454 kg     | **aluminum** (Fusion mass x 2700/7850, see Section 3)       |
| `base_ixx/iyy/izz`        | 0.738307 / 1.83627 / 2.470721 | **aluminum-scaled**, remapped to ROS frame |
| `wheel_radius`            | 0.1524 m       | ≈ 6 in, from CAD mesh diameter (~304.7 mm)                  |
| `wheel_width`             | 0.08 m         | from CAD mesh                                               |
| `wheel_separation`        | 0.60 m         | center-to-center, from CAD wheel positions (±0.300 m)       |
| `wheel_mass`              | 6.637983 kg    | **solid rubber** (Fusion mass x 1200/7850, per wheel)       |
| `wheel_ixx/iyy/izz`       | 0.043167 / 0.079915 / 0.043408 | **rubber-scaled**, remapped to ROS frame  |
| `caster_radius`           | 0.045 m        | from CAD mesh of caster wheel (90 mm dia)                   |
| `caster_x_offset`         | 0.45 m         | from CAD caster positions (CAD_Y = ±0.450 m)                 |
| `caster_mass`             | 0.6 kg         | **estimated** (aluminum fork + rubber wheel, see Section 3) |
| `support_frame_length`    | 0.348 m        | from CAD bbox (CAD_Y span)                                  |
| `support_frame_width`     | 0.268 m        | from CAD bbox (CAD_X span)                                  |
| `support_frame_height`    | 0.283 m        | from CAD bbox; sits directly on `base_link` top face         |
| `support_frame_mass`      | 3.5 kg         | **estimated** (aluminum, see Section 3)                     |
| `pelvis_length`           | 0.225 m        | from CAD bbox (CAD_Y span)                                  |
| `pelvis_width`            | 0.285 m        | from CAD bbox (CAD_X span)                                  |
| `pelvis_height`           | 0.152 m        | from CAD bbox; sits directly on `support_frame_link` top face|
| `pelvis_mass`             | 3.0 kg         | **estimated** (aluminum, see Section 3)                     |

All inertia tensors for the support frame, pelvis mount, and casters are
computed from these properties via the `box_inertial` / `sphere_inertial`
macros in `urdf/base.xacro` (solid-body formulas). `base_link` and the two
wheels use the real Fusion-computed inertia tensors directly.

---

## 3. Notes on mass/material choices and total robot mass

Fusion's export only included `<inertial>` data for `base_link` and the
two drive wheels (these were the only bodies modeled as separate rigid
bodies with revolute joints in the assembly), and that data was computed
assuming a **steel** material (~7850 kg/m³) for everything — which gave an
implausible ~94%-solid-steel wheel (43.4 kg each) and a 172.5 kg chassis.

This package instead assumes:

- **`base_link`, `support_frame_link`, `pelvis_mount_link` = aluminum**
  (~2700 kg/m³)
- **wheels = solid rubber/PU** (~1200 kg/m³)
- **casters = aluminum fork + rubber wheel**

Since geometry is unchanged, mass and inertia both scale linearly with
density. `base_mass`/`base_ixx/iyy/izz` were rescaled from Fusion's steel
values by `2700/7850 = 0.34395`, and `wheel_mass`/`wheel_ixx/iyy/izz` by
`1200/7850 = 0.15287`. The original steel values are kept in comments in
`urdf/base.xacro` for reference.

The support frame, pelvis mount, and casters exist as real mesh geometry
but had no Fusion mass properties at all, so their masses were estimated
from each mesh's bounding-box volume at the densities above and a rough
solid-fraction (frames and mounting plates are rarely solid blocks):

- `caster_mass = 0.6 kg` (~0.40 kg aluminum fork + ~0.20 kg rubber wheel, ~90 mm wheel)
- `support_frame_mass = 3.5 kg` (0.348 × 0.268 × 0.283 m envelope, ~5% solid)
- `pelvis_mass = 3.0 kg` (0.225 × 0.285 × 0.152 m envelope, ~11% solid)

**Total robot mass ≈ 80.3 kg** (59.35 + 2×6.64 + 2×0.6 + 3.5 + 3.0) — a much
more reasonable figure for a ~1 m aluminum-framed mobile robot with rubber
tires. If the real materials differ, edit the density ratios (or directly
the `*_mass`/`*_ixx/iyy/izz` values) in `urdf/base.xacro`; everything else
derives from them automatically.

---

## 4. Building

```bash
cd ~/ros2_ws
colcon build --packages-select zeko_description
source install/setup.bash
```

## 5. Usage

### Visualize in RViz

```bash
ros2 launch zeko_description display.launch.py
```

This runs `xacro` on `urdf/zeko.xacro`, starts `robot_state_publisher`,
`joint_state_publisher_gui` (sliders for `left_wheel_joint` /
`right_wheel_joint`), and RViz2.

### ros2_control + diff_drive_controller

`urdf/zeko.xacro` embeds a `<ros2_control>` block (hardware plugin
`mock_components/GenericSystem` by default) exposing:

- `left_wheel_joint`  — command_interface: velocity, state_interfaces: position, velocity
- `right_wheel_joint` — command_interface: velocity, state_interfaces: position, velocity

`config/ros2_control.yaml` loads `joint_state_broadcaster` and
`diff_drive_controller` with `wheel_separation = 0.60` and
`wheel_radius = 0.1524` (matching `urdf/base.xacro`). Launch
`controller_manager` with this URDF + YAML, then:

```bash
ros2 control load_controller --set-state active joint_state_broadcaster
ros2 control load_controller --set-state active diff_drive_controller

ros2 topic pub /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped \
  "{twist: {linear: {x: 0.2}, angular: {z: 0.0}}}"
```

> For a Gazebo simulation, change the `<plugin>` in the `<ros2_control>`
> block of `zeko.xacro` to `gazebo_ros2_control/GazeboSystem`. For a real
> robot, replace it with your hardware interface plugin.

### Nav2

`base_link` + `left_wheel_joint`/`right_wheel_joint` +
`diff_drive_controller` (publishing `odom` → `base_link` TF and
`/diff_drive_controller/odom`) provide everything Nav2's default
`nav2_params.yaml` expects for a differential-drive base
(`odom_topic`, `base_frame_id: base_link`, `cmd_vel_topic`).

### Isaac Sim 6.0

1. Use the included flattened `urdf/zeko.urdf` (or regenerate it with
   `xacro urdf/zeko.xacro > urdf/zeko.urdf`).
2. In Isaac Sim: **File > Import > URDF**, point it at `urdf/zeko.urdf`.
   Enable "Fix Base Link" = **false** (ZEKO is a mobile robot). If the
   importer can't resolve `package://zeko_description/...` mesh paths
   (no ROS workspace on the import machine), either source a workspace
   containing this package first, or edit the `<mesh filename="...">`
   entries in `zeko.urdf` to absolute `file://` paths.
3. After import, the **Articulation Inspector** exposes
   `left_wheel_joint` and `right_wheel_joint` as the only actuated DOFs
   (all caster/support/pelvis joints are `fixed` and do not appear as
   DOFs). Both wheel joints are `continuous` with `axis = (0, 1, 0)`,
   `effort = 60 N·m`, `velocity = 20 rad/s`.
4. Drive the robot directly via the **Articulation Controller**. Set the
   wheel joint drives to **velocity** mode with low/zero stiffness (the
   importer often defaults to position drives with nonzero stiffness,
   which will fight a velocity command), then apply
   `velocityCommand = [v, v]` to `[left_wheel_joint, right_wheel_joint]` —
   both wheels spin about the shared Y axis, producing forward motion in
   +X.
5. For ROS2 differential-drive control inside Isaac Sim, wire:
   `ROS2 Subscribe Twist` → `Differential Controller`
   (`wheel_radius = 0.1524`, `wheel_distance = 0.60`,
   `left_joint = left_wheel_joint`, `right_joint = right_wheel_joint`) →
   `Articulation Controller`. `linear.x` / `angular.z` map directly to
   wheel velocities with these parameters.

---

## 6. Validation summary

The generated `urdf/zeko.urdf` was checked for:

- ✅ 7 links (`base_link`, `left_wheel_link`, `right_wheel_link`,
  `front_caster_link`, `rear_caster_link`, `support_frame_link`,
  `pelvis_mount_link`), no duplicates.
- ✅ 6 joints (`left_wheel_joint`, `right_wheel_joint`,
  `front_caster_joint`, `rear_caster_joint`, `support_frame_joint`,
  `pelvis_mount_joint`), no duplicates.
- ✅ Every link has `<visual>`, `<collision>`, and `<inertial>` (no empty
  inertials). The two caster links each have **two** `<visual>` mesh
  entries (fork + wheel bodies) sharing one sphere `<collision>`.
- ✅ Every joint has `<parent>`, `<child>`, `<origin>`, and (for
  non-fixed joints) `<axis>`.
- ✅ Single root link (`base_link`), all links reachable — no orphans.
- ✅ `left_wheel_joint` / `right_wheel_joint` are `continuous` with
  `axis="0 1 0"`; all other joints are `fixed`.
- ✅ `<ros2_control>` block exposes `left_wheel_joint` and
  `right_wheel_joint` with velocity command + position/velocity state
  interfaces.
- ✅ `config/ros2_control.yaml` wheel geometry matches `urdf/base.xacro`
  (`wheel_radius = 0.1524`, `wheel_separation = 0.60`).
- ✅ All 9 mesh files referenced by `urdf/zeko.urdf` exist in `meshes/`
  and use `scale="0.001 0.001 0.001"` (CAD source units = mm).
- ✅ Visual mesh placement for `support_frame_link` and `pelvis_mount_link`
  verified: each mesh's world-frame Z span (`[0.221, 0.504]` and
  `[0.504, 0.656]` respectively) matches the parent link's collision box
  top face exactly, so the chassis -> support frame -> pelvis mount stack
  has no visual gaps. (An earlier revision had these mesh `<origin>` z
  offsets set to `0` instead of `-0.221` / `-0.504`, which floated both
  meshes well above their correct position — fixed.)
- ✅ Total mass ≈ 80.3 kg with aluminum/rubber material assumptions (see
  Section 3), vs. ~267 kg with Fusion's default steel-everywhere export.
