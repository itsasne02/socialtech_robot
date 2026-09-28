# socialtech_robot_oak

Robot-side wrapper for the OAK-D-PRO camera on the SocialTech robot.

This package starts the external `depthai_ros_driver_v3` composable node
directly, centralizes the initial OAK configuration, and documents the
expected frames and topics. It does not copy the driver and does not modify
vendor source code.

OAK is physically independent from Aurora: it mounts on `base_link` as its
own `oak_link` frame (see `socialtech_robot_description`), not on
`aurora_link`. Its position can be tuned on its own via
`config/oak_mounts/*.yaml` in that package.

## Scope

This package does:

- Launch the OAK camera on the real robot via `depthai_ros_driver_v3`'s
  `depthai_ros_driver::Driver` composable node, the same way
  `socialtech_robot_aurora` starts its driver Node directly.
- Configure camera model, frame names, and bandwidth-sensitive parameters.
- Optionally include the upstream `depthai_descriptions_v3` URDF so the
  camera-internal frames (rgb/left/right/imu optical frames) and their
  visual meshes appear in TF and RViz at runtime.
- Offer an explicit compatibility path
  (`use_upstream_launch:=true`) to run the vendor's own
  `driver_as_part_of_a_robot.launch.py` instead, for debug/comparison.

This package does not:

- Launch RViz, Nav2, SLAM desktop tools, Gazebo, teleop, Tracer, or Aurora.
- Publish the physical `base_link -> oak_link` mount transform. That is owned
  by `socialtech_robot_description` (`use_oak:=true`).
- Assume a confirmed camera model or mount location: both are placeholders
  pending validation on the physical robot (see
  `socialtech_robot_description/config/physical_layout.yaml`).

## Upstream Driver

The vendor repository, `depthai-ros`, is vendored into the workspace the same
way `aurora_ros` is: pinned and cloned by
`socialtech_setup/tools/install_dependencies.sh`, with patches applied by
`apply_patches.sh`. It is treated as an optional add-on, not part of the
`robot` target, since not every robot has an OAK camera:

```bash
cd ws
./src/socialtech_setup/tools/install_dependencies.sh oak
./src/socialtech_setup/tools/apply_patches.sh oak
```

`DEPTHAI_ROS_BRANCH` in `install_dependencies.sh` is pinned to `v3_jazzy`.
Package names on that branch are `depthai_ros_driver_v3` and
`depthai_descriptions_v3` (not the plain `depthai_ros_driver`/
`depthai_descriptions` names used by older depthai-ros branches, and by the
`alumnos` reference code, which targeted an older API). If `DEPTHAI_ROS_BRANCH`
is ever changed, re-verify package names, the composable node plugin name,
and the launch files this package relies on
(`depthai_ros_driver_v3/launch/driver_as_part_of_a_robot.launch.py`,
`depthai_descriptions_v3/launch/urdf_launch.py`,
`depthai_descriptions_v3/urdf/include/depthai_macro.urdf.xacro`) with a
throwaway clone of the new branch before trusting this package again.

Do not modify `depthai-ros` sources directly. If a driver change is needed,
keep it as a reviewed patch in `socialtech_setup/patches/depthai-ros`, the
same policy `socialtech_robot_aurora` follows for `aurora_ros`.

## Install Dependencies

From the workspace root:

```bash
cd ws
./src/socialtech_setup/tools/install_dependencies.sh oak
./src/socialtech_setup/tools/apply_patches.sh oak
rosdep install --from-paths src/depthai-ros src/socialtech_robot/socialtech_robot_oak -y --ignore-src
```

## Launch

Default robot launch (camera + upstream camera-internal URDF):

```bash
ros2 launch socialtech_robot_oak oak.launch.py
```

With an explicit camera model, once confirmed on the physical unit:

```bash
ros2 launch socialtech_robot_oak oak.launch.py camera_model:=OAK-D-PRO
```

Without the upstream camera-internal URDF, if something else already
publishes those frames (for example `socialtech_robot_description` launched
with `oak_include_upstream_urdf:=true`):

```bash
ros2 launch socialtech_robot_oak oak.launch.py publish_camera_urdf:=false
```

**Do not** run this package's default `publish_camera_urdf:=true` at the same
time as `socialtech_robot_description`'s `oak_include_upstream_urdf:=true`:
both would start a `robot_state_publisher` for the same camera-internal
frames. Pick one path.

Compatibility with the vendor's own robot-integration launch file:

```bash
ros2 launch socialtech_robot_oak oak.launch.py use_upstream_launch:=true
```

From the full robot bringup, use `socialtech_robot_bringup`'s `robot.launch.py`:
it declares `use_oak`/`oak_mount_profile`/`oak_include_upstream_urdf` and
forwards them into `socialtech_robot_description`'s `description.launch.py`,
so the physical `oak_link` mount exists before this package attaches the
camera-internal frames to it. It also has `oak_pointcloud_processing`, which
turns the OAK's depth image into a Nav2 obstacle point cloud without this
package's own `pointcloud_enable` (see that argument's description in
`robot.launch.py` for why: an unresolved upstream color/depth sync bug):

```bash
ros2 launch socialtech_robot_bringup robot.launch.py use_oak:=true
ros2 launch socialtech_robot_bringup robot.launch.py use_aurora:=true use_oak:=true
```

## Validate Topics

```bash
ros2 topic list
ros2 topic echo /oak/rgb/image_raw --once
ros2 topic hz /oak/rgb/image_raw
ros2 topic hz /oak/stereo/image_raw
```

Enable the pointcloud only after checking bandwidth/CPU on the real robot:

```bash
ros2 launch socialtech_robot_oak oak.launch.py pointcloud_enable:=true
ros2 topic hz /oak/rgbd/points
```

## Validate TF

```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo base_link oak_link
ros2 run tf2_ros tf2_echo oak_link oak_rgb_camera_optical_frame
```

`base_link -> oak_link` must come from `socialtech_robot_description`
(`use_oak:=true`). Everything below `oak_link` must come from exactly one
source: either the upstream `depthai_descriptions_v3` URDF launched by this
package (`publish_camera_urdf:=true`), or `socialtech_robot_description`
(`oak_include_upstream_urdf:=true`), never both. If `oak_link` is missing
entirely, the description package was not launched with `use_oak:=true`.

## Common Errors

- OAK is not connected over USB, or the USB link is too slow for the
  selected resolution/fps.
- `camera_model` does not match the physical unit: some features silently
  disable instead of failing loudly.
- `oak_link` is missing or `oak`/its children are disconnected root frames:
  `use_oak:=true` was not passed to `socialtech_robot_description` (only
  `robot_oak_aurora.launch.py` forwards it), or the driver's
  `i_tf_parent_frame`/`i_tf_base_frame` parameters (set from this launch's
  own `parent_frame`/`camera_name` args) disagree with what
  `depthai_descriptions_v3`'s `urdf_launch.py` used.
- Camera-internal frames published twice because both
  `publish_camera_urdf:=true` here and `oak_include_upstream_urdf:=true` in
  the description package are active at once.
- Pointcloud or high-resolution RGB enabled without checking CPU/USB
  bandwidth budget on the robot computer first.
- Package-not-found errors mentioning `depthai_ros_driver`/
  `depthai_descriptions` (without `_v3`): those are the older depthai-ros
  package names; this repo targets the `v3_jazzy` branch.

## Build And Smoke Test

```bash
cd ~/socialtech_ws
./src/socialtech_setup/tools/install_dependencies.sh oak
./src/socialtech_setup/tools/apply_patches.sh oak
rosdep install --from-paths src/depthai-ros src/socialtech_robot/socialtech_robot_oak -y --ignore-src
colcon build --packages-up-to socialtech_robot_oak
source install/setup.bash

ros2 launch socialtech_robot_oak oak.launch.py --show-args
ros2 launch socialtech_robot_oak oak.launch.py
ros2 launch socialtech_robot_bringup robot_oak_aurora.launch.py use_oak:=true
```
