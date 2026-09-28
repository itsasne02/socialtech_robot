# socialtech_robot_calibration

Sensor mount calibration tools for the SocialTech robot: estimate a depth
sensor's real mount position/orientation from its own live point cloud,
instead of measuring with a tape measure to an ambiguous point on the
sensor housing.

Works against any `sensor_msgs/PointCloud2` topic -- both `/oak/points`
(OAK-D-PRO, see `socialtech_robot_bringup`'s `oak_pointcloud_processing`)
and Aurora's `/slamware_ros_sdk_server_node/depth_point_cloud` use the same
two scripts.

## Scope

Both scripts are read-only against the ROS graph: they only subscribe to
the cloud topic and to `/tf`/`/tf_static`. Neither publishes anything or
can move the robot. They print a suggested correction for the relevant
`config/*_mounts/*.yaml` (in `socialtech_robot_description`, part of
`socialtech_common`); neither edits that file itself.

## Why two scripts

A flat floor alone only determines 3 of the 6 mount degrees of freedom
(height and tilt) -- it looks geometrically identical no matter the
sensor's forward/lateral position or yaw, so those need something that
breaks that symmetry: real objects at known positions.

- **`fit_ground_plane.py`** -- height (z) + pitch/roll, from the floor plane alone.
- **`fit_xy_yaw.py`** -- x/y + yaw, from two objects placed at known positions.

## Method

Both scripts transform the live cloud into `--reference-frame` (default
`base_footprint`, where a correctly calibrated setup places the floor
exactly at Z=0) using whatever mount calibration is *currently* live, then
measure how far reality is from that ideal. The residual is attributed
entirely to the `--parent-frame -> --output-frame` mount joint you are
solving for (e.g. `base_link -> oak_link`) and folded into its currently
published transform (read live via `tf2`, i.e. whatever the yaml currently
has) to print corrected `xyz_from_parent`/`rpy_from_parent` values.

This is a best-effort single-shot estimate, not a proof. **Re-run the
script after applying its suggested correction to the yaml** (and
rebuilding/relaunching): a well-calibrated mount should then report a
near-zero residual. If it does not, iterate -- do not trust a single run
blindly, the same way Aurora's own calibration in this project needed a
second, more careful pass after the first one looked wrong in RViz (see
`docs/project_context/06_perception_modes.md` Modo F on the desktop
repo for that history).

## Usage

Bringup and the sensor must already be running (e.g.
`ros2 launch socialtech_robot_bringup robot.launch.py use_oak:=true oak_pointcloud_processing:=true`).

Height, pitch, roll (point the sensor at a clear, flat floor patch):

```bash
ros2 run socialtech_robot_calibration fit_ground_plane.py \
  --topic /oak/points --output-frame oak_link
```

X, Y, yaw (clear the area in front of the robot first, then follow the
prompts -- you will be asked to place one object, then a second one, each
at a position you measure and type in):

```bash
ros2 run socialtech_robot_calibration fit_xy_yaw.py \
  --topic /oak/points --output-frame oak_link
```

`--parent-frame` (default `base_link`) and `--reference-frame` (default
`base_footprint`) match `config/oak_mounts/*.yaml`'s `parent_frame` key and
this project's floor-projection frame respectively -- override them only if
calibrating a mount with a different parent, or a sensor whose config uses
different names.

Run `--help` on either script for every tuning parameter (frame count,
RANSAC thresholds, timeouts).

## Not Implemented Here

Neither script writes the yaml file automatically -- review the suggested
numbers and the current-vs-suggested diff before pasting them in, same as
every other measured value in `socialtech_robot_description`'s
`config/*_mounts/`.
