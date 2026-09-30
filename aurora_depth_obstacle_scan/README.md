# Aurora native depth → diagnostic LaserScan

**Used on Robot 2 only as a `collision_monitor` source** (Itsasne,
2026-09-30): `socialtech_robot_bringup robot.launch.py aurora_depth_scan:=true`
starts it with `config/local_near.yaml`, and `socialtech_robot_navigation
navigation.launch.py collision_source:=aurora_depth` slows and stops before
what it sees. Not in any costmap (see "Near-field local costmap trial").
The node does not start Nav2 or publish TF/PointCloud2. No clearing. No OpenCV,
CUDA, VPI or Isaac dependency. Native DEPTH_MAP = optical Z was checked against
paired SDK POINT3D; physical accuracy, camera extrinsics and floor rejection
remain to be validated. No scale or offset correction is applied.

Build only this package with Release and source the resulting install:

```bash
colcon build --packages-select aurora_depth_obstacle_scan --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/local_setup.bash
ros2 launch aurora_depth_obstacle_scan diagnostic.launch.py
```

This launch starts only the new node. The existing Aurora driver must provide
32FC1 depth and exact-stamp CameraInfo; the existing robot_state_publisher must
provide camera → base_link. Do not launch duplicate drivers or TF publishers.
Physical LiDAR remains `/slamware_ros_sdk_server_node/scan`. Output is
`/aurora/depth_obstacle_scan`, frame **base_link**, stamped with the input image.

On the desktop (with RViz already installed), same ROS domain/network as robot:

```bash
rviz2 -d "$(ros2 pkg prefix --share aurora_depth_obstacle_scan)/rviz/depth_diagnostic.rviz"
```

Green: depth scan. Red: physical LiDAR. RViz uses base_link as fixed frame.
The LiDAR requires the existing full TF chain through map/odom. Do not create
a replacement TF to make the display work. Existing camera translation and
rotation are provisional for Robot 2. Missing TF is a finding, not permission
to publish an invented transform. The scan projects accepted heights onto
base_link z=0; the LiDAR display can sit at a different z. Compare in top view.

Parameters are in `config/diagnostic.yaml`, startup-only. Copy that file,
change the height/range/stride/angle limits, and restart using
`params_file:=/absolute/path/to/copy.yaml`. All heights are in base_link,
never base_footprint. Start at 5 Hz and stride 2×2. `min_depth/max_depth`
bound optical Z; `min_range/max_range` bound horizontal range in base_link.
Those depth limits are **provisional**, not a physically validated range.
`transform_tolerance` bounds the wait for TF at the image stamp; there is no
latest-transform fallback. `max_input_age` rejects stale/future ROS stamps.

The node uses a bounded 8-pair pointer cache with exact stamp matching,
processes only the latest new pair on each rate-limited timer tick,
precomputes pinhole rays only when K/dimensions change, and reuses the scan
range buffer. The projection loop allocates no buffers. ROS transport and
TF can still allocate; this is not a claim of allocation-free middleware.
Malformed frames, nonzero distortion, unsupported binning/ROI, missing TF,
NaN/Inf/nonpositive/out-of-bound depths are rejected. Each angular bin keeps
the closest surviving point. Empty bins are +Inf: **unobserved, not free**.
Missing input or TF produces no replacement scan. Never enable clearing
from this diagnostic output. No confidence map is assumed available.

The driver currently shares a ROS publication-time stamp between depth and
CameraInfo. Device acquisition timestamps use a different, unverified clock;
they are not copied into ROS Time. Logs report processing+publication time
and ROS stamp age, which is **not acquisition latency**. Static camera mount
allows this MVP; motion timing and full benchmark remain pending.

RViz acceptance sequence, stationary robot: empty floor; wall; box; person;
table leg; tabletop above LiDAR; low object below LiDAR; difficult surfaces.
Check floor rejection and height bands before navigation. Empty bins should
replace old detections when the object leaves view; RViz decay limits stale
display if the stream stops. Record failures, do not fit one wall by changing
TF or scale. Compare several distances and orientations before calibration.

Tests: `colcon test --packages-select aurora_depth_obstacle_scan`. Ten C++
geometry tests and an isolated ROS contract test cover TF absence, exact
stamps in both arrival orders, malformed/stale images, invalid depth and
no replay when inputs stop. The ROS test uses LOCALHOST domain 199 only; its
synthetic TF never reaches robot domain 43.

## Near-field local costmap trial

`config/local_near.yaml` limits **horizontal range from base_link** to 1.50 m,
while retaining the 0.05–0.90 m height band, stride 2×2 and 5 Hz. Optical Z
limits are independent. The previous 3 m diagnostic profile stays available.
For the current 0.70 m long centered footprint, a frontal return at 1.50 m
is about 1.15 m ahead of the bumper. This is a trial range, not an accepted
stopping distance; include speed, latency, braking and thin-object detection
in physical acceptance.

```bash
ros2 launch aurora_depth_obstacle_scan diagnostic.launch.py \
  params_file:="$(ros2 pkg prefix --share aurora_depth_obstacle_scan)/config/local_near.yaml"
```

Start only one instance of this node (the bringup's `aurora_depth_scan:=true`
is one). The optional costmap integration lives
in `socialtech_robot_navigation/config/local_costmap_sources/scan_and_aurora_depth.yaml`.
It uses a separate marking-only ObstacleLayer so that LiDAR rays cannot clear
tabletop marks. Missing depth does not clear cells: marks may persist even
though the current LaserScan no longer contains that object. See that
package's stationary diagnostic instructions. On Robot 2 a person walking
past left lethal cells 0.10 m from the bumper that did not clear with the
robot still (2026-09-30), so this overlay is not used for navigation.
Full physical acceptance and A/B benchmark remain pending.
