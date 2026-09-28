# socialtech_robot_bringup

Real-robot bringup for the SocialTech AgileX Tracer platform: Tracer +
description + Aurora + OAK-D-PRO, coordinated from a single launch file.

This package wraps existing drivers and coordinates startup. It does not copy or
modify `tracer_ros2`, `ugv_sdk`, `aurora_ros`, or `depthai-ros`.

## Scope

`socialtech_robot_bringup` does:

- start the AgileX Tracer base through the upstream `tracer_base` launch;
- parameterize the CAN interface, odometry frame, base frame, and odometry topic;
- optionally start `robot_state_publisher` from `socialtech_robot_description`;
- optionally start SLAMTEC Aurora via `socialtech_robot_aurora` (on by
  default: Aurora is this robot's only localization source, there is no
  AMCL fallback);
- optionally start the OAK-D-PRO via `socialtech_robot_oak`, and optionally
  turn its depth image into an obstacle point cloud for Nav2
  (`oak_pointcloud_processing`).

It does not include URDF/Xacro (owned by `socialtech_robot_description`),
RViz, Gazebo, Nav2, SLAM, maps, teleoperation, vendor driver code, or
patches to vendor packages.

## Relationship With Description

The robot model belongs to `socialtech_robot_description`. This bringup package
uses that package when `use_description:=true`; it must not duplicate robot
frames, URDF, or Xacro files.

## Build

```bash
cd ~/socialtech_ws
rosdep install \
  --from-paths \
    src/socialtech_common \
    src/socialtech_robot \
    src/tracer_ros2 \
    src/ugv_sdk \
    src/aurora_ros \
    src/depthai-ros \
    src/ugv_gazebo_sim \
  -y --ignore-src
colcon build --packages-up-to socialtech_robot_bringup
source install/setup.bash
```

## Launch The Real Robot

Full stack (Tracer + description + Aurora; this is the default -- OAK stays
off unless asked for):

```bash
ros2 launch socialtech_robot_bringup robot.launch.py
```

With the OAK-D-PRO, camera only (no obstacle point cloud yet):

```bash
ros2 launch socialtech_robot_bringup robot.launch.py use_oak:=true
```

With the OAK-D-PRO feeding Nav2's `oak_point_cloud` obstacle source
(`socialtech_robot_navigation`, on the operator PC) -- validate CPU/bandwidth
first, see `oak_pointcloud_processing`'s description in `robot.launch.py`:

```bash
ros2 launch socialtech_robot_bringup robot.launch.py use_oak:=true oak_pointcloud_processing:=true
```

Tracer (+ OAK, optionally) only, without Aurora -- e.g. while diagnosing
Tracer or OAK on their own, or while Aurora is disconnected/unreachable:

```bash
ros2 launch socialtech_robot_bringup robot.launch.py use_aurora:=false
```

Every argument `robot.launch.py` declares is listed by:

```bash
ros2 launch socialtech_robot_bringup robot.launch.py --show-args
```

The Tracer adapter is `can_tracer` once socialtech_setup's udev rule is
installed (`config/tracer.yaml`). Check it before launch:

```bash
ip -details link show can_tracer
```

## Launch Tracer Only

```bash
ros2 launch socialtech_robot_bringup tracer.launch.py port_name:=can_tracer
```

## Preflight Checks

There is no automated safety-check launch file yet. Before moving the
robot, check manually:

```bash
ip -details link show <can-name>
ros2 node list
ros2 topic list
ros2 topic echo /odom --once
ros2 run tf2_tools view_frames
```

`socialtech_robot_tests`'s `check_tf_contract.py` validates the live TF
graph against the documented contract (single tree, no duplicate
publishers, Aurora/OAK frames reachable from `base_footprint`) --
run it as an additional check:

```bash
ros2 run socialtech_robot_tests check_tf_contract.py --require-oak
```

## Not Implemented Here

Teleoperation, `twist_mux`-based command arbitration, and an automated
preflight/safety-check launch are not part of this package yet -- if
needed, add them as a separate task rather than assuming they already
exist. Nav2, SLAM, and RViz live in `socialtech_desktop`
(`socialtech_robot_navigation`, `socialtech_robot_slam`,
`socialtech_robot_rviz`) on the operator PC, not here.
