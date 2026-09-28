# socialtech_robot

Real-robot packages for the SocialTech AgileX Tracer + SLAMTEC Aurora platform
(ROS 2 Jazzy, native on the robot's Jetson).

Packages:

- `socialtech_robot_bringup`: `robot.launch.py`, the single real-robot entry
  point (Tracer + description + Aurora, OAK opt-in), and `tracer.launch.py`
  with `config/tracer.yaml`.
- `socialtech_robot_aurora`: Aurora wrapper launch, frame/topic contract and
  the identity `map -> odom` static transform (`map_to_odom_static`).
- `socialtech_robot_oak`: optional OAK-D-PRO bringup (`depthai_ros_driver_v3`).
- `socialtech_robot_calibration`: offline mount fitting scripts.
- `socialtech_robot_tests`: launch argument, Xacro/URDF and TF contract tests.

This repository depends on `socialtech_common`, `tracer_ros2`, `ugv_sdk` and
`aurora_ros` (patched). It must not depend on desktop or simulation packages.
Pinned revisions and patches live in `socialtech_setup`
(`config/robot_stack.repos`, `patches/`); installation is described in
`socialtech_setup/docs/robot_setup.md`.

Per-robot values are launch arguments, not edits to this repository:

```bash
ros2 launch socialtech_robot_bringup robot.launch.py \
  port_name:=can2 aurora_ip_address:=192.168.11.1 mount_profile:=default
```

`port_name` is the Tracer's USB-CAN interface; without it the value in
`config/tracer.yaml` is used.
