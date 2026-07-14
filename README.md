# socialtech_robot

Real-robot packages for the SocialTech AgileX Tracer + SLAMTEC Aurora platform.

Current packages:

- `socialtech_robot_bringup`: Tracer bringup, velocity mux, teleop entry points,
  and preflight checks.
- `socialtech_robot_aurora`: Aurora wrapper launch and configuration.
- `socialtech_robot_tests`: robot-side package contract tests.

This repository may depend on `socialtech_common`, `tracer_ros2`, `ugv_sdk`, and
`aurora_ros`. It must not depend on desktop or simulation packages.

External source dependencies:

- `ugv_gazebo_sim`, via `socialtech_common`, for `tracer_description`.
- `tracer_ros2`, for `tracer_base` and `tracer_msgs`.
- `ugv_sdk`, used by the AgileX Tracer driver.
- `aurora_ros`, for `slamware_ros_sdk`.

Robot-only build:

```bash
./src/socialtech_setup/tools/install_dependencies.sh robot
./src/socialtech_setup/tools/apply_patches.sh robot
rosdep install \
  --from-paths \
    src/socialtech_common \
    src/socialtech_robot \
    src/tracer_ros2 \
    src/ugv_sdk \
    src/aurora_ros \
    src/ugv_gazebo_sim \
  -y --ignore-src

colcon build --packages-up-to socialtech_robot_bringup socialtech_robot_aurora
```
