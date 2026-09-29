# socialtech_robot_aurora

Robot-side wrapper for SLAMTEC Aurora on the SocialTech robot.

This package launches the external `slamware_ros_sdk` driver, centralizes the
initial Aurora configuration, and documents the expected frames, topics, and
patch policy. It does not copy the SDK and does not modify vendor source code.

## Scope

This package does:

- Launch Aurora on the real robot.
- Configure the Aurora IP and frame names.
- Keep a documented robot-side contract in `config/`.
- Offer an explicit compatibility path to include the upstream XML launch.

This package does not:

- Launch RViz, Nav2, SLAM desktop tools, Gazebo, teleop, or Tracer.
- Publish physical sensor transforms already owned by
  `socialtech_robot_description`.
- Correct Aurora mounting height in C++ vendor code.
- Carry unvalidated vendor patches.

## Upstream Driver

The vendor package found in this workspace is `slamware_ros_sdk`, provided under
`src/aurora_ros/src/slamware_ros_sdk`. Its upstream launch file is:

```text
slamware_ros_sdk/launch/slamware_ros_sdk_server_node.xml
```

That XML accepts `ip_address` and `raw_ladar_data`, but it hard-codes frame
values such as `slamware_map` and `base_link` and starts static transforms for
camera/IMU frames. For the clean SocialTech path, `aurora.launch.py` starts the
driver node directly and passes the frame contract as parameters. Use
`use_upstream_launch:=true` only for compatibility or comparison; only
`ip_address` is passed to the upstream XML.

## Install Dependencies

From the workspace root:

```bash
./src/socialtech_setup/tools/install_dependencies.sh robot
./src/socialtech_setup/tools/apply_patches.sh robot
```

## Launch

Default robot launch:

```bash
ros2 launch socialtech_robot_aurora aurora.launch.py
```

With explicit IP:

```bash
ros2 launch socialtech_robot_aurora aurora.launch.py ip_address:=192.168.11.1
```

Debug logging:

```bash
ros2 launch socialtech_robot_aurora aurora.launch.py log_level:=debug
```

Compatibility with the vendor XML:

```bash
ros2 launch socialtech_robot_aurora aurora.launch.py use_upstream_launch:=true
```

### Base frame at the axle (`aurora_base_adapter`)

The Aurora SDK gives the pose of the Aurora unit, and `slamware_ros_sdk`
publishes it as `odom -> robot_frame` without any mount offset. With the
default `aurora_base_adapter:=false`, `base_footprint` is therefore the
Aurora itself, not the axle: on Robot 2 it sits ~0.19 m ahead of it, turning
in place moves Nav2's robot origin along an arc, and every frame the URDF
hangs under `aurora_link` gets the mount applied twice (open question #16,
`socialtech_setup/docs/tf_contract.md`).

```bash
ros2 launch socialtech_robot_aurora aurora.launch.py aurora_base_adapter:=true
```

With `true` the driver publishes `odom -> aurora_pose` on
`/slamware_ros_sdk_server_node/odom_device`, and `scripts/aurora_base_adapter.py`
publishes `odom -> base_footprint` at the axle plus
`/slamware_ros_sdk_server_node/odom` (pose at the axle, twist in
`base_footprint` axes). The mount is read once from the URDF TF
(`base_footprint -> aurora_link`), so `robot_state_publisher` must be running.
`map -> slamware_laser` and `robot_pose` stay at the Aurora unit.

## Validate Network

```bash
ping 192.168.11.1
```

## Validate Topics

```bash
ros2 topic list
ros2 topic echo /scan --once
ros2 topic echo /imu --once
ros2 topic hz /scan
ros2 topic hz /imu
```

The vendor defaults are also documented in `config/topics.yaml`; inspect the
live graph because the upstream driver may publish names under
`/slamware_ros_sdk_server_node`.

## Validate TF

```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo base_link aurora_link
ros2 run tf2_ros tf2_echo aurora_link laser
ros2 run tf2_ros tf2_echo odom base_footprint
```

`map -> odom` and `odom -> base_footprint` must each have exactly one active
publisher in the selected integration mode. Physical transforms such as
`base_link -> aurora_link`, `aurora_link -> laser`, and `aurora_link -> imu_link`
belong in `socialtech_robot_description`.

## Common Errors

- Aurora is powered off or unreachable.
- The configured IP does not match the Aurora network.
- More than one node publishes `map -> odom`.
- More than one source publishes `odom -> base_footprint`.
- Physical sensor frames are duplicated by both URDF and static transforms.
- Sensor height is corrected in vendor C++ instead of in URDF/Xacro.

## Build And Smoke Test

```bash
cd ~/socialtech_ws
rosdep install \
  --from-paths \
    src/socialtech_common \
    src/socialtech_robot \
    src/tracer_ros2 \
    src/ugv_sdk \
    src/aurora_ros \
    src/ugv_gazebo_sim \
  -y --ignore-src

colcon build --packages-up-to socialtech_robot_aurora
source install/setup.bash

ros2 pkg prefix socialtech_robot_aurora
ros2 launch socialtech_robot_aurora aurora.launch.py --show-args
ros2 launch socialtech_robot_aurora aurora.launch.py
```

## From The Full Robot Bringup

`socialtech_robot_bringup`'s `robot.launch.py` includes this package behind
`use_aurora` (default `true`: Aurora is this robot's only localization
source), forwarding `ip_address`, `map_frame`, `odom_frame`, `robot_frame`,
`laser_frame`, `imu_frame` and `aurora_base_adapter` (only when
`aurora_owns_odom` is true):

```bash
ros2 launch socialtech_robot_bringup robot.launch.py
ros2 launch socialtech_robot_bringup robot.launch.py use_aurora:=false
```
