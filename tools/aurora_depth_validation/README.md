# Aurora native depth validation (diagnostic only)

Phase 1 probes for the existing Aurora Remote SDK 2.1.1 and ROS 2 Jazzy.
No OpenCV, CUDA, VPI, point-cloud publication or navigation. This directory
is excluded from colcon; build into `/tmp`, using the already installed SDK.

From `socialtech_robot` on either x86_64 or aarch64:

```bash
cmake -S tools/aurora_depth_validation -B /tmp/aurora_depth_probe_build \
  -DAURORA_SDK_ROOT="$PWD/../aurora_ros/src/aurora_remote_public" \
  -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/aurora_depth_probe_build -j2
timeout 45 /tmp/aurora_depth_probe_build/sdk_probe 192.168.11.1 15
```

The SDK probe explicitly checks `getLastDeviceBasicInfo` success before
interpreting feature bits. Failure means **unknown**, never unsupported.
It records model, firmware, feature bits, both depth-support helpers, depth
configuration, frame layouts, SDK timestamps and frequencies measured with
a local monotonic clock. It only enables depth reception for its own SDK
session; it never changes mapping/localization mode. DEPTH_MAP/POINT3D
pairs count as matched only with equal SDK timestamps and dimensions.
It does not yet validate metric accuracy or the interpretation of depth.
Exit codes: 0 presence passes, 2 query/communication/argument failure,
3 explicit unsupported capability, 4 missing/invalid frame streams.

With **one existing Aurora driver** running, no Nav2 required:

```bash
source /opt/ros/jazzy/setup.bash
source ~/socialtech_ws/install/local_setup.bash
export ROS_DOMAIN_ID=43
python3 tools/aurora_depth_validation/ros_probe.py --seconds 15
```

The ROS probe only subscribes to the already defined depth and CameraInfo
topics. Acceptance requires at least 20 exact-stamp pairs, matching frame
IDs/dimensions, finite positive focal lengths and valid 32FC1 layout.
Both probes must pass before phase 2. These are presence checks, not proof
that the calibration, depth ranges, floor geometry or obstacle detection
are correct. The timestamp age is publication-to-reception, **not sensor
latency**. Nothing converts SDK timestamps into ROS time.

Keep output under a new timestamped `socialtech_context` evidence directory.
Do not commit recorded raw image buffers. Robot motion, Nav2 and changes to
the vendor driver are outside this tool's scope.

## Phase 2: SDK convention check and physical measurements

After both phase-1 probes pass, capture at most 20 paired raw frames (at
most 2 Hz). The output directory must not exist; this avoids overwriting
measurements. No ROS PointCloud2 is created. These are temporary SDK image
buffers for diagnosis only.

```bash
timeout 45 /tmp/aurora_depth_probe_build/sdk_probe 192.168.11.1 15 \
  /tmp/aurora_depth_convention_01
python3 tools/aurora_depth_validation/analyze_capture.py \
  /tmp/aurora_depth_convention_01 --ros-probe-json /path/to/ros_phase1.json
```

The offline analyzer uses the existing NumPy installation, not OpenCV. It
honours endianness and row strides, rejects mismatched SDK timestamps,
counts invalid values and compares DEPTH_MAP against Z, Euclidean XYZ norm,
and both reconstructions from CameraInfo. `--roi X Y WIDTH HEIGHT` reports
the positive-depth distribution within a selected target region.

Do not mistake agreement between two SDK outputs for ground truth. The
analyzer deliberately leaves `phase2_pass=false`: an operator must place
and measure independent flat targets at approximately 0.5, 1.0, 1.5 and
2.0 m. Record actual distance, reference point (camera/robot surface),
target orientation, ROI and repeated measurements. Compare LiDAR only on
a surface seen by both sensors, accounting for their distinct origins and
ray directions. Document units, bias, spread and a validated operating
range before accepting phase 2. Observed min/max depths are not an accepted
range. Do not continue to TF acceptance, the production node or Nav2 while
physical measurements are missing or fail.

## Read-only physical LaserScan comparison

With the existing Aurora driver running, execute `python3 depth_lidar_probe.py
--seconds 20` from this directory and save stdout to JSON. Requires existing
ROS sensor_msgs/tf2_msgs/rosidl_runtime_py and NumPy; no new dependencies.
Records concurrent depth ROI statistics, CameraInfo, full scans at <=2 Hz,
scan frequency and existing TF messages. It publishes no TF or sensor data.
The ROI [191,137,20,20] is specific to this 416x224 validation capture;
verify CameraInfo/principal point before reusing it with another calibration.
Laser statistics use +/-10 degrees about the scan frame +X and include
both radial range and x=range*cos(angle). Those axes/origins are not assumed
equal to depth axes/origins. Nonfinite scan values are retained as strings.
Keep wall and robot stationary; publication stamps do not prove acquisition
synchronization. Exact alignment and metric acceptance remain pending.
