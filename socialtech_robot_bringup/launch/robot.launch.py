"""Single entry point for real-robot bringup: Tracer + description + Aurora
(on by default) + OAK-D-PRO (opt-in) + OAK obstacle point cloud (opt-in).

Replaces the previous robot.launch.py / robot_aurora.launch.py /
robot_oak_aurora.launch.py trio -- those three duplicated the same ~10
Tracer arguments three times over, each file a superset of the last, with
no single obvious "the" launch file to use. This is that one file.

use_aurora defaults to true: Aurora is this robot's only localization
source (no AMCL), so a normal run needs it. Set use_aurora:=false only to
bring up Tracer (+ OAK, if wanted) without it -- e.g. diagnosing Tracer or
OAK on their own while Aurora is unreachable/disconnected.

use_oak defaults to false: not every robot run needs the camera, and it is
optional hardware.

oak_pointcloud_processing defaults to false and requires use_oak:=true to
do anything. It does NOT enable depthai_ros_driver_v3's own RGBD/
pointcloud_enable path (oak_pointcloud_enable below, also default false) --
that path's internal color/depth Sync does not converge on this robot
(rate collapses to ~1.6-1.8 Hz, open upstream bug
github.com/luxonis/depthai-ros/issues/559, confirmed on the real robot
2026-09-02/03). Instead it loads depth_image_proc::PointCloudXyzNode
(depth-only, no color/intensity input, so no cross-stream sync to fail)
into the OAK driver's own composable node container, turning
/oak/stereo/image_raw + /oak/stereo/camera_info into /oak/points --
confirmed live: 0 sync warnings, ~16-19 Hz, but ~64-67 MB/s at the driver's
default 640x400 depth resolution, far above every other obstacle source in
this workspace. This is what socialtech_robot_navigation's
local_costmap_sources/oak_point_cloud.yaml (in socialtech_desktop, the
operator-PC repo) expects to subscribe to. Deliberately not part of
socialtech_robot_oak: that package's own README scopes it to camera
bringup only (depthai_ros_driver_v3), not obstacle-source processing --
depth_image_proc is a different library entirely. This bringup package is
what decides which sensors run together for a given robot configuration,
so the toggle lives here.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    tracer_config = LaunchConfiguration("tracer_config")
    can_interface = LaunchConfiguration("can_interface")
    port_name = LaunchConfiguration("port_name")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_description = LaunchConfiguration("use_description")
    use_aurora = LaunchConfiguration("use_aurora")
    aurora_owns_odom = LaunchConfiguration("aurora_owns_odom")
    odom_frame = LaunchConfiguration("odom_frame")
    base_frame = LaunchConfiguration("base_frame")
    odom_topic_name = LaunchConfiguration("odom_topic_name")
    is_tracer_mini = LaunchConfiguration("is_tracer_mini")
    simulated_robot = LaunchConfiguration("simulated_robot")
    control_rate = LaunchConfiguration("control_rate")
    aurora_ip_address = LaunchConfiguration("aurora_ip_address")
    aurora_map_frame = LaunchConfiguration("aurora_map_frame")
    aurora_odom_frame = LaunchConfiguration("aurora_odom_frame")
    aurora_robot_frame = LaunchConfiguration("aurora_robot_frame")
    aurora_laser_frame = LaunchConfiguration("aurora_laser_frame")
    aurora_imu_frame = LaunchConfiguration("aurora_imu_frame")
    aurora_use_upstream_launch = LaunchConfiguration("aurora_use_upstream_launch")
    aurora_publish_map_to_odom_static = LaunchConfiguration("aurora_publish_map_to_odom_static")
    aurora_log_level = LaunchConfiguration("aurora_log_level")
    aurora_base_adapter = LaunchConfiguration("aurora_base_adapter")
    use_oak = LaunchConfiguration("use_oak")
    mount_profile = LaunchConfiguration("mount_profile")
    oak_mount_profile = LaunchConfiguration("oak_mount_profile")
    oak_camera_model = LaunchConfiguration("oak_camera_model")
    oak_pointcloud_enable = LaunchConfiguration("oak_pointcloud_enable")
    oak_include_upstream_urdf = LaunchConfiguration("oak_include_upstream_urdf")
    oak_pointcloud_processing = LaunchConfiguration("oak_pointcloud_processing")

    # If the description package already publishes the OAK camera-internal
    # frames (oak_include_upstream_urdf:=true), the driver launch must not
    # publish them a second time -- both would start a robot_state_publisher
    # for the same frames, and whichever one is not namespaced away colliding
    # on the global /robot_description topic (confirmed live 2026-09-03: two
    # publishers on /robot_description, ambiguous which one a subscriber
    # actually gets). oak_include_upstream_urdf:=true is the single-publisher
    # path and is recommended over the driver's own calibration-based one.
    oak_publish_camera_urdf = PythonExpression([
        "'false' if '", oak_include_upstream_urdf, "' == 'true' else 'true'",
    ])

    tracer_odom_topic_name = PythonExpression([
        "'", odom_topic_name, "' if '", odom_topic_name, "' != '' else (",
        "'tracer/odom' if '", use_aurora, "' == 'true' and '",
        aurora_owns_odom, "' == 'true' else '')",
    ])
    # base_frame stays "base_footprint" (from tracer.yaml) regardless of who
    # owns odom -> base_footprint: publish_odom_tf, not frame renaming, is
    # what prevents two publishers of that transform. Requires the
    # socialtech_setup/patches/tracer_ros2 publish_odom_tf patch.
    tracer_publish_odom_tf = PythonExpression([
        "'false' if '", use_aurora, "' == 'true' and '",
        aurora_owns_odom, "' == 'true' else ''",
    ])
    # The adapter publishes odom -> base_footprint itself, so it only makes
    # sense while Aurora owns odom; otherwise Tracer does and two publishers
    # of the same transform would appear.
    resolved_aurora_base_adapter = PythonExpression([
        "'true' if '", aurora_base_adapter, "' == 'true' and '",
        aurora_owns_odom, "' == 'true' else 'false'",
    ])
    resolved_aurora_robot_frame = PythonExpression([
        "'", aurora_robot_frame, "' if '", aurora_robot_frame, "' != '' else (",
        "'base_footprint' if '", aurora_owns_odom, "' == 'true' else 'slamware_base')",
    ])
    run_oak_pointcloud_processing = PythonExpression([
        "'", use_oak, "' == 'true' and '", oak_pointcloud_processing, "' == 'true'",
    ])

    bringup_share = FindPackageShare("socialtech_robot_bringup")
    tracer_launch = PathJoinSubstitution([
        bringup_share,
        "launch",
        "tracer.launch.py",
    ])

    # This description launch is intentionally lightweight: it only starts
    # robot_state_publisher from socialtech_robot_description and does not open RViz.
    description_launch = PathJoinSubstitution([
        FindPackageShare("socialtech_robot_description"),
        "launch",
        "description.launch.py",
    ])

    aurora_launch = PathJoinSubstitution([
        FindPackageShare("socialtech_robot_aurora"),
        "launch",
        "aurora.launch.py",
    ])

    oak_launch = PathJoinSubstitution([
        FindPackageShare("socialtech_robot_oak"),
        "launch",
        "oak.launch.py",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "tracer_config",
            default_value=PathJoinSubstitution([
                FindPackageShare("socialtech_robot_bringup"),
                "config",
                "tracer.yaml",
            ]),
            description="YAML file with Tracer bringup defaults.",
        ),
        DeclareLaunchArgument(
            "can_interface",
            default_value="",
            description="Alias for port_name. Empty uses port_name/can_interface from YAML.",
        ),
        DeclareLaunchArgument(
            "port_name",
            default_value="",
            description="Override tracer_config port_name. Empty uses the YAML value.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="",
            description="Override tracer_config use_sim_time. Empty uses the YAML value.",
        ),
        DeclareLaunchArgument(
            "use_description",
            default_value="true",
            description="Start robot_state_publisher from socialtech_robot_description.",
        ),
        DeclareLaunchArgument(
            "use_aurora",
            default_value="true",
            description=(
                "Start the SLAMTEC Aurora driver via socialtech_robot_aurora. "
                "Aurora is this robot's only localization source (no AMCL), so "
                "normal runs need it. Set to false only to bring up Tracer "
                "(and optionally OAK) on their own, e.g. while Aurora is "
                "disconnected/unreachable."
            ),
        ),
        DeclareLaunchArgument(
            "aurora_owns_odom",
            default_value="true",
            description=(
                "When use_aurora is true, let Aurora own odom -> base_footprint "
                "and disable Tracer's own odom TF broadcast (publish_odom_tf:=false), "
                "keeping only its /tracer/odom topic for diagnostics."
            ),
        ),
        DeclareLaunchArgument(
            "odom_frame",
            default_value="",
            description="Override tracer_config odom_frame. Empty uses the YAML value.",
        ),
        DeclareLaunchArgument(
            "base_frame",
            default_value="",
            description="Override tracer_config base_frame. Empty uses the YAML value.",
        ),
        DeclareLaunchArgument(
            "odom_topic_name",
            default_value="",
            description="Override tracer_config odom_topic_name. Empty uses the YAML value.",
        ),
        DeclareLaunchArgument(
            "is_tracer_mini",
            default_value="",
            description="Override tracer_config is_tracer_mini. Empty uses the YAML value.",
        ),
        DeclareLaunchArgument(
            "simulated_robot",
            default_value="",
            description="Override tracer_config simulated_robot. Empty uses the YAML value.",
        ),
        DeclareLaunchArgument(
            "control_rate",
            default_value="",
            description="Override tracer_config control_rate. Empty uses the YAML value.",
        ),
        DeclareLaunchArgument(
            "aurora_ip_address",
            default_value="192.168.11.1",
            description="IPv4 address of the SLAMTEC Aurora device.",
        ),
        DeclareLaunchArgument(
            "aurora_map_frame",
            default_value="map",
            description="Map frame passed to socialtech_robot_aurora.",
        ),
        DeclareLaunchArgument(
            "aurora_odom_frame",
            default_value="odom",
            description="Odometry frame passed to socialtech_robot_aurora.",
        ),
        DeclareLaunchArgument(
            "aurora_robot_frame",
            default_value="",
            description=(
                "Robot frame passed to socialtech_robot_aurora. Empty selects "
                "base_footprint when aurora_owns_odom is true, otherwise slamware_base."
            ),
        ),
        DeclareLaunchArgument(
            "aurora_laser_frame",
            default_value="slamware_laser",
            description="Laser frame passed to socialtech_robot_aurora.",
        ),
        DeclareLaunchArgument(
            "aurora_imu_frame",
            default_value="imu_link",
            description="IMU frame passed to socialtech_robot_aurora.",
        ),
        DeclareLaunchArgument(
            "aurora_use_upstream_launch",
            default_value="false",
            description="Use the vendor Aurora XML launch for compatibility/debug.",
        ),
        DeclareLaunchArgument(
            "aurora_publish_map_to_odom_static",
            default_value="true",
            description="Publish identity map -> odom from the Aurora wrapper.",
        ),
        DeclareLaunchArgument(
            "aurora_log_level",
            default_value="info",
            description="ROS log level for the Aurora driver node.",
        ),
        DeclareLaunchArgument(
            "aurora_base_adapter",
            default_value="false",
            description=(
                "Put base_footprint at the axle instead of at the Aurora unit "
                "(socialtech_robot_aurora aurora_base_adapter.py, open question #16). "
                "Only applied when aurora_owns_odom is true. Robot profiles set it "
                "with AURORA_BASE_ADAPTER (socialtech_setup/robots)."
            ),
        ),
        DeclareLaunchArgument(
            "use_oak",
            default_value="false",
            description="Add the OAK-D-PRO mount frame to the description and start socialtech_robot_oak.",
        ),
        DeclareLaunchArgument(
            "mount_profile",
            default_value="default",
            description="Aurora mount profile from socialtech_robot_description/config/aurora_mounts.",
        ),
        DeclareLaunchArgument(
            "oak_mount_profile",
            default_value="default",
            description="OAK mount profile from socialtech_robot_description/config/oak_mounts.",
        ),
        DeclareLaunchArgument(
            "oak_camera_model",
            default_value="OAK-D-PRO",
            description="depthai camera model string passed to socialtech_robot_oak.",
        ),
        DeclareLaunchArgument(
            "oak_pointcloud_enable",
            default_value="false",
            description=(
                "Enable depthai_ros_driver_v3's own RGBD/pointcloud path "
                "(/oak/rgbd/points). NOT RECOMMENDED: its internal color/depth "
                "Sync does not converge on this robot (open upstream bug, see "
                "this file's docstring) -- rate collapses to ~1.6-1.8 Hz. Use "
                "oak_pointcloud_processing instead. Kept as an explicit opt-in "
                "for debugging/comparison only."
            ),
        ),
        DeclareLaunchArgument(
            "oak_include_upstream_urdf",
            default_value="false",
            description=(
                "Attach the upstream depthai_descriptions_v3 visual mesh and camera-internal "
                "frames to oak_link from the description package instead of the driver "
                "launch. Verified against depthai-ros@v3_jazzy: see "
                "socialtech_robot_description/urdf/oak_mount.xacro if that branch changes."
            ),
        ),
        DeclareLaunchArgument(
            "oak_pointcloud_processing",
            default_value="false",
            description=(
                "Load depth_image_proc::PointCloudXyzNode into the OAK driver's "
                "composable node container, publishing /oak/points from "
                "/oak/stereo/image_raw + /oak/stereo/camera_info (depth only, no "
                "color -- avoids oak_pointcloud_enable's Sync bug). This is what "
                "socialtech_robot_navigation's local_costmap_sources/"
                "oak_point_cloud.yaml expects. Requires use_oak:=true; does "
                "nothing otherwise. ~64-67 MB/s at the driver's default 640x400 "
                "depth resolution -- measure before trusting this over Aurora's "
                "scan as the default obstacle source."
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tracer_launch),
            launch_arguments={
                "tracer_config": tracer_config,
                "can_interface": can_interface,
                "port_name": port_name,
                "use_sim_time": use_sim_time,
                "odom_frame": odom_frame,
                "base_frame": base_frame,
                "odom_topic_name": tracer_odom_topic_name,
                "publish_odom_tf": tracer_publish_odom_tf,
                "is_tracer_mini": is_tracer_mini,
                "simulated_robot": simulated_robot,
                "control_rate": control_rate,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch),
            condition=IfCondition(use_description),
            launch_arguments={
                "mount_profile": mount_profile,
                "use_oak": use_oak,
                "oak_mount_profile": oak_mount_profile,
                "oak_include_upstream_urdf": oak_include_upstream_urdf,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(aurora_launch),
            condition=IfCondition(use_aurora),
            launch_arguments={
                "ip_address": aurora_ip_address,
                "map_frame": aurora_map_frame,
                "odom_frame": aurora_odom_frame,
                "robot_frame": resolved_aurora_robot_frame,
                "laser_frame": aurora_laser_frame,
                "imu_frame": aurora_imu_frame,
                "use_upstream_launch": aurora_use_upstream_launch,
                "publish_map_to_odom_static": aurora_publish_map_to_odom_static,
                "log_level": aurora_log_level,
                "aurora_base_adapter": resolved_aurora_base_adapter,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(oak_launch),
            condition=IfCondition(use_oak),
            launch_arguments={
                "camera_model": oak_camera_model,
                "pointcloud_enable": oak_pointcloud_enable,
                "publish_camera_urdf": oak_publish_camera_urdf,
            }.items(),
        ),
        LoadComposableNodes(
            condition=IfCondition(run_oak_pointcloud_processing),
            target_container="/oak_container",
            composable_node_descriptions=[
                ComposableNode(
                    package="depth_image_proc",
                    plugin="depth_image_proc::PointCloudXyzNode",
                    name="point_cloud_xyz_node",
                    remappings=[
                        ("image_rect", "/oak/stereo/image_raw"),
                        ("camera_info", "/oak/stereo/camera_info"),
                        ("points", "/oak/points"),
                    ],
                ),
            ],
        ),
    ])
