from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


AURORA_POSE_FRAME = "aurora_pose"
ODOM_TOPIC = "/slamware_ros_sdk_server_node/odom"
DEVICE_ODOM_TOPIC = "/slamware_ros_sdk_server_node/odom_device"


def generate_launch_description():
    ip_address = LaunchConfiguration("ip_address")
    map_frame = LaunchConfiguration("map_frame")
    odom_frame = LaunchConfiguration("odom_frame")
    robot_frame = LaunchConfiguration("robot_frame")
    laser_frame = LaunchConfiguration("laser_frame")
    imu_frame = LaunchConfiguration("imu_frame")
    use_upstream_launch = LaunchConfiguration("use_upstream_launch")
    publish_map_to_odom_static = LaunchConfiguration("publish_map_to_odom_static")
    log_level = LaunchConfiguration("log_level")
    aurora_base_adapter = LaunchConfiguration("aurora_base_adapter")
    enable_dense_point_cloud = LaunchConfiguration("enable_dense_point_cloud")
    dense_point_cloud_stride = LaunchConfiguration("dense_point_cloud_stride")

    # With the adapter the driver keeps the SDK pose on its own frame and
    # topic, and aurora_base_adapter.py publishes robot_frame at the axle
    # (open question #16, socialtech_setup/docs/tf_contract.md).
    driver_robot_frame = PythonExpression([
        "'", AURORA_POSE_FRAME, "' if '", aurora_base_adapter, "' == 'true' else '", robot_frame, "'",
    ])
    driver_odom_topic = PythonExpression([
        "'", DEVICE_ODOM_TOPIC, "' if '", aurora_base_adapter, "' == 'true' else '", ODOM_TOPIC, "'",
    ])
    run_adapter = PythonExpression([
        "'", aurora_base_adapter, "' == 'true' and '", use_upstream_launch, "' == 'false'",
    ])

    upstream_launch = PathJoinSubstitution([
        FindPackageShare("slamware_ros_sdk"),
        "launch",
        "slamware_ros_sdk_server_node.xml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "ip_address",
            default_value="192.168.11.1",
            description="IPv4 address of the SLAMTEC Aurora device on the robot network.",
        ),
        DeclareLaunchArgument(
            "map_frame",
            default_value="map",
            description="Global frame used by Aurora when the direct Node wrapper is active.",
        ),
        DeclareLaunchArgument(
            "odom_frame",
            default_value="odom",
            description="Odometry frame used by Aurora when the direct Node wrapper is active.",
        ),
        DeclareLaunchArgument(
            "robot_frame",
            default_value="base_footprint",
            description="Robot frame used by Aurora when the direct Node wrapper is active.",
        ),
        DeclareLaunchArgument(
            "laser_frame",
            default_value="slamware_laser",
            description=(
                "Frame used by slamware_ros_sdk for scan messages. The SDK "
                "broadcasts this dynamically itself (in practice as a child of "
                "map_frame, not of robot_frame -- see server_workers.cpp "
                "ServerLaserScanWorker). Keep it outside the URDF physical tree: "
                "aurora_link must never also define a frame with this name, or "
                "it would have two TF parents."
            ),
        ),
        DeclareLaunchArgument(
            "imu_frame",
            default_value="imu_link",
            description="IMU frame expected from the robot URDF/Xacro.",
        ),
        DeclareLaunchArgument(
            "use_upstream_launch",
            default_value="false",
            description=(
                "Include slamware_ros_sdk_server_node.xml instead of the clean wrapper. "
                "Only ip_address is passed because that XML does not expose frame arguments."
            ),
        ),
        DeclareLaunchArgument(
            "publish_map_to_odom_static",
            default_value="true",
            description=(
                "PROVISIONAL / PENDING VALIDATION, not a confirmed-correct "
                "architecture: publishes an identity map -> odom transform. "
                "Aurora's SDK has no real local odometry (ServerOdometryWorker "
                "republishes the same absolute VSLAM pose used for the map-frame "
                "robot_pose topic under the odom_frame->robot_frame TF label), so "
                "any loop-closure/relocalization/global-bundle-adjustment jump "
                "already propagates into odom -> base_footprint, not just into "
                "map -> odom as REP-105 would expect. This identity transform is "
                "a placeholder kept until that is validated (or replaced by a "
                "proper localizer/adapter node) -- see "
                "socialtech_setup/docs/tf_contract.md. Set false if another "
                "localization node owns map -> odom."
            ),
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="ROS log level for the Aurora driver node.",
        ),
        DeclareLaunchArgument(
            "aurora_base_adapter",
            default_value="false",
            description=(
                "Put robot_frame at the robot's axle instead of at the Aurora unit. "
                "The SDK pose is the pose of the Aurora itself and the driver applies "
                "no mount offset, so with false robot_frame (base_footprint) is the "
                "Aurora. With true the driver publishes odom_frame -> aurora_pose on "
                "/slamware_ros_sdk_server_node/odom_device and aurora_base_adapter.py "
                "publishes odom_frame -> robot_frame and the usual odom topic, removing "
                "the mount read from the URDF (robot_frame -> aurora_link). Needs "
                "robot_state_publisher. Ignored with use_upstream_launch."
            ),
        ),
        DeclareLaunchArgument(
            "enable_dense_point_cloud",
            default_value="false",
            description=(
                "Publish the depth camera's organized point cloud "
                "(/slamware_ros_sdk_server_node/dense_point_cloud, "
                "camera_depth_optical_frame; aurora_ros patch 0006). Computed only "
                "while someone subscribes. Off by default: the depth was found "
                "systematically wrong on Robot 1 (06_perception_modes.md, Modo F); "
                "enable it to measure, not to navigate."
            ),
        ),
        DeclareLaunchArgument(
            "dense_point_cloud_stride",
            default_value="1",
            description="Keep every stride-th row/column of the dense point cloud (3 = ~1/9 of the points).",
        ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(upstream_launch),
            condition=IfCondition(use_upstream_launch),
            # The upstream XML exposes only ip_address/raw_ladar_data as launch
            # arguments; frame arguments below are available in the clean Node path.
            launch_arguments={
                "ip_address": ip_address,
            }.items(),
        ),
        Node(
            package="slamware_ros_sdk",
            executable="slamware_ros_sdk_server_node",
            name="slamware_ros_sdk_server_node",
            output="both",
            condition=UnlessCondition(use_upstream_launch),
            parameters=[{
                "ip_address": ip_address,
                "angle_compensate": True,
                "map_frame": map_frame,
                "robot_frame": driver_robot_frame,
                "odom_topic": driver_odom_topic,
                "enable_dense_point_cloud": enable_dense_point_cloud,
                "dense_point_cloud_stride": dense_point_cloud_stride,
                "odom_frame": odom_frame,
                "laser_frame": laser_frame,
                "imu_frame": imu_frame,
                "camera_left": "camera_left",
                "camera_right": "camera_right",
                "robot_pose_pub_period": 0.05,
                "scan_pub_period": 0.1,
                "map_pub_period": 0.2,
                "imu_raw_data_period": 0.005,
                "ladar_data_clockwise": True,
                "no_preview_image": False,
                "raw_image_on": False,
            }],
            arguments=["--ros-args", "--log-level", log_level],
        ),
        Node(
            package="socialtech_robot_aurora",
            executable="aurora_base_adapter.py",
            name="aurora_base_adapter",
            output="both",
            condition=IfCondition(run_adapter),
            parameters=[{
                "parent_frame": odom_frame,
                "child_frame": robot_frame,
                "mount_frame": "aurora_link",
                "input_topic": DEVICE_ODOM_TOPIC,
                "output_topic": ODOM_TOPIC,
            }],
            arguments=["--ros-args", "--log-level", log_level],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_odom_static",
            output="screen",
            condition=IfCondition(PythonExpression([
                "'", publish_map_to_odom_static, "' == 'true' and '",
                use_upstream_launch, "' == 'false'",
            ])),
            arguments=[
                "0", "0", "0",
                "0", "0", "0", "1",
                map_frame,
                odom_frame,
            ],
        ),
    ])
