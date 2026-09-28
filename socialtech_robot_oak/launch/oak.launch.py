from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode, ParameterFile
from launch_ros.substitutions import FindPackageShare


# Verified against depthai-ros@v3_jazzy (the branch pinned in
# socialtech_setup/tools/install_dependencies.sh, DEPTHAI_ROS_BRANCH). If
# that branch changes, re-check package names, the composable node plugin,
# and the launch files below with a throwaway clone of
# github.com/luxonis/depthai-ros before relying on this file again.
DRIVER_PACKAGE = "depthai_ros_driver_v3"
DRIVER_PLUGIN = "depthai_ros_driver::Driver"


def _launch_setup(context):
    camera_name = LaunchConfiguration("camera_name").perform(context)
    parent_frame = LaunchConfiguration("parent_frame").perform(context)
    params_file = ParameterFile(LaunchConfiguration("params_file"), allow_substs=True)

    # driver_as_part_of_a_robot.launch.py (the vendor's own "clean" entry
    # point for this exact use case) exposes pointcloud/color/depth toggles
    # only through params_file. We replicate its ComposableNodeContainer
    # directly, the same way socialtech_robot_aurora starts its Node
    # directly, so we can layer a pointcloud_enable launch arg on top
    # without depending on a richer upstream launch file that may change.
    parameter_overrides = {}
    if LaunchConfiguration("pointcloud_enable").perform(context) == "true":
        parameter_overrides["pipeline_gen"] = {"i_enable_rgbd": True}

    # Forward our own parent_frame/camera_name into the driver's TF
    # parameters (declared as "driver.i_tf_parent_frame"/"i_tf_base_frame"/
    # "i_tf_device_name" -- BaseParamHandler prefixes every DriverParamHandler
    # param with "driver."). Without this, the driver keeps its own
    # defaults (i_tf_parent_frame="oak_parent_frame", a frame nobody
    # publishes) instead of "oak_link", and ~1s after start
    # depthai_bridge::TFPublisher remotely overwrites the urdf_launch.py
    # robot_state_publisher's robot_description and reparents "oak" away
    # from oak_link to that unpublished frame -- silently orphaning the
    # whole OAK TF branch. i_publish_tf_from_calibration is left at its
    # vendor default (true): it gives real calibration-derived camera/imu
    # extrinsics, more accurate than any offset we could put in a generic
    # xacro, as long as it agrees with urdf_launch.py on oak_link/oak.
    parameter_overrides["driver"] = {
        "i_tf_parent_frame": parent_frame,
        "i_tf_base_frame": camera_name,
        "i_tf_device_name": camera_name,
    }

    return [
        ComposableNodeContainer(
            name=f"{camera_name}_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container",
            composable_node_descriptions=[
                ComposableNode(
                    package=DRIVER_PACKAGE,
                    plugin=DRIVER_PLUGIN,
                    name=camera_name,
                    parameters=[params_file, parameter_overrides],
                )
            ],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
            output="both",
            condition=UnlessCondition(LaunchConfiguration("use_upstream_launch")),
        ),
    ]


def generate_launch_description():
    camera_name = LaunchConfiguration("camera_name")
    camera_model = LaunchConfiguration("camera_model")
    parent_frame = LaunchConfiguration("parent_frame")
    publish_camera_urdf = LaunchConfiguration("publish_camera_urdf")
    use_upstream_launch = LaunchConfiguration("use_upstream_launch")

    # The vendor launch file written specifically for "driver as part of a
    # robot": unlike driver.launch.py, it does not embed its own
    # robot_state_publisher, matching this repo's policy that physical
    # frames only come from socialtech_robot_description. Included here
    # only for compatibility/debug, same role as Aurora's
    # use_upstream_launch.
    upstream_launch = PathJoinSubstitution([
        FindPackageShare(DRIVER_PACKAGE),
        "launch",
        "driver_as_part_of_a_robot.launch.py",
    ])

    # depthai_descriptions_v3 publishes the camera-internal frames
    # (rgb/left/right camera optical frames, imu_frame) and their visual
    # meshes. socialtech_robot_description only owns the physical bracket,
    # oak_link, so this is the frame depthai_descriptions_v3 attaches to.
    # cam_pos_*/cam_roll/pitch/yaw stay at zero here: the real-world mount
    # offset already lives in socialtech_robot_description/config/oak_mounts.
    #
    # Do not enable this at the same time as description.launch.py's
    # oak_include_upstream_urdf:=true: both would publish the same
    # camera-internal frames from two different robot_state_publisher
    # instances.
    urdf_launch = PathJoinSubstitution([
        FindPackageShare("depthai_descriptions_v3"),
        "launch",
        "urdf_launch.py",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "camera_name",
            default_value="oak",
            description="Node/topic name for the OAK camera. Must match the top-level key in params_file.",
        ),
        DeclareLaunchArgument(
            "camera_model",
            default_value="OAK-D-PRO",
            description="depthai camera model string. Verify against the physical unit before deployment.",
        ),
        DeclareLaunchArgument(
            "parent_frame",
            default_value="oak_link",
            description=(
                "Physical mount frame published by socialtech_robot_description "
                "when use_oak:=true (child of base_link, independent from "
                "aurora_link). Forwarded to both urdf_launch.py and the driver's "
                "own i_tf_parent_frame parameter so they agree on the same "
                "parent. Do not change unless the description package "
                "oak_mounts output_frame is also changed."
            ),
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("socialtech_robot_oak"),
                "config",
                "oak.yaml",
            ]),
            description=f"{DRIVER_PACKAGE} parameter overrides for the OAK camera.",
        ),
        DeclareLaunchArgument(
            "pointcloud_enable",
            default_value="false",
            description="Enable the depthai RGBD pointcloud. High bandwidth/CPU: validate before enabling.",
        ),
        DeclareLaunchArgument(
            "publish_camera_urdf",
            default_value="true",
            description=(
                "Include the upstream depthai_descriptions_v3 URDF/robot_state_publisher "
                "for the camera-internal frames. Set to false if "
                "socialtech_robot_description was already launched with "
                "oak_include_upstream_urdf:=true, to avoid two robot_state_publisher "
                "instances publishing the same camera-internal frames."
            ),
        ),
        DeclareLaunchArgument(
            "use_upstream_launch",
            default_value="false",
            description=(
                f"Include {DRIVER_PACKAGE}'s own driver_as_part_of_a_robot.launch.py "
                "instead of the direct ComposableNodeContainer here, for "
                "compatibility/debug. Only camera_name, camera_model, parent_frame, "
                "and params_file are passed through."
            ),
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="ROS log level for the OAK camera container.",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(upstream_launch),
            condition=IfCondition(use_upstream_launch),
            launch_arguments={
                "name": camera_name,
                "camera_model": camera_model,
                "parent_frame": parent_frame,
                "params_file": LaunchConfiguration("params_file"),
            }.items(),
        ),
        OpaqueFunction(function=_launch_setup),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(urdf_launch),
            condition=IfCondition(publish_camera_urdf),
            launch_arguments={
                "tf_prefix": camera_name,
                "base_frame": camera_name,
                "camera_model": camera_model,
                "parent_frame": parent_frame,
                "cam_pos_x": "0.0",
                "cam_pos_y": "0.0",
                "cam_pos_z": "0.0",
                "cam_roll": "0.0",
                "cam_pitch": "0.0",
                "cam_yaw": "0.0",
            }.items(),
        ),
    ])
