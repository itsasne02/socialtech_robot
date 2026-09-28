from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare

try:
    import yaml
except ImportError:
    yaml = None


def _load_tracer_config(config_path):
    if yaml is None:
        raise RuntimeError(
            "The python3-yaml dependency is required to read tracer_config files."
        )

    path = Path(config_path).expanduser()
    if not path.exists():
        raise FileNotFoundError(f"tracer_config does not exist: {path}")

    data = yaml.safe_load(path.read_text()) or {}
    return data.get("tracer", {}).get("ros__parameters", {}) or {}


def _configured_value(context, params, launch_argument, yaml_key, fallback):
    launch_value = LaunchConfiguration(launch_argument).perform(context)
    if launch_value:
        return launch_value
    return str(params.get(yaml_key, fallback))


def _configured_port_name(context, params):
    port_name = LaunchConfiguration("port_name").perform(context)
    if port_name:
        return port_name

    can_interface = LaunchConfiguration("can_interface").perform(context)
    if can_interface:
        return can_interface

    return str(params.get("port_name", params.get("can_interface", "can0")))


def _launch_setup(context):
    params = _load_tracer_config(LaunchConfiguration("tracer_config").perform(context))

    use_sim_time = _configured_value(context, params, "use_sim_time", "use_sim_time", "false")
    port_name = _configured_port_name(context, params)
    odom_frame = _configured_value(context, params, "odom_frame", "odom_frame", "odom")
    base_frame = _configured_value(context, params, "base_frame", "base_frame", "base_link")
    odom_topic_name = _configured_value(
        context,
        params,
        "odom_topic_name",
        "odom_topic_name",
        "odom",
    )
    publish_odom_tf = _configured_value(
        context,
        params,
        "publish_odom_tf",
        "publish_odom_tf",
        "true",
    )
    is_tracer_mini = _configured_value(
        context,
        params,
        "is_tracer_mini",
        "is_tracer_mini",
        "false",
    )
    simulated_robot = _configured_value(
        context,
        params,
        "simulated_robot",
        "simulated_robot",
        "false",
    )
    control_rate = _configured_value(context, params, "control_rate", "control_rate", "50")

    tracer_base_launch = PathJoinSubstitution([
        FindPackageShare("tracer_base"),
        "launch",
        "tracer_base.launch.py",
    ])

    # Keep tracer_base as an external dependency: this launch reads the
    # SocialTech YAML contract and maps can_interface to the upstream port_name.
    return [
        LogInfo(msg=[
            "Including upstream tracer_base.launch.py with port_name: ",
            port_name,
        ]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tracer_base_launch),
            launch_arguments={
                "use_sim_time": TextSubstitution(text=use_sim_time),
                "port_name": TextSubstitution(text=port_name),
                "odom_frame": TextSubstitution(text=odom_frame),
                "base_frame": TextSubstitution(text=base_frame),
                "odom_topic_name": TextSubstitution(text=odom_topic_name),
                "publish_odom_tf": TextSubstitution(text=publish_odom_tf),
                "is_tracer_mini": TextSubstitution(text=is_tracer_mini),
                "simulated_robot": TextSubstitution(text=simulated_robot),
                "control_rate": TextSubstitution(text=control_rate),
            }.items(),
        )
    ]


def generate_launch_description():
    default_tracer_config = PathJoinSubstitution([
        FindPackageShare("socialtech_robot_bringup"),
        "config",
        "tracer.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "tracer_config",
            default_value=default_tracer_config,
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
            "publish_odom_tf",
            default_value="",
            description=(
                "Override tracer_config publish_odom_tf. Empty uses the YAML value. "
                "Set false when another source (e.g. Aurora) owns odom -> base_footprint; "
                "the /odom topic keeps publishing either way. Requires the "
                "socialtech_setup/patches/tracer_ros2 publish_odom_tf patch."
            ),
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
        OpaqueFunction(function=_launch_setup),
    ])
