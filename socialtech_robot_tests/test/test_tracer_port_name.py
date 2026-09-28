"""
Check that the Tracer CAN interface reaches tracer_base.launch.py.

robot.launch.py -> tracer.launch.py -> tracer_base.launch.py (upstream), whose
port_name launch argument becomes the tracer_base_node parameter. This runs
tracer.launch.py's OpaqueFunction setup against a LaunchContext, without
starting any node, and reads the port_name handed to the upstream include.
"""

import ast
import importlib.util

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import IncludeLaunchDescription
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions

BRINGUP_PKG = 'socialtech_robot_bringup'


def _load_tracer_launch_module():
    share = get_package_share_directory(BRINGUP_PKG)
    spec = importlib.util.spec_from_file_location(
        'tracer_launch', f'{share}/launch/tracer.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module, share


def _resolved_port_name(port_name_argument):
    module, share = _load_tracer_launch_module()
    context = LaunchContext()
    context.launch_configurations.update({
        'tracer_config': f'{share}/config/tracer.yaml',
        'can_interface': '',
        'port_name': port_name_argument,
        'use_sim_time': '',
        'odom_frame': '',
        'base_frame': '',
        'odom_topic_name': '',
        'publish_odom_tf': '',
        'is_tracer_mini': '',
        'simulated_robot': '',
        'control_rate': '',
    })
    actions = module._launch_setup(context)
    includes = [a for a in actions if isinstance(a, IncludeLaunchDescription)]
    assert len(includes) == 1
    for name, value in includes[0].launch_arguments:
        if name == 'port_name':
            return perform_substitutions(context, normalize_to_list_of_substitutions(value))
    raise AssertionError('tracer.launch.py does not forward port_name')


def test_port_name_argument_reaches_tracer_base():
    assert _resolved_port_name('can_test') == 'can_test'


def test_port_name_defaults_to_tracer_yaml():
    assert _resolved_port_name('') == 'can2'


def test_robot_launch_forwards_port_name_to_tracer_launch():
    share = get_package_share_directory(BRINGUP_PKG)
    with open(f'{share}/launch/robot.launch.py') as f:
        tree = ast.parse(f.read())
    forwarded = set()
    for node in ast.walk(tree):
        is_include = (
            isinstance(node, ast.Call)
            and getattr(node.func, 'id', None) == 'IncludeLaunchDescription'
        )
        if not is_include:
            continue
        if not any(isinstance(n, ast.Name) and n.id == 'tracer_launch' for n in ast.walk(node)):
            continue
        for kw in node.keywords:
            if kw.arg == 'launch_arguments':
                dict_node = kw.value.func.value
                forwarded.update(k.value for k in dict_node.keys if isinstance(k, ast.Constant))
    assert 'port_name' in forwarded
