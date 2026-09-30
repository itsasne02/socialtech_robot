"""
Guard against a class of "argument silently ignored" launch bug.

This audit found robot.launch.py forwarding use_oak/
oak_mount_profile/oak_include_upstream_urdf into description.launch.py,
which used to not declare them at all (ros2 launch does not error on
unused launch_arguments, it just silently drops them). This statically
parses both launch files' source with ast, without executing them or
requiring a sourced ROS environment, and fails if a forwarded argument is
ever not declared downstream again.
"""

import ast

from ament_index_python.packages import get_package_share_directory

DESCRIPTION_PKG = 'socialtech_robot_description'
BRINGUP_PKG = 'socialtech_robot_bringup'
AURORA_PKG = 'socialtech_robot_aurora'
DEPTH_SCAN_PKG = 'aurora_depth_obstacle_scan'


def _parse(pkg, relative_path):
    share = get_package_share_directory(pkg)
    with open(f'{share}/{relative_path}') as f:
        return ast.parse(f.read())


def _declared_launch_arguments(tree):
    names = set()
    for node in ast.walk(tree):
        is_declare = (
            isinstance(node, ast.Call)
            and getattr(node.func, 'id', None) == 'DeclareLaunchArgument'
        )
        if is_declare and node.args and isinstance(node.args[0], ast.Constant):
            names.add(node.args[0].value)
    return names


def _forwarded_argument_keys(tree, target_variable_name):
    """
    Find launch_arguments dict keys forwarded to target_variable_name.

    Looks for IncludeLaunchDescription calls whose
    PythonLaunchDescriptionSource references target_variable_name, and
    returns the keys of the dict passed as their launch_arguments keyword.
    """
    forwarded = set()
    for node in ast.walk(tree):
        is_include = (
            isinstance(node, ast.Call)
            and getattr(node.func, 'id', None) == 'IncludeLaunchDescription'
        )
        if not is_include:
            continue

        references_target = any(
            isinstance(n, ast.Name) and n.id == target_variable_name
            for n in ast.walk(node)
        )
        if not references_target:
            continue

        for kw in node.keywords:
            if kw.arg != 'launch_arguments':
                continue
            # launch_arguments={...}.items() -> kw.value is a Call to .items() on a Dict
            value = kw.value
            dict_node = None
            is_items_call = (
                isinstance(value, ast.Call)
                and isinstance(value.func, ast.Attribute)
                and value.func.attr == 'items'
            )
            if is_items_call:
                dict_node = value.func.value
            elif isinstance(value, ast.Dict):
                dict_node = value
            if isinstance(dict_node, ast.Dict):
                for key in dict_node.keys:
                    if isinstance(key, ast.Constant):
                        forwarded.add(key.value)
    return forwarded


def test_description_launch_declares_everything_robot_launch_forwards():
    description_tree = _parse(DESCRIPTION_PKG, 'launch/description.launch.py')
    bringup_tree = _parse(BRINGUP_PKG, 'launch/robot.launch.py')

    declared = _declared_launch_arguments(description_tree)
    forwarded = _forwarded_argument_keys(bringup_tree, 'description_launch')

    assert forwarded, (
        'Expected robot.launch.py to forward at least one argument '
        'to description_launch; the detection logic may need updating if the '
        "launch file's structure changed."
    )

    undeclared = forwarded - declared
    assert not undeclared, (
        f'robot.launch.py forwards {sorted(undeclared)} to '
        'description.launch.py, but description.launch.py does not declare '
        'them -- ros2 launch silently drops unknown launch_arguments, so '
        'this reproduces the exact TF-audit bug where use_oak never reached '
        'the Xacro.'
    )


def test_description_launch_declares_oak_arguments():
    description_tree = _parse(DESCRIPTION_PKG, 'launch/description.launch.py')
    declared = _declared_launch_arguments(description_tree)
    for name in ('use_oak', 'oak_mount_profile', 'oak_include_upstream_urdf'):
        assert name in declared, f"description.launch.py must declare '{name}'"


def test_aurora_launch_declares_everything_robot_launch_forwards():
    aurora_tree = _parse(AURORA_PKG, 'launch/aurora.launch.py')
    bringup_tree = _parse(BRINGUP_PKG, 'launch/robot.launch.py')

    declared = _declared_launch_arguments(aurora_tree)
    forwarded = _forwarded_argument_keys(bringup_tree, 'aurora_launch')

    assert 'aurora_base_adapter' in forwarded, (
        'robot.launch.py must forward aurora_base_adapter to aurora.launch.py')
    undeclared = forwarded - declared
    assert not undeclared, (
        f'robot.launch.py forwards {sorted(undeclared)} to aurora.launch.py, '
        'which does not declare them; ros2 launch would drop them silently.'
    )


def test_depth_scan_launch_declares_everything_robot_launch_forwards():
    depth_tree = _parse(DEPTH_SCAN_PKG, 'launch/diagnostic.launch.py')
    bringup_tree = _parse(BRINGUP_PKG, 'launch/robot.launch.py')

    declared = _declared_launch_arguments(depth_tree)
    forwarded = _forwarded_argument_keys(bringup_tree, 'aurora_depth_scan_launch')

    assert 'params_file' in forwarded, (
        'robot.launch.py must forward params_file to aurora_depth_obstacle_scan')
    undeclared = forwarded - declared
    assert not undeclared, (
        f'robot.launch.py forwards {sorted(undeclared)} to diagnostic.launch.py, '
        'which does not declare them; ros2 launch would drop them silently.'
    )
