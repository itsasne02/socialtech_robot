"""
Static validation of the robot Xacro/URDF TF contract.

Expands socialtech_robot_description's main Xacro with use_oak:=false and
use_oak:=true, validates the result with check_urdf, and checks the
resulting link/joint set against the TF contract documented in
socialtech_setup/docs/tf_contract.md: single physical tree, no duplicate
frame names, no leftover "laser" frame, Aurora sensors always present,
OAK present only when requested.
"""

import shutil
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import pytest

DESCRIPTION_PKG = 'socialtech_robot_description'
XACRO_RELATIVE_PATH = 'urdf/tracer_aurora.urdf.xacro'

ALWAYS_EXPECTED_LINKS = {
    'base_footprint',
    'base_link',
    'aurora_link',
    'imu_link',
    'camera_left',
    'camera_right',
    'right_wheel_link',
    'left_wheel_link',
    'front_left_castor_link',
    'front_left_castor_wheel_link',
    'front_right_castor_link',
    'front_right_castor_wheel_link',
    'rear_left_castor_link',
    'rear_left_castor_wheel_link',
    'rear_right_castor_link',
    'rear_right_castor_wheel_link',
}

FORBIDDEN_LINKS = {'laser'}


def _xacro_path():
    share = get_package_share_directory(DESCRIPTION_PKG)
    return f'{share}/{XACRO_RELATIVE_PATH}'


def _require_executable(name):
    if shutil.which(name) is None:
        pytest.skip(f"'{name}' executable not found on PATH")


def _expand_xacro(use_oak):
    _require_executable('xacro')
    xacro_path = _xacro_path()
    result = subprocess.run(
        ['xacro', xacro_path, f"use_oak:={'true' if use_oak else 'false'}"],
        capture_output=True,
        text=True,
        timeout=60,
    )
    assert result.returncode == 0, (
        f'xacro expansion failed (use_oak={use_oak}):\n{result.stderr}'
    )
    return result.stdout


def _check_urdf(urdf_xml):
    _require_executable('check_urdf')
    tmp_path = '/tmp/socialtech_robot_tests_check_urdf.urdf'
    with open(tmp_path, 'w') as f:
        f.write(urdf_xml)
    result = subprocess.run(
        ['check_urdf', tmp_path], capture_output=True, text=True, timeout=30
    )
    assert result.returncode == 0, (
        f'check_urdf reported an invalid URDF:\n{result.stdout}\n{result.stderr}'
    )
    return result.stdout


def _parse_links_and_joints(urdf_xml):
    root = ET.fromstring(urdf_xml)
    link_names = [link.get('name') for link in root.findall('link')]
    joint_names = [joint.get('name') for joint in root.findall('joint')]
    child_to_parent = {}
    for joint in root.findall('joint'):
        parent = joint.find('parent').get('link')
        child = joint.find('child').get('link')
        child_to_parent[child] = parent
    return link_names, joint_names, child_to_parent


def _assert_single_tree(link_names, child_to_parent):
    roots = [name for name in link_names if name not in child_to_parent]
    assert len(roots) == 1, f'Expected exactly one root link, found: {roots}'

    for child in child_to_parent:
        visited = {child}
        current = child
        steps = 0
        while current in child_to_parent:
            current = child_to_parent[current]
            steps += 1
            assert current not in visited, (
                f"Cycle detected in physical tree while walking up from '{child}'"
            )
            visited.add(current)
            assert steps <= len(link_names), (
                f"Cycle detected in physical tree while walking up from '{child}'"
            )


@pytest.mark.parametrize('use_oak', [False, True])
def test_xacro_expands_and_urdf_is_valid(use_oak):
    urdf_xml = _expand_xacro(use_oak)
    _check_urdf(urdf_xml)


@pytest.mark.parametrize('use_oak', [False, True])
def test_no_duplicate_link_or_joint_names(use_oak):
    urdf_xml = _expand_xacro(use_oak)
    link_names, joint_names, _ = _parse_links_and_joints(urdf_xml)

    duplicate_links = {n for n in link_names if link_names.count(n) > 1}
    duplicate_joints = {n for n in joint_names if joint_names.count(n) > 1}

    assert not duplicate_links, f'Duplicate link names: {duplicate_links}'
    assert not duplicate_joints, f'Duplicate joint names: {duplicate_joints}'


@pytest.mark.parametrize('use_oak', [False, True])
def test_single_connected_physical_tree(use_oak):
    urdf_xml = _expand_xacro(use_oak)
    link_names, _, child_to_parent = _parse_links_and_joints(urdf_xml)
    _assert_single_tree(link_names, child_to_parent)


@pytest.mark.parametrize('use_oak', [False, True])
def test_base_frames_present(use_oak):
    urdf_xml = _expand_xacro(use_oak)
    link_names, _, _ = _parse_links_and_joints(urdf_xml)
    link_set = set(link_names)
    assert 'base_footprint' in link_set
    assert 'base_link' in link_set


@pytest.mark.parametrize('use_oak', [False, True])
def test_aurora_connected_and_no_forbidden_frames(use_oak):
    urdf_xml = _expand_xacro(use_oak)
    link_names, _, child_to_parent = _parse_links_and_joints(urdf_xml)
    link_set = set(link_names)

    missing = ALWAYS_EXPECTED_LINKS - link_set
    assert not missing, f'Missing expected frames: {missing}'

    forbidden_present = FORBIDDEN_LINKS & link_set
    assert not forbidden_present, (
        f'Forbidden/removed frames reappeared in the URDF: {forbidden_present}'
    )

    assert child_to_parent.get('aurora_link') == 'base_link'
    assert child_to_parent.get('imu_link') == 'aurora_link'
    assert child_to_parent.get('camera_left') == 'aurora_link'
    assert child_to_parent.get('camera_right') == 'camera_left'


def test_oak_absent_when_disabled():
    urdf_xml = _expand_xacro(use_oak=False)
    link_names, _, _ = _parse_links_and_joints(urdf_xml)
    oak_links = {n for n in link_names if n == 'oak_link' or n.startswith('oak_') or n == 'oak'}
    assert not oak_links, f'OAK frames present with use_oak:=false: {oak_links}'


def test_oak_present_when_enabled():
    urdf_xml = _expand_xacro(use_oak=True)
    link_names, _, child_to_parent = _parse_links_and_joints(urdf_xml)
    link_set = set(link_names)
    assert 'oak_link' in link_set, 'oak_link missing with use_oak:=true'
    assert child_to_parent.get('oak_link') == 'base_link', (
        'oak_link must be a direct child of base_link'
    )
