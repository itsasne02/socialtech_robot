"""
Check the geometry of socialtech_robot_aurora's aurora_base_adapter.py.

Without ROS running: device pose + mount -> base pose, and twist in body
axes.

The case that motivated it (open question #16): on Robot 2 the Aurora sits
0.2 m ahead of the axle and the SDK pose is the Aurora's. Turning in place
moves the device along a 0.2 m arc while the axle stays put; the adapter
must return a fixed base position for the whole turn.
"""

import importlib.util
import math
import os

from ament_index_python.packages import get_package_prefix


def _adapter():
    path = os.path.join(get_package_prefix('socialtech_robot_aurora'),
                        'lib', 'socialtech_robot_aurora', 'aurora_base_adapter.py')
    spec = importlib.util.spec_from_file_location('aurora_base_adapter', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


A = _adapter()
MOUNT = (0.2, 0.0, 0.0)   # Robot 2, socialtech_robot_itsasne mount profile


def _close(a, b, tol=1e-9):
    return all(abs(x - y) <= tol for x, y in zip(a, b))


def test_zero_mount_is_identity():
    assert _close(A.base_from_device(1.5, -2.0, 0.7, 0.0, 0.0, 0.0), (1.5, -2.0, 0.7))


def test_mount_ahead_heading_zero():
    # Aurora at (1.2, 0) looking along +x: the axle is 0.2 m behind it.
    assert _close(A.base_from_device(1.2, 0.0, 0.0, *MOUNT), (1.0, 0.0, 0.0))


def test_mount_ahead_heading_90_and_180():
    assert _close(A.base_from_device(1.0, 0.2, math.pi / 2, *MOUNT), (1.0, 0.0, math.pi / 2))
    assert _close(A.base_from_device(0.8, 0.0, math.pi, *MOUNT), (1.0, 0.0, math.pi))


def test_mount_with_lateral_offset_and_yaw():
    # Robot 1 style mount, behind and to the right, rotated 90 deg.
    mount = (-0.325, -0.115, math.pi / 2)
    base = (2.0, 1.0, 0.3)
    c, s = math.cos(base[2]), math.sin(base[2])
    dev = (base[0] + c * mount[0] - s * mount[1],
           base[1] + s * mount[0] + c * mount[1],
           base[2] + mount[2])
    assert _close(A.base_from_device(*dev, *mount), base)


def test_turn_in_place_keeps_the_axle_still():
    for deg in range(0, 361, 15):
        yaw = math.radians(deg)
        dev = (1.0 + 0.2 * math.cos(yaw), 2.0 + 0.2 * math.sin(yaw), yaw)
        x, y, _ = A.base_from_device(*dev, *MOUNT)
        assert _close((x, y), (1.0, 2.0)), deg


def test_body_twist_forward_at_any_heading():
    for deg in (-131.5, -90, 0, 45, 180):
        yaw = math.radians(deg)
        prev = (0.0, 0.0, yaw, 10.0)
        cur = (0.018 * math.cos(yaw), 0.018 * math.sin(yaw), yaw, 10.1)
        vx, vy, wz = A.body_twist(prev, cur)
        assert _close((vx, vy, wz), (0.18, 0.0, 0.0), 1e-9), deg


def test_body_twist_turn_across_pi():
    prev = (0.0, 0.0, math.radians(179.0), 0.0)
    cur = (0.0, 0.0, math.radians(-179.0), 0.1)
    _, _, wz = A.body_twist(prev, cur)
    assert abs(wz - math.radians(2.0) / 0.1) < 1e-9


def test_body_twist_without_time_step_is_zero():
    assert A.body_twist((0, 0, 0, 5.0), (1, 1, 1, 5.0)) == (0.0, 0.0, 0.0)
