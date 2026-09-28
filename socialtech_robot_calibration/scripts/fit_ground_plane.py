#!/usr/bin/env python3
"""
Estimate a depth sensor's mount height/pitch/roll from its floor plane.

Fits the real floor plane in a live point cloud instead of measuring with
a tape measure to an ambiguous point on the sensor housing.

Works for any depth sensor publishing sensor_msgs/PointCloud2 -- OAK's
/oak/points (socialtech_robot_bringup's oak_pointcloud_processing) or
Aurora's own depth point cloud
(/slamware_ros_sdk_server_node/depth_point_cloud) both work with the same
script and the same --topic argument.

Method:
  1. Accumulate a few frames of the raw cloud (in the sensor's own optical
     frame) and transform them into --reference-frame (default
     base_footprint) using whatever mount calibration is CURRENTLY live
     (from config/*_mounts/*.yaml + the robot's running TF tree).
  2. base_footprint is, by ROS convention, the projection of the robot
     onto the ground: a perfectly calibrated setup would show the floor
     exactly at Z=0 with a perfectly vertical ([0, 0, 1]) normal. Fit a
     plane (RANSAC) to the largest near-horizontal surface in view and
     measure how far it actually is from that ideal -- that residual IS
     the current mount calibration's error.
  3. Attribute 100% of that residual to the mount joint being solved for
     (--parent-frame -> --output-frame, e.g. base_link -> oak_link): fold
     the residual correction into the currently-published
     parent_frame -> output_frame transform (read live via tf2, i.e.
     whatever config/*_mounts/*.yaml currently has) and print the
     corrected xyz_from_parent[2] (z) and rpy_from_parent (pitch, roll)
     ready to paste into that file. x/y and yaw are NOT determined by a
     flat floor (translationally/rotationally symmetric in those 3 DOF)
     -- use fit_xy_yaw.py for those.

This is read-only: it only subscribes to /tf, /tf_static, and the cloud
topic. It never publishes anything and cannot move the robot. It prints a
suggested correction; it does not edit any yaml file itself, and the
suggestion is a best-effort single-shot estimate, not a proof -- re-run
this script after applying it. If the setup was fixed correctly, the
second run should report a near-zero residual; if it still reports a
large one, something about the correction (or the assumption that all
error is in this one mount joint) was wrong, and that is a real, useful
signal rather than a silent wrong answer.
"""

import argparse
import faulthandler
import os
import signal
import subprocess
import sys
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from tf2_ros import Buffer
from tf2_ros import ConnectivityException
from tf2_ros import ExtrapolationException
from tf2_ros import LookupException
from tf2_ros import TransformListener


def parse_args(argv=None):
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        '--topic', required=True, help='PointCloud2 topic to calibrate against, e.g. /oak/points.'
    )
    parser.add_argument(
        '--parent-frame',
        default='base_link',
        help="Mount joint's parent_frame, matching config/*_mounts/*.yaml (default: base_link).",
    )
    parser.add_argument(
        '--output-frame',
        required=True,
        help="Mount joint's output_frame, matching config/*_mounts/*.yaml, e.g. oak_link.",
    )
    parser.add_argument(
        '--reference-frame',
        default='base_footprint',
        help='Frame in which the floor is assumed to be exactly Z=0 (default: base_footprint).',
    )
    parser.add_argument(
        '--frames',
        type=int,
        default=10,
        help='Number of cloud messages to accumulate (default: 10).',
    )
    parser.add_argument(
        '--timeout',
        type=float,
        default=30.0,
        help='Seconds to wait for --frames messages (default: 30).',
    )
    parser.add_argument(
        '--distance-threshold',
        type=float,
        default=0.02,
        help='RANSAC inlier distance in meters (default: 0.02).',
    )
    parser.add_argument(
        '--ransac-iterations',
        type=int,
        default=2000,
        help='RANSAC iterations per plane candidate (default: 2000).',
    )
    parser.add_argument(
        '--max-planes',
        type=int,
        default=3,
        help='Number of plane candidates to extract and report (default: 3).',
    )
    parser.add_argument(
        '--min-inlier-ratio',
        type=float,
        default=0.05,
        help='Discard plane candidates with fewer inliers than this fraction of all points '
        '(default: 0.05).',
    )
    parser.add_argument(
        '--kill-existing',
        action='store_true',
        help=(
            'If another instance of this script is already running, terminate it first '
            'instead of refusing to start. Without this flag, a leftover/orphaned instance '
            '(e.g. from a previous run that was not fully closed) is left alone and this one '
            'exits with an error -- accumulating orphaned instances is a real, '
            'previously-hit problem in this project (see fit_xy_yaw.py / project history).'
        ),
    )
    return parser.parse_args(argv)


def find_other_instances(script_name):
    """Return PIDs of other running processes with this script's name in their command line."""
    my_pid = os.getpid()
    my_ppid = os.getppid()  # excludes the "ros2 run" wrapper process, if any
    try:
        output = subprocess.check_output(['pgrep', '-f', script_name], text=True)
    except subprocess.CalledProcessError:
        return []  # pgrep exits 1 when there are no matches
    except FileNotFoundError:
        return []  # pgrep not installed -- skip the check rather than fail the whole script
    pids = [int(p) for p in output.split()]
    return [p for p in pids if p not in (my_pid, my_ppid)]


def terminate_pids(pids):
    """SIGTERM, then SIGKILL after a grace period, for every PID that is still alive."""
    for pid in pids:
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
    time.sleep(2.0)
    for pid in pids:
        try:
            os.kill(pid, signal.SIGKILL)
        except ProcessLookupError:
            pass


def transform_to_matrix(transform):
    """Convert a geometry_msgs/Transform into a 4x4 homogeneous matrix."""
    t = transform.translation
    q = transform.rotation
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    matrix[:3, 3] = [t.x, t.y, t.z]
    return matrix


def apply_matrix(points, matrix):
    """Apply a 4x4 homogeneous matrix to an (N, 3) array of points."""
    homogeneous = np.hstack([points, np.ones((points.shape[0], 1))])
    return (matrix @ homogeneous.T).T[:, :3]


def fit_plane_ransac(points, distance_threshold, iterations, rng):
    """Return (normal, d, inlier_mask) for the best-fitting plane normal.x + d = 0, or None."""
    n_points = points.shape[0]
    if n_points < 3:
        return None
    best_inliers = None
    best_count = -1
    for _ in range(iterations):
        idx = rng.choice(n_points, size=3, replace=False)
        p0, p1, p2 = points[idx]
        normal = np.cross(p1 - p0, p2 - p0)
        norm = np.linalg.norm(normal)
        if norm < 1e-9:
            continue
        normal = normal / norm
        d = -normal.dot(p0)
        distances = np.abs(points @ normal + d)
        inliers = distances < distance_threshold
        count = int(inliers.sum())
        if count > best_count:
            best_count = count
            best_inliers = inliers
    if best_inliers is None or best_count < 3:
        return None
    # Refine on all inliers via least-squares (SVD), not just the 3 sample points.
    inlier_pts = points[best_inliers]
    centroid = inlier_pts.mean(axis=0)
    _, _, vh = np.linalg.svd(inlier_pts - centroid)
    normal = vh[-1]
    normal = normal / np.linalg.norm(normal)
    d = -normal.dot(centroid)
    distances = np.abs(points @ normal + d)
    inliers = distances < distance_threshold
    return normal, d, inliers


def extract_plane_candidates(points, args, rng):
    """Iterate: fit and remove the best-fitting plane, up to --max-planes times."""
    remaining = points
    candidates = []
    min_inliers = int(args.min_inlier_ratio * points.shape[0])
    for _ in range(args.max_planes):
        result = fit_plane_ransac(remaining, args.distance_threshold, args.ransac_iterations, rng)
        if result is None:
            break
        normal, d, inliers = result
        if int(inliers.sum()) < min_inliers:
            break
        # Orient the normal to point "up" (positive Z) for a consistent report.
        if normal[2] < 0:
            normal = -normal
            d = -d
        candidates.append((normal, d, int(inliers.sum())))
        remaining = remaining[~inliers]
        if remaining.shape[0] < 3:
            break
    return candidates


def residual_transform(normal, d):
    """
    Build the 4x4 matrix mapping the fitted plane onto the ideal floor plane.

    The fitted plane satisfies normal.x + d = 0; the ideal floor plane is
    z = 0 with normal [0, 0, 1].
    """
    target = np.array([0.0, 0.0, 1.0])
    rotation, _ = Rotation.align_vectors([target], [normal])
    matrix = np.eye(4)
    matrix[:3, :3] = rotation.as_matrix()
    # After rotation alone, plane points sit at constant z = -d; lift by +d.
    matrix[:3, 3] = [0.0, 0.0, d]
    return matrix


class GroundPlaneCollector(Node):
    """Accumulates PointCloud2 frames and looks up the TF needed to place them."""

    def __init__(self, args):
        """Set up the subscription and TF listener."""
        super().__init__('fit_ground_plane')
        self.args = args
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.clouds = []
        self.create_subscription(PointCloud2, args.topic, self._on_cloud, 10)

    def _on_cloud(self, msg):
        if len(self.clouds) >= self.args.frames:
            return
        self.clouds.append(msg)
        self.get_logger().info(f'Collected frame {len(self.clouds)}/{self.args.frames}')


def collect_points_in_reference_frame(node, args):
    """Collect --frames cloud messages and return an (N, 3) array in --reference-frame."""
    start = node.get_clock().now()
    timeout = Duration(seconds=args.timeout)
    while len(node.clouds) < args.frames:
        rclpy.spin_once(node, timeout_sec=0.2)
        if node.get_clock().now() - start > timeout:
            break
    if not node.clouds:
        raise RuntimeError(
            f'No messages received on {args.topic} within {args.timeout}s. '
            'Is the sensor/bringup running?'
        )
    if len(node.clouds) < args.frames:
        node.get_logger().warn(
            f'Only collected {len(node.clouds)}/{args.frames} frames before timeout -- '
            'continuing anyway.'
        )

    all_points = []
    for msg in node.clouds:
        try:
            transform = node.tf_buffer.lookup_transform(
                args.reference_frame,
                msg.header.frame_id,
                Time.from_msg(msg.header.stamp),
                Duration(seconds=1.0),
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as exc:
            node.get_logger().warn(f'Skipping a frame: could not look up transform ({exc})')
            continue
        matrix = transform_to_matrix(transform.transform)
        points = pc2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if points.shape[0] == 0:
            continue
        all_points.append(apply_matrix(points.astype(np.float64), matrix))

    if not all_points:
        raise RuntimeError(
            f'Collected {len(node.clouds)} cloud messages but none had a usable transform to '
            f'{args.reference_frame}. Is the full TF tree (base_footprint, {args.parent_frame}, '
            f"{args.output_frame}, and the cloud's own frame) actually being published?"
        )
    return np.vstack(all_points)


def report_candidates(candidates, total_points):
    """Print every plane candidate found, with its inlier count and tilt."""
    print('=== Plane candidates in the reference frame (near-horizontal if the floor) ===')
    for i, (normal, d, count) in enumerate(candidates):
        tilt_deg = np.degrees(np.arccos(np.clip(normal[2], -1.0, 1.0)))
        pct = 100.0 * count / total_points
        print(
            f'  [{i}] inliers={count} ({pct:.1f}%)  height_offset={-d:+.4f} m  '
            f'tilt_from_vertical={tilt_deg:.2f} deg  normal={np.round(normal, 4)}'
        )


def main(argv=None):
    """Collect a cloud, fit the floor plane, and print a corrected mount transform."""
    faulthandler.register(signal.SIGUSR1)
    args = parse_args(argv)

    # See fit_xy_yaw.py for why this check exists: a leftover/orphaned
    # instance from a previous run can pile up with a new one and degrade
    # /oak/points throughput for everyone until it looks like a hang.
    others = find_other_instances(os.path.basename(__file__))
    if others:
        if args.kill_existing:
            print(f'Terminating existing instance(s) first: PIDs {others}', flush=True)
            terminate_pids(others)
        else:
            print(
                f'ERROR: another instance of this script is already running: PIDs {others}. '
                'Refusing to start a second one -- leftover instances compete for CPU and can '
                'make /oak/points throughput collapse for everyone. Kill it yourself and retry, '
                'or re-run with --kill-existing.',
                flush=True,
            )
            return 1

    rclpy.init(args=None)
    node = GroundPlaneCollector(args)
    rng = np.random.default_rng()

    try:
        points = collect_points_in_reference_frame(node, args)
        print(
            f'Collected {points.shape[0]} points across {len(node.clouds)} frames '
            f"in '{args.reference_frame}'."
        )

        candidates = extract_plane_candidates(points, args, rng)
        if not candidates:
            print(
                'RESULT: FAIL -- no plane with enough inliers found. '
                'Point the sensor at a clear, flat floor patch.'
            )
            return 1
        report_candidates(candidates, points.shape[0])

        # The floor is the candidate whose normal is closest to vertical --
        # reliable here because we are already in a frame where a correctly
        # calibrated floor would be exactly vertical, so even a fairly wrong
        # current calibration should still leave the floor as the
        # most-vertical large surface (walls are much further from vertical).
        floor_normal, floor_d, floor_count = min(candidates, key=lambda c: abs(c[0][2] - 1.0))
        chosen_idx = candidates.index((floor_normal, floor_d, floor_count))
        print(f'\nUsing candidate [{chosen_idx}] as the floor (closest to vertical).')

        try:
            t_bf_parent = node.tf_buffer.lookup_transform(
                args.reference_frame, args.parent_frame, Time()
            )
            t_parent_output_current = node.tf_buffer.lookup_transform(
                args.parent_frame, args.output_frame, Time()
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as exc:
            print(f'RESULT: FAIL -- could not look up the mount chain to correct: {exc}')
            return 1

        t_bf_parent_m = transform_to_matrix(t_bf_parent.transform)
        t_parent_output_current_m = transform_to_matrix(t_parent_output_current.transform)
        t_residual_m = residual_transform(floor_normal, floor_d)

        # See this file's docstring for the derivation: the residual found in
        # reference_frame is attributed entirely to the parent_frame ->
        # output_frame mount joint.
        t_parent_output_new_m = (
            np.linalg.inv(t_bf_parent_m) @ t_residual_m @ t_bf_parent_m @ t_parent_output_current_m
        )

        new_xyz = t_parent_output_new_m[:3, 3]
        new_rpy = Rotation.from_matrix(t_parent_output_new_m[:3, :3]).as_euler('xyz')
        current_xyz = t_parent_output_current_m[:3, 3]
        current_rpy = Rotation.from_matrix(t_parent_output_current_m[:3, :3]).as_euler('xyz')

        print(f'\n=== Current {args.parent_frame} -> {args.output_frame} (from the live TF) ===')
        cx, cy, cz = current_xyz
        print(f'  xyz_from_parent: [{cx:.4f}, {cy:.4f}, {cz:.4f}]')
        cr, cp, cyaw = current_rpy
        print(f'  rpy_from_parent: [{cr:.4f}, {cp:.4f}, {cyaw:.4f}]')

        print("\n=== Suggested corrected values (paste into the mount's *_mounts/*.yaml) ===")
        print(f'  xyz_from_parent: [{new_xyz[0]:.4f}, {new_xyz[1]:.4f}, {new_xyz[2]:.4f}]')
        print(f'  rpy_from_parent: [{new_rpy[0]:.4f}, {new_rpy[1]:.4f}, {new_rpy[2]:.4f}]')
        print(
            '\nOnly z (xyz index 2) and pitch/roll (rpy indices 0, 1) are meaningfully determined '
            'by a flat floor -- x, y, and yaw above are carried over unchanged from the current '
            'calibration, not actually measured. Use fit_xy_yaw.py for those.\n'
            'Re-run this script after applying the correction: a well-calibrated mount should '
            'report a height_offset and tilt close to zero for the floor candidate.'
        )
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
