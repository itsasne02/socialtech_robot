#!/usr/bin/env python3
"""
Estimate a depth sensor's mount X/Y/yaw using two known-position objects.

A flat floor alone cannot determine X/Y/yaw (see fit_ground_plane.py's
docstring for why -- a flat floor looks identical no matter the sensor's
X/Y position or yaw, so those 3 degrees of freedom need something that
breaks that symmetry: real objects at known positions).

Interactive, guided flow, run once:
  1. Capture a baseline cloud with nothing placed in front of the robot.
  2. Twice: you place one object at a position you measure by hand
     relative to --reference-frame's origin (default base_footprint --
     measure from the point on the floor directly below the robot's own
     center, not from the camera), type in that real X/Y, and the script
     captures again and isolates the object by diffing against the
     baseline (same technique already used to validate Aurora's mount:
     remove a real obstacle, compare the point cloud before/after).
  3. With both correspondences (real position vs. what the sensor actually
     saw), solves the rigid 2D transform (rotation + translation) that
     explains the difference, and folds it into the currently-published
     parent_frame -> output_frame mount transform the same way
     fit_ground_plane.py does for height/tilt.

Use two REAL positions that are clearly apart (e.g. one ahead-left, one
ahead-right) -- two points close together or nearly colinear with the
sensor make the solved rotation unreliable.

Read-only: only subscribes to /tf, /tf_static, and the cloud topic. Prints
a suggested x/y/yaw correction; does not edit any yaml file itself.
Re-run after applying it -- a well-calibrated mount should then place a
newly-placed object very close to its real measured position.
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
from scipy.spatial import cKDTree
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
        help="Mount joint's parent_frame (default: base_link).",
    )
    parser.add_argument(
        '--output-frame', required=True, help="Mount joint's output_frame, e.g. oak_link."
    )
    parser.add_argument(
        '--reference-frame',
        default='base_footprint',
        help='Frame the real X/Y positions you type in are measured relative to '
        '(default: base_footprint).',
    )
    parser.add_argument(
        '--frames',
        type=int,
        default=10,
        help='Cloud messages to accumulate per capture (default: 10).',
    )
    parser.add_argument(
        '--timeout',
        type=float,
        default=30.0,
        help='Seconds to wait for --frames messages (default: 30).',
    )
    parser.add_argument(
        '--diff-threshold',
        type=float,
        default=0.04,
        help='Minimum distance (m) from any baseline point to count as a new/object point '
        '(default: 0.04).',
    )
    parser.add_argument(
        '--min-object-points',
        type=int,
        default=15,
        help='Minimum diff points required to accept an object detection (default: 15).',
    )
    parser.add_argument(
        '--max-diff-points',
        type=int,
        default=30000,
        help=(
            'Randomly subsample each capture to at most this many points before comparing '
            'them (default: 30000). A raw capture can be 400-700k points; comparing that many '
            'took multiple minutes on a Jetson core sharing CPU with the OAK driver and looked '
            'like a hang (confirmed via faulthandler 2026-09-03, not an actual bug). Lower this '
            'if it is still slow; a downsampled centroid is accurate enough for this script.'
        ),
    )
    parser.add_argument(
        '--kill-existing',
        action='store_true',
        help=(
            'If another instance of this script is already running, terminate it first '
            'instead of refusing to start. Without this flag, a leftover/orphaned instance '
            '(e.g. from a previous run that was not fully closed) is left alone and this one '
            'exits with an error -- see this file docstring / project history for why '
            'accumulating orphaned instances is a real, previously-hit problem.'
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


class CloudCollector(Node):
    """Captures PointCloud2 frames on demand, transformed into --reference-frame."""

    def __init__(self, args):
        """Set up one persistent TF listener and cloud subscription for the whole run."""
        super().__init__('fit_xy_yaw')
        print('[startup] rclpy Node created', flush=True)
        self.args = args
        self.tf_buffer = Buffer()
        print('[startup] tf2_ros.Buffer created', flush=True)
        self.tf_listener = TransformListener(self.tf_buffer, self)
        print('[startup] TransformListener created', flush=True)
        self.clouds = []
        self._recording = False
        self._received_count = 0
        # One subscription for the whole script run, not recreated per capture:
        # destroying and recreating a subscription on the same topic forces
        # DDS to rediscover/rematch the publisher, which can stall well past
        # a single spin_once cycle. capture() below just toggles _recording
        # instead, so every capture reuses the same already-matched
        # subscription.
        self.create_subscription(PointCloud2, args.topic, self._on_cloud, 10)
        print(f'[startup] subscribed to {args.topic}, ready', flush=True)

    def _on_cloud(self, msg):
        self._received_count += 1
        if self._recording and len(self.clouds) < self.args.frames:
            self.clouds.append(msg)

    def capture(self, label):
        """Reset, collect --frames messages, and return an (N, 3) array in --reference-frame."""
        args = self.args
        self.clouds = []
        received_before = self._received_count
        self._recording = True
        self.get_logger().info(
            f"Capturing '{label}': waiting for {args.frames} frames on {args.topic}..."
        )
        start = self.get_clock().now()
        timeout = Duration(seconds=args.timeout)
        loop_iters = 0
        while len(self.clouds) < args.frames:
            rclpy.spin_once(self, timeout_sec=0.2)
            loop_iters += 1
            if loop_iters % 25 == 0:
                elapsed = (self.get_clock().now() - start).nanoseconds / 1e9
                print(
                    f'[capture] {elapsed:.1f}s elapsed, {loop_iters} spin_once calls, '
                    f'{self._received_count - received_before} messages seen on callback, '
                    f'{len(self.clouds)}/{args.frames} kept',
                    flush=True,
                )
            if self.get_clock().now() - start > timeout:
                print('[capture] timeout reached, breaking out of the wait loop', flush=True)
                break
        self._recording = False
        if not self.clouds:
            raise RuntimeError(f'No messages received on {args.topic} within {args.timeout}s.')
        if len(self.clouds) < args.frames:
            self.get_logger().warn(
                f'Only got {len(self.clouds)}/{args.frames} frames -- continuing anyway.'
            )

        all_points = []
        for msg in self.clouds:
            try:
                transform = self.tf_buffer.lookup_transform(
                    args.reference_frame,
                    msg.header.frame_id,
                    Time.from_msg(msg.header.stamp),
                    Duration(seconds=1.0),
                )
            except (LookupException, ConnectivityException, ExtrapolationException) as exc:
                self.get_logger().warn(f'Skipping a frame: could not look up transform ({exc})')
                continue
            matrix = transform_to_matrix(transform.transform)
            points = pc2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
            if points.shape[0] == 0:
                continue
            all_points.append(apply_matrix(points.astype(np.float64), matrix))

        if not all_points:
            raise RuntimeError(
                f'No usable transform to {args.reference_frame} for any captured frame.'
            )
        return np.vstack(all_points)


def subsample(points, max_points, rng):
    """Randomly keep at most max_points rows -- cheap and fine for a centroid estimate."""
    if points.shape[0] <= max_points:
        return points
    idx = rng.choice(points.shape[0], size=max_points, replace=False)
    return points[idx]


def isolate_object(
    baseline_points, with_object_points, diff_threshold, min_object_points, max_diff_points, rng
):
    """
    Return points whose nearest baseline neighbor is farther than diff_threshold.

    A raw capture from /oak/points can be 400-700k points (see this project's
    own measured OAK bandwidth); building a KD-tree over that many points and
    querying it with just as many, on a Jetson core sharing CPU with the OAK
    driver, took multiple minutes in practice and looked like a hang from the
    outside (confirmed 2026-09-03 via faulthandler: it was genuinely still
    inside cKDTree.query, not stuck). Subsampling first keeps this to
    seconds; a downsampled centroid is still accurate enough for this script's
    purpose (a rough X/Y correction, not a precision measurement).
    """
    print(
        f'  Comparando nube ({with_object_points.shape[0]} puntos vs '
        f'{baseline_points.shape[0]} de referencia, reducido a <= {max_diff_points} cada uno)...',
        flush=True,
    )
    baseline_points = subsample(baseline_points, max_diff_points, rng)
    with_object_points = subsample(with_object_points, max_diff_points, rng)
    tree = cKDTree(baseline_points)
    distances, _ = tree.query(with_object_points, k=1)
    object_points = with_object_points[distances > diff_threshold]
    if object_points.shape[0] < min_object_points:
        raise RuntimeError(
            f'Only found {object_points.shape[0]} new points (need >= {min_object_points}). '
            'Is the object actually new/different from the baseline scene, and within the '
            "sensor's view?"
        )
    return object_points


def solve_rigid_2d(observed_xy, real_xy):
    """Solve the 2D rotation + translation (Kabsch/Procrustes) mapping observed_xy onto real_xy."""
    observed_xy = np.asarray(observed_xy, dtype=np.float64)
    real_xy = np.asarray(real_xy, dtype=np.float64)
    obs_centroid = observed_xy.mean(axis=0)
    real_centroid = real_xy.mean(axis=0)
    obs_centered = observed_xy - obs_centroid
    real_centered = real_xy - real_centroid
    h = obs_centered.T @ real_centered
    u, _, vt = np.linalg.svd(h)
    r = vt.T @ u.T
    if np.linalg.det(r) < 0:
        vt[-1, :] *= -1
        r = vt.T @ u.T
    t = real_centroid - r @ obs_centroid
    return r, t


def residual_transform_2d(r2d, t2d):
    """Build the 4x4 matrix that rotates/translates in X/Y only, leaving Z untouched."""
    matrix = np.eye(4)
    matrix[0:2, 0:2] = r2d
    matrix[0, 3] = t2d[0]
    matrix[1, 3] = t2d[1]
    return matrix


def prompt_real_xy(label):
    """Ask the operator to place the object and type in its real measured position."""
    print(
        f'\nColoca el objeto en la posicion {label} y mide su X, Y reales desde el origen de '
        '--reference-frame (por defecto base_footprint: el punto en el suelo justo debajo del '
        'centro del robot). X positivo = adelante, Y positivo = izquierda (REP103).'
    )
    x = float(input(f'  X real del objeto {label} (m): '))
    y = float(input(f'  Y real del objeto {label} (m): '))
    input('  Pulsa Enter cuando el objeto este colocado y quieto...')
    return x, y


def main(argv=None):
    """Guide the operator through two object placements and print a corrected mount transform."""
    # Diagnostic only: `kill -USR1 <pid>` dumps every thread's exact Python
    # stack to stderr without killing the process, if it ever appears to
    # hang again.
    faulthandler.register(signal.SIGUSR1)
    args = parse_args(argv)

    # Confirmed real 2026-09-03: a killed/closed run can leave an orphaned
    # copy of this script still running (e.g. only the "ros2 run" wrapper
    # got killed, not its child) -- one such orphan was found consuming
    # ~90% of a Jetson core continuously. Each new run piling on top of
    # leftover ones degrades /oak/points throughput for everyone until it
    # looks like a hang, the same class of bug already hit once with
    # Aurora relaunches (see docs/project_context/10_open_questions.md #4b
    # on the desktop repo). Check for and refuse to start next to one,
    # rather than silently add to the pile.
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
    node = CloudCollector(args)
    rng = np.random.default_rng()

    try:
        input(
            '\nAsegurate de que NO hay ningun objeto extra delante del robot (solo el entorno '
            'normal). Pulsa Enter para capturar la referencia...'
        )
        baseline = node.capture('baseline (sin objeto)')
        print(f'Referencia: {baseline.shape[0]} puntos.')

        real_points = []
        observed_points = []
        for label in ('1', '2'):
            real_x, real_y = prompt_real_xy(label)
            with_object = node.capture(f'con objeto en posicion {label}')
            object_points = isolate_object(
                baseline,
                with_object,
                args.diff_threshold,
                args.min_object_points,
                args.max_diff_points,
                rng,
            )
            centroid = object_points.mean(axis=0)
            print(
                f'  Objeto {label} detectado: {object_points.shape[0]} puntos nuevos, '
                f'centroide observado = ({centroid[0]:.3f}, {centroid[1]:.3f}, {centroid[2]:.3f})'
            )
            real_points.append([real_x, real_y])
            observed_points.append([centroid[0], centroid[1]])

        r2d, t2d = solve_rigid_2d(observed_points, real_points)
        yaw_correction_deg = np.degrees(np.arctan2(r2d[1, 0], r2d[0, 0]))
        print('\n=== Correccion 2D resuelta ===')
        print(f'  yaw_correction: {yaw_correction_deg:.2f} deg')
        print(f'  xy_correction: [{t2d[0]:.4f}, {t2d[1]:.4f}] m')

        try:
            t_ref_parent = node.tf_buffer.lookup_transform(
                args.reference_frame, args.parent_frame, Time()
            )
            t_parent_output_current = node.tf_buffer.lookup_transform(
                args.parent_frame, args.output_frame, Time()
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as exc:
            print(f'RESULT: FAIL -- could not look up the mount chain to correct: {exc}')
            return 1

        t_ref_parent_m = transform_to_matrix(t_ref_parent.transform)
        t_parent_output_current_m = transform_to_matrix(t_parent_output_current.transform)
        t_residual_m = residual_transform_2d(r2d, t2d)

        t_parent_output_new_m = (
            np.linalg.inv(t_ref_parent_m)
            @ t_residual_m
            @ t_ref_parent_m
            @ t_parent_output_current_m
        )

        new_xyz = t_parent_output_new_m[:3, 3]
        new_rpy = Rotation.from_matrix(t_parent_output_new_m[:3, :3]).as_euler('xyz')
        current_xyz = t_parent_output_current_m[:3, 3]
        current_rpy = Rotation.from_matrix(t_parent_output_current_m[:3, :3]).as_euler('xyz')

        print(f'\n=== Current {args.parent_frame} -> {args.output_frame} ===')
        cx, cy, cz = current_xyz
        print(f'  xyz_from_parent: [{cx:.4f}, {cy:.4f}, {cz:.4f}]')
        cr, cp, cyaw = current_rpy
        print(f'  rpy_from_parent: [{cr:.4f}, {cp:.4f}, {cyaw:.4f}]')

        print("\n=== Suggested corrected values (paste into the mount's *_mounts/*.yaml) ===")
        print(f'  xyz_from_parent: [{new_xyz[0]:.4f}, {new_xyz[1]:.4f}, {current_xyz[2]:.4f}]')
        print(f'  rpy_from_parent: [{cr:.4f}, {cp:.4f}, {new_rpy[2]:.4f}]')
        print(
            '\nOnly x, y (kept from the solved xyz above) and yaw are meaningfully determined '
            '-- z and pitch/roll are carried over unchanged from the current calibration (use '
            'fit_ground_plane.py for those). Re-run this script after applying the correction: a '
            'well-calibrated mount should then place a newly-placed object very close to its real '
            'measured position.'
        )
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
