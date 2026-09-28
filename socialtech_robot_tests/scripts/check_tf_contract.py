#!/usr/bin/env python3
"""
Read-only TF contract validator for the SocialTech Tracer+Aurora(+OAK) robot.

Passively listens to /tf and /tf_static and checks the live TF graph against
the contract documented in socialtech_setup/docs/tf_contract.md:

  * a single connected tree (no cycles, no child frame with more than one
    parent, no pair published as both static and dynamic);
  * a chain from --global-frame through --odom-frame to --base-frame;
  * Aurora's physical frames (aurora_link and its children) transformable
    to --base-frame;
  * OAK's oak_link transformable to --base-frame when --require-oak is set;
  * dynamic transforms are fresh (age <= --max-dynamic-age);
  * static transforms are stable (the same pair never changes value).

This tool subscribes only. It never creates a publisher and never modifies
the ROS graph. Exit code is 0 on success, non-zero if any check fails.
"""

import argparse
from collections import defaultdict
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from tf2_msgs.msg import TFMessage


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--require-oak',
        action='store_true',
        help='Also require oak_link to be connected to --base-frame.',
    )
    parser.add_argument(
        '--timeout',
        type=float,
        default=5.0,
        help='Seconds to listen to /tf and /tf_static before evaluating (default: 5.0).',
    )
    parser.add_argument(
        '--max-dynamic-age',
        type=float,
        default=1.0,
        help='Maximum allowed age in seconds for the most recent sample of each '
        'dynamic transform (default: 1.0).',
    )
    parser.add_argument('--global-frame', default='map')
    parser.add_argument('--odom-frame', default='odom')
    parser.add_argument('--base-frame', default='base_footprint')
    return parser.parse_args(argv)


class TfContractListener(Node):
    """Subscribe-only accumulator of /tf and /tf_static edges."""

    def __init__(self, args):
        super().__init__('check_tf_contract')
        self.args = args
        self.static_values = {}
        self.static_unstable = set()
        self.dynamic_last_stamp = {}
        self.child_parents = defaultdict(set)

        static_qos = QoSProfile(
            depth=200,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(TFMessage, '/tf_static', self._on_static, static_qos)
        self.create_subscription(TFMessage, '/tf', self._on_dynamic, 200)

    def _on_static(self, msg):
        for t in msg.transforms:
            parent, child = t.header.frame_id, t.child_frame_id
            key = (parent, child)
            value = (
                round(t.transform.translation.x, 6),
                round(t.transform.translation.y, 6),
                round(t.transform.translation.z, 6),
                round(t.transform.rotation.x, 6),
                round(t.transform.rotation.y, 6),
                round(t.transform.rotation.z, 6),
                round(t.transform.rotation.w, 6),
            )
            if key in self.static_values and self.static_values[key] != value:
                self.static_unstable.add(key)
            self.static_values.setdefault(key, value)
            self.child_parents[child].add(parent)

    def _on_dynamic(self, msg):
        for t in msg.transforms:
            parent, child = t.header.frame_id, t.child_frame_id
            key = (parent, child)
            self.dynamic_last_stamp[key] = t.header.stamp
            self.child_parents[child].add(parent)


def _stamp_to_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def find_ancestor_chain(child_parents, start):
    """
    Walk single-parent pointers from `start` toward the root.

    Returns the chain (list, start first) or None if a cycle is found.
    Stops (without failing) at any frame with zero or more-than-one parent;
    that ambiguity is reported separately by the multi-parent check.
    """
    chain = [start]
    visited = {start}
    current = start
    while len(child_parents.get(current, ())) == 1:
        parent = next(iter(child_parents[current]))
        if parent in visited:
            return None
        chain.append(parent)
        visited.add(parent)
        current = parent
    return chain


def run_checks(node):
    args = node.args
    failures = []
    warnings = []

    if not node.child_parents:
        failures.append(
            'No TF data received at all on /tf or /tf_static within the timeout '
            'window. Is the bringup running?'
        )
        return failures, warnings

    multi_parent = {c: p for c, p in node.child_parents.items() if len(p) > 1}
    for child, parents in multi_parent.items():
        failures.append(f"Frame '{child}' has more than one parent: {sorted(parents)}")

    dynamic_pairs = set(node.dynamic_last_stamp.keys())
    static_pairs = set(node.static_values.keys())
    both = dynamic_pairs & static_pairs
    for parent, child in both:
        failures.append(
            f'Transform {parent} -> {child} is published as BOTH static and dynamic'
        )

    for key in node.static_unstable:
        parent, child = key
        failures.append(
            f'Static transform {parent} -> {child} changed value between messages '
            '(a static transform must be constant)'
        )

    base_chain = find_ancestor_chain(node.child_parents, args.base_frame)
    if base_chain is None:
        failures.append(f"Cycle detected while walking up from '{args.base_frame}'")
    else:
        if args.odom_frame not in base_chain:
            failures.append(
                f"'{args.odom_frame}' is not an ancestor of '{args.base_frame}' "
                f"(chain found: {' -> '.join(base_chain)})"
            )
        if args.global_frame not in base_chain:
            failures.append(
                f"'{args.global_frame}' is not an ancestor of '{args.base_frame}' "
                f"(chain found: {' -> '.join(base_chain)})"
            )

    def _check_connected(frame_name, required):
        if frame_name == args.base_frame:
            return
        if frame_name not in node.child_parents:
            if required:
                failures.append(f"'{frame_name}' has no known parent (not connected to the tree)")
            return
        chain = find_ancestor_chain(node.child_parents, frame_name)
        if chain is None:
            failures.append(f"Cycle detected while walking up from '{frame_name}'")
        elif args.base_frame not in chain:
            failures.append(
                f"'{frame_name}' is not transformable to '{args.base_frame}' "
                f"(chain found: {' -> '.join(chain)})"
            )

    _check_connected('aurora_link', required=True)
    _check_connected('oak_link', required=args.require_oak)

    now_seconds = time.time()
    for (parent, child), stamp in node.dynamic_last_stamp.items():
        age = now_seconds - _stamp_to_seconds(stamp)
        if age > args.max_dynamic_age:
            failures.append(
                f'Dynamic transform {parent} -> {child} is stale: '
                f'{age:.2f}s old (max allowed {args.max_dynamic_age}s)'
            )

    if not dynamic_pairs:
        warnings.append('No dynamic (/tf) transforms observed during the timeout window.')

    return failures, warnings


def main(argv=None):
    args = parse_args(argv)
    rclpy.init(args=None)
    node = TfContractListener(args)

    start = time.time()
    try:
        while time.time() - start < args.timeout:
            rclpy.spin_once(node, timeout_sec=0.2)
        failures, warnings = run_checks(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    print('=== check_tf_contract report ===')
    print(
        f'global_frame={args.global_frame} odom_frame={args.odom_frame} '
        f'base_frame={args.base_frame} require_oak={args.require_oak}'
    )
    print(
        f'frames observed: {len(node.child_parents)}  '
        f'static pairs: {len(node.static_values)}  '
        f'dynamic pairs: {len(node.dynamic_last_stamp)}'
    )

    for w in warnings:
        print(f'WARN: {w}')

    if failures:
        for f in failures:
            print(f'FAIL: {f}')
        print(f'RESULT: FAIL ({len(failures)} failure(s))')
        return 1

    print('RESULT: PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
