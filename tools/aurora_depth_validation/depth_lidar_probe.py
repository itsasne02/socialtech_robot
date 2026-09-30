#!/usr/bin/env python3
"""Bounded read-only depth/LaserScan diagnostic; no extrinsic or accuracy acceptance."""
import argparse
import json
import math
import time

import numpy as np
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

from ros_probe import Probe


def finite_json(value):
    if isinstance(value, dict):
        return {k: finite_json(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [finite_json(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)
    return value


def stats(values):
    values = np.asarray(values)
    values = values[np.isfinite(values)]
    return None if not values.size else dict(zip(
        ('p05', 'median', 'p95'), map(float, np.percentile(values, [5, 50, 95]))))


class ComparisonProbe(Probe):
    def __init__(self, args):
        super().__init__(args)
        self.scans, self.depth_rois, self.transforms = [], [], {}
        self.scan_count = 0
        self.last_saved_scan = -math.inf
        self.scan_times = []
        self.create_subscription(LaserScan, args.scan_topic, self.on_scan, qos_profile_sensor_data)
        self.create_subscription(TFMessage, '/tf', self.on_tf, qos_profile_sensor_data)
        self.create_subscription(TFMessage, '/tf_static', self.on_tf,
                                 QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def on_tf(self, msg):
        for t in msg.transforms:
            self.transforms[t.child_frame_id] = message_to_ordereddict(t)

    def on_depth(self, msg):
        super().on_depth(msg)
        if (msg.encoding != '32FC1' or msg.width < 211 or msg.height < 157
                or msg.step < msg.width * 4 or len(msg.data) < msg.height * msg.step):
            return
        d = np.ndarray((msg.height, msg.width), buffer=bytes(msg.data),
                       dtype='>f4' if msg.is_bigendian else '<f4', strides=(msg.step, 4))
        roi = d[137:157, 191:211]
        valid = roi[np.isfinite(roi) & (roi > 0)]
        self.depth_rois.append({'stamp': message_to_ordereddict(msg.header),
                               'receive_monotonic': time.monotonic(),
                               'valid': int(valid.size), 'total': int(roi.size),
                               'depth_sdk_units': stats(valid)})

    def on_scan(self, msg):
        now = time.monotonic()
        self.scan_count += 1
        self.scan_times.append(now)
        if now - self.last_saved_scan < 0.5:
            return
        self.last_saved_scan = now
        r = np.asarray(msg.ranges)
        angles = msg.angle_min + np.arange(len(r)) * msg.angle_increment
        wrapped = np.arctan2(np.sin(angles), np.cos(angles))
        valid = (np.isfinite(r) & (r > 0) & (r >= msg.range_min)
                 & (r <= msg.range_max) & (np.abs(wrapped) <= math.radians(10)))
        self.scans.append({'receive_monotonic': now, 'message': message_to_ordereddict(msg),
                           'sector': '+X of scan frame, +/-10 degrees; alignment unvalidated',
                           'sector_valid': int(valid.sum()), 'sector_range_m': stats(r[valid]),
                           'sector_x_m': stats(r[valid] * np.cos(angles[valid]))})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=20)
    parser.add_argument('--depth-topic', default='/slamware_ros_sdk_server_node/depth_image_raw')
    parser.add_argument('--info-topic', default='/slamware_ros_sdk_server_node/camera_info')
    parser.add_argument('--scan-topic', default='/slamware_ros_sdk_server_node/scan')
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 1 <= args.seconds <= 60:
        parser.error('--seconds must be in [1, 60]')
    rclpy.init()
    node = ComparisonProbe(args)
    try:
        deadline = time.monotonic() + args.seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        result = node.result()
        times = node.scan_times
        result.update(scan_count=node.scan_count,
                      scan_receive_hz=(len(times)-1)/(times[-1]-times[0]) if len(times)>1 else 0,
                      scans=node.scans, depth_rois=node.depth_rois,
                      latest_tf=list(node.transforms.values()), roi_xywh=[191, 137, 20, 20],
                      phase2_pass=False,
                      note='Separate sensor origins/directions. No common-frame transform applied. '
                           'ROS stamps are publication stamps, not acquisition synchronization. '
                           'Static wall/robot required. Nonfinite scan values encoded as strings.')
        print(json.dumps(finite_json(result), indent=2, allow_nan=False))
        return 0 if result['ros_presence_pass'] and node.scan_count >= 20 else 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
