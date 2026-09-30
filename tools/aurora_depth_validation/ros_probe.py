#!/usr/bin/env python3
"""Read-only phase-1 depth/CameraInfo presence and exact-header check."""
import argparse
import json
import math
import statistics
import time
from collections import OrderedDict

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class Probe(Node):
    def __init__(self, args):
        super().__init__('aurora_depth_validation_probe')
        self.depth = OrderedDict()
        self.info = OrderedDict()
        self.count = {'depth': 0, 'camera_info': 0, 'matched': 0, 'bad_pairs': 0, 'bad_images': 0}
        self.arrivals = {'depth': [], 'camera_info': []}
        self.ages = []
        self.last_info = None
        self.last_image = None
        self.create_subscription(Image, args.depth_topic, self.on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, args.info_topic, self.on_info, qos_profile_sensor_data)

    def record(self, msg, kind, store):
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        self.count[kind] += 1
        self.arrivals[kind].append(time.monotonic())
        store[stamp] = msg
        while len(store) > 64:
            store.popitem(last=False)
        if stamp in self.depth and stamp in self.info:
            d, i = self.depth.pop(stamp), self.info.pop(stamp)
            good = (d.header.frame_id == i.header.frame_id and bool(d.header.frame_id)
                    and d.width == i.width and d.height == i.height
                    and all(math.isfinite(v) for v in i.k) and i.k[0] > 0 and i.k[4] > 0)
            self.count['matched' if good else 'bad_pairs'] += 1

    def on_depth(self, msg):
        if (msg.encoding != '32FC1' or not msg.width or not msg.height
                or msg.step < 4 * msg.width or len(msg.data) < msg.step * msg.height):
            self.count['bad_images'] += 1
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        self.ages.append((self.get_clock().now().nanoseconds - stamp) / 1e6)
        self.last_image = {'frame_id': msg.header.frame_id, 'width': msg.width, 'height': msg.height,
                           'encoding': msg.encoding, 'step': msg.step, 'is_bigendian': msg.is_bigendian}
        self.record(msg, 'depth', self.depth)

    def on_info(self, msg):
        self.last_info = {'frame_id': msg.header.frame_id, 'width': msg.width, 'height': msg.height,
                          'distortion_model': msg.distortion_model, 'k': list(msg.k), 'd': list(msg.d),
                          'r': list(msg.r), 'p': list(msg.p)}
        self.record(msg, 'camera_info', self.info)

    def result(self):
        def hz(times):
            return (len(times) - 1) / (times[-1] - times[0]) if len(times) > 1 else 0
        return {'ros_presence_pass': self.count['matched'] >= 20 and not self.count['bad_pairs']
                and not self.count['bad_images'], 'counts': self.count,
                'receive_hz': {k: hz(v) for k, v in self.arrivals.items()},
                'unmatched_at_exit': {'depth': len(self.depth), 'camera_info': len(self.info)},
                'last_image': self.last_image, 'last_camera_info': self.last_info,
                'ros_stamp_to_receive_median_ms': statistics.median(self.ages) if self.ages else None,
                'latency_note': 'Publication-to-reception approximation only; acquisition clock unverified.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=15)
    parser.add_argument('--depth-topic', default='/slamware_ros_sdk_server_node/depth_image_raw')
    parser.add_argument('--info-topic', default='/slamware_ros_sdk_server_node/camera_info')
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 1 <= args.seconds <= 60:
        parser.error('--seconds must be in [1, 60]')
    rclpy.init()
    node = Probe(args)
    try:
        deadline = time.monotonic() + args.seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        result = node.result()
        print(json.dumps(result, indent=2, allow_nan=False))
        return 0 if result['ros_presence_pass'] else 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
