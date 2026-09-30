#!/usr/bin/env python3
"""Isolated synthetic ROS test. Use LOCALHOST and a domain separate from the robot."""
import math
import os
from pathlib import Path
import struct
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from tf2_ros import StaticTransformBroadcaster


def main():
    binary = sys.argv[1]
    if os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST':
        raise RuntimeError('Synthetic test requires ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST')
    if os.environ.get('ROS_DOMAIN_ID') != '199':
        raise RuntimeError('Synthetic test requires isolated ROS_DOMAIN_ID=199')
    rclpy.init()
    node = rclpy.create_node('depth_scan_contract_test')
    messages = []
    free = []
    depth_pub = node.create_publisher(Image, '/test/depth', qos_profile_sensor_data)
    info_pub = node.create_publisher(CameraInfo, '/test/info', qos_profile_sensor_data)
    node.create_subscription(LaserScan, '/test/scan', messages.append, qos_profile_sensor_data)
    node.create_subscription(LaserScan, '/test/free', free.append, qos_profile_sensor_data)
    tf = StaticTransformBroadcaster(node)
    log_path = Path(os.environ.get('DEPTH_TEST_LOG', '/tmp/aurora_depth_scan_ros_test.log'))
    with log_path.open('w') as log:
        process = subprocess.Popen([binary, '--ros-args',
            '-p', 'depth_topic:=/test/depth', '-p', 'camera_info_topic:=/test/info',
            '-p', 'scan_topic:=/test/scan', '-p', 'free_scan_topic:=/test/free',
            '-p', 'publish_rate:=20.0',
            '-p', 'pixel_stride_x:=1', '-p', 'pixel_stride_y:=1',
            '-p', 'transform_tolerance:=0.0'], stdout=log, stderr=subprocess.STDOUT)
        try:
            def spin(seconds):
                deadline = time.monotonic() + seconds
                while time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=0.01)
                    assert process.poll() is None, log_path.read_text()

            def send(value=1.0, mismatch=False, frame='test_depth', malformed=False,
                     age=0.0, info_first=False):
                d = Image()
                stamp = node.get_clock().now().nanoseconds - int(age * 1e9)
                d.header.stamp.sec, d.header.stamp.nanosec = divmod(stamp, 10**9)
                d.header.frame_id = frame
                d.height = d.width = 1
                d.encoding, d.step = '32FC1', 4
                d.data = list(struct.pack('<f', value)) if not malformed else [0]
                i = CameraInfo()
                i.header.stamp.sec, i.header.stamp.nanosec = divmod(stamp + int(mismatch), 10**9)
                i.header.frame_id = frame
                i.height = i.width = 1
                i.k = [100., 0., 0., 0., 100., 0., 0., 0., 1.]
                if info_first:
                    info_pub.publish(i)
                    spin(0.01)
                    depth_pub.publish(d)
                else:
                    depth_pub.publish(d)
                    info_pub.publish(i)
                return stamp

            def silent(**kwargs):
                spin(0.1)
                before = len(messages)
                for _ in range(7):
                    send(**kwargs)
                    spin(0.07)
                assert len(messages) == before, kwargs

            def receive(**kwargs):
                before = len(messages)
                stamps = set()
                for _ in range(20):
                    stamps.add(send(**kwargs))
                    spin(0.07)
                    if len(messages) > before:
                        msg = messages[-1]
                        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
                        assert stamp in stamps, 'Output must preserve exact matched input stamp'
                        spin(0.05)
                        match = [f for f in free if f.header.stamp == msg.header.stamp]
                        assert match, 'Free scan missing for ' + str(stamp)
                        return msg, match[-1]
                raise AssertionError('No scan received; see ' + str(log_path))

            spin(1.0)
            silent()  # no TF -> no scan, not an artificial empty/free scan
            transform = TransformStamped()
            transform.header.frame_id = 'base_link'
            transform.child_frame_id = 'test_depth'
            transform.header.stamp = node.get_clock().now().to_msg()
            transform.transform.translation.x = 0.2
            transform.transform.translation.z = 0.6
            q = transform.transform.rotation
            q.x, q.y, q.z, q.w = -0.5, 0.5, -0.5, 0.5
            tf.sendTransform(transform)
            spin(0.2)
            silent(mismatch=True)
            silent(frame='unknown_camera')
            silent(malformed=True)
            silent(age=2.0)
            for info_first in (False, True):
                msg, seen = receive(info_first=info_first)
                assert msg.header.frame_id == 'base_link'
                assert len(msg.ranges) == 181
                assert abs(msg.ranges[90] - 1.2) < 1e-5
                assert sum(math.isfinite(v) for v in msg.ranges) == 1
                assert seen.header.frame_id == 'base_link' and len(seen.ranges) == 181
                assert abs(seen.ranges[90] - 1.2) < 1e-5, 'Free only up to the obstacle'
                assert sum(math.isnan(v) for v in seen.ranges) == 180
            for invalid in (float('nan'), float('inf'), 0., -1.):
                msg, seen = receive(value=invalid)
                assert all(math.isinf(v) and v > 0 for v in msg.ranges)
                assert all(math.isnan(v) for v in seen.ranges), 'Invalid depth is not free'
            spin(0.2)
            before = len(messages)
            spin(0.3)
            assert len(messages) == before, 'Must not republish stale frames without new input'
            print('PASS: exact stamps, both arrival orders, base_link geometry, missing TF, '
                  'malformed/stale input, invalid depth -> +Inf, no stale replay, '
                  'free scan with the same stamp, invalid depth -> NaN free')
        finally:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
