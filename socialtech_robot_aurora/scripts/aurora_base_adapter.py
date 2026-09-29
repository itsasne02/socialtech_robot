#!/usr/bin/env python3
"""
Publish odom -> base_footprint at the robot's axle from Aurora's own pose.

slamware_ros_sdk publishes the SLAMTEC SDK pose as odom_frame -> robot_frame
without any mount offset (server_workers.cpp, ServerOdometryWorker), and the
SDK has no setting for one. With robot_frame:=base_footprint that makes
base_footprint the Aurora unit itself: on Robot 2 it sits ~0.19 m ahead of
the axle (measured 2026-09-29, docs/experiments/2026-09-29_robot2_nav2.md,
open question #16), so an in-place turn moves Nav2's robot origin along an
arc, goals with a final heading never converge, and every frame the URDF
hangs under aurora_link gets the mount applied twice.

With aurora.launch.py aurora_base_adapter:=true the driver publishes its pose
as odom -> aurora_pose on odom_topic .../odom_device instead, and this node:

  * reads the mount once from the URDF TF (child_frame -> mount_frame, i.e.
    base_footprint -> aurora_link from robot_state_publisher), keeping only
    x, y and yaw: the mount has a single source of truth, the measured
    mount profile in socialtech_robot_description;
  * for every driver odometry message, publishes odom -> base_footprint with
    the same stamp, planar (yaw only, REP-120) and z as the SDK gives it;
  * republishes nav_msgs/Odometry on the usual topic, pose at the axle and
    twist in base_footprint axes (the driver's twist is d/dt of the pose in
    odom axes, open question #11d).

If this node is down there is no odom -> base_footprint at all: Nav2 stops on
TF errors instead of driving on a wrong frame.
"""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformBroadcaster, TransformListener


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_of(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def base_from_device(dev_x, dev_y, dev_yaw, mount_x, mount_y, mount_yaw):
    """
    Planar pose of the base given the pose of the device and the device's
    pose in the base frame: base = device * inverse(mount).
    """
    yaw = wrap(dev_yaw - mount_yaw)
    c, s = math.cos(yaw), math.sin(yaw)
    return dev_x - (c * mount_x - s * mount_y), dev_y - (s * mount_x + c * mount_y), yaw


def body_twist(prev, cur):
    """
    (vx, vy, wz) in the body axes of `cur` from two planar poses
    (x, y, yaw, t). Zero when the stamps do not advance.
    """
    dt = cur[3] - prev[3]
    if dt <= 1e-9:
        return 0.0, 0.0, 0.0
    vx_w = (cur[0] - prev[0]) / dt
    vy_w = (cur[1] - prev[1]) / dt
    c, s = math.cos(cur[2]), math.sin(cur[2])
    return c * vx_w + s * vy_w, -s * vx_w + c * vy_w, wrap(cur[2] - prev[2]) / dt


class AuroraBaseAdapter(Node):

    def __init__(self):
        super().__init__('aurora_base_adapter')
        self.parent_frame = self.declare_parameter('parent_frame', 'odom').value
        self.child_frame = self.declare_parameter('child_frame', 'base_footprint').value
        self.mount_frame = self.declare_parameter('mount_frame', 'aurora_link').value
        input_topic = self.declare_parameter(
            'input_topic', '/slamware_ros_sdk_server_node/odom_device').value
        output_topic = self.declare_parameter(
            'output_topic', '/slamware_ros_sdk_server_node/odom').value

        self.mount = None
        self.prev = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.pub = self.create_publisher(Odometry, output_topic, 10)
        self.create_subscription(Odometry, input_topic, self._on_device_odom, 10)
        self.mount_timer = self.create_timer(0.5, self._lookup_mount)
        self.get_logger().info(
            f'waiting for {self.child_frame} -> {self.mount_frame}; '
            f'{input_topic} -> {self.parent_frame} -> {self.child_frame} + {output_topic}')

    def _lookup_mount(self):
        try:
            t = self.tf_buffer.lookup_transform(self.child_frame, self.mount_frame, Time())
        except Exception as exc:
            self.get_logger().warn(f'mount not available yet: {exc}', throttle_duration_sec=5.0)
            return
        tr, q = t.transform.translation, t.transform.rotation
        self.mount = (tr.x, tr.y, yaw_of(q.x, q.y, q.z, q.w))
        self.mount_timer.cancel()
        self.get_logger().info(
            f'mount {self.child_frame} -> {self.mount_frame}: x={tr.x:.3f} y={tr.y:.3f} '
            f'yaw={math.degrees(self.mount[2]):.2f} deg (z={tr.z:.3f} not used)')

    def _on_device_odom(self, msg):
        if self.mount is None:
            self.get_logger().warn('device odometry dropped: mount not known yet',
                                   throttle_duration_sec=5.0)
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        x, y, yaw = base_from_device(p.x, p.y, yaw_of(q.x, q.y, q.z, q.w), *self.mount)
        stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        cur = (x, y, yaw, stamp_s)
        vx, vy, wz = body_twist(self.prev, cur) if self.prev is not None else (0.0, 0.0, 0.0)
        self.prev = cur

        qz, qw = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = p.z
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.parent_frame
        out.child_frame_id = self.child_frame
        out.pose.pose.position.x = x
        out.pose.pose.position.y = y
        out.pose.pose.position.z = p.z
        out.pose.pose.orientation.z = qz
        out.pose.pose.orientation.w = qw
        out.twist.twist.linear.x = vx
        out.twist.twist.linear.y = vy
        out.twist.twist.angular.z = wz
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = AuroraBaseAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
