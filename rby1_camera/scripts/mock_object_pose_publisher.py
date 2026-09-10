#!/usr/bin/env python3
"""Publish a configurable object pose without a physical camera."""

from __future__ import annotations

import math
from typing import Iterable, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from rby1_msgs.msg import DetectedObjectPose


def _finite_values(
    values: Iterable[object],
    count: int,
    label: str,
) -> Tuple[float, ...]:
    try:
        result = tuple(float(value) for value in values)
    except (TypeError, ValueError) as exc:
        raise ValueError(f'{label} must contain {count} finite numbers') from exc
    if len(result) != count or not all(math.isfinite(value) for value in result):
        raise ValueError(f'{label} must contain {count} finite numbers')
    return result


class MockObjectPosePublisher(Node):
    """Publish one valid, timestamped detection at a configurable rate."""

    def __init__(self) -> None:
        super().__init__('mock_object_pose_publisher')

        self.declare_parameter('object_pose_topic', 'perception/object_pose')
        self.declare_parameter('object_id', 'tag_0')
        self.declare_parameter('frame_id', 'base')
        self.declare_parameter('position', [0.45, -0.20, 0.85])
        self.declare_parameter('orientation_xyzw', [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter('confidence', 1.0)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('enabled', True)

        topic = str(self.get_parameter('object_pose_topic').value).strip()
        self.object_id = str(self.get_parameter('object_id').value).strip()
        self.frame_id = str(self.get_parameter('frame_id').value).strip()
        self.position = _finite_values(
            self.get_parameter('position').value,
            3,
            'position',
        )
        orientation = _finite_values(
            self.get_parameter('orientation_xyzw').value,
            4,
            'orientation_xyzw',
        )
        confidence = float(self.get_parameter('confidence').value)
        rate_hz = float(self.get_parameter('publish_rate_hz').value)

        if not topic:
            raise ValueError('object_pose_topic must not be empty')
        if not self.object_id:
            raise ValueError('object_id must not be empty')
        if not self.frame_id:
            raise ValueError('frame_id must not be empty')
        if not math.isfinite(confidence) or not 0.0 <= confidence <= 1.0:
            raise ValueError('confidence must be in the range [0.0, 1.0]')
        if not math.isfinite(rate_hz) or rate_hz <= 0.0:
            raise ValueError('publish_rate_hz must be a positive finite number')

        quaternion_norm = math.sqrt(sum(value * value for value in orientation))
        if quaternion_norm <= 1.0e-12:
            raise ValueError('orientation_xyzw must be a non-zero quaternion')
        self.orientation = tuple(
            value / quaternion_norm
            for value in orientation
        )
        self.confidence = confidence

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.object_pose_pub = self.create_publisher(
            DetectedObjectPose,
            topic,
            qos,
        )
        self.publish_timer = self.create_timer(
            1.0 / rate_hz,
            self._publish_object_pose,
        )

        self.get_logger().info(
            'Mock object pose ready: '
            f'{self.resolve_topic_name(topic)} '
            f'object_id={self.object_id!r} frame_id={self.frame_id!r}'
        )

    def _publish_object_pose(self) -> None:
        if not bool(self.get_parameter('enabled').value):
            return

        message = DetectedObjectPose()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id
        message.object_id = self.object_id
        (
            message.pose.position.x,
            message.pose.position.y,
            message.pose.position.z,
        ) = self.position
        (
            message.pose.orientation.x,
            message.pose.orientation.y,
            message.pose.orientation.z,
            message.pose.orientation.w,
        ) = self.orientation
        message.confidence = self.confidence
        self.object_pose_pub.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MockObjectPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
