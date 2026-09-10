"""Perception interface contract used by the control UI frontend.

This module will later contain the non-Node ``CameraClient`` described below:

* subscribe through the existing frontend node;
* cache the newest observation for each object ID;
* transform observations from their source frame into ``base``;
* combine base-to-object and base-to-end-effector transforms at Task runtime.

Step 1 intentionally defines only the ROS boundary. It does not subscribe,
perform TF lookup, publish mock data, or execute robot motion yet.

Object-pose producer contract
-----------------------------
Topic
    ``perception/object_pose`` relative to the node namespace, normally
    ``/rby1/perception/object_pose``.
Type
    ``rby1_msgs/msg/DetectedObjectPose``.
Semantics
    Each message is one valid observation. ``header.frame_id`` is the frame in
    which the pose is expressed, position is in metres, and orientation is a
    normalized quaternion in ROS x/y/z/w field order. ``header.stamp`` is the
    image capture/detection time. Absence or stale data means "not detected";
    producers must not represent that condition with a zero pose.
"""

OBJECT_POSE_TOPIC = "perception/object_pose"
OBJECT_POSE_MESSAGE_TYPE = "rby1_msgs/msg/DetectedObjectPose"
OBJECT_POSE_TARGET_FRAME = "base"

# The subscriber will use sensor-data semantics: newest samples matter, and a
# slow consumer must not build an unbounded queue of obsolete detections.
OBJECT_POSE_QOS_DEPTH = 5
OBJECT_POSE_QOS_RELIABILITY = "best_effort"


__all__ = [
    "OBJECT_POSE_MESSAGE_TYPE",
    "OBJECT_POSE_QOS_DEPTH",
    "OBJECT_POSE_QOS_RELIABILITY",
    "OBJECT_POSE_TARGET_FRAME",
    "OBJECT_POSE_TOPIC",할
]

# <camera.py의 역할>
# ee tf 계산
# CameraClient 로 Realsense/AprilTag  노드가 발행하는 토픽 구독 -> TF 변환(ee to object)