"""Reusable Nav2 NavigateToPose action client."""

from math import cos, sin
from typing import Optional

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


class Nav2GoalClient:
    """Small wrapper around Nav2's NavigateToPose action client."""

    def __init__(
        self,
        node: Node,
        action_name: str = "/navigate_to_pose",
    ) -> None:
        self._node = node
        self._action_client = ActionClient(
            node,
            NavigateToPose,
            action_name,
        )

    def server_is_ready(self) -> bool:
        """Return whether the Nav2 action server is available."""
        return self._action_client.server_is_ready()

    def wait_for_server(self, timeout_sec: float = 1.0) -> bool:
        """Wait for the Nav2 action server."""
        return self._action_client.wait_for_server(
            timeout_sec=timeout_sec
        )

    def make_pose(
        self,
        frame_id: str,
        x: float,
        y: float,
        yaw: float,
    ) -> PoseStamped:
        """Create a 2-D PoseStamped from x, y and yaw."""
        pose = PoseStamped()

        pose.header.frame_id = frame_id
        pose.header.stamp = self._node.get_clock().now().to_msg()

        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0

        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = sin(yaw * 0.5)
        pose.pose.orientation.w = cos(yaw * 0.5)

        return pose

    def send_goal_async(
        self,
        frame_id: str,
        x: float,
        y: float,
        yaw: float,
        feedback_callback: Optional[object] = None,
    ):
        """Send a NavigateToPose goal asynchronously."""
        goal = NavigateToPose.Goal()

        goal.pose = self.make_pose(
            frame_id=frame_id,
            x=x,
            y=y,
            yaw=yaw,
        )

        return self._action_client.send_goal_async(
            goal,
            feedback_callback=feedback_callback,
        )