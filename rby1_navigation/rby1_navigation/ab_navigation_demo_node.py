"""Automatically navigate back and forth between P1 and P2."""

from __future__ import annotations

import time
from typing import List, Tuple

from action_msgs.msg import GoalStatus
import rclpy
from rclpy.node import Node

from rby1_navigation.nav2_goal_client import Nav2GoalClient


Pose2D = Tuple[float, float, float]


class ABNavigationDemoNode(Node):
    """Send alternating P1 and P2 goals to Nav2."""

    def __init__(self) -> None:
        super().__init__("ab_navigation_demo")

        self.declare_parameter("action_name", "/navigate_to_pose")
        self.declare_parameter("frame_id", "map")

        self.declare_parameter("p1", [0.0, 0.0, 0.0])
        self.declare_parameter("p2", [15.8, 5.0, 1.5708])

        self.declare_parameter("first_goal", "p2")
        self.declare_parameter("start_delay", 3.0)
        self.declare_parameter("wait_at_goal", 2.0)
        self.declare_parameter("retry_delay", 3.0)
        self.declare_parameter("repeat_forever", True)

        action_name = str(
            self.get_parameter("action_name").value
        )
        self._frame_id = str(
            self.get_parameter("frame_id").value
        )

        self._poses: List[Pose2D] = [
            self._read_pose_parameter("p1"),
            self._read_pose_parameter("p2"),
        ]

        first_goal = str(
            self.get_parameter("first_goal").value
        ).lower()

        if first_goal not in ("p1", "p2"):
            raise ValueError(
                "first_goal must be either 'p1' or 'p2'"
            )

        self._target_index = 0 if first_goal == "p1" else 1

        self._start_delay = float(
            self.get_parameter("start_delay").value
        )
        self._wait_at_goal = float(
            self.get_parameter("wait_at_goal").value
        )
        self._retry_delay = float(
            self.get_parameter("retry_delay").value
        )
        self._repeat_forever = bool(
            self.get_parameter("repeat_forever").value
        )

        self._goal_client = Nav2GoalClient(
            node=self,
            action_name=action_name,
        )

        self._goal_in_progress = False
        self._stopped = False
        self._successful_goal_count = 0

        self._next_send_time = (
            time.monotonic() + self._start_delay
        )

        self._last_server_log_time = 0.0
        self._last_feedback_log_time = 0.0

        self._timer = self.create_timer(
            0.2,
            self._timer_callback,
        )

        self.get_logger().info(
            "P1-P2 navigation demo initialized"
        )
        self.get_logger().info(
            f"P1={self._poses[0]}, P2={self._poses[1]}"
        )
        self.get_logger().info(
            f"First goal: {self._target_name}"
        )

    @property
    def _target_name(self) -> str:
        return "P1" if self._target_index == 0 else "P2"

    def _read_pose_parameter(self, name: str) -> Pose2D:
        values = list(self.get_parameter(name).value)

        if len(values) != 3:
            raise ValueError(
                f"{name} must contain [x, y, yaw]"
            )

        return (
            float(values[0]),
            float(values[1]),
            float(values[2]),
        )

    def _timer_callback(self) -> None:
        if self._stopped or self._goal_in_progress:
            return

        now = time.monotonic()

        if now < self._next_send_time:
            return

        if not self._goal_client.server_is_ready():
            if now - self._last_server_log_time >= 2.0:
                self.get_logger().info(
                    "Waiting for /navigate_to_pose..."
                )
                self._last_server_log_time = now
            return

        self._send_current_goal()

    def _send_current_goal(self) -> None:
        x, y, yaw = self._poses[self._target_index]

        self._goal_in_progress = True

        self.get_logger().info(
            f"Sending {self._target_name}: "
            f"x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}"
        )

        future = self._goal_client.send_goal_async(
            frame_id=self._frame_id,
            x=x,
            y=y,
            yaw=yaw,
            feedback_callback=self._feedback_callback,
        )

        future.add_done_callback(
            self._goal_response_callback
        )

    def _goal_response_callback(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(
                f"Goal request failed: {exc}"
            )
            self._schedule_retry()
            return

        if not goal_handle.accepted:
            self.get_logger().warning(
                f"{self._target_name} goal was rejected"
            )
            self._schedule_retry()
            return

        self.get_logger().info(
            f"{self._target_name} goal accepted"
        )

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            self._result_callback
        )

    def _feedback_callback(self, feedback_message) -> None:
        now = time.monotonic()

        if now - self._last_feedback_log_time < 2.0:
            return

        feedback = feedback_message.feedback
        distance = float(feedback.distance_remaining)

        self.get_logger().info(
            f"{self._target_name} distance remaining: "
            f"{distance:.2f} m"
        )

        self._last_feedback_log_time = now

    def _result_callback(self, future) -> None:
        self._goal_in_progress = False

        try:
            wrapped_result = future.result()
            status = wrapped_result.status
        except Exception as exc:
            self.get_logger().error(
                f"Failed to receive result: {exc}"
            )
            self._schedule_retry()
            return

        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warning(
                f"{self._target_name} navigation failed "
                f"with status={status}"
            )
            self._schedule_retry()
            return

        reached_name = self._target_name
        self._successful_goal_count += 1

        self.get_logger().info(
            f"Arrived at {reached_name}"
        )

        if (
            not self._repeat_forever
            and self._successful_goal_count >= 2
        ):
            self._stopped = True
            self.get_logger().info(
                "One P1-P2 round trip completed"
            )
            return

        # P1 성공 후 P2, P2 성공 후 P1
        self._target_index = 1 - self._target_index

        self._next_send_time = (
            time.monotonic() + self._wait_at_goal
        )

        self.get_logger().info(
            f"Next goal: {self._target_name} "
            f"after {self._wait_at_goal:.1f} s"
        )

    def _schedule_retry(self) -> None:
        self._goal_in_progress = False
        self._next_send_time = (
            time.monotonic() + self._retry_delay
        )

        self.get_logger().warning(
            f"Retrying {self._target_name} "
            f"after {self._retry_delay:.1f} s"
        )


def main(args=None) -> None:
    rclpy.init(args=args)

    node = ABNavigationDemoNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()