#!/usr/bin/env python3

from pathlib import Path

import mujoco
import numpy as np
import rclpy
from mujoco_lidar import MjLidarWrapper, create_lidar_single_line
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped


class DualLidarNode(Node):
    def __init__(self) -> None:
        super().__init__("dual_lidar_node")

        self.declare_parameter(
            "model_path",
            "/root/sdk/rby1-sdk/models/rby1m/mujoco/lidar_shadow.xml",
        )
        self.declare_parameter("scan_frequency", 30.0)
        self.declare_parameter("range_min", 0.05)
        self.declare_parameter("range_max", 20.0)
        self.declare_parameter("fov_deg", 270.0)
        self.declare_parameter("num_rays", 541)
        self.declare_parameter("pose_timeout", 0.5)
        self.declare_parameter(
            "front_topic",
            "/scan_front_right",
        )
        self.declare_parameter(
            "rear_topic",
            "/scan_rear_left",
        )

        self.declare_parameter(
            "front_frame",
            "laser_front_right",
        )
        self.declare_parameter(
            "rear_frame",
            "laser_rear_left",
        )

        self.declare_parameter(
            "front_site",
            "lidar_front_site",
        )
        self.declare_parameter(
            "rear_site",
            "lidar_rear_site",
        )
        self.declare_parameter(
            "pose_topic",
            "/sim/ground_truth_pose",
        )

        self.pose_topic = str(
            self.get_parameter("pose_topic").value
        )
        self.model_path = Path(
            self.get_parameter("model_path").value
        )
        self.scan_frequency = float(
            self.get_parameter("scan_frequency").value
        )
        self.range_min = float(
            self.get_parameter("range_min").value
        )
        self.range_max = float(
            self.get_parameter("range_max").value
        )
        self.fov_deg = float(
            self.get_parameter("fov_deg").value
        )
        self.num_rays = int(
            self.get_parameter("num_rays").value
        )
        self.front_topic = str(
            self.get_parameter("front_topic").value
        )
        self.rear_topic = str(
            self.get_parameter("rear_topic").value
        )

        self.front_frame = str(
            self.get_parameter("front_frame").value
        )
        self.rear_frame = str(
            self.get_parameter("rear_frame").value
        )

        self.front_site = str(
            self.get_parameter("front_site").value
        )
        self.rear_site = str(
            self.get_parameter("rear_site").value
        )

        self.pose_timeout = float(
            self.get_parameter("pose_timeout").value
        )

        if self.pose_timeout <= 0.0:
            raise ValueError("pose_timeout must be positive")

        if not self.model_path.is_file():
            raise FileNotFoundError(
                f"Shadow MuJoCo model not found: {self.model_path}"
            )

        if self.scan_frequency <= 0.0:
            raise ValueError("scan_frequency must be positive")

        if self.num_rays < 2:
            raise ValueError("num_rays must be at least 2")

        self.model = mujoco.MjModel.from_xml_path(
            str(self.model_path)
        )
        self.data = mujoco.MjData(self.model)

        mocap_ids = np.asarray(
            self.model.body("shadow_base").mocapid
        ).reshape(-1)

        if mocap_ids.size == 0 or int(mocap_ids[0]) < 0:
            raise RuntimeError(
                "shadow_base must be defined as a mocap body"
            )

        self.mocap_id = int(mocap_ids[0])

        self.theta, self.phi = create_lidar_single_line(
            horizontal_resolution=self.num_rays,
            horizontal_fov=np.deg2rad(self.fov_deg),
        )

        self.theta = np.ascontiguousarray(
            self.theta,
            dtype=np.float64,
        )
        self.phi = np.ascontiguousarray(
            self.phi,
            dtype=np.float64,
        )

        self.angle_min = float(self.theta[0])
        self.angle_max = float(self.theta[-1])
        self.angle_increment = float(
            (self.angle_max - self.angle_min)
            / (self.num_rays - 1)
        )
        self.scan_period = 1.0 / self.scan_frequency

        self.front_lidar = MjLidarWrapper(
            self.model,
            site_name=self.front_site,
            backend="cpu",
            cutoff_dist=self.range_max,
        )

        self.rear_lidar = MjLidarWrapper(
            self.model,
            site_name=self.rear_site,
            backend="cpu",
            cutoff_dist=self.range_max,
        )

        self.front_publisher = self.create_publisher(
            LaserScan,
            self.front_topic,
            qos_profile_sensor_data,
        )
        self.rear_publisher = self.create_publisher(
            LaserScan,
            self.rear_topic,
            qos_profile_sensor_data,
        )

        self.pose_received = False
        self.latest_position = None
        self.latest_quaternion = None

        self.first_pose_logged = False
        self.first_scan_started_logged = False
        self.first_scan_published_logged = False

        self.pose_subscription = self.create_subscription(
            PoseStamped,
            self.pose_topic,
            self.pose_callback,
            qos_profile_sensor_data,
        )

        self.timer = self.create_timer(
            self.scan_period,
            self.scan_callback,
        )

        self.get_logger().info(
            f"Subscribing to ground-truth pose: "
            f"{self.pose_topic}"
        )
        self.get_logger().info(
            f"Loaded shadow model: {self.model_path}"
        )
        self.get_logger().info(
            f"Publishing {self.front_topic} and "
            f"{self.rear_topic} at "
            f"{self.scan_frequency:.1f} Hz"
        )

    def pose_callback(self, message: PoseStamped) -> None:
        position = message.pose.position
        orientation = message.pose.orientation

        quaternion = np.array(
            [
                orientation.w,
                orientation.x,
                orientation.y,
                orientation.z,
            ],
            dtype=np.float64,
        )

        norm = np.linalg.norm(quaternion)

        if norm < 1.0e-9:
            self.get_logger().warning(
                "Received invalid zero-norm quaternion"
            )
            return

        quaternion /= norm

        self.latest_position = np.array(
            [
                position.x,
                position.y,
                position.z,
            ],
            dtype=np.float64,
        )

        self.latest_quaternion = quaternion
        self.pose_received = True

        if not self.first_pose_logged:
            self.get_logger().info(
                "Received first ground-truth pose: "
                f"x={position.x:.3f}, "
                f"y={position.y:.3f}, "
                f"z={position.z:.3f}"
            )
            self.first_pose_logged = True

    def scan_callback(self) -> None:
        if (
            not self.pose_received
            or self.latest_position is None
            or self.latest_quaternion is None
        ):
            return

        if not self.first_scan_started_logged:
            self.get_logger().info(
                "Starting first dual LiDAR ray trace"
            )
            self.first_scan_started_logged = True

        self.data.mocap_pos[self.mocap_id] = (
            self.latest_position
        )

        self.data.mocap_quat[self.mocap_id] = (
            self.latest_quaternion
        )

        mujoco.mj_forward(
            self.model,
            self.data,
        )

        front_ranges = self.front_lidar.trace_rays(
            self.data,
            self.theta,
            self.phi,
        )

        rear_ranges = self.rear_lidar.trace_rays(
            self.data,
            self.theta,
            self.phi,
        )

        stamp = self.get_clock().now().to_msg()

        front_message = self.make_scan_message(
            front_ranges,
            self.front_frame,
            stamp,
        )

        rear_message = self.make_scan_message(
            rear_ranges,
            self.rear_frame,
            stamp,
        )

        self.front_publisher.publish(front_message)
        self.rear_publisher.publish(rear_message)

        if not self.first_scan_published_logged:
            self.get_logger().info(
                "Published first front and rear LaserScan"
            )
            self.first_scan_published_logged = True

    def make_scan_message(
        self,
        raw_ranges,
        frame_id: str,
        stamp,
    ) -> LaserScan:
        ranges = np.asarray(
            raw_ranges,
            dtype=np.float64,
        ).reshape(-1)

        if ranges.size != self.num_rays:
            raise RuntimeError(
                f"Expected {self.num_rays} ranges, "
                f"received {ranges.size}"
            )

        invalid = (
            ~np.isfinite(ranges)
            | (ranges < self.range_min)
            | (ranges > self.range_max)
        )

        ranges[invalid] = np.inf

        message = LaserScan()

        message.header.stamp = stamp
        message.header.frame_id = frame_id

        message.angle_min = self.angle_min
        message.angle_max = self.angle_max
        message.angle_increment = self.angle_increment

        message.time_increment = 0.0
        message.scan_time = self.scan_period

        message.range_min = self.range_min
        message.range_max = self.range_max

        message.ranges = ranges.astype(
            np.float32
        ).tolist()

        message.intensities = []

        return message


def main(args=None) -> None:
    rclpy.init(args=args)

    node = None

    try:
        node = DualLidarNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        if node is not None:
            node.get_logger().error(str(error))
        else:
            print(f"dual_lidar_node error: {error}")
        raise
    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()