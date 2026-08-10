#!/usr/bin/env python3

import math
import socket

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class MujocoPoseBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("mujoco_pose_bridge_node")

        self.declare_parameter(
            "bind_address",
            "0.0.0.0",
        )
        self.declare_parameter("port", 15000)
        self.declare_parameter(
            "output_topic",
            "/sim/ground_truth_pose",
        )
        self.declare_parameter(
            "frame_id",
            "sim_world",
        )
        self.declare_parameter(
            "poll_frequency",
            200.0,
        )
        self.declare_parameter(
            "stale_timeout",
            0.5,
        )

        self.bind_address = str(
            self.get_parameter("bind_address").value
        )
        self.port = int(
            self.get_parameter("port").value
        )
        self.output_topic = str(
            self.get_parameter("output_topic").value
        )
        self.frame_id = str(
            self.get_parameter("frame_id").value
        )
        self.poll_frequency = float(
            self.get_parameter("poll_frequency").value
        )
        self.stale_timeout = float(
            self.get_parameter("stale_timeout").value
        )

        if not 1 <= self.port <= 65535:
            raise ValueError(
                "port must be between 1 and 65535"
            )

        if self.poll_frequency <= 0.0:
            raise ValueError(
                "poll_frequency must be positive"
            )

        if self.stale_timeout <= 0.0:
            raise ValueError(
                "stale_timeout must be positive"
            )

        self.publisher = self.create_publisher(
            PoseStamped,
            self.output_topic,
            qos_profile_sensor_data,
        )

        self.socket = socket.socket(
            socket.AF_INET,
            socket.SOCK_DGRAM,
        )

        self.socket.setsockopt(
            socket.SOL_SOCKET,
            socket.SO_REUSEADDR,
            1,
        )

        self.socket.bind(
            (self.bind_address, self.port)
        )
        self.socket.setblocking(False)

        self.last_receive_time = None
        self.first_packet_received = False
        self.stale_warning_sent = False
        self.last_sequence = None

        self.timer = self.create_timer(
            1.0 / self.poll_frequency,
            self.poll_socket,
        )

        self.get_logger().info(
            f"Listening for MuJoCo pose UDP on "
            f"{self.bind_address}:{self.port}"
        )
        self.get_logger().info(
            f"Publishing actual pose to "
            f"{self.output_topic}"
        )

    def poll_socket(self) -> None:
        latest_packet = None
        latest_sender = None

        while True:
            try:
                packet, sender = self.socket.recvfrom(
                    2048
                )
            except BlockingIOError:
                break
            except OSError as error:
                self.get_logger().error(
                    f"UDP receive failed: {error}"
                )
                return

            latest_packet = packet
            latest_sender = sender

        if latest_packet is None:
            self.check_stale_pose()
            return

        try:
            parsed = self.parse_packet(latest_packet)
        except ValueError as error:
            self.get_logger().warning(
                f"Rejected UDP pose packet: {error}"
            )
            return

        (
            sequence,
            simulation_time,
            x,
            y,
            z,
            qw,
            qx,
            qy,
            qz,
        ) = parsed

        message = PoseStamped()
        message.header.stamp = (
            self.get_clock().now().to_msg()
        )
        message.header.frame_id = self.frame_id

        message.pose.position.x = x
        message.pose.position.y = y
        message.pose.position.z = z

        message.pose.orientation.w = qw
        message.pose.orientation.x = qx
        message.pose.orientation.y = qy
        message.pose.orientation.z = qz

        self.publisher.publish(message)

        self.last_receive_time = self.get_clock().now()
        self.stale_warning_sent = False
        self.last_sequence = sequence

        if not self.first_packet_received:
            self.get_logger().info(
                "Received first MuJoCo pose: "
                f"sender={latest_sender}, "
                f"seq={sequence}, "
                f"sim_t={simulation_time:.3f}, "
                f"xyz=({x:.3f}, {y:.3f}, {z:.3f})"
            )
            self.first_packet_received = True

    def parse_packet(self, packet: bytes):
        try:
            text = packet.decode("ascii").strip()
        except UnicodeDecodeError as error:
            raise ValueError(
                "packet is not ASCII"
            ) from error

        fields = text.split(",")

        if len(fields) != 10:
            raise ValueError(
                f"expected 10 fields, received "
                f"{len(fields)}"
            )

        if fields[0] != "RBY1POSE":
            raise ValueError(
                f"invalid magic value: {fields[0]}"
            )

        try:
            sequence = int(fields[1])

            values = [
                float(value)
                for value in fields[2:]
            ]
        except ValueError as error:
            raise ValueError(
                "packet contains invalid numeric data"
            ) from error

        if not all(math.isfinite(v) for v in values):
            raise ValueError(
                "packet contains non-finite values"
            )

        (
            simulation_time,
            x,
            y,
            z,
            qw,
            qx,
            qy,
            qz,
        ) = values

        quaternion_norm = math.sqrt(
            qw * qw
            + qx * qx
            + qy * qy
            + qz * qz
        )

        if quaternion_norm < 1.0e-12:
            raise ValueError(
                "received a zero-length quaternion"
            )

        qw /= quaternion_norm
        qx /= quaternion_norm
        qy /= quaternion_norm
        qz /= quaternion_norm

        return (
            sequence,
            simulation_time,
            x,
            y,
            z,
            qw,
            qx,
            qy,
            qz,
        )

    def check_stale_pose(self) -> None:
        if self.last_receive_time is None:
            return

        age = (
            self.get_clock().now()
            - self.last_receive_time
        ).nanoseconds * 1.0e-9

        if (
            age > self.stale_timeout
            and not self.stale_warning_sent
        ):
            self.get_logger().warning(
                f"MuJoCo pose packets stopped: "
                f"last packet age={age:.3f} s"
            )
            self.stale_warning_sent = True

    def destroy_node(self) -> bool:
        self.socket.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)

    node = None

    try:
        node = MujocoPoseBridgeNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        if node is not None:
            node.get_logger().error(str(error))
        else:
            print(
                f"mujoco_pose_bridge_node error: "
                f"{error}"
            )
        raise
    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()