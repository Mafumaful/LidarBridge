#!/usr/bin/env python3

import os
import math
from typing import List

from PIL import Image
import rclpy
from nav_msgs.msg import MapMetaData, OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
import yaml


ANSI_RESET = "\033[0m"
ANSI_CYAN = "\033[1;36m"


def color_text(text: str, color_code: str) -> str:
    return f"{color_code}{text}{ANSI_RESET}"


def occupancy_from_pixel(
    pixel_value: int,
    negate: bool,
    occupied_thresh: float,
    free_thresh: float,
    mode: str,
) -> int:
    normalized = float(pixel_value) / 255.0
    occupancy = normalized if negate else 1.0 - normalized

    if mode == "raw":
        return max(0, min(100, int(round(occupancy * 100.0))))

    if occupancy >= occupied_thresh:
        return 100
    if occupancy <= free_thresh:
        return 0

    if mode == "scale":
        scaled = (occupancy - free_thresh) / (occupied_thresh - free_thresh)
        return max(0, min(100, int(round(scaled * 100.0))))

    return -1


class Nav2MapPublisherNode(Node):
    def __init__(self) -> None:
        super().__init__("nav2_map_publisher_node")

        self.declare_parameter("map_yaml_file", "")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("map_metadata_topic", "/map_metadata")
        self.declare_parameter("map_frame_id", "map")
        self.declare_parameter("republish_period_sec", 2.0)

        map_yaml_file = self.get_parameter("map_yaml_file").get_parameter_value().string_value
        self.map_topic = self.get_parameter("map_topic").get_parameter_value().string_value
        self.map_metadata_topic = (
            self.get_parameter("map_metadata_topic").get_parameter_value().string_value
        )
        self.map_frame_id = self.get_parameter("map_frame_id").get_parameter_value().string_value
        republish_period_sec = (
            self.get_parameter("republish_period_sec").get_parameter_value().double_value
        )

        if not map_yaml_file:
            raise RuntimeError("map_yaml_file parameter must not be empty")

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE

        self.map_publisher = self.create_publisher(OccupancyGrid, self.map_topic, qos)
        self.metadata_publisher = self.create_publisher(
            MapMetaData, self.map_metadata_topic, qos
        )

        self.map_msg = self.load_map(map_yaml_file)
        self.publish_map()

        if republish_period_sec > 0.0:
            self.timer = self.create_timer(republish_period_sec, self.publish_map)
        else:
            self.timer = None

        self.get_logger().info(
            color_text(
                f"Loaded Nav2 map '{map_yaml_file}' and publishing on '{self.map_topic}'",
                ANSI_CYAN,
            )
        )

    def load_map(self, map_yaml_file: str) -> OccupancyGrid:
        with open(map_yaml_file, "r", encoding="utf-8") as stream:
            map_config = yaml.safe_load(stream)

        image_path = map_config["image"]
        if not os.path.isabs(image_path):
            image_path = os.path.join(os.path.dirname(map_yaml_file), image_path)

        image = Image.open(image_path).convert("L")
        width, height = image.size
        pixel_access = image.load()

        negate = bool(map_config.get("negate", 0))
        occupied_thresh = float(map_config.get("occupied_thresh", 0.65))
        free_thresh = float(map_config.get("free_thresh", 0.196))
        mode = str(map_config.get("mode", "trinary")).lower()
        resolution = float(map_config["resolution"])
        origin = map_config["origin"]

        if mode not in {"trinary", "scale", "raw"}:
            raise RuntimeError(f"Unsupported map mode '{mode}'")

        data: List[int] = []
        for y in range(height - 1, -1, -1):
            for x in range(width):
                pixel_value = pixel_access[x, y]
                data.append(
                    occupancy_from_pixel(
                        pixel_value,
                        negate,
                        occupied_thresh,
                        free_thresh,
                        mode,
                    )
                )

        msg = OccupancyGrid()
        msg.header.frame_id = self.map_frame_id
        msg.info.resolution = resolution
        msg.info.width = width
        msg.info.height = height
        msg.info.origin.position.x = float(origin[0])
        msg.info.origin.position.y = float(origin[1])
        msg.info.origin.position.z = 0.0
        yaw_rad = float(origin[2]) if len(origin) > 2 else 0.0
        msg.info.origin.orientation.z = math.sin(yaw_rad * 0.5)
        msg.info.origin.orientation.w = math.cos(yaw_rad * 0.5)
        msg.data = data
        return msg

    def publish_map(self) -> None:
        stamp = self.get_clock().now().to_msg()
        self.map_msg.header.stamp = stamp
        self.map_msg.info.map_load_time = stamp
        self.map_publisher.publish(self.map_msg)
        self.metadata_publisher.publish(self.map_msg.info)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Nav2MapPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
