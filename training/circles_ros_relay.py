#!/usr/bin/env python3
"""Run on the Pi: republish circle_detect_live.py's /objects JSON on ROS 2.

Topic /circles/positions (std_msgs/String), body:
{"circles": [{"x_mm", "y_mm", "px", "py", "r_px"}, ...], "updated_at": unix_s}
An empty list means no circles seen. If the detector is unreachable nothing
is published, so consumers must reject stale updated_at.
"""
import argparse
import urllib.request

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Relay(Node):
    def __init__(self, url, hz):
        super().__init__("circles_relay")
        self.url, self.publisher = url, self.create_publisher(String, "/circles/positions", 10)
        self.create_timer(1 / hz, self.tick)

    def tick(self):
        try:
            with urllib.request.urlopen(self.url, timeout=0.5) as response:
                body = response.read().decode()
        except OSError as error:
            self.get_logger().warn(f"detector unreachable: {error}", throttle_duration_sec=5)
            return
        # serve.py names the list "objects"; rename so the topic reads on its own.
        self.publisher.publish(String(data=body.replace('"objects"', '"circles"', 1)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default="http://10.42.0.50:8771/objects")
    parser.add_argument("--hz", type=float, default=10)
    args = parser.parse_args()
    rclpy.init()
    rclpy.spin(Relay(args.url, args.hz))


if __name__ == "__main__":
    main()
