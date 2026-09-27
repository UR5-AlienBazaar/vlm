#!/usr/bin/env python3
"""Republish a detector's JSON endpoint on ROS 2 as std_msgs/String.

Default (on the Pi): circle_detect_live.py's /objects on /circles/positions, body
{"circles": [{"x_mm", "y_mm", "px", "py", "r_px"}, ...], "updated_at": unix_s}

On hackaton-gpu, over the tunnel to live_bottle_infer.py:
  --url http://localhost:8779/objects  --topic /bottles/objects --key bottles
  --url http://localhost:8779/position --topic /cup/position
If the detector is unreachable nothing is published, so consumers must reject
stale updated_at (and check "found" on /cup/position).
"""
import argparse
import urllib.request

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Relay(Node):
    def __init__(self, url, hz, topic, key):
        super().__init__(topic.strip("/").replace("/", "_") + "_relay")
        self.url, self.key, self.publisher = url, key, self.create_publisher(String, topic, 10)
        self.create_timer(1 / hz, self.tick)

    def tick(self):
        try:
            with urllib.request.urlopen(self.url, timeout=0.5) as response:
                body = response.read().decode()
        except OSError as error:
            self.get_logger().warn(f"detector unreachable: {error}", throttle_duration_sec=5)
            return
        # serve.py names the list "objects"; rename so the topic reads on its own.
        self.publisher.publish(String(data=body.replace('"objects"', f'"{self.key}"', 1)))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--url", default="http://10.42.0.50:8771/objects")
    parser.add_argument("--hz", type=float, default=10)
    parser.add_argument("--topic", default="/circles/positions")
    parser.add_argument("--key", default="circles", help='what the body\'s "objects" list is renamed to')
    args = parser.parse_args()
    rclpy.init()
    rclpy.spin(Relay(args.url, args.hz, args.topic, args.key))


if __name__ == "__main__":
    main()
