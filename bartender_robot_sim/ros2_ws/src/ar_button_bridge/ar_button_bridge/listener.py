"""Prints every button published on /ar/button.

    ros2 run ar_button_bridge ar_button_listener

Stands in for the robot side while testing: if this prints "button 2" when
you pinch button 2 on the glasses, the whole chain works and the robot node
only has to subscribe to the same topic.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

TOPIC = '/ar/button'


class Listener(Node):
    def __init__(self):
        super().__init__('ar_button_listener')
        self.create_subscription(Int32, TOPIC, self._on_button, 10)
        self.get_logger().info('waiting for buttons on %s' % TOPIC)

    def _on_button(self, msg):
        self.get_logger().info('button %d' % msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = Listener()
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
