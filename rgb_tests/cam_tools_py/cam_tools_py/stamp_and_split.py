#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class StampAndSplit(Node):
    """
    Simple image stamper and splitter.

    Subscribes to a camera image topic, sets the header timestamp to
    node time if it is zero, and republishes the same frame on 4 topics.
    """

    def __init__(self):
        super().__init__('stamp_and_split')

        # Parameters
        self.in_topic = self.declare_parameter(
            'in_topic', '/camera/image_raw'
        ).get_parameter_value().string_value

        depth = self.declare_parameter(
            'qos_depth', 10
        ).get_parameter_value().integer_value

        # Subscriber
        self.sub = self.create_subscription(
            Image,
            self.in_topic,
            self.cam_callback,
            depth
        )

        # Publishers (4 outputs)
        self.pub_main = self.create_publisher(Image, '/image_raw_stamped', depth)
        self.pub_1 = self.create_publisher(Image, '/image_copy_1', depth)
        self.pub_2 = self.create_publisher(Image, '/image_copy_2', depth)
        self.pub_3 = self.create_publisher(Image, '/image_copy_3', depth)

        self.get_logger().info(
            f"StampAndSplit node started. Subscribing to {self.in_topic}"
        )

    def cam_callback(self, msg: Image):
        # If timestamp is zero, use node time
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            msg.header.stamp = self.get_clock().now().to_msg()

        # Publish same frame on 4 topics
        self.pub_main.publish(msg)
        self.pub_1.publish(msg)
        self.pub_2.publish(msg)
        self.pub_3.publish(msg)


def main():
    rclpy.init()
    node = StampAndSplit()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
