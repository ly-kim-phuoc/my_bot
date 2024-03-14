#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Range

from ultrasonic_hc_sr04 import UltrasonicHCSR04


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.min_range = 0.2
        self.max_range = 5.0
        self.fov = 0.26179938779915 # 15 degrees
        self.ultrasonic_sensor = UltrasonicHCSR04(11, 23)
        self.publisher_ = self.create_publisher(Range, 'sonar', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Range()
        distance = self.ultrasonic_sensor.Distance()
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = self.fov
        msg.header.frame_id = "/base_link"
        # msg.header.stamp = minimal_publisher.get_clock().now()
        msg.min_range = self.min_range
        msg.max_range = self.max_range
        msg.range = distance
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.range}')


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()