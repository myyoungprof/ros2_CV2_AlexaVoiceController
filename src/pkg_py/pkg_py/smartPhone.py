#!/usr/bin/env python3
import rclpy

from rclpy.node import Node
from example_interfaces.msg import String

class MySmartPhone(Node):

    def __init__(self):
        super().__init__("smartPhone")
        self.publishers_ = self.create_subscription(String, "robot", self.callback_robot, 10)
        self.get_logger().info("THe subscriber is on")


    def callback_robot(self, msg):
        self.get_logger().info(msg.data)


def main(args =None):
    rclpy.init(args=args)
    node=MySmartPhone()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ =="__main__":
    main()


