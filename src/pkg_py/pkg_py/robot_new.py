#!/usr/bin/env python3
import rclpy

from rclpy.node import Node
from example_interfaces.msg import String

class MyRobotNews(Node):

    def __init__(self):
        super().__init__(" robot_new") 
        self.subscriber_= self.create_publisher(String, "robot", 10)
        self.get_logger().info("THe publisher node has started")
        self.create_timer(0.5, self.pub_news)

    def pub_news(self):
        msg = String()
        msg.data = "hello ros2"
        self.publishers_.publish(msg)


def main(args =None):
    rclpy.init(args=args)
    node=MyRobotNews()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ =="__main__":
    main()





