#!/usr/bin/env python3
import rclpy

from rclpy.node import Node

class MyNode(Node):

    def __init__(self):
        super().__init__("py_node")
        self.counter_ = 0
        self.get_logger().info("i love ros2")
        self.create_timer(0.5, self.time_callback)


    def time_callback(self):
        self.counter_ +=1 
        self.get_logger().info("this is my ros code: " + str(self.counter_)) 

        


def main(args =None):
    rclpy.init(args=args)
    node=MyNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ =="__main__":
    main()


