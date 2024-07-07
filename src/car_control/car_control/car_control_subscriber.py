#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2

class GestureSubscriber(Node):
    def __init__(self):
        super().__init__('gesture_subscriber')
        self.image_subscription = self.create_subscription(Image, 'gesture_image',
                                                           self.image_callback,10)
        self.gesture_subscription = self.create_subscription(String, 'gesture',
                                                             self.gesture_callback,10)
        
        self.bridge = CvBridge()
        self.frame = None

    def image_callback(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.display_frame()

    def gesture_callback(self, msg):
        self.get_logger().info(f'I heard: {msg.data}')

    def display_frame(self):
        if self.frame is not None:
            cv2.imshow('Hand Tracking', self.frame)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = GestureSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
