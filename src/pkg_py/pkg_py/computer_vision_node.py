import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
...

class FramesPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        ...
        self.bridge = CvBridge()
        ...
    def publish_frames(self):
        ret, frame = self.vid.read() #opencv returned video frame.
        
        #convert to ROS2 Image msg.
        img = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        img.header.stamp = self.get_clock().now().to_msg()
        img.header.frame_id = self.camera_name        
        
        #publish
        self.img_publisher.publish(img)
def main(args =None):
    rclpy.init(args=args)
    node=FramesPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ =="__main__":
    main()

