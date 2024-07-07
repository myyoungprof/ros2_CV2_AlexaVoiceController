#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import serial

class GesturePublisher(Node):
    def __init__(self):
        super().__init__('car_control_publisher')
        #arduino 
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200)
        self.port_ = self.get_parameter("port").value
        self.baudrate_ = self.get_parameter("baudrate").value

        self.arduino_ = serial.Serial(port=self.port_, baudrate=self.baudrate_, timeout=0.1)
        
        #publisher 
        self.image_publisher_ = self.create_publisher(Image, 'gesture_image', 10)

        #computer vision 
        self.gesture_publisher_ = self.create_publisher(String, 'gesture', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.cap = cv2.VideoCapture(0)
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands()
        self.mp_drawing = mp.solutions.drawing_utils
        self.bridge = CvBridge()

    def count_fingers(self, hand_landmarks):
        thumb_is_open = hand_landmarks.landmark[4].x > hand_landmarks.landmark[3].x
        fingers_are_open = [
            hand_landmarks.landmark[8].y < hand_landmarks.landmark[6].y,
            hand_landmarks.landmark[12].y < hand_landmarks.landmark[10].y,
            hand_landmarks.landmark[16].y < hand_landmarks.landmark[14].y,
            hand_landmarks.landmark[20].y < hand_landmarks.landmark[18].y,
        ]
        return sum([thumb_is_open] + fingers_are_open)

    def timer_callback(self):
        if rclpy.ok() and self.arduino_.is_open:
            data = self.arduino_.readline()

            try:
                data.decode("utf-8")
            except:
                return
            
        ret, frame = self.cap.read()
        if not ret:
            return
        
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result = self.hands.process(frame_rgb)
        
        if result.multi_hand_landmarks:
            for hand_landmarks in result.multi_hand_landmarks:
                self.mp_drawing.draw_landmarks(frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                num_fingers = self.count_fingers(hand_landmarks)
                gesture_msg = String()
                gesture_msg.data = f'Fingers: {num_fingers}'
                self.gesture_publisher_.publish(gesture_msg)
        
        image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")

        self.image_publisher_.publish(image_msg)

def main(args=None):
    rclpy.init(args=args)
    node = GesturePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
