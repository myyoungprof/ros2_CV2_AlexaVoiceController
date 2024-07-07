import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import mediapipe as mp

class GesturePublisher(Node):
    def __init__(self):
        super().__init__('gesture_publisher')
        self.publisher_ = self.create_publisher(String, 'gesture', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.cap = cv2.VideoCapture(0)
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands()
        self.mp_drawing = mp.solutions.drawing_utils

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
        ret, frame = self.cap.read()
        if not ret:
            return
        
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result = self.hands.process(frame_rgb)
        
        if result.multi_hand_landmarks:
            for hand_landmarks in result.multi_hand_landmarks:
                self.mp_drawing.draw_landmarks(frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                num_fingers = self.count_fingers(hand_landmarks)
                msg = String()
                msg.data = f'Fingers: {num_fingers}'
                self.publisher_.publish(msg)
        
        if cv2.waitKey(1) & 0xFF == 27:
            self.cap.release()
            cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = GesturePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
