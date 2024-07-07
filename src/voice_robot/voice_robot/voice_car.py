import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from robot_msg.action import Move
import serial  # For communicating with Arduino

class MoveActionServer(Node):
    def __init__(self):
        super().__init__('move_action_server')
        self._action_server = ActionServer(
            self,
            Move,
            'move',
            self.execute_callback)
        self.serial_port = serial.Serial('/dev/ttyUSB0', 9600)  # Adjust this port to match your setup

    def execute_callback(self, goal_handle):
        self.get_logger().info(f'Received goal request with direction {goal_handle.request.direction}')
        
        feedback_msg = Move.Feedback()
        feedback_msg.feedback = 'Command received'

        direction = goal_handle.request.direction
        command = self.direction_to_command(direction)
        self.serial_port.write(command.encode('utf-8'))

        while True:
            if self.serial_port.in_waiting > 0:
                response = self.serial_port.readline().decode('utf-8').strip()
                feedback_msg.feedback = response
                goal_handle.publish_feedback(feedback_msg)
                if response == 'DONE':
                    goal_handle.succeed()
                    result = Move.Result()
                    result.success = True
                    result.message = "Motion completed"
                    return result

    def direction_to_command(self, direction):
        if direction == 0:
            return 'STOP'
        elif direction == 1:
            return 'FORWARD'
        elif direction == -1:
            return 'BACKWARD'
        elif direction == 2:
            return 'LEFT'
        elif direction == 3:
            return 'RIGHT'

def main(args=None):
    rclpy.init(args=args)
    move_action_server = MoveActionServer()
    rclpy.spin(move_action_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
