#!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from rclpy.action import ActionServer
# from robot_msg.action import Move
# import serial
# import time

# class MoveActionServer(Node):
#     def __init__(self):
#         super().__init__('move_action_server')
#         self._action_server = ActionServer(
#             self,
#             Move,
#             'move',
#             self.execute_callback)
#         self.serial_port = serial.Serial('/dev/ttyACM0', 115200)  # Adjust this port to match your setup

#     def execute_callback(self, goal_handle):
#         self.get_logger().info(f'Received goal request with direction {goal_handle.request.direction} and distance {goal_handle.request.distance}')
        
#         feedback_msg = Move.Feedback()
#         feedback_msg.feedback = 'Command received'

#         direction = goal_handle.request.direction
#         distance = goal_handle.request.distance
#         command = self.direction_to_command(direction, distance)
#         self.get_logger().info(command)
#         self.serial_port.write(command.encode('utf-8'))

#         while True:
#             if self.serial_port.in_waiting > 0:
#                 response = self.serial_port.readline().decode('utf-8').strip()
#                 feedback_msg.feedback = response
#                 goal_handle.publish_feedback(feedback_msg)

#                 if response == 'DONE':
#                     goal_handle.succeed()
#                     result = Move.Result()
#                     result.success = True
#                     result.message = "Motion completed"
#                     return result
#                 elif response.startswith('DISTANCE'):
#                     # Example response format: DISTANCE:45
#                     distance_value = response.split(':')[1]
#                     feedback_msg.feedback = f'Distance: {distance_value} cm'
#                     goal_handle.publish_feedback(feedback_msg)
#                     # Add logic to handle distance if necessary

#     def direction_to_command(self, direction, distance):
        
#         self.get_logger().info(str(type(direction)))
#         self.get_logger().info(str(direction))
#         direction_map = {
#             0: '5', #stop
#             1: '1',  #forward
#             -1: '2', #backward
#             2: '3',  #left
#             3: '4'    #right
#         }
#         cmd = f'{direction_map[direction]} {distance}'
#         # 
#         return cmd

# def main(args=None):
#     rclpy.init(args=args)
#     move_action_server = MoveActionServer()
#     rclpy.spin(move_action_server)
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

#trying my code 


# import rclpy
# from rclpy.node import Node
# from rclpy.action import ActionServer, CancelResponse, GoalResponse
# from robot_msg.action import Move
# import serial
# import time

# class MoveActionServer(Node):
#     def __init__(self):
#         super().__init__('move_action_server')
#         self._action_server = ActionServer(
#             self,
#             Move,
#             'move',
#             execute_callback=self.execute_callback,
#             goal_callback=self.goal_callback,
#             handle_accepted_callback=self.handle_accepted_callback,
#             cancel_callback=self.cancel_callback)
#         self.serial_port = serial.Serial('/dev/ttyACM0', 115200)  # Adjust this port to match your setup
#         self.current_goal_handle = None

#     def goal_callback(self, goal_request):
#         self.get_logger().info(f'Received goal request with direction {goal_request.direction} and distance {goal_request.distance}')
#         return GoalResponse.ACCEPT

#     def cancel_callback(self, goal_handle):
#         self.get_logger().info(f'Received cancel request for goal with direction {goal_handle.request.direction} and distance {goal_handle.request.distance}')
#         if self.current_goal_handle == goal_handle:
#             self.stop_robot()
#         return CancelResponse.ACCEPT

#     def handle_accepted_callback(self, goal_handle):
#         if self.current_goal_handle:
#             self.get_logger().info(f'Preempting current goal')
#             self.stop_robot()
#             self.current_goal_handle.canceled()
#         self.current_goal_handle = goal_handle
#         goal_handle.execute()

#     def execute_callback(self, goal_handle):
#         self.get_logger().info(f'Executing goal with direction {goal_handle.request.direction} and distance {goal_handle.request.distance}')
        
#         feedback_msg = Move.Feedback()
#         feedback_msg.feedback = 'Command received'

#         direction = goal_handle.request.direction
#         distance = goal_handle.request.distance
#         command = self.direction_to_command(direction, distance)
#         self.serial_port.write(command.encode('utf-8'))

#         while True:
#             if goal_handle.is_cancel_requested:
#                 self.get_logger().info('Goal canceled')
#                 goal_handle.canceled()
#                 self.stop_robot()
#                 self.current_goal_handle = None
#                 return Move.Result()

#             if self.serial_port.in_waiting > 0:
#                 response = self.serial_port.readline().decode('utf-8').strip()
#                 feedback_msg.feedback = response
#                 goal_handle.publish_feedback(feedback_msg)

#                 if response == 'DONE':
#                     goal_handle.succeed()
#                     result = Move.Result()
#                     result.success = True
#                     result.message = "Motion completed"
#                     self.current_goal_handle = None
#                     return result
#                 elif response.startswith('DISTANCE'):
#                     distance_value = response.split(':')[1]
#                     feedback_msg.feedback = f'Distance: {distance_value} cm'
#                     goal_handle.publish_feedback(feedback_msg)

#     def direction_to_command(self, direction, distance):
#         # self.get_logger().info(str(type(direction)))
#         # self.get_logger().info(str(type(distance)))
#         # self.get_logger().info(str(direction))
#         # self.get_logger().info(str(distance))
#         direction_map = {
#             0: '5',   # stop
#             1: '1',   # forward
#             -1: '2',  # backward
#             2: '3',   # left
#             3: '4'    # right
#         }
#         cmd = f'{direction_map[direction]} {distance}'
#         return cmd

#     def stop_robot(self):
#         stop_command = self.direction_to_command(0, 0)
#         self.serial_port.write(stop_command.encode('utf-8'))
#         self.get_logger().info("Sent stop command to halt current movement")
#         time.sleep(0.1)  # Ensure the stop command is processed

# def main(args=None):
#     rclpy.init(args=args)
#     move_action_server = MoveActionServer()
#     rclpy.spin(move_action_server)
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


#third code part













import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from robot_msg.action import Move
import serial
import time

class MoveActionServer(Node):
    def __init__(self):
        super().__init__('move_action_server')
        self._action_server = ActionServer(
            self,
            Move,
            'move',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback)
        self.serial_port = serial.Serial('/dev/ttyACM0', 115200)  # Adjust this port to match your Arduino setup
        self.current_goal_handle = None

    def goal_callback(self, goal_request):
        self.get_logger().info(f'Received goal request with direction {goal_request.direction} and distance {goal_request.distance}')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info(f'Received cancel request for goal with direction {goal_handle.request.direction} and distance {goal_handle.request.distance}')
        if self.current_goal_handle == goal_handle:
            self.stop_robot()
        return CancelResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        if self.current_goal_handle:
            self.get_logger().info(f'Preempting current goal')
            self.stop_robot()
            self.current_goal_handle.canceled()
        self.current_goal_handle = goal_handle
        goal_handle.execute()

    def execute_callback(self, goal_handle):
        self.get_logger().info(f'Executing goal with direction {goal_handle.request.direction} and distance {goal_handle.request.distance}')
        
        feedback_msg = Move.Feedback()
        feedback_msg.feedback = 'Command received'

        direction = goal_handle.request.direction
        distance = goal_handle.request.distance
        command = self.direction_to_command(direction, distance)
        self.serial_port.write(command.encode('utf-8'))

        while True:
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal canceled')
                goal_handle.canceled()
                self.stop_robot()
                self.current_goal_handle = None
                return Move.Result()

            # Receive distance from Arduino and update feedback
            distance_value = self.receive_from_arduino()
            feedback_msg.feedback = f'Distance: {distance_value} cm'
            goal_handle.publish_feedback(feedback_msg)

            # Check if distance is below 20 cm
            if distance_value <= 20:
                self.stop_robot()
                feedback_msg.feedback = "Obstacle detected, stopped"
                goal_handle.publish_feedback(feedback_msg)
                self.current_goal_handle = None
                return Move.Result()

            time.sleep(1)  # Adjust as needed for frequency of distance checks

    def direction_to_command(self, direction, distance):
        direction_map = {
            0: '5',   # stop
            1: '1',   # forward
            -1: '2',  # backward
            2: '3',   # left
            3: '4'    # right
        }
        cmd = f'{direction_map[direction]} {distance}'
        return cmd

    def stop_robot(self):
        stop_command = self.direction_to_command(0, 0)
        self.serial_port.write(stop_command.encode('utf-8'))
        self.get_logger().info("Sent stop command to halt current movement")
        time.sleep(0.1)  # Ensure the stop command is processed

    def receive_from_arduino(self):
        distance_value = 0
        while self.serial_port.in_waiting > 0:
            response = self.serial_port.readline().decode('utf-8').strip()
            try:
                distance_value = int(response)
                self.get_logger().info(f'Received distance from Arduino: {distance_value} cm')
            except ValueError:
                self.get_logger().warning(f'Invalid distance value received from Arduino: {response}')
        return distance_value

def main(args=None):
    rclpy.init(args=args)
    move_action_server = MoveActionServer()
    rclpy.spin(move_action_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

