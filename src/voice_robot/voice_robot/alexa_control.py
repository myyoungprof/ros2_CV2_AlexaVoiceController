#!/usr/bin/env python3
from flask import Flask
from ask_sdk_core.skill_builder import SkillBuilder
from flask_ask_sdk.skill_adapter import SkillAdapter
from ask_sdk_core.utils import is_request_type, is_intent_name
from ask_sdk_core.handler_input import HandlerInput
from ask_sdk_model import Response
from ask_sdk_model.ui import SimpleCard
from ask_sdk_core.dispatch_components import AbstractRequestHandler, AbstractExceptionHandler
from robot_msg.action import Move
import rclpy
from rclpy.node import Node
import threading
from rclpy.action import ActionClient

# Initialize ROS in a separate thread to avoid blocking
def init_ros():
    rclpy.init()

ros_thread = threading.Thread(target=init_ros)
ros_thread.start()

app_node = Node('alexa_interface')
action_client = ActionClient(app_node, Move, 'move')

app = Flask(__name__)

class LaunchRequestHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        return is_request_type("LaunchRequest")(handler_input)

    def handle(self, handler_input):
        speech_text = "Hi, how can I control the car?"
        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Car Controller", speech_text)).set_should_end_session(False)
        return handler_input.response_builder.response

class MoveIntentHandler(AbstractRequestHandler):
    def __init__(self, direction, distance, name):
        super().__init__()
        self.direction = direction
        self.distance = distance
        self.name = name

    def can_handle(self, handler_input):
        return is_intent_name(self.name)(handler_input)

    def handle(self, handler_input):
        speech_text = f"Moving in direction {self.direction} for {self.distance} meters"
        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard(f"Move {self.direction}", speech_text)).set_should_end_session(True)

        goal = Move.Goal()
        goal.direction = self.direction
        goal.distance = self.distance
        send_goal_future = action_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self.goal_response_callback)

        return handler_input.response_builder.response

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            app_node.get_logger().info('Goal rejected')
            return
        app_node.get_logger().info('Goal accepted')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        app_node.get_logger().info(f'Result: {result.success} - {result.message}')

class TurnIntentHandler(AbstractRequestHandler):
    def __init__(self, direction, name):
        super().__init__()
        self.direction = direction
        self.name = name

    def can_handle(self, handler_input):
        return is_intent_name(self.name)(handler_input)

    def handle(self, handler_input):
        speech_text = f"Turning {self.name.replace('Turn', '').lower()}"
        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard(f"Turn {self.direction}", speech_text)).set_should_end_session(True)

        goal = Move.Goal()
        goal.direction = self.direction
        goal.distance = 0  # Assuming distance is 0 for turning
        send_goal_future = action_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self.goal_response_callback)

        return handler_input.response_builder.response

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            app_node.get_logger().info('Goal rejected')
            return
        app_node.get_logger().info('Goal accepted')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        app_node.get_logger().info(f'Result: {result.success} - {result.message}')

class StopIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        return is_intent_name("StopIntent")(handler_input)

    def handle(self, handler_input):
        speech_text = "Stopping"
        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Stop", speech_text)).set_should_end_session(True)

        goal = Move.Goal()
        goal.direction = 0
        goal.distance = 0
        send_goal_future = action_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self.goal_response_callback)

        return handler_input.response_builder.response

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            app_node.get_logger().info('Goal rejected')
            return
        app_node.get_logger().info('Goal accepted')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        app_node.get_logger().info(f'Result: {result.success} - {result.message}')

class AllExceptionHandler(AbstractExceptionHandler):
    def can_handle(self, handler_input, exception):
        return True

    def handle(self, handler_input, exception):
        speech = "Hmm, I don't know that. Can you please say it again?"
        handler_input.response_builder.speak(speech).ask(speech)
        return handler_input.response_builder.response

skill_builder = SkillBuilder()
skill_builder.add_request_handler(LaunchRequestHandler())
skill_builder.add_request_handler(MoveIntentHandler(1, 100, "MoveForwardIntent"))
skill_builder.add_request_handler(MoveIntentHandler(1, 200, "MoveForwardIntentX"))
skill_builder.add_request_handler(MoveIntentHandler(-1, 100, "MoveBackwardIntent"))
skill_builder.add_request_handler(MoveIntentHandler(-1, 200, "MoveBackwardIntentX"))
skill_builder.add_request_handler(TurnIntentHandler(2, "TurnLeftIntent"))
skill_builder.add_request_handler(TurnIntentHandler(3, "TurnRightIntent"))
skill_builder.add_request_handler(StopIntentHandler())
skill_builder.add_exception_handler(AllExceptionHandler())

skill_adapter = SkillAdapter(
    skill=skill_builder.create(),
    skill_id="amzn1.ask.skill.71d8d7ed-37d5-49c6-a1a4-e90a8d546251",
    app=app)

skill_adapter.register(app=app, route="/")

if __name__ == '__main__':
    app.run()
