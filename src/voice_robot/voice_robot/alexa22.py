#!/usr/bin/env python3
from flask import Flask
from ask_sdk_core.skill_builder import SkillBuilder
from flask_ask_sdk.skill_adapter import SkillAdapter
from ask_sdk_core.utils import is_request_type, is_intent_name
from ask_sdk_core.handler_input import HandlerInput
from ask_sdk_model import Response
from ask_sdk_model.ui import SimpleCard
from ask_sdk_core.dispatch_components import AbstractRequestHandler, AbstractExceptionHandler
from example_interfaces.action import Move
import rclpy
from rclpy.node import Node
import threading
from rclpy.action import ActionClient

threading.Thread(target=lambda: rclpy.init()).start()
action_client = ActionClient(Node('alexa_interface'), Move, 'move')

app = Flask(__name__)

class LaunchRequestHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        return is_request_type('LaunchRequest')(handler_input)

    def handle(self, handler_input):
        speech_text = "Hi, how can we help?"
        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Online", speech_text)).set_should_end_session(False)
        return handler_input.response_builder.response

class MoveIntentHandler(AbstractRequestHandler):
    def __init__(self, intent_name, direction, distance=0):
        self.intent_name = intent_name
        self.direction = direction
        self.distance = distance

    def can_handle(self, handler_input):
        return is_intent_name(self.intent_name)(handler_input)

    def handle(self, handler_input):
        direction_map = {
            0: "Stopping",
            1: "Moving forward",
            -1: "Moving backward",
            2: "Turning left",
            3: "Turning right"
        }
        speech_text = direction_map[self.direction]
        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard(self.intent_name, speech_text)).set_should_end_session(True)

        goal = Move.Goal()
        goal.direction = self.direction
        goal.distance = self.distance
        action_client.send_goal_async(goal)
        return handler_input.response_builder.response

class AllExceptionHandler(AbstractExceptionHandler):
    def can_handle(self, handler_input, exception):
        return True

    def handle(self, handler_input, exception):
        speech = "Hmm, I don't know that. Can you please say it again?"
        handler_input.response_builder.speak(speech).ask(speech)
        return handler_input.response_builder.response

skill_builder = SkillBuilder()
skill_builder.add_request_handler(LaunchRequestHandler())
skill_builder.add_request_handler(MoveIntentHandler("MoveForwardIntent", 1))
skill_builder.add_request_handler(MoveIntentHandler("MoveBackwardIntent", -1))
skill_builder.add_request_handler(MoveIntentHandler("TurnLeftIntent", 2))
skill_builder.add_request_handler(MoveIntentHandler("TurnRightIntent", 3))
skill_builder.add_request_handler(MoveIntentHandler("StopIntent", 0))
skill_builder.add_request_handler(MoveIntentHandler("MoveForwardIntent_100m", 1, 100))
skill_builder.add_request_handler(MoveIntentHandler("MoveForwardIntent_200m", 1, 200))
skill_builder.add_request_handler(MoveIntentHandler("MoveBackwardIntent_100m", -1, 100))
skill_builder.add_request_handler(MoveIntentHandler("MoveBackwardIntent_200m", -1, 200))
skill_builder.add_exception_handler(AllExceptionHandler())

skill_adapter = SkillAdapter(
    skill=skill_builder.create(),
    skill_id="amzn1.ask.skill.71d8d7ed-37d5-49c6-a1a4-e90a8d546251",
    app=app
)
skill_adapter.register(app=app, route="/")

if __name__ == '__main__':
    app.run()
