#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time
import sys

# pip install keyboard
import keyboard

from cyber_py import cyber
from cyber_py import cyber_time
from modules.control.proto.control_pb2 import Control_Command

sys.path.append("../")


class Exercise(object):
    def __init__(self, node):
        self.node = node
        self.msg = Control_Command()
        # create writer
        self.writer = self.node.create_writer(
            "/control", Control_Command)
        # keyboard event listen
        keyboard.add_hotkey('w', self.hotkey_w)
        keyboard.add_hotkey('s', self.hotkey_s)
        keyboard.add_hotkey('a', self.hotkey_a)
        keyboard.add_hotkey('d', self.hotkey_d)
        self.loop()

    def hotkey_w(self):
        self.msg.throttle == 20.0 if self.msg.throttle > 20.0 else self.msg.throttle += 2.0

    def hotkey_s(self):
        self.msg.throttle == -20.0 if self.msg.throttle < -20.0 else self.msg.throttle -= 2.0

    def hotkey_a(self):
        self.msg.steer_angle = 45.0 if self.msg.steer_angle > 45.0 else self.msg.steer_angle += 2.0

    def hotkey_d(self):
        self.msg.steer_angle = -45.0 if self.msg.steer_angle < -45.0 else self.msg.steer_angle -= 2.0

    def loop(self):
        rate = cyber_time.Rate(100)
        self.write_to_channel()
        rate.sleep()

    def write_to_channel(self):
        # write control command message
        self.writer.write(self.msg)


if __name__ == '__main__':
    cyber.init()

    # TODO update node to your name
    exercise_node = cyber.Node("exercise1.1_node_name")
    exercise = Exercise(exercise_node)

    exercise_node.spin()

    cyber.shutdown()
