#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Exercise 1.2 write a control script, need to run with root
1. sudo -s 
2. source /apollo/scripts/apollo_base.sh
3. python or python3 run this script
"""
import time
import sys
import random

# python -m pip install pynput
from pynput import keyboard
# import pynput.keyboard import Key, Controller

from cyber_py import cyber
from cyber_py import cyber_time
from modules.control.proto.control_pb2 import Control_Command

sys.path.append("../")

THROTTLE_MAX = 20.0
THROTTLE_MIN = -20.0

THROTTLE_STEP = 0.5

STEER_ANGLE_MAX = 45.0
STEER_ANGLE_MIN = -45.0
STEER_ANGLE_STEP = 0.8


class Exercise(object):
    def __init__(self, node):
        self.msg = Control_Command()
        # create writer
        self.writer = node.create_writer(
            "/control", Control_Command)
        # keyboard event listen non blocking fashion
        listener= keyboard.Listener(on_press = self.onpress_w, 
                                    on_release = self.onrelease_w)
        listener.start()
        self.loop()

    def onpress_w(self, key):
        print('press key %s' % key)
        throttle = self.msg.throttle + THROTTLE_STEP
        self.msg.throttle = THROTTLE_MAX if throttle >= THROTTLE_MAX else throttle

    def onrelease_w(self, key):
        print('release key %s' % key)
        # throttle = self.msg.throttle + THROTTLE_STEP
        # self.msg.throttle = THROTTLE_MAX if throttle >= THROTTLE_MAX else throttle
        print('self.msg.throttle %s' % self.msg.throttle)

    def onpress_s(self):
        throttle = self.msg.throttle - THROTTLE_STEP
        self.msg.throttle = THROTTLE_MIN if throttle <= THROTTLE_MIN else throttle

    def onpress_a(self):
        steer_angle = self.msg.steer_angle + STEER_ANGLE_STEP
        self.msg.steer_angle = STEER_ANGLE_MAX if steer_angle >= STEER_ANGLE_MAX else steer_angle

    def onpress_d(self):
        steer_angle = self.msg.steer_angle - STEER_ANGLE_STEP
        self.msg.steer_angle = STEER_ANGLE_MIN if steer_angle <= STEER_ANGLE_MIN else steer_angle

    def loop(self):
        while not cyber.is_shutdown():
            # self.msg.steer_angle = random.random()
            # self.msg.throttle = random.random()
            self.writer.write(self.msg)
            # ratio domain 100hz
            time.sleep(0.01)

if __name__ == '__main__':
    cyber.init()

    # TODO update node to your name
    exercise_node = cyber.Node("exercise1.1_node_name")
    exercise = Exercise(exercise_node)

    exercise_node.spin()
    cyber.shutdown()
