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

import termios, fcntl, os

from cyber_py import cyber
from cyber_py import cyber_time
from modules.control.proto.control_pb2 import Control_Command

sys.path.append("../")

fd = sys.stdin.fileno()

oldterm = termios.tcgetattr(fd)
newattr = termios.tcgetattr(fd)
newattr[3] = newattr[3] & ~termios.ICANON & ~termios.ECHO
termios.tcsetattr(fd, termios.TCSANOW, newattr)

oldflags = fcntl.fcntl(fd, fcntl.F_GETFL)
fcntl.fcntl(fd, fcntl.F_SETFL, oldflags | os.O_NONBLOCK)

THROTTLE_MAX = 20.0
THROTTLE_MIN = -20.0
THROTTLE_STEP = 0.5

STEER_ANGLE_MAX = 45.0
STEER_ANGLE_MIN = -45.0
STEER_ANGLE_STEP = 2


class Exercise(object):
    def __init__(self, node):
        self.msg = Control_Command()
        # TODO create writer
        self.writer = node.create_writer(
            "/control", Control_Command)
        self.loop()

    def hotkey_w(self):
        throttle = self.msg.throttle + THROTTLE_STEP
        self.msg.throttle = THROTTLE_MAX if throttle >= THROTTLE_MAX else throttle

    def hotkey_s(self):
        throttle = self.msg.throttle - THROTTLE_STEP
        self.msg.throttle = THROTTLE_MIN if throttle <= THROTTLE_MIN else throttle

    def hotkey_a(self):
        steer_angle = self.msg.steer_angle + STEER_ANGLE_STEP
        self.msg.steer_angle = STEER_ANGLE_MAX if steer_angle >= STEER_ANGLE_MAX else steer_angle

    def hotkey_d(self):
        steer_angle = self.msg.steer_angle - STEER_ANGLE_STEP
        self.msg.steer_angle = STEER_ANGLE_MIN if steer_angle <= STEER_ANGLE_MIN else steer_angle

    def loop(self):
        try:
            while not cyber.is_shutdown():
                try :
                    c = sys.stdin.read(1)
                    if c:
                        # print("Got character", repr(c))
                        if c == 'w': self.hotkey_w()
                        if c == 's': self.hotkey_s()
                        if c == 'a': self.hotkey_a()
                        if c == 'd': self.hotkey_d()
                        print(self.msg)
                        self.writer.write(self.msg)
                        # ratio domain 100hz
                        time.sleep(0.01)
                except IOError: pass
        finally:
            termios.tcsetattr(fd, termios.TCSAFLUSH, oldterm)
            fcntl.fcntl(fd, fcntl.F_SETFL, oldflags)

if __name__ == '__main__':
    cyber.init()

    # TODO update node to your name
    exercise_node = cyber.Node("exercise1.2_node_name")
    exercise = Exercise(exercise_node)

    exercise_node.spin()
    cyber.shutdown()
