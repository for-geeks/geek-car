#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import cv2
import time
from modules.sensors.proto.sensors_pb2 import Image
from cyber_py import cyber
import modules.exercises.common.util as util
import sys

sys.path.append("../")


class Exercise(object):

    def __init__(self, node):
        self.node = node
        self.msg = Image()

        # create reader
        self.node.create_reader("/realsense/color_image", Image, self.callback)
        # create writer
        self.writer = self.node.create_writer(
            "/realsense/color_image/compressed", Image)

    def callback(self, data):
        # print frame number
        print(data.frame_no)
        # api to reshape image
        self.msg = data
        self.msg.data = util.reshape(data.data)
        # publish, write to channel
        self.write_to_channel()

    def write_to_channel(self):
        # write compressed image
        self.writer.write(self.msg)


if __name__ == '__main__':
    cyber.init()

    # TODO update node to your name
    exercise_node = cyber.Node("exercise1_node_name")
    exercise = Exercise(exercise_node)

    exercise_node.spin()

    cyber.shutdown()
