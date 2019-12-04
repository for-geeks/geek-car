#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import cv2
import time
import sys

from cyber_py import cyber
from modules.sensors.proto.sensor_image_pb2 import Image, CompressImage
import modules.exercises.common.util as util


sys.path.append("../")


class Exercise(object):

    def __init__(self, node):
        self.node = node
        self.msg = Image()

        # create reader
        self.node.create_reader("/realsense/color_image", Image, self.callback)
        # TODO create writer

    def callback(self, data):
        # TODO call reshape method
        # TODO publish compressed image, call write_to_channel method

    def write_to_channel(self):
        # TODO
        pass


if __name__ == '__main__':
    cyber.init()

    # TODO update node to your name
    exercise_node = cyber.Node("exercise1_node_name")
    exercise = Exercise(exercise_node)

    exercise_node.spin()

    cyber.shutdown()
