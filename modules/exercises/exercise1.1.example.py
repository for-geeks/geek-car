#!/usr/bin/env python
import sys

import cv2
import numpy as np

from cyber_py import cyber
from modules.sensors.proto.sensor_image_pb2 import Image
from common.image_utils import reshape

sys.path.append("../")


class Exercise(object):

    def __init__(self, node):
        self.node = node
        self.msg = Image()

        # create reader
        self.node.create_reader("/realsense/color_image", Image, self.callback)
        # create writer
        self.writer = self.node.create_writer("/realsense/color_image/compressed", Image)

    def callback(self, data):
        # print frame number
        print('Frame number is :%s' % data.frame_no)
        # api to reshape image
        self.msg = data
        self.msg.data = reshape(data.data)
        # publish, write compressed image to channel
        self.writer.write(self.msg)
        print('Comressed image have wrote to channel /realsense/color_image/compressed')


if __name__ == '__main__':
    cyber.init()

    # TODO update node to your name
    exercise_node = cyber.Node("exercise1_node_name")
    exercise = Exercise(exercise_node)

    exercise_node.spin()

    cyber.shutdown()
