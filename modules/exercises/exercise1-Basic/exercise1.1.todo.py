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

        # TODO create reader

        # TODO create writer
        

    def callback(self, data):
        # TODO print frame number

        # TODO api to reshape image

        # TODO publish, write compressed image
        pass




if __name__ == '__main__':
    cyber.init()

    # TODO update node to your gourp_name or other thing
    exercise_node = cyber.Node("exercise1_node_name")
    exercise = Exercise(exercise_node)

    exercise_node.spin()

    cyber.shutdown()
