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

        # TODO create reader
        self.node.create_reader("/realsense/raw_image", Image, self.callback)
        # TODO create writer
        self.writer = self.node.create_writer(
            "/realsense/compressed_image", Image)

    def callback(self, data):
        # TODO
        print(data.frame_no)
        # TODO reshape
        self.msg = data
        self.msg.data = util.reshape(data.data)
        # TODO publish, write to channel
        self.write_to_channel()

    def write_to_channel(self):
        # TODO
        self.writer.write(self.msg)


if __name__ == '__main__':
    cyber.init()

    # TODO update node to your name
    exercise_node = cyber.Node("your_name")
    exercise = Exercise(exercise_node)

    exercise_node.spin()

    cyber.shutdown()
