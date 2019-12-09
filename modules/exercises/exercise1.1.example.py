#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import sys
import time

import cv2

from cyber_py import cyber

from modules.sensors.proto.sensor_image_pb2 import Image

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
        print('Frame number is :%s' % data.frame_no)
        # api to reshape image
        self.msg = data
        self.msg.data = self.reshape(data.data)
        # publish, write to channel
        self.write_to_channel()

    def write_to_channel(self):
        # write compressed image
        self.writer.write(self.msg)

    def reshape(self, data):
        """api to reshape and encodes image, you can call self.reshape(data)"""
        new_image = np.frombuffer(data, dtype=np.uint8)
        img_param = [int(cv2.IMWRITE_JPEG_QUALITY), 30]
        new_image = new_image.reshape(480, 640*3)
        img_encode = cv2.imencode('.jpeg', new_image, img_param)[1]
        data_encode = np.array(img_encode)
        return data_encode.tostring()


if __name__ == '__main__':
    cyber.init()

    # TODO update node to your name
    exercise_node = cyber.Node("exercise1_node_name")
    exercise = Exercise(exercise_node)

    exercise_node.spin()

    cyber.shutdown()
