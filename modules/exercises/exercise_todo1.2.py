# -*- coding: utf-8 -*-
import numpy as np
import cv2
import time
from modules.sensors.proto.sensor_image_pb2 import Image, CompressImage
from cyber_py import cyber
import sys

sys.path.append("../")


class Exercise(object):

    def __init__(self, node):
        self.node = node
        self.msg = Image()

        # create reader
        self.node.create_reader("/realsense/raw_image", Image, self.callback)
        # TODO create writer

    def callback(self, data):
        # TODO call reshape method
        # TODO publish compressed image, call write_to_channel method

    def write_to_channel(self):
        # TODO
        pass

    def reshape(self, data):
        new_image = np.frombuffer(data.data, dtype=np.uint8)
        img_param = [int(cv2.IMWRITE_JPEG_QUALITY), 30]
        new_image = new_image.reshape(816/2, 848/2)
        img_encode = cv2.imencode('.jpeg', new_image, img_param)[1]
        data_encode = np.array(img_encode)
        str_encode = data_encode.tostring()
        data.data = str_encode
        self.msg = data


if __name__ == '__main__':
    cyber.init()

    # TODO update node to your name
    exercise_node = cyber.Node("your_name")
    exercise = Exercise(exercise_node)

    exercise_node.spin()

    cyber.shutdown()
