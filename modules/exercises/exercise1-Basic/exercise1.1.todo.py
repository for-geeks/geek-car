#!/usr/bin/env python
import sys
import time

import cv2
import numpy as np

from cyber_py import cyber

from modules.sensors.proto.sensor_image_pb2 import Image

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

    def reshape(self, data):
        """api to reshape and encodes image, you can call self.reshape(data)"""
        new_image = np.frombuffer(data, dtype=np.uint8)
        img_param = [int(cv2.IMWRITE_JPEG_QUALITY), 12]
        new_image = new_image.reshape(360, 640*3)
        img_encode = cv2.imencode('.jpeg', new_image, img_param)[1]
        data_encode = np.array(img_encode)
        return data_encode.tostring()


if __name__ == '__main__':
    cyber.init()

    # TODO update node to your gourp_name or other thing
    exercise_node = cyber.Node("exercise1_node_name")
    exercise = Exercise(exercise_node)

    exercise_node.spin()

    cyber.shutdown()
