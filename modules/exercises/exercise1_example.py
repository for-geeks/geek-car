# -*- coding: utf-8 -*-
import numpy as np
import cv2
import time
from modules.sensors.proto.sensors_pb2 import Image
from cyber_py import cyber
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
        self.reshape(data)
        # TODO publish, write to channel
        self.write_to_channel()

    def write_to_channel(self):
        # TODO
        self.writer.write(self.msg)

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
