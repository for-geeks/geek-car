#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import cv2
import time
from modules.sensors.proto.sensors_pb2 import Image
from cyber_py import cyber
import sys

sys.path.append("../")

# roll
src_corners = [[191, 223], [272, 223], [182, 269], [297, 269]]

# turn to
dst_corners = [[152, 270], [267, 270], [152, 339], [267, 339]]
M = cv2.getPerspectiveTransform(
    np.float32(src_corners), np.float32(dst_corners))


def perspective_transform(image, m, img_size=None):
    if img_size is None:
        img_size = (image.shape[1], image.shape[0])
    warped = cv2.warpPerspective(image, m, img_size, flags=cv2.INTER_LINEAR)
    return warped


class Exercise(object):

    def __init__(self, node):
        self.node = node
        self.msg = Image()

        # TODO create reader
        self.node.create_reader(
            "/realsense/compressed_image", Image, self.callback)
        # TODO create writer
        self.writer = self.node.create_writer(
            "/perception/vertical_view", Image)

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

        # TODO e begin
        new_image = np.frombuffer(data.data, dtype=np.uint8)
        new_image = cv2.imdecode(new_image, cv2.IMREAD_COLOR)
        # TODO e end

        img = cv2.resize(new_image, (424, 408))

        wrap_img = perspective_transform(img, M, img_size=(444, 343))

        # TODO f begin
        gray_d = cv2.cvtColor(wrap_img, cv2.COLOR_BGR2GRAY)
        # TODO f end

        # TODO h begin, threshold
        wrap_img = cv2.threshold(gray_d, 100, 220, cv2.THRESH_BINARY_INV)[1]
        # TODO h end

        # TODO i begin
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        img_d = cv2.erode(wrap_img, kernel, iterations=2)
        wrap_img = cv2.dilate(img_d, kernel, iterations=3)
        # TODO i end

        img_param = [int(cv2.IMWRITE_JPEG_QUALITY), 30]

        img_encode = cv2.imencode('.jpeg', wrap_img, img_param)[1]
        data_encode = np.array(img_encode)
        str_encode = data_encode.tostring()
        data.data = str_encode
        self.msg = data


if __name__ == '__main__':
    cyber.init()

    # TODO update node to your name
    exercise_node = cyber.Node("threshold")
    exercise = Exercise(exercise_node)

    exercise_node.spin()

    cyber.shutdown()
