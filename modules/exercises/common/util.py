# -*- coding: utf-8 -*-
import time
import sys
import cv2
import numpy as np

from cyber_py import cyber
from modules.sensors.proto.sensors_pb2 import Image


def reshape(self, data):
    """reshape and encodes image"""
    new_image = np.frombuffer(data, dtype=np.uint8)
    img_param = [int(cv2.IMWRITE_JPEG_QUALITY), 30]
    new_image = new_image.reshape(816/2, 848/2)
    img_encode = cv2.imencode('.jpeg', new_image, img_param)[1]
    data_encode = np.array(img_encode)
    return data_encode.tostring()
