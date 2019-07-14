# -*- coding: utf-8 -*-
import time
import sys
import cv2
import numpy as np

from cyber_py import cyber
from modules.sensors.proto.sensors_pb2 import Image


def reshape(data):
    """
    Reshape Image from T265.
    """
    new_image = np.frombuffer(data.data, dtype=np.uint8)
    img_param = [int(cv2.IMWRITE_JPEG_QUALITY), 30]
    new_image = new_image.reshape(data.height * 0.5, data.width * 0.5)
    img_encode = cv2.imencode('.jpeg', new_image, img_param)[1]
    data_encode = np.array(img_encode)
    str_encode = data_encode.tostring()
    data.data = str_encode

    return data
