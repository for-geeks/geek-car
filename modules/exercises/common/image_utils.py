#!/usr/bin/env python

import cv2
import numpy as np

def reshape(data):
        """api to reshape and encodes image, you can call self.reshape(data)"""
        new_image = np.frombuffer(data, dtype=np.uint8)
        img_param = [int(cv2.IMWRITE_JPEG_QUALITY), 20]
        # https://stackoverflow.com/questions/50306863/valueerror-cannot-reshape-array-of-size-50176-into-shape-1-224-224-3
        new_image = new_image.reshape((360, 640, 3))
        img_encode = cv2.imencode('.jpeg', new_image, img_param)[1]
        data_encode = np.array(img_encode)
        return data_encode.tostring()