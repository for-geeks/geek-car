# ****************************************************************************
# Copyright 2018 The Apollo Authors. All Rights Reserved.

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ****************************************************************************
# -*- coding: utf-8 -*-
from cyber_py import cyber
from modules.sensors.proto.sensors_pb2 import Image
"""Module for example of talker."""

import time
import sys
import cv2
import numpy as np
sys.path.append("../")

msg = Image()
send_flag = 0


def image_router():
    """
    Test talker.
    """
    g_count = 1
    test_node = cyber.Node("image_router_py_out")
    writer = test_node.create_writer("/realsense/compressed_image",
                                     Image)
    while not cyber.is_shutdown():
        # time.sleep(1)
        global send_flag
        if (send_flag):
            g_count = g_count + 1
            print "write msg"
            global msg
            # print msg
            writer.write(msg)
            send_flag = 0


def callback(data):
    """
    Reader message callback.
    """
    new_image = np.frombuffer(data.data, dtype=np.uint8)
    img_param = [int(cv2.IMWRITE_JPEG_QUALITY), 30]
    new_image = new_image.reshape(816/2, 848/2)
    img_encode = cv2.imencode('.jpeg', new_image, img_param)[1]
    data_encode = np.array(img_encode)
    str_encode = data_encode.tostring()
    data.data = str_encode
    global msg
    msg = data
    global send_flag
    send_flag = 1


def image_reader():
    """
    Reader message.
    """
    # print "=" * 120
    test_node = cyber.Node("image_router_py")
    test_node.create_reader("/realsense/raw_image",
                            Image, callback)
    # test_node.spin()


if __name__ == '__main__':
    cyber.init("image_router")
    image_reader()
    image_router()
    cyber.shutdown()
