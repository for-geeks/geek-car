# -*- coding:utf-8 -*-

import sys
from cyber_py import cyber
from modules.sensors.proto.sensors_pb2 import Image
import time
import cv2
import numpy as np
sys.path.append("../")

msg = Image()
send_flag = 0

def callback(data):
    print "=" * 80
    print "py:reader callback msg->:"
    print data.frame_no
    print "=" * 80

def test_listener_class():
    print "=" * 120
    test_node = cyber.Node("image reader")
    test_node.create_reader("/realsense/raw_image",Image, callback)
    test_node.spin()

#if __name__ == '__main__':
#    cyber.init()
#    test_listener_class()
#    cyber.shutdown()

def image_router(node):

    g_count = 1

    writer = node.create_writer("/realsense/compressed_image", Image)

    while not cyber.is_shutdown():
        global send_flag
        if(send_flag):
            g_count += 1
            global msg
            writer.write(msg)
            send_flag = 0

def callback_reader(data):
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
    test_node = cyber.Node("image_router_py")
    test_node.create_reader("/realsense/raw_image", Image, callback_reader)
    return test_node

if __name__ == '__main__':
    cyber.init("image_router")
    #test_listener_class()
    cyber_node = image_reader()
    image_router(cyber_node)
    cyber.shutdown()

