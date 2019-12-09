#!/usr/bin/env python
import sys

sys.path.append("../")
from cyber_py import cyber
from modules.sensors.proto.sensor_image_pb2 import Image

def callback(data):
    print("="*80)
    print("py:reader callback msg->:")
    print(data.frame_no)
    print("="*80)

def test_listener_class():
    print("="*120)
    test_node=cyber.Node("image_reader_lujun")
    test_node.create_reader("/realsense/raw_image", Image, callback)
    test_node.spin()

if __name__ == '__main__':
    cyber.init()
    test_listener_class()
    cyber.shutdown()


