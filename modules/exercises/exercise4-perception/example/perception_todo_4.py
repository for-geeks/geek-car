#!/usr/bin/env python
import os
import sys

import cv2
import numpy as np

from cyber_py3 import cyber

from modules.sensors.proto.sensor_image_pb2 import Image
from modules.perception.proto.perception_label_box_pb2 import LBox2DList
from modules.perception.proto.perception_label_box_pb2 import LBox2D

import codecs
import time
import paddle.fluid as fluid

sys.path.append("../")

train_parameters = {
    "data_dir": "data/",
    "train_list": "train.txt",
    "eval_list": "eval.txt",
    "class_dim": -1,
    "label_dict": {},
    "num_dict": {},
    "image_count": -1,
    "continue_train": False,     # 是否加载前一次的训练参数，接着训练
    "pretrained": False,
    "pretrained_model_dir": "./pretrained-model",
    "save_model_dir": "./yolo-model",
    "model_prefix": "yolo-v3",
    "freeze_dir": "freeze_model",
    "use_tiny": True, 
    "max_box_num": 5, 
    "num_epochs": 80,
    "train_batch_size": 32, 
    "use_gpu": True,
    "yolo_cfg": {
        # 424 408
        "input_size": [3, 448, 448],
        "anchors": [7, 10, 12, 22, 24, 17, 22, 45, 46, 33, 43, 88, 85, 66, 115, 146, 275, 240],
        "anchor_mask": [[6, 7, 8], [3, 4, 5], [0, 1, 2]]
    },
    "yolo_tiny_cfg": {
        "input_size": [3, 256, 256],
        "anchors": [21, 21, 50, 50, 81, 81, 133, 133, 167, 167, 220, 220],
        "anchor_mask": [[3, 4, 5], [0, 1, 2]]
    },
    "ignore_thresh": 0.7,
    "mean_rgb": [127.5, 127.5, 127.5],
    "mode": "train",
    "multi_data_reader_count": 4,
    "apply_distort": True,
    "nms_top_k": 300,
    "nms_pos_k": 300,
    "valid_thresh": 0.01,
    "nms_thresh": 0.45,
    "image_distort_strategy": {
        "expand_prob": 0.5,
        "expand_max_ratio": 4,
        "hue_prob": 0.5,
        "hue_delta": 18,
        "contrast_prob": 0.5,
        "contrast_delta": 0.5,
        "saturation_prob": 0.5,
        "saturation_delta": 0.5,
        "brightness_prob": 0.5,
        "brightness_delta": 0.125
    },
    "sgd_strategy": {
        "learning_rate": 0.002,
        "lr_epochs": [30, 50, 65],
        "lr_decay": [1, 0.5, 0.25, 0.1]
    },
    "early_stop": {
        "sample_frequency": 50,
        "successive_limit": 3,
        "min_loss": 2.5,
        "min_curr_map": 0.84
    }
}


def init_train_parameters():
    """
    :return:
    """
    label_list = os.path.join(train_parameters['data_dir'], "label_list")
    index = 0
    with codecs.open(label_list, encoding='utf-8') as flist:
        lines = [line.strip() for line in flist]
        for line in lines:
            train_parameters['num_dict'][index] = line.strip()
            train_parameters['label_dict'][line.strip()] = index
            index += 1
        train_parameters['class_dim'] = index


init_train_parameters()
ues_tiny = train_parameters['use_tiny']
yolo_config = train_parameters['yolo_tiny_cfg'] if ues_tiny else train_parameters['yolo_cfg']

target_size = yolo_config['input_size']
anchors = yolo_config['anchors']
anchor_mask = yolo_config['anchor_mask']
label_dict = train_parameters['num_dict']
class_dim = train_parameters['class_dim']
print("label_dict:{} class dim:{}".format(label_dict, class_dim))
place = fluid.CUDAPlace(0) if train_parameters['use_gpu'] else fluid.CPUPlace()
exe = fluid.Executor(place)
path = train_parameters['freeze_dir']
[inference_program, feed_target_names, fetch_targets] = fluid.io.load_inference_model(dirname=path, executor=exe)


def read_image_cv(origin):
    """
    读取图片
    :param img_path:
    :return:
    """
    img = cv2.resize(origin,(target_size[1:][0],target_size[1:][1]))
    resized_img = img.copy()
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = np.array(img).astype('float32').transpose((2, 0, 1))  # HWC to CHW
    img -= 127.5
    img *= 0.007843
    img = img[np.newaxis, :]
    return origin, img, resized_img


class Exercise(object):

    def __init__(self, node):
        self.node = node
        self.infer_boxs = LBox2DList()

        # TODO create reader
        self.node.create_reader("/realsense/color_image/compressed", Image, self.callback)
        # TODO create writer
        self.writer = self.node.create_writer(
            "/perception/inference_box", LBox2DList)

    def callback(self, data):
        # TODO
        # print(data.frame_no)
        # TODO infer
        self.infer(data)
        # TODO publish, write to channel
        if not cyber.is_shutdown():
            self.write_to_channel()

    def write_to_channel(self):
        # TODO
        self.writer.write(self.infer_boxs)

    def infer(self, data):

        # TODO e
        image = np.frombuffer(data.data, dtype=np.uint8)
        image = cv2.imdecode(image, cv2.IMREAD_COLOR)
        
        origin, tensor_img, resized_img = read_image_cv(image)
        input_w, input_h = image.shape[1], image.shape[0]
        image_shape = np.array([input_h, input_w], dtype='int32')
        
        t1 = time.time()
        batch_outputs = exe.run(inference_program,
                            feed={feed_target_names[0]: tensor_img,
                                  feed_target_names[1]: image_shape[np.newaxis, :]},
                            fetch_list=fetch_targets,
                            return_numpy=False)
        period = time.time() - t1
        print("predict cost time:{0}".format("%2.2f sec" % period))
        bboxs = np.array(batch_outputs[0])
        # print(bboxs)
        
        self.infer_boxs = LBox2DList()
        if bboxs.shape[1] != 6:
            print("No object found in this")
        else:
            labels = bboxs[:, 0].astype('int32')
            scores = bboxs[:, 1].astype('float32')
            boxes = bboxs[:, 2:].astype('int32')

            for box, label, score in zip(boxes, labels, scores):
                if score > 0.05 and ((box[2]-box[0])>30 and (box[3]-box[1])>30):
                    box_one = LBox2D()
                    box_one.xmin = int(box[0])
                    box_one.ymin = int(box[1])
                    box_one.xmax = int(box[2])
                    box_one.ymax = int(box[3])
                    box_one.label = str(label)
                    box_one.probability = score
                    self.infer_boxs.box.append(box_one)


if __name__ == '__main__':
    cyber.init()

    # TODO update node to your name
    exercise_node = cyber.Node("inference_box")
    exercise = Exercise(exercise_node)

    exercise_node.spin()

    cyber.shutdown()
