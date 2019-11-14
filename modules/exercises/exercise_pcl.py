# -*- coding: utf-8 -*-

import time
import sys
import random

import pcl
import numpy as np
import cv2

import pcl.pcl_visualization

from modules.sensors.proto.sensors_pb2 import Image
from cyber_py import cyber
import modules.exercises.common.util as util

sys.path.append("../")


class Exercise(object):

    def __init__(self, node):
        self.node = node
        self.msg = Image()

        # TODO create reader
        self.node.create_reader("/realsense/point_cloud", Image, self.callback)
        # TODO create writer

    def callback(self, data):
        # TODO
        print(data.frame_no)
        # TODO reshape
        #self.msg = data
        #self.msg.data = util.reshape(data.data)
        # TODO publish, write to channel
        #self.write_to_channel()
        # pcl::PointCloud<pcl::PointXYZRGB> cloud;
        cloud = pcl.PointCloud_PointXYZRGB()

        # Fill in the cloud data
        # cloud.width  = 15;
        # cloud.height = 10;
        # cloud.points.resize (cloud.width * cloud.height)
        # cloud.resize (np.array([15, 10], dtype=np.float))
        # points = np.zeros((10, 15, 4), dtype=np.float32)
        points = np.zeros((150, 4), dtype=np.float32)
        RAND_MAX = 1.0
        # Generate the data
        for i in range(0, 75):
            # set Point Plane
            points[i][0] = 1024 * random.random() / (RAND_MAX + 1.0)
            points[i][1] = 1024 * random.random() / (RAND_MAX + 1.0)
            points[i][2] = 0.1 * random.random() / (RAND_MAX + 1.0)
            points[i][3] = 255 << 16 | 255 << 8 | 255

        for i in range(75, 150):
            # set Point Randomize
            points[i][0] = 1024 * random.random() / (RAND_MAX + 1.0)
            points[i][1] = 1024 * random.random() / (RAND_MAX + 1.0)
            points[i][2] = 1024 * random.random() / (RAND_MAX + 1.0)
            points[i][3] = 255 << 16 | 255 << 8 | 255

        # Set a few outliers
        points[0][2] = 2.0
        points[3][2] = -2.0
        points[6][2] = 4.0

        print(cloud)

        for i in range(0, 150):
            print(points[i][0], points[i][1], points[i][2], points[i][3])

        cloud.from_array(points)

        # Create the segmentation object
        # pcl::SACSegmentation<pcl::PointXYZRGB> seg
        seg = cloud.make_segmenter()
        # Optional
        seg.set_optimize_coefficients(True)
        # Mandatory
        seg.set_model_type(pcl.SACMODEL_PLANE)
        seg.set_method_type(pcl.SAC_RANSAC)
        seg.set_distance_threshold(0.1)

        # pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients)
        # pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        inliers, model = seg.segment()

        # if inliers.size
        #   return
        # end

        print(model)
        # std::cerr << "Model coefficients: " << coefficients->values[0] << " "
        # << coefficients->values[1] << " "
        # << coefficients->values[2] << " "
        # << coefficients->values[3] << std::endl;
        #
        # std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
        # for (size_t i = 0; i < inliers->indices.size (); ++i)
        # {
        #   std::cerr << inliers->indices[i] << "    " << cloud.points[inliers->indices[i]].x << " "
        #   << cloud.points[inliers->indices[i]].y << " "
        #   << cloud.points[inliers->indices[i]].z << std::endl;
        #   cloud.points[inliers->indices[i]].r = 255;
        #   cloud.points[inliers->indices[i]].g = 0;
        #   cloud.points[inliers->indices[i]].b = 0;
        # }
        for i in inliers:
            points[i][3] = 255 << 16 | 255 << 8 | 0

        cloud.from_array(points)

        #
        # pcl::visualization::CloudViewer viewer("Cloud Viewer");
        # viewer.showCloud(cloud.makeShared());
        # while (!viewer.wasStopped ())
        visual = pcl.pcl_visualization.CloudViewing()
        visual.ShowColorCloud(cloud)

        v = True
        while v:
            v = not(visual.WasStopped())

    def write_to_channel(self):
        # TODO
        self.writer.write(self.msg)


if __name__ == '__main__':
    cyber.init()

    # TODO update node to your name
    exercise_node = cyber.Node("your_name")
    exercise = Exercise(exercise_node)

    exercise_node.spin()

    cyber.shutdown()
