#!/usr/bin/env python
import os
import sys

import cv2
import numpy as np

from cyber_py3 import cyber

from modules.sensors.proto.sensor_image_pb2 import Image
from modules.planning.proto.planning_pb2 import Trajectory
from modules.planning.proto.planning_pb2 import Point

sys.path.append("../")

# roll
src_corners = [[274, 250], [438, 252], [250, 339], [502, 341]]

# turn to
dst_corners = [[262, 470], [342, 470], [262, 550], [342, 550]]

M = cv2.getPerspectiveTransform(np.float32(src_corners), np.float32(dst_corners))

car_mid_point = 228


def perspective_transform(image, m, img_size=None):
    if img_size is None:
        img_size = (image.shape[1], image.shape[0])
    warped = cv2.warpPerspective(image, m, img_size, flags=cv2.INTER_LINEAR)
    return warped


def color_mask(hsv, low, high):
    # Return mask from HSV
    mask = cv2.inRange(hsv, low, high)
    return mask


def apply_yellow_white_mask(img):
    gradx = abs_sobel_thresh(img, orient='x', sobel_kernel=3, thresh=(20, 255))
    # TODO h
    image_HSV = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    yellow_hsv_low = np.array([0,  100,  100])
    yellow_hsv_high = np.array([149, 255, 255])
    mask_yellow = color_mask(image_HSV, yellow_hsv_low, yellow_hsv_high)
    mask_yellow[(mask_yellow != 0)] = 1
    combined_lsx = np.zeros_like(gradx)
    combined_lsx[((gradx == 1) | (mask_yellow == 1))] = 1
    return mask_yellow


def get_win_point(leftx, lefty, shape):

    left_x_re = []

    tag_y = shape[0] - 1

    left_tmp = -1
    get_polt_tag = 0

    bs_tag = int(shape[1] / 50)

    pox_arr_x_l = []
    pox_arr_y = []

    while tag_y >= 0:
        left_x_0 = -1

        if tag_y in lefty:
            for i_y, ic in enumerate(lefty):
                if ic == tag_y:
                    left_x_0 = leftx[i_y]
                    break

        if left_x_0 == -1:
            tag_y -= 1
            get_polt_tag += 1
            continue

        left_tmp = left_x_0

        if get_polt_tag < bs_tag:
            pox_arr_x_l.append(left_x_0)

            pox_arr_y.append(tag_y)
        else:
            if len(pox_arr_y) > 0:
                x_l = int(np.sum(pox_arr_x_l) // len(pox_arr_x_l))

                left_x_re.append(x_l)

                pox_arr_x_l = []
                pox_arr_y = []

            get_polt_tag = 0

        tag_y -= 1
        get_polt_tag += 1

    return left_x_re


def abs_sobel_thresh(image, sobel_kernel=3, orient='x', thresh=(0, 255)):
    # TODO f
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

    # INFO g
    if orient == 'x':
        sobel = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=sobel_kernel)
    else:
        sobel = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=sobel_kernel)

    abs_sobel = np.absolute(sobel)
    scaled_sobel = np.uint8(255 * abs_sobel / np.max(abs_sobel))
    sxbinary = np.zeros_like(scaled_sobel)
    sxbinary[(scaled_sobel >= thresh[0]) & (scaled_sobel <= thresh[1])] = 1
    return sxbinary


def translation_view(x, y):
    x_r = 125.3 - 0.003918 * x - 0.1418 * y
    y_r = 48.78 - 0.1446 * x - y * 0.008061
    return x_r, y_r


def find_line_fit(img, midpoint=None, nwindows=20, margin=100, minpix=30):
    histogram = np.sum(img[img.shape[0] // 2:, :], axis=0)
    leftx_base = np.argmax(histogram)

    # Set height of windows
    window_height = np.int(img.shape[0] / nwindows)
    # Identify the x and y positions of all nonzero pixels in the image
    nonzero = img.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    # Current positions to be updated for each window
    leftx_current = leftx_base
    line_rank_x = []
    line_rank_y = []

    # Create empty lists to receive left and right lane pixel indices
    left_lane_inds = []

    # Step through the windows one by one
    for window in range(nwindows):
        # Identify window boundaries in x and y (and right and left)
        win_y_low = img.shape[0] - (window + 1) * window_height
        win_y_high = img.shape[0] - window * window_height
        win_xleft_low = leftx_current - margin
        win_xleft_high = leftx_current + margin

        good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                          (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
        # Append these indices to the lists

        left_lane_inds.append(good_left_inds)

        # If you found > minpix pixels, recenter next window on their mean position
        if len(good_left_inds) > minpix:
            leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
            line_rank_x.append(leftx_current)
            line_rank_y.append((win_y_low+win_y_high)//2)

    left_list = get_win_point(line_rank_x, line_rank_y, img.shape)

    return left_list, line_rank_x, line_rank_y


class Exercise(object):

    def __init__(self, node):
        self.node = node
        self.planning_path = Trajectory()

        # TODO create reader
        self.node.create_reader("/realsense/color_image/compressed", Image, self.callback)
        # TODO create writer
        self.writer = self.node.create_writer(
            "/perception/road_mean_point", Trajectory)

    def callback(self, data):
        # TODO
        # print(data.frame_no)
        # TODO reshape
        self.getmeanpoint(data)
        # TODO publish, write to channel
        if not cyber.is_shutdown():
            self.write_to_channel()

    def write_to_channel(self):
        # TODO
        self.writer.write(self.planning_path)

    def getmeanpoint(self, data):

        # TODO e
        image = np.frombuffer(data.data, dtype=np.uint8)
        image = cv2.imdecode(image, cv2.IMREAD_COLOR)

        wrap_img = perspective_transform(image, M, (580, 560))

        yellow_line = apply_yellow_white_mask(wrap_img)

        # TODO I
        line_list, mean_x, mean_y = find_line_fit(
            yellow_line, midpoint=car_mid_point, nwindows=10, margin=100)

        self.planning_path = Trajectory()
        #print("point size:",str(len(mean_y)))
        if len(mean_y) > 0:
            mean_x_real, mean_y_real = translation_view(np.asarray(mean_x), np.asarray(mean_y))

            for i, point in enumerate(mean_y_real):
                point_xy = Point()

                point_xy.x = mean_x_real[i]
                point_xy.y = point
                self.planning_path.point.append(point_xy)


if __name__ == '__main__':
    cyber.init()

    # TODO update node to your name
    exercise_node = cyber.Node("to_mid_point")
    exercise = Exercise(exercise_node)

    exercise_node.spin()

    cyber.shutdown()
