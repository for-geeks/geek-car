#!/usr/bin/env python
import numpy as np
import cv2
import time
from modules.perception.proto.perception_pb2 import LaneInfo
from modules.sensors.proto.sensors_pb2 import Image
from cyber_py import cyber
import sys

sys.path.append("../")


line_msg = LaneInfo()
send_flag = 0

# roll
src_corners = [[191, 223], [272, 223], [182, 269], [297, 269]]

# turn to
dst_corners = [[152, 270], [267, 270], [152, 339], [267, 339]]
M = cv2.getPerspectiveTransform(
    np.float32(src_corners), np.float32(dst_corners))

picnum = 664

# weight 240
# midline_x 228
car_mid_point = 228
weight = 240

road_weight = 260

mask_right_cor = np.array(
    [[[443, 301], [443, 342], [376, 342]]], dtype=np.int32)
mask_right_cor_a = (mask_right_cor[0][0][1] - mask_right_cor[0]
                    [2][1])/(mask_right_cor[0][0][0] - mask_right_cor[0][2][0])
mask_right_cor_b = (mask_right_cor[0][0][1]) - \
    (mask_right_cor[0][0][0])*mask_right_cor_a


def clean_right(x_c, y_c):
    val_ri_x = [tesd(x_c[ir], y_c[ir]) for ir in range(len(x_c))]

    rightx_d = []
    righty_d = []

    for i_i, ir in enumerate(val_ri_x):
        if ir == 0:
            rightx_d.append(x_c[i_i])
            righty_d.append(y_c[i_i])

    return np.array(rightx_d), np.array(righty_d)


def tesd(x, y):
    if (mask_right_cor_b + x*mask_right_cor_a - 4) < y:
        return 1
    else:
        return 0


def get_midpoint(leftx, lefty, rightx, righty, shape):
    mean_x = []
    mean_y = []
    tag_y = shape[0] - 1

    while tag_y >= 0:
        left_x_0 = -1
        right_x_0 = shape[1]
        if tag_y in lefty:
            for i_y, ic in enumerate(lefty):
                if ic == tag_y:
                    left_x_0 = leftx[i_y]
                    break
        if tag_y in righty:
            for i_y, ic in enumerate(righty):
                if ic == tag_y:
                    right_x_0 = rightx[i_y]
                    break

        if left_x_0 == -1 and right_x_0 == shape[1]:
            tag_y -= 1
            continue

        # left no
        if right_x_0 < shape[1] and left_x_0 == -1:
            left_x_0 = right_x_0 - road_weight

        if right_x_0 == shape[1] and left_x_0 > -1:
            right_x_0 = left_x_0 + road_weight

        mean_x.append(int((right_x_0 - left_x_0) / 2 + left_x_0))
        mean_y.append(int(tag_y))

        tag_y -= 1

    return mean_x, mean_y


def laneInfo_router(node):
    g_count = 1
    writer = node.create_writer("/perception/lane_line", LaneInfo)

    while not cyber.is_shutdown():
        global send_flag
        if send_flag:
            g_count = g_count + 1
            global line_msg
            writer.write(line_msg)
            send_flag = 0


def perspective_transform(image, m, img_size=None):
    if img_size is None:
        img_size = (image.shape[1], image.shape[0])
    warped = cv2.warpPerspective(image, m, img_size, flags=cv2.INTER_LINEAR)
    return warped


def get_tag_mask(image_input, tag_roi=(228, 340)):

    gray_d = cv2.cvtColor(image_input, cv2.COLOR_BGR2GRAY)

    img_d = cv2.threshold(gray_d, 100, 220, cv2.THRESH_BINARY_INV)[1]

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    img_d = cv2.erode(img_d, kernel, iterations=2)
    img_d = cv2.dilate(img_d, kernel, iterations=3)

    cv2.fillPoly(img_d, mask_right_cor, 0)

    image, contours, hierarchy = cv2.findContours(img_d, cv2.RETR_EXTERNAL,
                                                  cv2.CHAIN_APPROX_SIMPLE)

    vis = np.array([(tag_roi[0], tag_roi[1]), (tag_roi[0] - 20,
                                               tag_roi[1]), (tag_roi[0] + 20, tag_roi[1])])

    c_max = []
    max_area = 0
    max_cnt = contours[0]
    for i in range(len(contours)):
        cnt = contours[i]
        loca_tem = np.array([cv2.pointPolygonTest(cnt, (vis[0][0], vis[0][1]), False),
                             cv2.pointPolygonTest(
                                 cnt, (vis[1][0], vis[1][1]), False),
                             cv2.pointPolygonTest(cnt, (vis[2][0], vis[2][1]), False)])

        is_black = 0
        if loca_tem.max() <= 0:
            continue

        for i_i, poi in enumerate(loca_tem):
            if poi == 1 and img_d[vis[i_i][1]][vis[i_i][0]] == 220:
                is_black = 1
                break

        if is_black == 1:
            area = cv2.contourArea(cnt)

            if loca_tem.sum() == 3:
                max_cnt = cnt
                max_area = area
                break

            if area > max_area:
                max_cnt = cnt
                max_area = area
            else:
                continue

    temp = np.ones(image_input.shape, np.uint8) * 255
    if max_area > 0:
        c_max.append(max_cnt)
        cv2.drawContours(temp, c_max, -1, (0, 0, 0), thickness=-1)
    else:
        temp = np.dstack((img_d, img_d, img_d)) * 255

    return temp


def translation_view(x, y):
    # x2 = 100.7 + 0.00305 * x1 - 0.1677 * y1
    # y2 = 28.43 - 0.1554 * x1 + 0.008986 * y1
    x_r = (x * 0.00305 - y * 0.1677 + 100.7) / 100.00
    y_r = (x * (-0.1554) + y * 0.008986 + 28.43) / 100.00
    return x_r, y_r


def abs_sobel_thresh(image, sobel_kernel=3, orient='x', thresh=(0, 255)):
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    if orient == 'x':
        sobel = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=sobel_kernel)
    else:
        sobel = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=sobel_kernel)
    abs_sobel = np.absolute(sobel)
    scaled_sobel = np.uint8(255 * abs_sobel / np.max(abs_sobel))
    sxbinary = np.zeros_like(scaled_sobel)
    sxbinary[(scaled_sobel >= thresh[0]) & (scaled_sobel <= thresh[1])] = 1
    return sxbinary


def find_line_fit(img, midpoint=None, nwindows=9, margin=100, minpix=30):
    histogram = np.sum(img[img.shape[0]//2:, :], axis=0)
    # Create an output image to draw on and  visualize the result
    out_img = np.dstack((img, img, img)) * 255
    # Find the peak of the left and right halves of the histogram
    # These will be the starting point for the left and right lines
    if midpoint is None:
        midpoint = np.int(histogram.shape[0]/2)
    else:
        midpoint = np.int(midpoint)
    leftx_base = np.argmax(histogram[:midpoint])
    rightx_base = np.argmax(histogram[midpoint:]) + midpoint

    # Set height of windows
    window_height = np.int(img.shape[0]/nwindows)
    # Identify the x and y positions of all nonzero pixels in the image
    nonzero = img.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    # Current positions to be updated for each window
    leftx_current = leftx_base

    rightx_current = rightx_base
    # Create empty lists to receive left and right lane pixel indices
    left_lane_inds = []
    right_lane_inds = []

    list_line = []

    # Step through the windows one by one
    for window in range(nwindows):
        # Identify window boundaries in x and y (and right and left)
        win_y_low = img.shape[0] - (window+1)*window_height
        win_y_high = img.shape[0] - window*window_height
        win_xleft_low = leftx_current - margin
        win_xleft_high = leftx_current + margin
        win_xright_low = rightx_current - margin
        win_xright_high = rightx_current + margin

        good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                          (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
        good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                           (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]
        # Append these indices to the lists
        left_lane_inds.append(good_left_inds)
        right_lane_inds.append(good_right_inds)
        # If you found > minpix pixels, recenter next window on their mean position
        if len(good_left_inds) > minpix:
            leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
        if len(good_right_inds) > minpix:
            rightx_current = np.int(np.mean(nonzerox[good_right_inds]))

    # Concatenate the arrays of indices
    left_lane_inds = np.concatenate(left_lane_inds)
    right_lane_inds = np.concatenate(right_lane_inds)

    # Extract left and right line pixel positions
    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds]
    rightx = nonzerox[right_lane_inds]
    righty = nonzeroy[right_lane_inds]

    rightx, righty = clean_right(rightx, righty)

    if len(leftx) == 0:
        if len(rightx) > 0:
            #leftx = rightx-240
            leftx = rightx - road_weight
            lefty = righty
        else:
            leftx = [[0] for x_t in range(0, 5)]
            lefty = [[img.shape[0]-y_t*10] for y_t in range(0, 5)]

            rightx = [[img.shape[1]-1] for x_t in range(0, 5)]
            righty = [[img.shape[0] - y_t * 10] for y_t in range(0, 5)]

    if len(righty) == 0:
        if len(leftx) > 0:
            #rightx = leftx + 240
            rightx = leftx + road_weight
            righty = lefty

    mean_x, mean_y = get_midpoint(leftx, lefty, rightx, righty, img.shape)

    # to plot
    #out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]

    #out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 0, 255]

    # Fit a second order polynomial to each

    """
    left_fit = np.polyfit(lefty, leftx, 2)
    right_fit = np.polyfit(righty, rightx, 2)

    leftx_real, lefty_real = translation_view(leftx, lefty)
    rightx_real, righty_real = translation_view(rightx, righty)
    left_fit_real = np.polyfit(leftx_real, lefty_real, 2)
    right_fit_real = np.polyfit(rightx_real, righty_real, 2)

    return left_fit, right_fit, left_fit_real, right_fit_real, out_img

    """
    mean_x_real, mean_y_real = translation_view(mean_x, mean_y)
    return leftx, lefty, rightx, righty, mean_x, mean_y, out_img


def image_reader():
    test_node = cyber.Node("image_router_py")
    test_node.create_reader("/realsense/compressed_image", Image, callback)

    return test_node


def callback(data):
    new_image = np.frombuffer(data.data, dtype=np.uint8)
    new_image = cv2.imdecode(new_image, cv2.IMREAD_COLOR)
    #new_image = new_image.reshape(816/2, 848/2)

    img = cv2.resize(new_image, (424, 408))

    wrap_img = perspective_transform(img, M, img_size=(444, 343))

    wrap_img3 = get_tag_mask(wrap_img)

    binary = abs_sobel_thresh(wrap_img3, orient='y',
                              sobel_kernel=3, thresh=(20, 255))

    left_fit_view, right_fit_view, left_fit, right_fit, out_img2 = find_line_fit(binary,
                                                                                 midpoint=car_mid_point,
                                                                                 margin=100)
    print("left:", left_fit_view, "; right:", right_fit)

    global line_msg

    line_msg.left_lane.a = left_fit[0]
    line_msg.left_lane.b = left_fit[1]
    line_msg.left_lane.c = left_fit[2]

    line_msg.right_lane.a = right_fit[0]
    line_msg.right_lane.b = right_fit[1]
    line_msg.right_lane.c = right_fit[2]

    line_msg.left_lane_view.a = left_fit_view[0]
    line_msg.left_lane_view.b = left_fit_view[1]
    line_msg.left_lane_view.c = left_fit_view[2]

    line_msg.right_lane_view.a = right_fit_view[0]
    line_msg.right_lane_view.b = right_fit_view[1]
    line_msg.right_lane_view.c = right_fit_view[2]

    global send_flag
    send_flag = 1


if __name__ == '__main__':
    cyber.init()
    cyber_node = image_reader()
    laneInfo_router(cyber_node)

    cyber_node.spin()
    cyber.shutdown()
