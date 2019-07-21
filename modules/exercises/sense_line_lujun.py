import sys

sys.path.append("../")
from cyber_py import cyber
from modules.sensors.proto.sensors_pb2 import Image
from modules.perception.proto.control_pb2 import LaneInfo

import time
import sys
import cv2
import numpy as np

msg = Image()
send_flag = 0

# roll
src_corners = [[191, 223], [272, 223], [182, 269], [297, 269]]

# turn to
dst_corners = [[152, 270], [267, 270], [152, 339], [267, 339]]
M = cv2.getPerspectiveTransform(np.float32(src_corners), np.float32(dst_corners))

picnum = 664

# weight 240
# midline_x 228
car_mid_point = 228


def image_router(node):
    g_count = 1
    #writer = node.create_writer("/realsense/compressed_image", Image)

    while not cyber.is_shutdown():
        global send_flag
        if send_flag:
            g_count = g_count + 1
            global msg
            #writer.write(msg)
            send_flag = 1


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

    image, contours, hierarchy = cv2.findContours(img_d, cv2.RETR_EXTERNAL,
                                                  cv2.CHAIN_APPROX_SIMPLE)

    c_max = []
    max_area = 0
    max_cnt = contours[0]
    for i in range(len(contours)):
        cnt = contours[i]
        loca_tem = cv2.pointPolygonTest(cnt, (tag_roi[0], tag_roi[1]), False) \
                   + cv2.pointPolygonTest(cnt, (tag_roi[0]-10, tag_roi[1]), False) \
                   + cv2.pointPolygonTest(cnt, (tag_roi[0]+10, tag_roi[1]), False)
        if loca_tem >= 0:
            max_cnt = cnt
            area = cv2.contourArea(cnt)
            if loca_tem == 3:
                max_area = area
                break
            if area > max_area:
                max_cnt = cnt
                max_area = area
            else:
                contours
    temp = np.ones(image_input.shape, np.uint8) * 255
    if max_area > 0:
        c_max.append(max_cnt)
        cv2.drawContours(temp, c_max, -1, (0, 0, 0), thickness=-1)
    else:
        temp = np.dstack((img_d, img_d, img_d)) * 255

    return temp


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


    # to plot
    out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
    out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 0, 255]

    # Fit a second order polynomial to each
    left_fit = np.polyfit(lefty, leftx, 2)
    right_fit = np.polyfit(righty, rightx, 2)

    return left_fit, right_fit, out_img


def image_reader():
    test_node = cyber.Node("image_router_py")
    test_node.create_reader("/realsense/compressed_image", Image, callback)

    return test_node


def callback(data):
    new_image = np.frombuffer(data.data, dtype=np.uint8)
    new_image = cv2.imdecode(new_image, cv2.IMREAD_COLOR)
    #new_image = new_image.reshape(408, 424)

    img = cv2.resize(new_image, (424, 408))

    wrap_img = perspective_transform(img, M, img_size=(444, 343))

    wrap_img3 = get_tag_mask(wrap_img)

    binary = abs_sobel_thresh(wrap_img3, orient='y', sobel_kernel=3, thresh=(20, 255))

    left_fit, right_fit, out_img2 = find_line_fit(binary, midpoint=car_mid_point, margin=100)

    #data_encode = np.array([left_fit, right_fit])
    #data.data = data_encode.tostring()
    print(left_fit, right_fit)
    data.data = ""
    #print(data_encode.tostring())
    global msg
    msg = data
    global send_flag
    send_flag = 1


if __name__ == '__main__':
    cyber.init()
    cyber_node = image_reader()
    image_router(cyber_node)

    cyber_node.spin()
    cyber.shutdown()

