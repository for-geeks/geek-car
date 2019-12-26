/******************************************************************************
 * MIT License

 * Copyright (c) 2019 Geekstyle

 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
******************************************************************************/

#pragma once

#include "gflags/gflags.h"

// Arduino
DECLARE_string(arduino_device_name);
DECLARE_string(nooploop_device_name);

// RealSense
DECLARE_string(serial_number);

DECLARE_string(pose_channel);
DECLARE_string(gray_image_channel);
DECLARE_string(color_image_channel);
DECLARE_string(depth_image_channel);
DECLARE_string(point_cloud_channel);
DECLARE_string(compressed_gray_image_channel);
DECLARE_string(compressed_color_image_channel);
DECLARE_string(realsense_acc_channel);
DECLARE_string(realsense_gyro_channel);
DECLARE_string(uwb_pose_channel);
DECLARE_string(uwb_acc_channel);
DECLARE_string(uwb_gyro_channel);

// Module Channel
DECLARE_string(control_channel);
DECLARE_string(chassis_channel);
DECLARE_string(control_ref_channel);
DECLARE_string(control_coefficient);
DECLARE_string(tags_channel);
DECLARE_string(routing_channel);
DECLARE_string(planning_channel);
DECLARE_string(obstacle_channel);

// switch
DECLARE_bool(publish_realsense_acc);
DECLARE_bool(publish_realsense_gyro);
DECLARE_bool(publish_pose);
DECLARE_bool(publish_raw_gray_image);
DECLARE_bool(publish_compressed_gray_image);
DECLARE_bool(use_compressed_image_to_detect_tag);
DECLARE_bool(publish_depth_image);
DECLARE_bool(publish_color_image);
DECLARE_bool(publish_compressed_color_image);
DECLARE_bool(publish_point_cloud);
DECLARE_bool(enable_point_cloud_transform);
DECLARE_bool(publish_tagframe);

DECLARE_bool(publish_nooploop_acc);
DECLARE_bool(publish_nooploop_gyro);

DECLARE_bool(save_pcd);
DECLARE_bool(pcl_visualization);

// CONST
DECLARE_double(cruise_speed);
DECLARE_double(longitude_kp);
DECLARE_double(longitude_ki);
DECLARE_double(longitude_ff);
DECLARE_double(offset);

DECLARE_double(tagsize);
DECLARE_double(left_fx);
DECLARE_double(left_fy);
DECLARE_double(left_cx);
DECLARE_double(left_cy);
DECLARE_double(speed_feedback);
DECLARE_double(point_cloud_min_distance);
DECLARE_double(point_cloud_max_distance);
DECLARE_double(passthrough_y_min);
DECLARE_double(passthrough_y_max);
DECLARE_double(temp_filter_alpha);
DECLARE_double(temp_filter_delta);
DECLARE_double(angle_x);
DECLARE_double(angle_y);
DECLARE_double(angle_z);
DECLARE_double(leaf_size);
DECLARE_double(cluster_radius);
DECLARE_int32(min_cluster_size);
DECLARE_int32(max_cluster_size);

// IMAGE
DECLARE_int32(compress_rate);
DECLARE_int32(color_image_height);
DECLARE_int32(color_image_width);
DECLARE_int32(color_image_frequency);

DECLARE_int32(depth_image_height);
DECLARE_int32(depth_image_width);
DECLARE_int32(depth_image_frequency);

// TOOLS
DECLARE_string(image_export_dir);
DECLARE_string(odometry_file);

