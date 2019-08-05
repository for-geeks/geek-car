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
#include "modules/common/global_gflags.h"

// Arduino
DEFINE_string(device_name, "ttyACM0", "Arduino device name ");

// Realsense
DEFINE_string(device_model, "T265", "T265 model");
DEFINE_string(serial_number, "908412111198", "T265 serial no");
DEFINE_string(pose_channel, "/realsense/pose", "pose data from T265");
DEFINE_string(raw_image_channel, "/realsense/raw_image",
              "raw image Y8 data from T265");
DEFINE_string(compressed_image_channel, "/realsense/compressed_image",
              "compressed image Y8 data from T265");
DEFINE_string(acc_channel, "/realsense/acc", "acc data from T265");
DEFINE_string(gyro_channel, "/realsense/gyro", "gyro data from T265");
DEFINE_string(tags_channel, "/localization/tag", "localization tag result");
DEFINE_string(routing_channel, "/planning/target",
              "planning routing information");
DEFINE_string(planning_channel, "/planning/trajectory",
              "planning trajectory result");

// Module Channel
DEFINE_string(control_channel, "/control", "control message channel");
DEFINE_string(chassis_channel, "/chassis", "chassis message channel");
DEFINE_string(control_ref_channel, "/control_reference", "control message ref");
DEFINE_string(control_coefficient, "/control_coefficient",
              "control coefficient");

// switch
DEFINE_bool(publish_acc, false, "publish acc data");
DEFINE_bool(publish_gyro, false, "publish gyro data");
DEFINE_bool(publish_pose, true, "publish pose data");
DEFINE_bool(publish_raw_image, false, "publish raw gray image data");
DEFINE_bool(publish_compressed_image, true, "publish raw gray image data");
DEFINE_bool(use_compressed_image_to_detect_tag, false,
            "use compressed_image_channel to detect apriltag data");

// const
DEFINE_double(cruise_speed, 0.5, "cruise speed ");
DEFINE_double(longitude_kp, 5.0, "kp");
DEFINE_double(longitude_ki, 5.0, "ki");
DEFINE_double(longitude_ff, 5.0, "ff");
DEFINE_double(offset, 5.3, "ff");

DEFINE_double(tagsize, 0.07, "tag size in meters");
DEFINE_double(left_fx, 142.6265, "fx");
DEFINE_double(left_fy, 143.13, "fy");
DEFINE_double(left_cx, 211.345, "cx");
DEFINE_double(left_cy, 200.011, "cy");

// IMAGE
DEFINE_int32(compress_rate, 30, "compressed image rate");

// TOOLS
DEFINE_string(image_export_dir, "/home/raosiyue/out_test/",
              "tools image saver dir");
DEFINE_string(odometry_file,
              "../modules/sensors/conf/calibration_odometry.json",
              "odometry calibration file ");
