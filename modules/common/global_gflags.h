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
DECLARE_string(device_name);

// RealSense T265
DECLARE_string(device_model);
DECLARE_string(serial_number);
DECLARE_string(pose_channel);
DECLARE_string(raw_image_channel);
DECLARE_string(acc_channel);
DECLARE_string(gyro_channel);

// Module Channel
DECLARE_string(control_channel);
DECLARE_string(chassis_channel);
DECLARE_string(control_ref_channel);
DECLARE_string(control_coefficient);

// switch
DECLARE_bool(publish_acc);
DECLARE_bool(publish_gyro);
DECLARE_bool(publish_pose);
DECLARE_bool(publish_raw_image);

// CONST
DECLARE_double(cruise_speed);
