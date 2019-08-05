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
#include <cstring>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include "librealsense2/rs.hpp"
#include "modules/common/global_gflags.h"

rs2::device get_device(const std::string& serial_number = "") {
  rs2::context ctx;
  while (true) {
    for (auto&& dev : ctx.query_devices()) {
      if (((serial_number.empty() &&
            std::strstr(dev.get_info(RS2_CAMERA_INFO_NAME), "T265")) ||
           std::strcmp(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER),
                       serial_number.c_str()) == 0))
        return dev;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

int main() {
  // get the device given the serial number
  std::cout << "Waiting for device..." << std::endl;
  auto device = get_device();

  FLAGS_serial_number = device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
  std::cout << "Device with serial number " << FLAGS_serial_number << " got"
            << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(1));

  // open the profiles you need, or all of them
  auto sensor = device.first<rs2::sensor>();
  sensor.open(sensor.get_stream_profiles());

  // start the sensor providing a callback to get the frame
  sensor.start([](rs2::frame f) {
    if (f.get_profile().stream_type() == RS2_STREAM_POSE) {
      auto pose_frame = f.as<rs2::pose_frame>();
      auto pose_data = pose_frame.get_pose_data();
      std::cout << "pose " << pose_data.translation << std::endl;
    } else if (f.get_profile().stream_type() == RS2_STREAM_FISHEYE) {
      // this is one of the fisheye imagers
      auto fisheye_frame = f.as<rs2::video_frame>();
      std::cout << "fisheye " << f.get_profile().stream_index() << ", "
                << fisheye_frame.get_width() << "x"
                << fisheye_frame.get_height() << std::endl;
    }
  });

  std::this_thread::sleep_for(std::chrono::seconds(10));

  // and stop
  sensor.stop();
}
