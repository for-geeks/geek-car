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
#include "modules/sensors/realsense_component.h"

#include <chrono>
#include <iomanip>
#include <iostream>
#include <thread>
#include <utility>

#include "cyber/common/log.h"
#include "cyber/cyber.h"

#include "modules/sensors/realsense.h"

namespace apollo {
namespace sensors {

bool RealsenseComponent::Init() {
  // TODO(FENGZONGBAO): READ CONFIG

  // Init by config device
  InitDeviceAndSensor();
  AINFO << "Init realsense device successfuly";
  return true;
}

void RealsenseComponent::InitDeviceAndSensor() {
  device_ = first_connected_device();

  if (std::strstr(device_.get_info(RS2_CAMERA_INFO_NAME), "T265")) {
    // device_object_ = new T265();
  } else if (std::strstr(device_.get_info(RS2_CAMERA_INFO_NAME), "D435I")) {
    // device_object_ = D435I();
  } else {
    AERROR << "The device data is not yet supported for parsing";
  }

  if (!device_object_.Init()) {
    AERROR << "Failed to init Realsense device";
  }
}

RealsenseComponent::~RealsenseComponent() {
  if (sensor_) {
    sensor_.stop();
    sensor_.close();
  }
}

}  // namespace sensors
}  // namespace apollo
