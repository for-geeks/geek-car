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

#include "cyber/common/log.h"
#include "cyber/cyber.h"

namespace apollo {
namespace sensors {

std::map<std::string, int> device_map{{"Intel RealSense T265", 0},
                                      {"Intel RealSense D435", 1},
                                      {"Intel RealSense D435I", 2}};

bool RealsenseComponent::Init() {
  // Init by config device
  InitDeviceAndSensor();
  AINFO << "Init realsense device successfuly";
  return true;
}

void RealsenseComponent::InitDeviceAndSensor() {
  device_ = first_connected_device();

  AINFO << "CONNECTED FIRST DEVICE INFO:";
  RealSense::printDeviceInformation(device_);
  auto sensor = device_.first<rs2::depth_sensor>();
  RealSense::getSensorOption(sensor);

  AWARN << "RS2 CAMERA INFO: " << device_.get_info(RS2_CAMERA_INFO_NAME);
  std::string camera_info = device_.get_info(RS2_CAMERA_INFO_NAME);

  AWARN << "CAMERA MODEL: " << device_map[camera_info];
  switch (device_map[camera_info]) {
    case 0:
      device_object_ = new T265();
      break;
    case 1:
      device_object_ = new D435();
      break;
    case 2:
      device_object_ = new D435I();
      break;

    default:
      AERROR << "The device data is not yet supported for parsing";
      break;
  }

  // Channel writer
  if (!device_object_->Init(node_)) {
    AERROR << "Failed to init Realsense device";
  }
}

RealsenseComponent::~RealsenseComponent() {
  AINFO << "destructor from RealsenseComponent.";
  try {
    delete device_object_;
  } catch (const std::exception &e) {
    std::cerr << "error from destructor:" << e.what() << std::endl;
  }
}

}  // namespace sensors
}  // namespace apollo
