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

#include "modules/monitor/monitor_component.h"

#include <string>

#include "librealsense2/rs.hpp"
#include "modules/common/global_gflags.h"
#include "modules/monitor/proto/status.pb.h"
#include "modules/sensors/realsense.h"

namespace apollo {
namespace monitor {

using apollo::monitor::Status;

bool MonitorComponent::Init() {
  writer_ = node_->CreateWriter<Status>("/monitor");

  // async_result_ = cyber::Async(&MonitorComponent::Realsense, this);

  return true;
}

void MonitorComponent::Realsense() {
  // First, create a rs2::context.
  // The context represents the current platform with respect to connected
  // devices
  rs2::context ctx;

  // Using the context we can get all connected devices in a device list
  rs2::device_list devices = ctx.query_devices();

  rs2::device selected_device;

  auto status = std::make_shared<Status>();

  auto realsense = status->mutable_realsense();
  if (devices.size() == 0) {
    std::string message =
        "No device connected, please connect a RealSense device";

    AERROR << message;

    // To help with the boilerplate code of waiting for a device to connect
    // The SDK provides the rs2::device_hub class
    rs2::device_hub device_hub(ctx);

    // Using the device_hub we can block the program until a device connects
    // selected_device = device_hub.wait_for_device();
    realsense->set_connection_status(false);
    realsense->set_message(message);
  } else {
    // Update the selected device
    selected_device = devices[0];
    reader_ = node_->CreateReader<Pose>(
        FLAGS_pose_channel,
        [this](const std::shared_ptr<Pose>& pose) { pose_.CopyFrom(*pose); });
    RealsenseField();
  }

  // print device_info
  RealSense::printDeviceInformation(selected_device);

  // publish status;
  std::string serial_number =
      selected_device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);

  realsense->set_connection_status(true);
  realsense->set_message("Everything is ok");
  realsense->set_serial_number(serial_number);

  auto arduino = status->mutable_arduino();
  if (apollo::cyber::common::PathExists("/dev/ttyACM0")) {
    arduino->set_connection_status(true);
    arduino->set_message("Device mounted at /dev/ttyACM0");
  }
  arduino->set_connection_status(false);
  arduino->set_message("Arduino NOT FOUND at /dev/ttyACM0");

  writer_->Write(status);
}

void MonitorComponent::Arduino() {
  // check Arduino device

  // TODO(ALL) udev rules
  if (apollo::cyber::common::PathExists("/dev/ttyACM0")) {
    AINFO << "SUCCESS, Arduino Connected.";
  }

  // std::string cmd = "/apollo/scripts/realsense.sh";
  // const int ret = std::system(cmd.c_str());
  // if (ret == 0) {
  //   AINFO << "SUCCESS: " << cmd;
  // } else {
  //   AERROR << "FAILED(" << ret << "): " << cmd;
  // }
}

void MonitorComponent::RealsenseField() {
  if (std::to_string(pose_.translation().x()) == "nan") {
    // restart realsense_component
    std::string cmd = "/apollo/scripts/realsense.sh restart";

    AWARN << "realsense T265 return nan, waitting for respawn";
    const int ret = std::system(cmd.c_str());
    if (ret == 0) {
      AINFO << "SUCCESS: " << cmd;
    } else {
      AERROR << "FAILED(" << ret << "): " << cmd;
    }
  }
}

}  // namespace monitor
}  // namespace apollo
