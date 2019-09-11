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

  auto async = cyber::Async(&MonitorComponent::Realsense, this);

  return true;
}

void MonitorComponent::Realsense() {
  while (true) {
    // First, create a rs2::context.
    // The context represents the current platform with respect to connected
    // devices
    rs2::context ctx;

    // Using the context we can get all connected devices in a device list
    rs2::device_list devices = ctx.query_devices();

    rs2::device selected_device;
    AINFO << "Realsense Device size is :" << devices.size();

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
      selected_device = devices.front();

      // print device_info
      // RealSense::printDeviceInformation(selected_device);

      realsense->set_connection_status(true);
      realsense->set_message("Everything is ok");
      // publish status;
      std::string serial_number =
          selected_device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);

      realsense->set_serial_number(serial_number);

      if (reader_ == nullptr && realsense_ready_.load()) {
        reader_ = node_->CreateReader<Pose>(
            FLAGS_pose_channel, [this](const std::shared_ptr<Pose>& pose) {
              pose_.CopyFrom(*pose);
            });
      }

      realsense_ready_.exchange(true);
      RealsenseField();
    }

    auto arduino = status->mutable_arduino();
    if (Arduino()) {
      arduino->set_connection_status(true);
      arduino->set_message("Device mounted at /dev/ttyACM0");
    } else {
      arduino->set_connection_status(false);
      arduino->set_message("Arduino NOT FOUND or permission refused");
    }

    writer_->Write(status);
  }
}

bool MonitorComponent::Arduino() {
  // check Arduino device

  // TODO(ALL) udev rules
  auto arduino_path = "/dev/ttyACM0";
  // AINFO << "SUCCESS, Arduino Connected.";

  /**
   * @info
   * st_mode 则定义了下列数种情况：
    S_IFMT   0170000    文件类型的位遮罩
    S_IFSOCK 0140000    scoket
    S_IFLNK 0120000     符号连接
    S_IFREG 0100000     一般文件
    S_IFBLK 0060000     区块装置
    S_IFDIR 0040000     目录
    S_IFCHR 0020000     字符装置
    S_IFIFO 0010000     先进先出

    S_ISUID 04000     文件的(set user-id on execution)位
    S_ISGID 02000     文件的(set group-id on execution)位
    S_ISVTX 01000     文件的sticky位

    S_IRUSR(S_IREAD) 00400     文件所有者具可读取权限
    S_IWUSR(S_IWRITE)00200     文件所有者具可写入权限
    S_IXUSR(S_IEXEC) 00100     文件所有者具可执行权限

    S_IRGRP 00040             用户组具可读取权限
    S_IWGRP 00020             用户组具可写入权限
    S_IXGRP 00010             用户组具可执行权限
   *
   */

  struct stat info;
  return stat(arduino_path, &info) == 0 && (info.st_mode & S_IRGRP);
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
