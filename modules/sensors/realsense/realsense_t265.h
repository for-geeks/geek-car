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

#include <memory>

#include "cyber/node/node.h"
#include "cyber/node/reader.h"
#include "cyber/node/writer.h"
#include "modules/common/global_gflags.h"
#include "modules/control/proto/chassis.pb.h"
#include "modules/sensors/proto/sensor_image.pb.h"
#include "modules/sensors/proto/sensors.pb.h"
#include "modules/sensors/realsense/device_base.h"

namespace apollo {
namespace sensors {
namespace realsense {

using apollo::control::Chassis;
using apollo::cyber::Node;
using apollo::cyber::Reader;
using apollo::cyber::Writer;
using apollo::sensors::Image;
using apollo::sensors::Pose;

class T265 : public DeviceBase {
 public:
  T265(){};
  ~T265();

  bool Init(std::shared_ptr<Node> node_) override;
  void DeviceConfig() override;
  void InitChannelWriter(std::shared_ptr<Node> node_) override;

 private:
  void Run();
  void OnGrayImage(const rs2::frame &fisheye_frame);
  void OnPose(const rs2::pose_frame &pose_frame);

  void Calibration();
  void WheelOdometry();

  std::shared_ptr<Reader<Chassis>> chassis_reader_ = nullptr;

  std::shared_ptr<Writer<Image>> image_writer_ = nullptr;
  std::shared_ptr<Writer<Pose>> pose_writer_ = nullptr;

  Chassis chassis_;

  // fisheye calibration map
  cv::Mat map1_;
  cv::Mat map2_;

  double norm_max = 0;

  const int fisheye_sensor_idx = 1;  // for the left fisheye lens of T265
};
}  // namespace realsense
}  // namespace sensors
}  // namespace apollo
