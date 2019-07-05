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
#include <string>
#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "librealsense2/rs.hpp"
#include "opencv2/opencv.hpp"

#include "modules/sensors/proto/sensors.pb.h"

namespace apollo {
namespace sensors {

using apollo::cyber::Component;
using apollo::cyber::Writer;

class RealsenseComponent : public Component<> {
 public:
  bool Init() override;
  void run();
  ~RealsenseComponent();

 private:
  void Calibration();
  void OnImage(cv::Mat dst, uint64 frame_no);
  void OnPose(rs2_pose pose_data, uint64 frame_no);
  void OnAcc(rs2_vector acc, uint64 frame_no);
  void OnGyro(rs2_vector gyro, uint64 frame_no);

  std::shared_ptr<Writer<Image>> image_writer_ = nullptr;
  std::shared_ptr<Writer<Pose>> pose_writer_ = nullptr;
  std::shared_ptr<Writer<Acc>> acc_writer_ = nullptr;
  std::shared_ptr<Writer<Gyro>> gyro_writer_ = nullptr;

  std::future<void> async_result_;
  rs2::device device_;  // realsense device
  rs2::sensor sensor_;  // sensor include imu and camera;

  uint32_t device_wait_ = 2000;  // ms
  uint32_t spin_rate_ = 200;     // ms

  // frame queue
  rs2::frame_queue q_;

  /**
   * @brief from RS2_OPTION_FRAMES_QUEUE_SIZE
   * you are telling the SDK not to recycle frames for this sensor.
   * < Number of frames the user is allowed to keep per stream. Trying to
   * hold-on to more frames will cause frame-drops.
   * */
  float queue_size_ = 16.0;  // queue size

  std::string serial_number_ = "908412111198";  // serial number

  cv::Mat map1_;
  cv::Mat map2_;
};

CYBER_REGISTER_COMPONENT(RealsenseComponent)
}  // namespace sensors
}  // namespace apollo
