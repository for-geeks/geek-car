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

#include "modules/common/global_gflags.h"
#include "modules/control/proto/chassis.pb.h"
#include "modules/sensors/proto/sensors.pb.h"

namespace apollo {
namespace sensors {

using apollo::control::Chassis;
using apollo::cyber::Component;
using apollo::cyber::Writer;

class RealsenseComponent : public Component<> {
 public:
  bool Init() override;
  void run();
  ~RealsenseComponent();

 private:
  rs2::device GetDevice();
  void Calibration();
  void WheelOdometry();
  void OnImage(cv::Mat dst, uint64 frame_no);
  void OnDepthImage(cv::Mat mat, uint64 frame_no);
  void OnPose(rs2_pose pose_data, uint64 frame_no);
  void OnAcc(rs2_vector acc, uint64 frame_no);
  void OnGyro(rs2_vector gyro, uint64 frame_no);
  void CompressedImage(cv::Mat raw_image, uint64 frame_no);
  cv::Mat frame_to_mat(const rs2::frame& f);
  cv::Mat depth_frame_to_meters(const rs2::pipeline& pipe,
                                const rs2::depth_frame& f);

  std::shared_ptr<Reader<Chassis>> chassis_reader_ = nullptr;

  std::shared_ptr<Writer<Image>> image_writer_ = nullptr;
  std::shared_ptr<Writer<Image>> depth_image_writer_ = nullptr;
  std::shared_ptr<Writer<Pose>> pose_writer_ = nullptr;
  std::shared_ptr<Writer<Acc>> acc_writer_ = nullptr;
  std::shared_ptr<Writer<Gyro>> gyro_writer_ = nullptr;

  std::shared_ptr<Writer<Image>> compressed_image_writer_ = nullptr;
  Chassis chassis_;

  std::future<void> async_result_;
  rs2::device device_;     // realsense device
  rs2::sensor sensor_;     // sensor include imu and camera;
  uint32_t device_model_;  // realsense device model like T265 OR D435I
  rs2::sensor wheel_odometry_sensor_;

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

  cv::Mat map1_;
  cv::Mat map2_;
};

CYBER_REGISTER_COMPONENT(RealsenseComponent)
}  // namespace sensors
}  // namespace apollo
