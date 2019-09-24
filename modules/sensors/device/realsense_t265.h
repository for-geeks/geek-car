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
#include "modules/sensors/device/device_base.h"

#include <memory>

namespace apollo {
namespace sensors {

class T265 : public DeviceBase {
 public:
  T265();
  ~T265();

  bool Init();

 private:
  void InitDeviceAndSensor();
  void OnGrayImage(const rs2::frame &fisheye_frame);
  void OnCompressedImage(const rs2::frame &f, cv::Mat raw_image);
  void OnPose(const rs2::pose_frame &pose_frame);
  void OnAcc(const rs2::motion_frame &accel_frame);
  void OnGyro(const rs2::motion_frame &gyro_frame);

  void Calibration();
  void WheelOdometry();

  std::shared_ptr<Reader<Chassis>> chassis_reader_ = nullptr;

  std::shared_ptr<Writer<Image>> image_writer_ = nullptr;
  std::shared_ptr<Writer<Pose>> pose_writer_ = nullptr;
  std::shared_ptr<Writer<Acc>> acc_writer_ = nullptr;
  std::shared_ptr<Writer<Gyro>> gyro_writer_ = nullptr;

  std::shared_ptr<Writer<CompressedImage>> compressed_image_writer_ = nullptr;

  Chassis chassis_;

  rs2::device device_;  // realsense device
  rs2::sensor sensor_;  // sensor include imu and camera;

  // fisheye calibration map
  cv::Mat map1_;
  cv::Mat map2_;

  double norm_max = 0;

  const int fisheye_sensor_idx = 1;  // for the left fisheye lens of T265
};
}  // namespace sensors
}  // namespace apollo
