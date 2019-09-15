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

#include <memory>

#include "opencv2/opencv.hpp"

namespace apollo {
namespace sensors {

class DeviceBase {
 public:
  bool Init();

  void OnImage(cv::Mat raw_image, uint64 frame_no);
  void OnCompressedImage(cv::Mat raw_image, uint64 frame_no);
  void OnAcc(rs2::frame f);
  void OnGyro(rs2::frame f);

 private:
  std::shared_ptr<Writer<Acc>> acc_writer_ = nullptr;
  std::shared_ptr<Writer<Gyro>> gyro_writer_ = nullptr;
  std::shared_ptr<Writer<Image>> image_writer_ = nullptr;
  std::shared_ptr<Writer<Image>> compressed_image_writer_ = nullptr;

  std::future<void> async_result_;
  rs2::device device_;     // realsense device
  rs2::sensor sensor_;     // sensor include imu and camera;
  uint32_t device_model_;  // realsense device model like T265 OR D435I

  // Contruct a pipeline which abstracts the device
  rs2::pipeline pipe;

  // Create a configuration for configuring the pipeline with a non default
  // profile
  rs2::config cfg;
};
}  // namespace sensors
}  // namespace apollo
