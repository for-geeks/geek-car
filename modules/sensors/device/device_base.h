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

#include "librealsense2/rs.hpp"
#include "opencv2/opencv.hpp"

#include "cyber/cyber.h"
#include "modules/common/global_gflags.h"
#include "modules/control/proto/chassis.pb.h"
#include "modules/sensors/proto/pointcloud.pb.h"
#include "modules/sensors/proto/sensor_image.pb.h"
#include "modules/sensors/proto/sensors.pb.h"
#include "modules/sensors/realsense_motion.h"

namespace apollo {
namespace sensors {

using apollo::cyber::Writer;
using apollo::sensors::CompressedImage;
using apollo::sensors::Image;
using apollo::sensors::PointCloud;

class DeviceBase {
 public:
  DeviceBase() = default;
  virtual bool Init() = 0;
  virtual void InitChannelWriter() = 0;
  virtual void DeviceConfig() = 0;

  virtual void Run();

  virtual ~DeviceBase() = default;

  void OnCompressedImage(const rs2::frame &f, cv::Mat raw_image);
  void OnAcc(const rs2::motion_frame &accel_frame);
  void OnGyro(const rs2::motion_frame &gyro_frame);

  std::future<void> async_result_;

 private:
  std::shared_ptr<Writer<Acc>> acc_writer_ = nullptr;
  std::shared_ptr<Writer<Gyro>> gyro_writer_ = nullptr;
  std::shared_ptr<Writer<Image>> image_writer_ = nullptr;
  std::shared_ptr<Writer<CompressedImage>> compressed_image_writer_ = nullptr;

  
  rs2::device device_;  // realsense device
  rs2::sensor sensor_;  // sensor include imu and camera;

  // Contruct a pipeline which abstracts the device
  rs2::pipeline pipe;

  // Configuring the pipeline with a non default profile
  rs2::config cfg;

  // Declare object that handles camera pose calculations
  rotation_estimator algo_;
};

void DeviceBase::OnAcc(const rs2::motion_frame &accel_frame) {
  rs2_vector acc = accel_frame.get_motion_data();
  // Computes the angle of motion based on the retrieved measures
  algo_.process_accel(acc);
  AINFO << "Accel:" << acc.x << ", " << acc.y << ", " << acc.z;
  auto proto_accel = std::make_shared<Acc>();
  proto_accel->mutable_acc()->set_x(acc.x);
  proto_accel->mutable_acc()->set_y(acc.y);
  proto_accel->mutable_acc()->set_z(acc.z);

  acc_writer_->Write(proto_accel);
}

void DeviceBase::OnGyro(const rs2::motion_frame &gyro_frame) {
  rs2_vector gyro = gyro_frame.get_motion_data();
  // Computes the angle of motion based on the retrieved measures
  algo_.process_gyro(gyro, gyro_frame.get_timestamp());
  AINFO << "Gyro:" << gyro.x << ", " << gyro.y << ", " << gyro.z;
  auto proto_gyro = std::make_shared<Gyro>();
  proto_gyro->mutable_gyro()->set_x(gyro.x);
  proto_gyro->mutable_gyro()->set_y(gyro.y);
  proto_gyro->mutable_gyro()->set_z(gyro.z);

  gyro_writer_->Write(proto_gyro);
}
}  // namespace sensors
}  // namespace apollo
