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
#include <vector>

#include <Eigen/Core>
#include "librealsense2/rs.hpp"
#include "opencv2/opencv.hpp"

#include "cyber/cyber.h"
#include "cyber/node/node.h"

#include "modules/common/global_gflags.h"
#include "modules/control/proto/chassis.pb.h"
#include "modules/sensors/proto/pointcloud.pb.h"
#include "modules/sensors/proto/sensor_image.pb.h"
#include "modules/sensors/proto/sensors.pb.h"
#include "modules/sensors/realsense_motion.h"

namespace apollo {
namespace sensors {
namespace realsense {

using apollo::cyber::Node;
using apollo::cyber::Writer;
using apollo::sensors::CompressedImage;
using apollo::sensors::Image;
using apollo::sensors::PointCloud;

// const transform by 15 deg in y axis
static const ::Eigen::Matrix4d transform =
    (::Eigen::Matrix4d() << 1, -0, 0, 0, 0, 0.96596, 0.258691, 0, -0, -0.258691,
     0.96596, 0, 0, 0, 0, 1)
        .finished();

class DeviceBase {
 public:
  DeviceBase() = default;
  virtual bool Init(std::shared_ptr<Node> node_) = 0;
  virtual void InitChannelWriter(std::shared_ptr<Node> node_) = 0;
  virtual void DeviceConfig() = 0;

  virtual void Run() = 0;

  virtual ~DeviceBase() {
    AINFO << "Destructor from DeviceBase";
    if (async_result_.valid()) {
      async_result_.wait();
    }
  }

  void OnAcc(const rs2::motion_frame &accel_frame) {
    rs2_vector acc = accel_frame.get_motion_data();
    AINFO << "Accel:" << acc.x << ", " << acc.y << ", " << acc.z;
    auto proto_accel = std::make_shared<Acc>();
    proto_accel->mutable_acc()->set_x(acc.x);
    proto_accel->mutable_acc()->set_y(acc.y);
    proto_accel->mutable_acc()->set_z(acc.z);

    acc_writer_->Write(proto_accel);
  }

  void OnGyro(const rs2::motion_frame &gyro_frame) {
    rs2_vector gyro = gyro_frame.get_motion_data();
    AINFO << "Gyro:" << gyro.x << ", " << gyro.y << ", " << gyro.z;
    auto proto_gyro = std::make_shared<Gyro>();
    proto_gyro->mutable_gyro()->set_x(gyro.x);
    proto_gyro->mutable_gyro()->set_y(gyro.y);
    proto_gyro->mutable_gyro()->set_z(gyro.z);

    gyro_writer_->Write(proto_gyro);
  }

  void OnCompressedImage(const rs2::frame &f, cv::Mat raw_image) {
    std::vector<int> param = std::vector<int>(2);
    param[0] = cv::IMWRITE_JPEG_QUALITY;
    param[1] = FLAGS_compress_rate;
    cv::Mat tmp_mat;
    // cv::cvtColor(raw_image, tmp_mat, cv::COLOR_RGB2BGR);
    std::vector<uchar> data_encode;
    cv::imencode(".jpeg", raw_image, data_encode, param);
    std::string str_encode(data_encode.begin(), data_encode.end());

    auto compressedimage = std::make_shared<Image>();
    compressedimage->set_frame_no(f.get_frame_number());
    compressedimage->set_encoding(
        rs2_format_to_string(f.get_profile().format()));
    compressedimage->set_height(FLAGS_color_image_height);
    compressedimage->set_width(FLAGS_color_image_width);
    compressedimage->set_measurement_time(f.get_timestamp());
    compressedimage->set_data(str_encode);

    compressed_image_writer_->Write(compressedimage);
  }

 protected:
  std::shared_ptr<Writer<Acc>> acc_writer_ = nullptr;
  std::shared_ptr<Writer<Gyro>> gyro_writer_ = nullptr;
  std::shared_ptr<Writer<Image>> compressed_image_writer_ = nullptr;

  std::future<void> async_result_;

  rs2::device device_;  // realsense device
  rs2::sensor sensor_;  // sensor include imu and camera;

  // Contruct a pipeline which abstracts the device
  rs2::pipeline pipe;

  // Configuring the pipeline with a non default profile
  rs2::config cfg;
};

}  // namespace realsense
}  // namespace sensors
}  // namespace apollo
