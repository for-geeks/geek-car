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

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <memory>
#include <string>

#include "librealsense2/rs.hpp"
#include "opencv2/opencv.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"

#include "modules/common/global_gflags.h"
#include "modules/control/proto/chassis.pb.h"
#include "modules/sensors/proto/pointcloud.pb.h"
#include "modules/sensors/proto/sensor_image.pb.h"
#include "modules/sensors/proto/sensors.pb.h"
#include "modules/sensors/realsense_motion.h"

namespace apollo {
namespace sensors {

using apollo::control::Chassis;
using apollo::cyber::Component;
using apollo::cyber::Writer;
using apollo::sensors::CompressedImage;
using apollo::sensors::Image;

using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

class RealsenseComponent : public Component<> {
public:
  bool Init() override;
  void run();
  ~RealsenseComponent();

private:
  void InitDeviceAndSensor();
  void OnGrayImage(const rs2::frame &fisheye_frame);
  void OnColorImage(const rs2::frame &f);
  void OnCompressedImage(const rs2::frame &f, cv::Mat raw_image);
  void OnPointCloud(const rs2::frame &f);
  void OnPose(const rs2::pose_frame &pose_frame);
  void OnAcc(const rs2::motion_frame &accel_frame);
  void OnGyro(const rs2::motion_frame &gyro_frame);
  void Calibration();
  void WheelOdometry();

  std::shared_ptr<Reader<Chassis>> chassis_reader_ = nullptr;

  std::shared_ptr<Writer<Image>> image_writer_ = nullptr;
  std::shared_ptr<Writer<Image>> color_image_writer_ = nullptr;
  std::shared_ptr<Writer<Pose>> pose_writer_ = nullptr;
  std::shared_ptr<Writer<Acc>> acc_writer_ = nullptr;
  std::shared_ptr<Writer<Gyro>> gyro_writer_ = nullptr;
  std::shared_ptr<Writer<apollo::sensors::PointCloud>> point_cloud_writer_ =
      nullptr;

  std::shared_ptr<Writer<CompressedImage>> compressed_image_writer_ = nullptr;
  Chassis chassis_;

  std::future<void> async_result_;
  rs2::device device_;    // realsense device
  rs2::sensor sensor_;    // sensor include imu and camera;
  uint32_t device_model_; // realsense device model like T265 OR D435I

  // Contruct a pipeline which abstracts the device
  rs2::pipeline pipe;

  // Create a configuration for configuring the pipeline with a non default
  // profile
  rs2::config cfg;

  cv::Mat map1_;
  cv::Mat map2_;

  double norm_max = 0;
  // Declare object that handles camera pose calculations
  rotation_estimator algo_;
  Eigen::Matrix4f transform;
};

CYBER_REGISTER_COMPONENT(RealsenseComponent)
} // namespace sensors
} // namespace apollo
