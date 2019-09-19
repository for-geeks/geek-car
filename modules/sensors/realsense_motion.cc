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
#include "modules/sensors/realsense_motion.h"

#include <math.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <functional>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "librealsense2/rs.hpp"
#include "opencv2/opencv.hpp"

#include "cyber/cyber.h"
#include "cyber/task/task.h"
#include "modules/common/global_gflags.h"
#include "modules/sensors/proto/sensors.pb.h"

// #include "d435.h"

int main() {
  apollo::cyber::Init("motion");
  auto node = apollo::cyber::CreateNode("motion");

  // Declare object for rendering camera motion
  // camera_renderer camera;
  // Declare object that handles camera pose calculations
  rotation_estimator algo;

  apollo::cyber::ReaderConfig reader_config;
  reader_config.channel_name = FLAGS_acc_channel;
  reader_config.pending_queue_size = 1;

  // std::function<void(const std::shared_ptr<drivers::PointCloud>&)>
  //     lidar_register_call = std::bind(&MSFLocalization::OnPointCloud,
  //                                     &localization_, std::placeholders::_1);

  // lidar_listener_ = this->node_->CreateReader<drivers::PointCloud>(
  //     reader_config, lidar_register_call);
#if 0
  std::function<void(const std::shared_ptr<apollo::sensors::Acc>&)>
      acc_register_call = std::bind(&rotation_estimator::process_accel,
                                    &localization_, std::placeholders::_1);

  auto acc_reader = node->CreateReader<apollo::sensors::Acc>(reader_config,
                                                             acc_register_call);

  reader_config.channel_name = FLAGS_gyro_channel;

  std::function<void(const std::shared_ptr<apollo::sensors::Gyro>&)>
      gyro_register_call = std::bind(&rotation_estimator::process_accel,
                                     &localization_, std::placeholders::_1);

  auto gyro_reader = node->CreateReader<apollo::sensors::Gyro>(
      reader_config, gyro_register_call);

  // Main loop
  while (!apollo::cyber::IsShutdown()) {
    auto angle = algo.get_theta();
    Eigen::Matrix3f transform_1 = Eigen::Matrix3f::Identity();
    ::Eigen::Vector3d ea0(0, angle.x, angle.z);
    ::Eigen::Matrix3d R;
    R = ::Eigen::AngleAxisd(ea0[0], ::Eigen::Vector3d::UnitZ()) *
        ::Eigen::AngleAxisd(ea0[1], ::Eigen::Vector3d::UnitY()) *
        ::Eigen::AngleAxisd(ea0[2], ::Eigen::Vector3d::UnitX());
    AINFO << R;
    transform_1(0, 0) = R(0, 0);
    transform_1(0, 1) = R(0, 1);
    transform_1(0, 2) = R(0, 2);
    transform_1(1, 0) = R(1, 0);
    transform_1(1, 1) = R(1, 1);
    transform_1(1, 2) = R(1, 2);
    transform_1(2, 0) = R(2, 0);
    transform_1(2, 1) = R(2, 1);
    transform_1(2, 2) = R(2, 2);
    AINFO << transform_1;
    // use transform_1 as transform input param

    apollo::cyber::WaitForShutdown();
#endif
  return 0;
}
