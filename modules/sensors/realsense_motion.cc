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

#ifndef PI
const double PI = 3.14159265358979323846;
#endif
struct short3 {
  uint16_t x, y, z;
};

struct double3 {
  double x, y, z;
  double3 operator*(double t) { return {x * t, y * t, z * t}; }

  double3 operator-(double t) { return {x - t, y - t, z - t}; }

  void operator*=(double t) {
    x = x * t;
    y = y * t;
    z = z * t;
  }

  void operator=(double3 other) {
    x = other.x;
    y = other.y;
    z = other.z;
  }

  void add(double t1, double t2, double t3) {
    x += t1;
    y += t2;
    z += t3;
  }
};

// #include "d435.h"

class rotation_estimator {
  // theta is the angle of camera rotation in x, y and z components
  double3 theta;
  std::mutex theta_mtx;
  /* alpha indicates the part that gyro and accelerometer take in computation of
  theta; higher alpha gives more weight to gyro, but too high values cause
  drift; lower alpha gives more weight to accelerometer, which is more sensitive
  to disturbances */
  double alpha = 0.98;
  bool first = true;
  // Keeps the arrival time of previous gyro frame
  double last_ts_gyro = 0;

 public:
  // Function to calculate the change in angle of motion based on data from gyro
  void process_gyro(rs2_vector gyro_data, double ts) {
    // On the first iteration, use only data from accelerometer to
    // set the camera's initial position
    if (first) {
      last_ts_gyro = ts;
      return;
    }
    // Holds the change in angle, as calculated from gyro
    double3 gyro_angle;

    // Initialize gyro_angle with data from gyro
    gyro_angle.x = gyro_data.x;  // Pitch
    gyro_angle.y = gyro_data.y;  // Yaw
    gyro_angle.z = gyro_data.z;  // Roll

    // Compute the difference between arrival times of previous and current gyro
    // frames
    double dt_gyro = (ts - last_ts_gyro) / 1000.0;
    last_ts_gyro = ts;

    // Change in angle equals gyro measures * time passed since last measurement
    gyro_angle = gyro_angle * dt_gyro;

    // Apply the calculated change of angle to the current angle (theta)
    std::lock_guard<std::mutex> lock(theta_mtx);
    theta.add(-gyro_angle.z, -gyro_angle.y, gyro_angle.x);
  }

  void process_accel(rs2_vector accel_data) {
    // Holds the angle as calculated from accelerometer data
    double3 accel_angle;

    // Calculate rotation angle from accelerometer data
    accel_angle.z = atan2(accel_data.y, accel_data.z);
    accel_angle.x = atan2(accel_data.x, sqrt(accel_data.y * accel_data.y +
                                             accel_data.z * accel_data.z));

    // If it is the first iteration, set initial pose of camera according to
    // accelerometer data (note the different handling for Y axis)
    std::lock_guard<std::mutex> lock(theta_mtx);
    if (first) {
      first = false;
      theta = accel_angle;
      // Since we can't infer the angle around Y axis using accelerometer data,
      // we'll use PI as a convetion for the initial pose
      theta.y = PI;
    } else {
      /*
      Apply Complementary Filter:
          - high-pass filter = theta * alpha:  allows short-duration signals to
      pass through while filtering out signals that are steady over time, is
      used to cancel out drift.
          - low-pass filter = accel * (1- alpha): lets through long term
      changes, filtering out short term fluctuations
      */
      theta.x = theta.x * alpha + accel_angle.x * (1 - alpha);
      theta.z = theta.z * alpha + accel_angle.z * (1 - alpha);
    }
  }

  // Returns the current rotation angle
  double3 get_theta() {
    std::lock_guard<std::mutex> lock(theta_mtx);
    return theta;
  }
};

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

    return 0;
  }
