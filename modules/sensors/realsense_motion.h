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

#include "librealsense2/rs.hpp"

namespace apollo {
namespace sensors {

#ifndef PI
const double PI = 3.14159265358979323846;
#endif

struct double3 {
  double x, y, z;
  double3 operator*(double t) { return {x * t, y * t, z * t}; }

  double3 operator-(double t) { return {x - t, y - t, z - t}; }

  void operator*=(double t) {
    x = x * t;
    y = y * t;
    z = z * t;
  }

  void operator=(double3 d) {
    x = d.x;
    y = d.y;
    z = d.z;
  }

  void add(double t1, double t2, double t3) {
    x += t1;
    y += t2;
    z += t3;
  }
};

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
}  // namespace sensors
}  // namespace apollo
