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
#include "modules/sensors/device/realsense_t265.h"

#include "modules/common/global_gflags.h"
#include "modules/control/proto/chassis.pb.h"
#include "modules/sensors/proto/point_cloud.pb.h"
#include "modules/sensors/proto/sensors.pb.h"

namespace apollo {
namespace sensors {
bool T265::Init() {
  cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
  cfg.enable_stream(RS2_STREAM_FISHEYE, 1);
  cfg.enable_stream(RS2_STREAM_FISHEYE, 2);
  // Add streams of gyro and accelerometer to configuration
  cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
  cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);

  // Instruct pipeline to start streaming with the requested configuration
  pipe.start(cfg);
  return true;
}

void T265::Calibration() {
  cv::Mat intrinsicsL;
  cv::Mat distCoeffsL;
  rs2_intrinsics left = sensor_.get_stream_profiles()[0]
                            .as<rs2::video_stream_profile>()
                            .get_intrinsics();
  ADEBUG << " intrinsicksL, fx:" << left.fx << ", fy:" << left.fy
         << ", ppx:" << left.ppx << ", ppy:" << left.ppy;
  intrinsicsL = (cv::Mat_<double>(3, 3) << left.fx, 0, left.ppx, 0, left.fy,
                 left.ppy, 0, 0, 1);
  distCoeffsL = cv::Mat(1, 4, CV_32F, left.coeffs);
  cv::Mat R = cv::Mat::eye(3, 3, CV_32F);
  cv::Mat P = (cv::Mat_<double>(3, 4) << left.fx, 0, left.ppx, 0, 0, left.fy,
               left.ppy, 0, 0, 0, 1, 0);

  cv::fisheye::initUndistortRectifyMap(intrinsicsL, distCoeffsL, R, P,
                                       cv::Size(848, 816), CV_16SC2, map1_,
                                       map2_);
}

void T265::WheelOdometry() {
  auto wheel_odometry_sensor = device_.first<rs2::wheel_odometer>();
  std::string calibration_file_path =
      GetAbsolutePath(apollo::cyber::common::WorkRoot(), FLAGS_odometry_file);
  std::ifstream calibrationFile(calibration_file_path);
  const std::string json_str((std::istreambuf_iterator<char>(calibrationFile)),
                             std::istreambuf_iterator<char>());
  const std::vector<uint8_t> wo_calib(json_str.begin(), json_str.end());

  if (!wheel_odometry_sensor.load_wheel_odometery_config(wo_calib)) {
    AERROR << "Failed to load wheel odometry config file.";
  }
}

/**
 * @brief callback of Pose data
 *
 * @param pose_data
 * @return true
 * @return false
 */
void T265::OnPose(rs2::frame f) {
  auto pose_frame = f.as<rs2::pose_frame>();
  if (pose_frame.get_frame_number() % 5 == 0) {
    auto pose_data = pose_frame.get_pose_data();
    AINFO << "pose " << pose_data.translation;
    double norm = sqrt(pose_data.translation.x * pose_data.translation.x +
                       pose_data.translation.y * pose_data.translation.y +
                       pose_data.translation.z * pose_data.translation.z);
    if (norm > norm_max) {
      norm_max = norm;
    }

    ADEBUG << "norm_max:" << norm_max;

    // send vehicle speed to wheel odometry
    auto wheel_odometry_sensor = device_.first<rs2::wheel_odometer>();
    if (!wheel_odometry_sensor.send_wheel_odometry(0, 0,
                                                   {chassis_.speed(), 0, 0})) {
      AERROR << "Failed to send wheel odometry";
    }

    auto pose_proto = std::make_shared<Pose>();
    pose_proto->set_frame_no(pose_frame.get_frame_number());
    pose_proto->set_tracker_confidence(pose_data.tracker_confidence);
    pose_proto->set_mapper_confidence(pose_data.mapper_confidence);

    auto translation = pose_proto->mutable_translation();
    translation->set_x(pose_data.translation.x);
    translation->set_y(pose_data.translation.y);
    translation->set_z(pose_data.translation.z);

    auto velocity = pose_proto->mutable_velocity();
    velocity->set_x(pose_data.velocity.x);
    velocity->set_y(pose_data.velocity.y);
    velocity->set_z(pose_data.velocity.z);

    auto rotation = pose_proto->mutable_rotation();
    rotation->set_x(pose_data.rotation.x);
    rotation->set_y(pose_data.rotation.y);
    rotation->set_z(pose_data.rotation.z);
    rotation->set_w(pose_data.rotation.w);

    auto angular_velocity = pose_proto->mutable_angular_velocity();
    angular_velocity->set_x(pose_data.angular_velocity.x);
    angular_velocity->set_y(pose_data.angular_velocity.y);
    angular_velocity->set_z(pose_data.angular_velocity.z);

    pose_writer_->Write(pose_proto);
  }
}

}  // namespace sensors
}  // namespace apollo
