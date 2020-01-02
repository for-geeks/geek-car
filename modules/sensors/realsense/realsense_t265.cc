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
#include "modules/sensors/realsense/realsense_t265.h"

#include <string>
#include <vector>

#include "modules/sensors/realsense.h"

namespace apollo {
namespace sensors {
namespace realsense {

using apollo::cyber::Time;
using apollo::cyber::common::GetAbsolutePath;
using apollo::sensors::Acc;
using apollo::sensors::Gyro;
using apollo::sensors::Pose;

bool T265::Init(std::shared_ptr<Node> node_) {
  // 1. DeviceConfig
  DeviceConfig();

  InitChannelWriter(node_);

  async_result_ = cyber::Async(&T265::Run, this);

  return true;
}

void T265::DeviceConfig() {
  // Add desired streams to configuration
  cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
  cfg.enable_stream(RS2_STREAM_FISHEYE, 1);
  cfg.enable_stream(RS2_STREAM_FISHEYE, 2);
  // Add streams of gyro and accelerometer to configuration
  cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
  cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);

  pipe.start(cfg);
}

void T265::InitChannelWriter(std::shared_ptr<Node> node_) {
  // Channel Writer flags
  if (FLAGS_publish_pose) {
    pose_writer_ = node_->CreateWriter<Pose>(FLAGS_pose_channel);
  }
  if (FLAGS_publish_raw_gray_image) {
    image_writer_ = node_->CreateWriter<Image>(FLAGS_gray_image_channel);
  }

  if (FLAGS_publish_realsense_acc) {
    acc_writer_ = node_->CreateWriter<Acc>(FLAGS_realsense_acc_channel);
  }

  if (FLAGS_publish_realsense_gyro) {
    gyro_writer_ = node_->CreateWriter<Gyro>(FLAGS_realsense_gyro_channel);
  }

  if (FLAGS_publish_compressed_gray_image) {
    compressed_image_writer_ =
        node_->CreateWriter<Image>(FLAGS_compressed_gray_image_channel);
  }

  chassis_reader_ = node_->CreateReader<Chassis>(
      FLAGS_chassis_channel, [this](const std::shared_ptr<Chassis> &chassis) {
        chassis_.Clear();
        chassis_.CopyFrom(*chassis);
      });
}

void T265::Run() {
  while (!apollo::cyber::IsShutdown()) {
    // Camera warmup - dropping several first frames to let auto-exposure
    // stabilize
    rs2::frameset frames;
    // Wait for all configured streams to produce a frame
    frames = pipe.wait_for_frames();

    if (FLAGS_publish_realsense_acc) {
      rs2::motion_frame accel_frame = frames.first_or_default(RS2_STREAM_ACCEL);
      OnAcc(accel_frame);
    }

    if (FLAGS_publish_realsense_gyro) {
      rs2::motion_frame gyro_frame = frames.first_or_default(RS2_STREAM_GYRO);
      OnGyro(gyro_frame);
    }

    if (FLAGS_publish_pose) {
      auto pose_frame = frames.get_pose_frame();
      OnPose(pose_frame);
    }

    if (FLAGS_publish_compressed_gray_image) {
      auto fisheye_frame = frames.get_fisheye_frame(fisheye_sensor_idx);
      OnGrayImage(fisheye_frame);
    }
  }
}

void T265::OnPose(const rs2::pose_frame &pose_frame) {
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

void T265::OnGrayImage(const rs2::frame &fisheye_frame) {
  if (!FLAGS_publish_raw_gray_image) {
    AINFO << "Turn off the raw gray image";
    return;
  }

  cv::Mat image = frame_to_mat(fisheye_frame);
  cv::Mat dst;
  cv::remap(image, dst, map1_, map2_, cv::INTER_LINEAR);
  cv::Size dsize = cv::Size(static_cast<int>(dst.cols * 0.5),
                            static_cast<int>(dst.rows * 0.5));
  cv::Mat new_size_img(dsize, CV_8U);
  cv::resize(dst, new_size_img, dsize);

  auto image_proto = std::make_shared<Image>();
  image_proto->set_frame_no(fisheye_frame.get_frame_number());
  image_proto->set_height(dst.rows);
  image_proto->set_width(dst.cols);
  // encodings
  image_proto->set_encoding(rs2_format_to_string(RS2_FORMAT_Y8));
  image_proto->set_measurement_time(fisheye_frame.get_timestamp());
  auto m_size = dst.rows * dst.cols * dst.elemSize();
  image_proto->set_data(dst.data, m_size);
  image_writer_->Write(image_proto);

  if (FLAGS_publish_compressed_gray_image) {
    OnCompressedImage(fisheye_frame, dst);
  }
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

T265::~T265() { AINFO << "Destructor from T265"; }

}  // namespace realsense
}  // namespace sensors
}  // namespace apollo
