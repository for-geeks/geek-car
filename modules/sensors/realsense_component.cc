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
#include "modules/sensors/realsense_component.h"

#include <chrono>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

#include "librealsense2/rs.hpp"
#include "opencv2/opencv.hpp"

#include "cyber/common/log.h"
#include "cyber/cyber.h"
#include "cyber/time/rate.h"
#include "cyber/time/time.h"
#include "modules/sensors/proto/sensors.pb.h"
#include "modules/sensors/realsense.h"

namespace apollo {
namespace sensors {

using apollo::cyber::Rate;
using apollo::cyber::Time;
using apollo::cyber::common::GetAbsolutePath;
using apollo::sensors::Acc;
using apollo::sensors::Gyro;
using apollo::sensors::Image;
using apollo::sensors::Pose;

using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

bool RealsenseComponent::Init() {
  device_ = GetFirstConnectedDevice();

  // Print device information
  RealSense::printDeviceInformation(device_);

  if (std::strstr(device_.get_info(RS2_CAMERA_INFO_NAME), "T265")) {
    device_model_ = RealSenseDeviceModel::T265;
  } else if (std::strstr(device_.get_info(RS2_CAMERA_INFO_NAME), "D435I")) {
    device_model_ = RealSenseDeviceModel::D435I;
  } else {
    AWARN << "The device data is not yet supported for parsing";
  }

  // Get device serial_number
  FLAGS_serial_number = device_.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
  AINFO << "Got device with serial number: " << FLAGS_serial_number;
  cyber::SleepFor(std::chrono::milliseconds(device_wait_));

  // Get default as Sensor but for D435I is rs2::depth_sensor
  sensor_ = device_.first<rs2::sensor>();

  // RealSense::getSensorOption(sensor_);
  sensor_.set_option(RS2_OPTION_FRAMES_QUEUE_SIZE, 0);
  sensor_.open(sensor_.get_stream_profiles());

  Calibration();

  sensor_.start([this](rs2::frame f) {
    // enqueue any new frames into q
    q_.enqueue(std::move(f));
  });

  // load_wheel_odometery_config
  WheelOdometry();

  if (FLAGS_publish_pose && device_model_ == RealSenseDeviceModel::T265) {
    pose_writer_ = node_->CreateWriter<Pose>(FLAGS_pose_channel);
  }
  if (FLAGS_publish_raw_image) {
    image_writer_ = node_->CreateWriter<Image>(FLAGS_raw_image_channel);
  }

  if (FLAGS_publish_depth_image &&
      device_model_ == RealSenseDeviceModel::D435I) {
    depth_image_writer_ = node_->CreateWriter<Image>(FLAGS_depth_image_channel);
  }

  if (FLAGS_publish_acc) {
    acc_writer_ = node_->CreateWriter<Acc>(FLAGS_acc_channel);
  }

  if (FLAGS_publish_gyro) {
    gyro_writer_ = node_->CreateWriter<Gyro>(FLAGS_gyro_channel);
  }

  if (FLAGS_publish_compressed_image) {
    compressed_image_writer_ =
        node_->CreateWriter<Image>(FLAGS_compressed_image_channel);
  }

  chassis_reader_ = node_->CreateReader<Chassis>(
      FLAGS_chassis_channel, [this](const std::shared_ptr<Chassis>& chassis) {
        chassis_.Clear();
        chassis_.CopyFrom(*chassis);
      });

  // thread to handle frames
  async_result_ = cyber::Async(&RealsenseComponent::run, this);
  return true;
}

rs2::device RealsenseComponent::GetFirstConnectedDevice() {
  rs2::context ctx;
  auto list = ctx.query_devices();
  // Get a snapshot of currently connected devices
  if (list.size() == 0) {
    AWARN << "No device detected. Is it plugged in?";
  }

  return list.front();
}

/**
 * Sensor RealSense:: collect frames
 * @return void
 */
void RealsenseComponent::run() {
  double norm_max = 0;
  while (!cyber::IsShutdown()) {
    // wait for device is ready. in case of device in busy state
    if (!device_) {
      device_ = GetFirstConnectedDevice();
      continue;
    }

    // wait until new frame is available and dequeue it
    // handle frames in the main event loop
    rs2::frame f = q_.wait_for_frame();

    // core of data write to channel
    if (f.get_profile().stream_type() == RS2_STREAM_POSE &&
        FLAGS_publish_pose) {
      auto pose_frame = f.as<rs2::pose_frame>();
      auto pose_data = pose_frame.get_pose_data();
      AINFO << "pose " << pose_data.translation;

      double norm = sqrt(pose_data.translation.x * pose_data.translation.x +
                         pose_data.translation.y * pose_data.translation.y +
                         pose_data.translation.z * pose_data.translation.z);
      if (norm > norm_max) norm_max = norm;

      ADEBUG << "norm_max:" << norm_max;

      auto wo_sensor = device_.first<rs2::wheel_odometer>();
      // send vehicle speed to wheel odometry
      if (!wo_sensor.send_wheel_odometry(0, 0, {chassis_.speed(), 0, 0})) {
        AERROR << "Failed to send wheel odometry";
      }
      if (pose_frame.get_frame_number() % 5 == 0) {
        OnPose(pose_data, pose_frame.get_frame_number());
      }
    } else if (f.get_profile().stream_type() == RS2_STREAM_GYRO &&
               FLAGS_publish_gyro) {
      auto gyro_frame = f.as<rs2::motion_frame>();
      rs2_vector gyro = gyro_frame.get_motion_data();
      AINFO << "Gyro:" << gyro.x << ", " << gyro.y << ", " << gyro.z;
      OnGyro(gyro, gyro_frame.get_frame_number());
    } else if (f.get_profile().stream_type() == RS2_STREAM_ACCEL &&
               FLAGS_publish_acc) {
      auto accel_frame = f.as<rs2::motion_frame>();
      rs2_vector accel = accel_frame.get_motion_data();
      AINFO << "Accel:" << accel.x << ", " << accel.y << ", " << accel.z;
      OnAcc(accel, accel_frame.get_frame_number());
    } else if (f.get_profile().stream_type() == RS2_STREAM_FISHEYE &&
               f.get_profile().stream_index() == 1) {
      // left fisheye
      auto fisheye_frame = f.as<rs2::video_frame>();

      AINFO << "fisheye " << f.get_profile().stream_index() << ", "
            << fisheye_frame.get_width() << "x" << fisheye_frame.get_height();

      cv::Mat image = frame_to_mat(fisheye_frame);
      cv::Mat dst;
      cv::remap(image, dst, map1_, map2_, cv::INTER_LINEAR);
      cv::Size dsize = cv::Size(static_cast<int>(dst.cols * 0.5),
                                static_cast<int>(dst.rows * 0.5));
      cv::Mat new_size_img(dsize, CV_8U);
      cv::resize(dst, new_size_img, dsize);
      if (fisheye_frame.get_frame_number() % 2 == 0) {
        OnImage(new_size_img, fisheye_frame.get_frame_number());
      }
    } else if (f.get_profile().stream_type() == RS2_STREAM_COLOR) {
      // Notice that the color frame is of type `rs2::video_frame` and the depth
      // frame if of type `rs2::depth_frame` (which derives from
      // `rs2::video_frame` and adds special depth related functionality).
      auto color_frame = f.as<rs2::video_frame>();
      cv::Mat color_image = frame_to_mat(color_frame);
      OnImage(color_image, color_frame.get_frame_number());

    } else if (f.get_profile().stream_type() == RS2_STREAM_DEPTH) {
      auto depth_frame = f.as<rs2::depth_frame>();
      OnPointCloud(depth_frame);
      // cv::Mat depth_image = frame_to_mat(depth_frame);
      // OnDepthImage(depth_image, depth_frame.get_frame_number());
    }
  }
}

void RealsenseComponent::Calibration() {
  if (device_model_ == RealSenseDeviceModel::T265) {
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
  } else if (device_model_ == RealSenseDeviceModel::D435I) {
    // no need to calibration 435I
  }
}

void RealsenseComponent::WheelOdometry() {
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
 * @brief callback of Image data
 *
 * @param dst
 * @return true
 * @return false
 */
void RealsenseComponent::OnImage(cv::Mat dst, uint64 frame_no) {
  if (FLAGS_publish_raw_image) {
    auto image_proto = std::make_shared<Image>();
    image_proto->set_frame_no(frame_no);
    image_proto->set_height(dst.rows);
    image_proto->set_width(dst.cols);
    // encodings
    if (device_model_ == RealSenseDeviceModel::T265) {
      image_proto->set_encoding(rs2_format_to_string(RS2_FORMAT_Y8));
    } else if (device_model_ == RealSenseDeviceModel::D435I) {
      image_proto->set_encoding(rs2_format_to_string(RS2_FORMAT_RGBA8));
    }
    image_proto->set_measurement_time(Time::Now().ToSecond());
    auto m_size = dst.rows * dst.cols * dst.elemSize();
    image_proto->set_data(dst.data, m_size);
    image_writer_->Write(image_proto);
  }
  if (FLAGS_publish_compressed_image) {
    CompressedImage(dst, frame_no);
  }
}

void RealsenseComponent::OnDepthImage(cv::Mat mat, uint64 frame_no) {
  if (FLAGS_publish_depth_image) {
    auto image_proto = std::make_shared<Image>();
    image_proto->set_frame_no(frame_no);
    image_proto->set_height(mat.rows);
    image_proto->set_width(mat.cols);
    // encoding /**< 16-bit linear depth values. The depth is meters is equal to
    // depth scale * pixel value. */
    image_proto->set_encoding(rs2_format_to_string(RS2_FORMAT_Z16));

    image_proto->set_measurement_time(Time::Now().ToSecond());
    auto m_size = mat.rows * mat.cols * mat.elemSize();
    image_proto->set_data(mat.data, m_size);
    image_writer_->Write(image_proto);
  }
}

void RealsenseComponent::OnPointCloud(rs2::frame& f) {
  // Declare pointcloud object, for calculating pointclouds and texture mappings
  rs2::pointcloud pc;
  // We want the points object to be persistent so we can display the last cloud
  // when a frame drops
  rs2::points points;

  auto depth = f.get_depth_frame();

  // Generate the pointcloud and texture mappings
  points = pc.calculate(depth);

  auto pcl_points = points_to_pcl(points);

  pcl_ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(pcl_points);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.0, 1.0);
  pass.filter(*cloud_filtered);

  std::vector<pcl_ptr> layers;
  layers.push_back(pcl_points);
  layers.push_back(cloud_filtered);

// TODO(fengzongbao) need to complete publish pointcloud
#if 0
  point_new = std::make_shared<apollo::sensors::PointCloud>();
  point_new = pcl_ptr->add_point();
  point_new->set_x(nan);
  point_new->set_y(nan);
  point_new->set_z(nan);
  point_new->set_timestamp(timestamp);
  point_new->set_intensity(0);
#endif
}

/**
 * @brief callback of Pose data
 *
 * @param pose_data
 * @return true
 * @return false
 */
void RealsenseComponent::OnPose(rs2_pose pose_data, uint64 frame_no) {
  auto pose_proto = std::make_shared<Pose>();
  pose_proto->set_frame_no(frame_no);
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

void RealsenseComponent::OnAcc(rs2_vector acc, uint64 frame_no) {
  auto proto_accel = std::make_shared<Acc>();
  proto_accel->mutable_acc()->set_x(acc.x);
  proto_accel->mutable_acc()->set_y(acc.y);
  proto_accel->mutable_acc()->set_z(acc.z);

  acc_writer_->Write(proto_accel);
}

void RealsenseComponent::OnGyro(rs2_vector gyro, uint64 frame_no) {
  auto proto_gyro = std::make_shared<Gyro>();
  proto_gyro->mutable_gyro()->set_x(gyro.x);
  proto_gyro->mutable_gyro()->set_y(gyro.y);
  proto_gyro->mutable_gyro()->set_z(gyro.z);

  gyro_writer_->Write(proto_gyro);
}

void RealsenseComponent::CompressedImage(cv::Mat raw_image, uint64 frame_no) {
  std::vector<uchar> data_encode;
  std::vector<int> param = std::vector<int>(2);
  param[0] = CV_IMWRITE_JPEG_QUALITY;
  param[1] = FLAGS_compress_rate;
  cv::imencode(".jpeg", raw_image, data_encode, param);
  std::string str_encode(data_encode.begin(), data_encode.end());

  auto compressedimage = std::make_shared<Image>();
  compressedimage->set_frame_no(frame_no);
  compressedimage->set_height(raw_image.rows);
  compressedimage->set_width(raw_image.cols);
  // encodings
  if (device_model_ == RealSenseDeviceModel::T265) {
    compressedimage->set_encoding(rs2_format_to_string(RS2_FORMAT_Y8));
  } else if (device_model_ == RealSenseDeviceModel::D435I) {
    compressedimage->set_encoding(rs2_format_to_string(RS2_FORMAT_RGBA8));
  }
  compressedimage->set_measurement_time(Time::Now().ToSecond());
  compressedimage->set_data(str_encode);
  compressed_image_writer_->Write(compressedimage);
}

RealsenseComponent::~RealsenseComponent() {
  sensor_.stop();
  sensor_.close();
  async_result_.wait();
}

}  // namespace sensors
}  // namespace apollo
