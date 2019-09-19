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
#include "pcl/filters/passthrough.h"

#include "cyber/common/log.h"
#include "cyber/cyber.h"
#include "cyber/time/time.h"
#include "modules/sensors/proto/sensors.pb.h"
#include "modules/sensors/realsense.h"

namespace apollo {
namespace sensors {

using apollo::cyber::Time;
using apollo::cyber::common::GetAbsolutePath;
using apollo::sensors::Acc;
using apollo::sensors::Gyro;
using apollo::sensors::Image;
using apollo::sensors::Pose;

using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

bool RealsenseComponent::Init() {
  InitDeviceAndSensor();

  if (FLAGS_publish_pose && device_model_ == RealSenseDeviceModel::T265) {
    pose_writer_ = node_->CreateWriter<Pose>(FLAGS_pose_channel);
  }
  if (FLAGS_publish_raw_gray_image) {
    image_writer_ = node_->CreateWriter<Image>(FLAGS_gray_image_channel);
  }

  if (FLAGS_publish_color_image &&
      device_model_ == RealSenseDeviceModel::D435I) {
    color_image_writer_ = node_->CreateWriter<Image>(FLAGS_color_image_channel);
  }

  // Point cloud channel
  if (FLAGS_publish_point_cloud &&
      device_model_ == RealSenseDeviceModel::D435I) {
    point_cloud_writer_ = node_->CreateWriter<apollo::sensors::PointCloud>(
        FLAGS_point_cloud_channel);
  }

  if (FLAGS_publish_acc) {
    acc_writer_ = node_->CreateWriter<Acc>(FLAGS_acc_channel);
  }

  if (FLAGS_publish_gyro) {
    gyro_writer_ = node_->CreateWriter<Gyro>(FLAGS_gyro_channel);
  }

  if (FLAGS_publish_compressed_color_image &&
      device_model_ == RealSenseDeviceModel::D435I) {
    compressed_image_writer_ =
        node_->CreateWriter<Image>(FLAGS_compressed_color_image_channel);
  }

  if (FLAGS_publish_compressed_gray_image &&
      device_model_ == RealSenseDeviceModel::T265) {
    compressed_image_writer_ =
        node_->CreateWriter<Image>(FLAGS_compressed_gray_image_channel);
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

void RealsenseComponent::InitDeviceAndSensor() {
  device_ = first_connected_device();

  if (std::strstr(device_.get_info(RS2_CAMERA_INFO_NAME), "T265")) {
    device_model_ = RealSenseDeviceModel::T265;
  } else if (std::strstr(device_.get_info(RS2_CAMERA_INFO_NAME), "D435I")) {
    device_model_ = RealSenseDeviceModel::D435I;
  } else {
    AWARN << "The device data is not yet supported for parsing";
  }

  // Add desired streams to configuration
  cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
  // Use a configuration object to request only depth from the pipeline
  cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
  // Add streams of gyro and accelerometer to configuration
  cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
  cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);

  // Instruct pipeline to start streaming with the requested configuration
  pipe.start(cfg);

  // load_wheel_odometery_config
  if (device_model_ == RealSenseDeviceModel::T265) {
    Calibration();
    WheelOdometry();
  }
}

void RealsenseComponent::run() {
  while (!apollo::cyber::IsShutdown()) {
    // Camera warmup - dropping several first frames to let auto-exposure
    // stabilize
    rs2::frameset frames;
    // Wait for all configured streams to produce a frame
    frames = pipe.wait_for_frames();

    if (FLAGS_publish_color_image) {
      rs2::frame color_frame = frames.get_color_frame();
      OnColorImage(color_frame);
    }

    if (FLAGS_publish_point_cloud) {
      rs2::frame depth_frame = frames.get_depth_frame();
      OnPointCloud(depth_frame);
    }

    if (FLAGS_publish_acc) {
      rs2::motion_frame accel_frame = frames.first_or_default(RS2_STREAM_ACCEL);
      OnAcc(accel_frame);
    }

    if (FLAGS_publish_gyro) {
      rs2::motion_frame gyro_frame = frames.first_or_default(RS2_STREAM_GYRO);
      OnGyro(gyro_frame);
    }

    // const int fisheye_sensor_idx = 1;  // for the left fisheye lens of T265
    // auto fisheye_frame = frames.get_fisheye_frame(1);
    // OnGrayImage(fisheye_frame);
  }
}

void RealsenseComponent::OnGrayImage(rs2::frame fisheye_frame) {
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
    OnCompressedImage(dst, fisheye_frame.get_frame_number());
  }
}

void RealsenseComponent::OnColorImage(rs2::frame color_frame) {
  // Creating OpenCV Matrix from a color image
  cv::Mat mat(cv::Size(640, 480), CV_8UC3, (void*)color_frame.get_data(),
              cv::Mat::AUTO_STEP);
  AINFO << "FRAME NUMBER:" << color_frame.get_frame_number();
  auto image_proto = std::make_shared<Image>();
  image_proto->set_frame_no(color_frame.get_frame_number());
  image_proto->set_height(mat.rows);
  image_proto->set_width(mat.cols);
  // encoding /**< 16-bit linear depth values. The depth is meters is equal to
  // depth scale * pixel value. */
  image_proto->set_encoding(
      rs2_format_to_string(color_frame.get_profile().format()));

  image_proto->set_measurement_time(color_frame.get_timestamp());
  auto m_size = mat.rows * mat.cols * mat.elemSize();
  image_proto->set_data(mat.data, m_size);
  color_image_writer_->Write(image_proto);

  if (FLAGS_publish_compressed_color_image) {
    OnCompressedImage(mat, color_frame.get_frame_number());
  }
}

void RealsenseComponent::OnPointCloud(rs2::frame depth_frame) {
  // Declare pointcloud object, for calculating pointclouds and texture mappings
  rs2::pointcloud pc;
  // We want the points object to be persistent so we can display the last cloud
  // when a frame drops
  rs2::points points;

  // Generate the pointcloud and texture mappings
  points = pc.calculate(depth_frame);
  AINFO << "POINT SIZE BEFORE FILTER IS " << points.size();

  auto pcl_points = points_to_pcl(points);

  pcl_ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(pcl_points);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.0, 1.0);
  pass.setFilterLimitsNegative(true);
  pass.filter(*cloud_filtered);

  AINFO << "POINT SIZE AFTER FILTER IS " << (*cloud_filtered).size();

  // Apply an affine transform defined by an Eigen Transform.
  // pcl::transformPointCloud(*source_cloud, *transformed_cloud, transform_1);

  auto sp = points.get_profile().as<rs2::video_stream_profile>();

  auto point_cloud_proto = std::make_shared<apollo::sensors::PointCloud>();
  // point_cloud_proto->set_frame_no(depth_frame.get_frame_number());
  point_cloud_proto->set_is_dense(false);
  point_cloud_proto->set_measurement_time(depth_frame.get_timestamp());
  point_cloud_proto->set_width(sp.width());
  point_cloud_proto->set_height(sp.height());

  // from rs-pointcloud Sample, after z axis filter, we can get 130000+ points
  for (size_t i = 0; i < (*cloud_filtered).size(); i++) {
    if ((*cloud_filtered)[i].z) {
      // publish the point/texture coordinates only for points we have depth
      // data for
      apollo::sensors::PointXYZIT* p = point_cloud_proto->add_point();
      p->set_x((*cloud_filtered)[i].x);
      p->set_y((*cloud_filtered)[i].y);
      p->set_z((*cloud_filtered)[i].z);
      // p->set_intensity(0);
      // p->set_timestamp(depth_frame.get_timestamp());
    }
  }

  point_cloud_writer_->Write(point_cloud_proto);
}

void RealsenseComponent::OnPose(rs2::pose_frame pose_frame) {
  auto pose_data = pose_frame.get_pose_data();
  AINFO << "Pose: " << pose_data.translation;
  double norm = sqrt(pose_data.translation.x * pose_data.translation.x +
                     pose_data.translation.y * pose_data.translation.y +
                     pose_data.translation.z * pose_data.translation.z);
  if (norm > norm_max) {
    norm_max = norm;
  }

  ADEBUG << "norm_max:" << norm_max;

  // send vehicle speed to wheel odometry
  auto wo_sensor = device_.first<rs2::wheel_odometer>();
  if (!wo_sensor.send_wheel_odometry(0, 0, {chassis_.speed(), 0, 0})) {
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

void RealsenseComponent::OnAcc(rs2::motion_frame accel_frame) {
  rs2_vector acc = accel_frame.get_motion_data();
  AINFO << "Accel:" << acc.x << ", " << acc.y << ", " << acc.z;
  auto proto_accel = std::make_shared<Acc>();
  proto_accel->mutable_acc()->set_x(acc.x);
  proto_accel->mutable_acc()->set_y(acc.y);
  proto_accel->mutable_acc()->set_z(acc.z);

  acc_writer_->Write(proto_accel);
}

void RealsenseComponent::OnGyro(rs2::motion_frame gyro_frame) {
  rs2_vector gyro = gyro_frame.get_motion_data();
  AINFO << "Gyro:" << gyro.x << ", " << gyro.y << ", " << gyro.z;
  auto proto_gyro = std::make_shared<Gyro>();
  proto_gyro->mutable_gyro()->set_x(gyro.x);
  proto_gyro->mutable_gyro()->set_y(gyro.y);
  proto_gyro->mutable_gyro()->set_z(gyro.z);

  gyro_writer_->Write(proto_gyro);
}

void RealsenseComponent::OnCompressedImage(cv::Mat raw_image, uint64 frame_no) {
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
    compressedimage->set_encoding(rs2_format_to_string(RS2_FORMAT_BGR8));
  }
  compressedimage->set_measurement_time(Time::Now().ToSecond());
  compressedimage->set_data(str_encode);
  compressed_image_writer_->Write(compressedimage);
}

void RealsenseComponent::Calibration() {
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

RealsenseComponent::~RealsenseComponent() {
  if (sensor_) {
    sensor_.stop();
    sensor_.close();
  }
  async_result_.wait();
}

}  // namespace sensors
}  // namespace apollo
