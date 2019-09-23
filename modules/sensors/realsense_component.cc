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

#include <algorithm>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

#include "pcl/common/transforms.h"
#include "pcl/filters/passthrough.h"

#include "cyber/common/log.h"
#include "cyber/cyber.h"
#include "cyber/time/time.h"
#include "modules/sensors/realsense.h"

namespace apollo {
namespace sensors {

using apollo::cyber::Time;
using apollo::cyber::common::GetAbsolutePath;
using apollo::sensors::Acc;
using apollo::sensors::Gyro;
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
    point_cloud_writer_ =
        node_->CreateWriter<PointCloud>(FLAGS_point_cloud_channel);
  }

  if (FLAGS_publish_acc) {
    acc_writer_ = node_->CreateWriter<Acc>(FLAGS_acc_channel);
  }

  if (FLAGS_publish_gyro) {
    gyro_writer_ = node_->CreateWriter<Gyro>(FLAGS_gyro_channel);
  }

  if (FLAGS_publish_compressed_color_image &&
      device_model_ == RealSenseDeviceModel::D435I) {
    compressed_image_writer_ = node_->CreateWriter<CompressedImage>(
        FLAGS_compressed_color_image_channel);
  }

  if (FLAGS_publish_compressed_gray_image &&
      device_model_ == RealSenseDeviceModel::T265) {
    compressed_image_writer_ = node_->CreateWriter<CompressedImage>(
        FLAGS_compressed_gray_image_channel);
  }

  chassis_reader_ = node_->CreateReader<Chassis>(
      FLAGS_chassis_channel, [this](const std::shared_ptr<Chassis> &chassis) {
        chassis_.Clear();
        chassis_.CopyFrom(*chassis);
      });

  point_cloud_pool_.reset(new CCObjectPool<PointCloud>(8));
  point_cloud_pool_->ConstructAll();
  for (int i = 0; i < 8; i++) {
    auto point_cloud = point_cloud_pool_->GetObject();
    if (point_cloud == nullptr) {
      AERROR << "fail to getobject, i: " << i;
      return false;
    }
    point_cloud->mutable_point()->Reserve(210000);
  }

  // thread to handle frames
  async_result_ = cyber::Async(&RealsenseComponent::run, this);

  // thread to Publish point cloud
  std::thread(&RealsenseComponent::PublishPointCloud, this).detach();
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
  cfg.enable_stream(RS2_STREAM_COLOR, FLAGS_color_image_width,
                    FLAGS_color_image_height, RS2_FORMAT_BGR8,
                    FLAGS_color_image_frequency);
  // Use a configuration object to request only depth from the pipeline
  cfg.enable_stream(RS2_STREAM_DEPTH, FLAGS_color_image_width,
                    FLAGS_color_image_height, RS2_FORMAT_Z16,
                    FLAGS_color_image_frequency);
  // Add streams of gyro and accelerometer to configuration
  cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
  cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);

  // Instruct pipeline to start streaming with the requested configuration
  auto profile = pipe.start(cfg);
  auto sensor = profile.get_device().first<rs2::depth_sensor>();

  // Set the device to High Accuracy preset of the D400 stereoscopic cameras
  if (sensor && sensor.is<rs2::depth_stereo_sensor>())
  {
    sensor.set_option(RS2_OPTION_VISUAL_PRESET, RS2_RS400_VISUAL_PRESET_HIGH_ACCURACY);
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

    if (FLAGS_publish_acc) {
      rs2::motion_frame accel_frame = frames.first_or_default(RS2_STREAM_ACCEL);
      OnAcc(accel_frame);
    }

    if (FLAGS_publish_gyro) {
      rs2::motion_frame gyro_frame = frames.first_or_default(RS2_STREAM_GYRO);
      OnGyro(gyro_frame);
    }

    if (FLAGS_publish_point_cloud) {
      if (FLAGS_enable_point_cloud_transform) {
        auto angle = algo_.get_theta();
        AINFO << "CALCULATED ANGLE X:" << angle.x << " Z:" << angle.z;

        transform = Eigen::Matrix4f::Identity();
        ::Eigen::Vector3d ea0(0, angle.x, angle.z);
        ::Eigen::Matrix3d R;
        R = ::Eigen::AngleAxisd(ea0[0], ::Eigen::Vector3d::UnitZ()) *
            ::Eigen::AngleAxisd(ea0[1], ::Eigen::Vector3d::UnitY()) *
            ::Eigen::AngleAxisd(ea0[2], ::Eigen::Vector3d::UnitX());
        std::cout << "ROTATION :" << R << std::endl << std::endl;
        Eigen::MatrixXf RR = R.cast<float>();
        transform(0, 0) = RR(0, 0);
        transform(0, 1) = RR(0, 1);
        transform(0, 2) = RR(0, 2);
        transform(1, 0) = RR(1, 0);
        transform(1, 1) = RR(1, 1);
        transform(1, 2) = RR(1, 2);
        transform(2, 0) = RR(2, 0);
        transform(2, 1) = RR(2, 1);
        transform(2, 2) = RR(2, 2);
        std::cout << "TRANSFORM:" << transform << std::endl << std::endl;
      }

      rs2::frame depth_frame = frames.get_depth_frame();
      OnPointCloud(depth_frame);
    }

    // const int fisheye_sensor_idx = 1;  // for the left fisheye lens of T265
    // auto fisheye_frame = frames.get_fisheye_frame(1);
    // OnGrayImage(fisheye_frame);
  }
}

void RealsenseComponent::OnGrayImage(const rs2::frame &fisheye_frame) {
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

void RealsenseComponent::OnColorImage(const rs2::frame &color_frame) {
  // Creating OpenCV Matrix from a color image
  cv::Mat mat(cv::Size(FLAGS_color_image_width, FLAGS_color_image_height),
              CV_8UC3, const_cast<void *>(color_frame.get_data()),
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
    OnCompressedImage(color_frame, mat);
  }
}

void RealsenseComponent::OnPointCloud(rs2::frame depth_frame) {
  rs2::threshold_filter thr_filter(static_cast<float>(FLAGS_point_cloud_min_distance),
    static_cast<float>(FLAGS_point_cloud_max_distance));

  depth_frame = thr_filter.process(depth_frame);

  filtered_data.enqueue(depth_frame);
}

void RealsenseComponent::PublishPointCloud(){
  while(!apollo::cyber::IsShutdown()) {
  AINFO << "ENTERED PublishPointCloud METHOD";
  // Declare pointcloud object, for calculating pointclouds and texture mappings
  rs2::pointcloud pc;
  // We want the points object to be persistent so we can display the last cloud
  // when a frame drops
  rs2::points points;

  rs2::frame depth_frame;
  filtered_data.poll_for_frame(&depth_frame);
  if(!depth_frame) {
    AINFO << "FRAME QUEUE IS EMPTY, WAIT FOR ENQUEUE;";
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    continue;
  }
  // Generate the pointcloud and texture mappings
  points = pc.calculate(depth_frame);

  auto t1 = Time::Now().ToSecond();
  auto pcl_points = points_to_pcl(points);
  auto t2 = Time::Now().ToSecond();
  AINFO << "time for realsense point to point cloud:" << t2 - t1;

  pcl_ptr cloud_(new pcl::PointCloud<pcl::PointXYZ>);

  if (FLAGS_enable_point_cloud_transform) {
    // Apply an affine transform defined by an Eigen Transform.
    pcl::transformPointCloud(*pcl_points, *cloud_, transform);
  } else {
    *cloud_ = *pcl_points;
  }

  auto sp = points.get_profile().as<rs2::video_stream_profile>();

  std::shared_ptr<PointCloud> point_cloud_out = point_cloud_pool_->GetObject();
  if (point_cloud_out == nullptr) {
    AWARN << "point cloud pool return nullptr, will be create new.";
    point_cloud_out = std::make_shared<PointCloud>();
    point_cloud_out->mutable_point()->Reserve(210000);
  }
  if (point_cloud_out == nullptr) {
    AWARN << "point cloud out is nullptr";
    return;
  }
  point_cloud_out->Clear();

  point_cloud_out->set_is_dense(false);
  point_cloud_out->set_measurement_time(Time::Now().ToSecond());
  point_cloud_out->set_width(sp.width());
  point_cloud_out->set_height(sp.height());

  for (size_t i = 0; i < (*cloud_).size(); i++) {
    if ((*cloud_)[i].z) {
      apollo::sensors::PointXYZIT *p = point_cloud_out->add_point();
      p->set_x((*cloud_)[i].x);
      p->set_y((*cloud_)[i].y);
      p->set_z((*cloud_)[i].z);
      // p->set_intensity(0);
      // p->set_timestamp(depth_frame.get_timestamp());
    }
  }

  auto tt = Time::Now().ToSecond();
  AINFO << "all time for point cloud:" << tt - t1;

  point_cloud_writer_->Write(point_cloud_out);
  }

}

void RealsenseComponent::OnPose(const rs2::pose_frame &pose_frame) {
  auto pose_data = pose_frame.get_pose_data();
  AINFO << "Pose: " << pose_data.translation;
  double norm = std::sqrt(pose_data.translation.x * pose_data.translation.x +
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

void RealsenseComponent::OnAcc(const rs2::motion_frame &accel_frame) {
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

void RealsenseComponent::OnGyro(const rs2::motion_frame &gyro_frame) {
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

void RealsenseComponent::OnCompressedImage(const rs2::frame &f,
                                           cv::Mat raw_image) {
  std::vector<uchar> data_encode;
  std::vector<int> param = std::vector<int>(2);
  param[0] = CV_IMWRITE_JPEG_QUALITY;
  param[1] = FLAGS_compress_rate;
  cv::Mat tmp_mat;
  cv::cvtColor(raw_image, tmp_mat, cv::COLOR_RGB2BGR);
  cv::imencode(".jpeg", tmp_mat, data_encode, param);
  std::string str_encode(data_encode.begin(), data_encode.end());

  auto compressedimage = std::make_shared<CompressedImage>();
  compressedimage->set_frame_no(f.get_frame_number());
  compressedimage->set_format("jpeg");
  compressedimage->set_measurement_time(f.get_timestamp());
  compressedimage->set_data(str_encode);

  compressed_image_writer_->Write(compressedimage);
}

RealsenseComponent::~RealsenseComponent() {
  if (sensor_) {
    sensor_.stop();
    sensor_.close();
  }
  async_result_.wait();
}

} // namespace sensors
} // namespace apollo
