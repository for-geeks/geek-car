#include "modules/sensors/realsense_component.h"

#include <chrono>
#include <iomanip>
#include <iostream>
#include <librealsense2/rs.hpp>
#include <mutex>
#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include <thread>
#include <utility>

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
using apollo::sensors::Image;
using apollo::sensors::Pose;

rs2::device get_device(const std::string& serial_number = "") {
  rs2::context ctx;
  while (true) {
    for (auto&& dev : ctx.query_devices()) {
      if (((serial_number.empty() &&
            std::strstr(dev.get_info(RS2_CAMERA_INFO_NAME), "T265")) ||
           std::strcmp(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER),
                       serial_number.c_str()) == 0))
        return dev;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

/**
 * @brief device and sensor init
 *
 * @return true
 * @return false
 */
bool RealsenseComponent::Init() {
  device_ = get_device(serial_number_);

  // print device information
  RealSense::printDeviceInformation(device_);

  AINFO << "Got device with serial number "
        << device_.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
  cyber::SleepFor(std::chrono::milliseconds(device_wait_));

  sensor_ = device_.first<rs2::sensor>();
  // print sensor option to log
  RealSense::getSensorOption(sensor_);
  sensor_.set_option(RS2_OPTION_FRAMES_QUEUE_SIZE, 0);
  sensor_.open(sensor_.get_stream_profiles());

  pose_writer_ = node_->CreateWriter<Pose>("/realsense/pose");
  image_writer_ = node_->CreateWriter<Image>("/realsense/raw_image");

  sensor_.start([this](rs2::frame f) {
    q_.enqueue(std::move(f));  // enqueue any new frames into q
  });

  // async_result_ = cyber::Async(&RealsenseComponent::run, this);
  async_result_ =
      std::async(std::launch::async, &RealsenseComponent::run, this);
  return true;
}

/**
 * [RealSense:: collect frames of T265]
 * @return [description]
 */
void RealsenseComponent::run() {
  while (!cyber::IsShutdown()) {
    // wait for device is ready. in case of device is busy
    if (!device_) {
      device_ = get_device();
      continue;
    }

    // wait until new frame is available and dequeue it
    // handle frames in the main event loop
    rs2::frame f = q_.wait_for_frame();

    // core of data write to channel
    switch (f.get_profile().stream_type()) {
      case RS2_STREAM_POSE: {
        auto pose_frame = f.as<rs2::pose_frame>();
        auto pose_data = pose_frame.get_pose_data();
        AINFO << "pose " << pose_data.translation;
        OnPose(pose_data, pose_frame.get_frame_number());
      }
      case RS2_STREAM_GYRO: {
        auto gyro_frame = f.as<rs2::motion_frame>();
        rs2_vector gyro_sample = gyro_frame.get_motion_data();
        AINFO << "Gyro:" << gyro_sample.x << ", " << gyro_sample.y << ", "
              << gyro_sample.z;
      }
      case RS2_STREAM_ACCEL: {
        auto accel_frame = f.as<rs2::motion_frame>();
        rs2_vector accel_sample = accel_frame.get_motion_data();
        AINFO << "Accel:" << accel_sample.x << ", " << accel_sample.y << ", "
              << accel_sample.z;
      }
      case RS2_STREAM_FISHEYE: {
        // left fisheye
        if (f.get_profile().stream_index() == 1) {
          // this is one of the fisheye imagers
          auto fisheye_frame = f.as<rs2::video_frame>();
          // CalibrationLeft(f);

          AINFO << "fisheye " << f.get_profile().stream_index() << ", "
                << fisheye_frame.get_width() << "x"
                << fisheye_frame.get_height();

          cv::Mat image(
              cv::Size(fisheye_frame.get_width(), fisheye_frame.get_height()),
              CV_8U, (void*)fisheye_frame.get_data(), cv::Mat::AUTO_STEP);
          cv::Mat dst;
          cv::remap(image, dst, map1_, map2_, cv::INTER_LINEAR);
          OnImage(dst, fisheye_frame.get_frame_number());
        }
      }
      default:
        break;
    }
    // cyber::SleepFor(std::chrono::milliseconds(spin_rate_));
  }
}

/**
 * @brief
 *
 * @param f
 */
void CalibrationLeft(const rs2::frame& f) {
  rs2_intrinsics intrinsicsleft =
      f.as<rs2::video_stream_profile>().get_intrinsics();
  intrinsicsL_ =
      (cv::Mat_<double>(3, 3) << intrinsicsleft.fx, 0, intrinsicsleft.ppx, 0,
       intrinsicsleft.fy, intrinsicsleft.ppy, 0, 0, 1);
  distCoeffsL_ = cv::Mat(1, 4, CV_32F, intrinsicsleft.coeffs);
  cv::Mat R = cv::Mat::eye(3, 3, CV_32F);
  cv::Mat P =
      (cv::Mat_<double>(3, 4) << intrinsicsleft.fx, 0, intrinsicsleft.ppx, 0, 0,
       intrinsicsleft.fy, intrinsicsleft.ppy, 0, 0, 0, 1, 0);

  cv::fisheye::initUndistortRectifyMap(intrinsicsL_, distCoeffsL_, R, P,
                                       cv::Size(848, 816), CV_16SC2, map1_,
                                       map2_);

#if 1
  // Start streaming through the callback
  rs2::pipeline_profile profiles = pipe.start(cfg, callback);
  rs2::stream_profile fisheye_stream =
      profiles.get_stream(RS2_STREAM_FISHEYE, 1);
  rs2_intrinsics intrinsicsleft =
      fisheye_stream.as<rs2::video_stream_profile>().get_intrinsics();
  intrinsicsL =
      (cv::Mat_<double>(3, 3) << intrinsicsleft.fx, 0, intrinsicsleft.ppx, 0,
       intrinsicsleft.fy, intrinsicsleft.ppy, 0, 0, 1);
  distCoeffsL = cv::Mat(1, 4, CV_32F, intrinsicsleft.coeffs);
  cv::Mat R = cv::Mat::eye(3, 3, CV_32F);
  cv::Mat P =
      (cv::Mat_<double>(3, 4) << intrinsicsleft.fx, 0, intrinsicsleft.ppx, 0, 0,
       intrinsicsleft.fy, intrinsicsleft.ppy, 0, 0, 0, 1, 0);

  cv::fisheye::initUndistortRectifyMap(
      intrinsicsL, distCoeffsL, R, P, cv::Size(848, 816), CV_16SC2, map1, map2);
#endif
}

/**
 * @brief callback of Image data
 *
 * @param dst
 * @return true
 * @return false
 */
bool RealsenseComponent::OnImage(cv::Mat dst, uint64 frame_no) {
  auto image_proto = std::make_shared<Image>();
  image_proto->set_frame_no(frame_no);
  image_proto->set_encoding(rs2_format_to_string(RS2_FORMAT_Y8));
  image_proto->set_measurement_time(Time::Now().ToSecond());
  auto m_size = dst.rows * dst.cols * dst.elemSize();
  image_proto->set_data(dst.data, m_size);
  image_writer_->Write(image_proto);
  return true;
}

/**
 * @brief callback of Pose data
 *
 * @param pose_data
 * @return true
 * @return false
 */
bool RealsenseComponent::OnPose(rs2_pose pose_data, uint64 frame_no) {
  auto pose_proto = std::make_shared<Pose>();
  pose_proto->set_frame_no(frame_no);
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

  auto angular_velocity = pose_proto->mutable_angular_velocity();
  angular_velocity->set_x(pose_data.angular_velocity.x);
  angular_velocity->set_y(pose_data.angular_velocity.y);
  angular_velocity->set_z(pose_data.angular_velocity.z);

  // pose_proto->set_angular_accelaration(pose_data.angular_acceleration);
  // pose_proto->set_trancker_confidence(pose_data.tracker_confidence);
  // pose_proto->set_mapper_confidence(pose_data.mapper_confidence);

  pose_writer_->Write(pose_proto);
  return true;
}

RealsenseComponent::~RealsenseComponent() {
  sensor_.stop();
  sensor_.close();
  async_result_.wait();
}

}  // namespace sensors
}  // namespace apollo
