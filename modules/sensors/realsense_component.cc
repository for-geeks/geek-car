
#include "modules/sensors/realsense_component.h"

#include <chrono>
#include <iomanip>
#include <iostream>
#include <librealsense2/rs.hpp>
#include <mutex>
#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

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

/**
 * @brief device and sensor init
 *
 * @return true
 * @return false
 */
bool RealsenseComponent::Init() {
  try {
    device_ = RealSense::getDevice();
    if (!device_) {
      AERROR << "Device T265 is not ready or connected.";
      return false;
    }

    // print device infomation
    AINFO << "Device name :" << RealSense::getDeviceName(device_);
    RealSense::printDeviceInformation(device_);

    // select sensor default senser is 1. is it fisheye?
    sensor_ = RealSense::getSensorFromDevice(device_);
    if (!sensor_) {
      AERROR << "Sensor of Device T265 is not ready or selected.";
      return false;
    }
    AINFO << "Sensor Name: " << RealSense::getSensorName(sensor_);
    RealSense::getSensorOption(sensor_);

    // Add pose stream
    cfg_.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
    // Enable both image streams
    // Note: It is not currently possible to enable only one
    cfg_.enable_stream(RS2_STREAM_FISHEYE, 1, RS2_FORMAT_Y8);
    cfg_.enable_stream(RS2_STREAM_FISHEYE, 2, RS2_FORMAT_Y8);

    // Enable imu parameter
    // cfg_.enable_stream(RS2_STREAM_GYRO);
    // cfg_.enable_stream(RS2_STREAM_ACCEL);

    // Start streaming through the callback
    rs2::pipeline_profile profiles = pipe_.start(cfg_);

    // calibration
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
        (cv::Mat_<double>(3, 4) << intrinsicsleft.fx, 0, intrinsicsleft.ppx, 0,
         0, intrinsicsleft.fy, intrinsicsleft.ppy, 0, 0, 0, 1, 0);

    cv::fisheye::initUndistortRectifyMap(intrinsicsL, distCoeffsL, R, P,
                                         cv::Size(848, 816), CV_16SC2, map1_,
                                         map2_);
  } catch (const rs2::error& e) {
    AERROR << "RealSense error calling " << e.get_failed_function() << "("
           << e.get_failed_args() << "):\n    " << e.what();
    return false;
  } catch (const std::exception& e) {
    AERROR << e.what();
    return false;
  }
  image_writer_ = node_->CreateWriter<Image>("/realsense/raw_image");
  pose_writer_ = node_->CreateWriter<Pose>("/realsense/pose");

  async_result_ = cyber::Async(&RealsenseComponent::Proc, this);

  return true;
}

/**
 * [RealSense:: collect frames of T265]
 * @return [description]
 */
void RealsenseComponent::Proc() {
  while (!cyber::IsShutdown()) {
    if (!device_) {
      // sleep for next check
      cyber::SleepFor(std::chrono::milliseconds(device_wait_));
      continue;
    }

    // Wait for the next set of frames from the camera
    auto frames = pipe_.wait_for_frames();
    // Get a frame from the fisheye stream
    rs2::video_frame fisheye_frame = frames.get_fisheye_frame(1);

    cv::Mat image(
        cv::Size(fisheye_frame.get_width(), fisheye_frame.get_height()), CV_8U,
        (void*)fisheye_frame.get_data(), cv::Mat::AUTO_STEP);
    cv::Mat dst;
    cv::remap(image, dst, map1_, map2_, cv::INTER_LINEAR);
    OnImage(dst);
    // Get a frame from the pose stream
    rs2::pose_frame pose_frame = frames.get_pose_frame();

    // Copy current camera pose
    rs2_pose pose_data = pose_frame.get_pose_data();
    OnPose(pose_data);

    cyber::SleepFor(std::chrono::microseconds(spin_rate_));

    AINFO << "Pose rate: " << pose_counter_
          << " Frame Rate: " << frame_counter_;
  }
}

/**
 * @brief callback of Image data
 *
 * @param dst
 * @return true
 * @return false
 */
bool RealsenseComponent::OnImage(cv::Mat dst) {
  auto image_proto = std::make_shared<Image>();
  image_proto->set_frame_id(frame_counter_);
  image_proto->set_measurement_time(Time::Now().ToSecond());
  auto m_size = dst.rows * dst.cols * dst.elemSize();
  image_proto->set_data(dst.data, m_size);
  image_writer_->Write(image_proto);
  // rate.Sleep();
  frame_counter_++;
  return true;
}

/**
 * @brief callback of Pose data
 *
 * @param pose_data
 * @return true
 * @return false
 */
bool RealsenseComponent::OnPose(rs2_pose pose_data) {
  auto pose_proto = std::make_shared<Pose>();
  auto translation = pose_proto->mutable_translation();
  translation->set_x(pose_data.translation.x);
  translation->set_y(pose_data.translation.y);
  translation->set_z(pose_data.translation.z);

  // pose_proto->set_velocity(pose_data.velocity);
  // pose_proto->set_rotation(pose_data.rotation);
  // pose_proto->set_angular_velocity(pose_data.angular_velocity);
  // pose_proto->set_angular_accelaration(pose_data.angular_acceleration);
  // pose_proto->set_trancker_confidence(pose_data.tracker_confidence);
  // pose_proto->set_mapper_confidence(pose_data.mapper_confidence);

  pose_writer_->Write(pose_proto);
  pose_counter_++;

  return true;
}

RealsenseComponent::~RealsenseComponent() {
  sensor_.stop();
  sensor_.close();
  async_result_.wait();
}

}  // namespace sensors
}  // namespace apollo