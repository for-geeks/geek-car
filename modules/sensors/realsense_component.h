#pragma once

#include <librealsense2/rs.hpp>
#include <memory>
#include <opencv2/opencv.hpp>

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "modules/sensors/proto/sensors.pb.h"

namespace apollo {
namespace sensors {

using apollo::cyber::Component;
using apollo::cyber::Writer;

class RealsenseComponent : public Component<> {
 public:
  bool Init() override;
  void Proc();
  ~RealsenseComponent();

 private:
  bool OnImage(cv::Mat dst);
  bool OnPose(rs2_pose pose_data);
  std::shared_ptr<Writer<Image>> image_writer_ = nullptr;
  std::shared_ptr<Writer<Pose>> pose_writer_ = nullptr;

  // realsense device
  rs2::device device_;
  // sensor include imu and camera;
  rs2::sensor sensor_;
  std::future<void> async_result_;

  // ms
  uint32_t device_wait_ = 2000;

  int index_ = 0;

  int buffer_size_ = 16;
  // ms
  uint32_t spin_rate_ = 200;

  // pipe
  rs2::pipeline pipe_;
  // Create a configuration for configuring the pipeline with a non default
  // profile
  rs2::config cfg_;
  cv::Mat intrinsicsL;
  cv::Mat distCoeffsL;
  cv::Mat map1_;
  cv::Mat map2_;
};

CYBER_REGISTER_COMPONENT(RealsenseComponent)
}  // namespace sensors
}  // namespace apollo