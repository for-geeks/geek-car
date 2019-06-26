#pragma once

#include <librealsense2/rs.hpp>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
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
  void run();
  ~RealsenseComponent();

 private:
  void CalibrationLeft(rs2::frame);
  bool OnImage(cv::Mat dst, uint64 frame_on);
  bool OnPose(rs2_pose pose_data, uint64 frame_on);
  std::shared_ptr<Writer<Image>> image_writer_ = nullptr;
  std::shared_ptr<Writer<Pose>> pose_writer_ = nullptr;

  // realsense device
  rs2::device device_;
  // sensor include imu and camera;
  rs2::sensor sensor_;

  std::future<void> async_result_;

  // ms
  uint32_t device_wait_ = 2000;

  // ms
  uint32_t spin_rate_ = 200;

  // frame queue
  rs2::frame_queue q_;

  /**
   * @brief from RS2_OPTION_FRAMES_QUEUE_SIZE
   * you are telling the SDK not to recycle frames for this sensor.
   * < Number of frames the user is allowed to keep per stream. Trying to
   * hold-on to more frames will cause frame-drops.
   * */
  int queue_size_ = 16;  // queue size

  std::string serial_number_ = "908412111198";  // serial number

  // pipe
  // rs2::pipeline pipe_;
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
