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
  void CalibrationLeft(const rs2::frame& f);
  bool OnImage(cv::Mat dst, uint64 frame_no);
  bool OnPose(rs2_pose pose_data, uint64 frame_no);
  std::shared_ptr<Writer<Image>> image_writer_ = nullptr;
  std::shared_ptr<Writer<Pose>> pose_writer_ = nullptr;
  std::future<void> async_result_;
  rs2::device device_;  // realsense device
  rs2::sensor sensor_;  // sensor include imu and camera;

  uint32_t device_wait_ = 2000;  // ms
  uint32_t spin_rate_ = 200;     // ms

  // frame queue
  rs2::frame_queue q_;

  /**
   * @brief from RS2_OPTION_FRAMES_QUEUE_SIZE
   * you are telling the SDK not to recycle frames for this sensor.
   * < Number of frames the user is allowed to keep per stream. Trying to
   * hold-on to more frames will cause frame-drops.
   * */
  float queue_size_ = 16.0;  // queue size

  std::string serial_number_ = "908412111198";  // serial number

  cv::Mat intrinsicsL_;
  cv::Mat distCoeffsL_;
  cv::Mat map1_;
  cv::Mat map2_;
};

CYBER_REGISTER_COMPONENT(RealsenseComponent)
}  // namespace sensors
}  // namespace apollo
