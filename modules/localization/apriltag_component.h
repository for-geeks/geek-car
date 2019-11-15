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
#pragma once

#include <memory>

#include "apriltag.h"
#include "librealsense2/rs.hpp"

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/sensors/proto/sensor_image.pb.h"
#include "modules/sensors/proto/sensors.pb.h"

namespace apollo {
namespace localization {

using apollo::cyber::Component;
using apollo::cyber::Reader;
using apollo::cyber::Writer;
using apollo::localization::Tags;
using apollo::sensors::Image;
using apollo::localization::Matrix;
using apollo::localization::Tag;

class ApriltagComponent : public Component<> {
 public:
  bool Init() override;
  void ApriltagDetection(const std::shared_ptr<Image>& image);
  ~ApriltagComponent() override;

  std::shared_ptr<Reader<Pose>> pose_reader_ = nullptr;
  std::shared_ptr<Reader<Image>> image_reader_ = nullptr;
  std::shared_ptr<Writer<Tags>> tags_writer_ = nullptr;
  rs2_pose predicted_pose_;

  std::future<void> tag_async_;
  std::atomic<bool> image_ready_ = {false};
  apriltag_detector_t* td_ = nullptr;
  apriltag_family_t* tf_ = nullptr;
};

CYBER_REGISTER_COMPONENT(ApriltagComponent);
}  // namespace localization
}  // namespace apollo
