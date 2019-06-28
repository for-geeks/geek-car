#pragma once

#include "cyber/component/component.h"
#include "modules/sensors/proto/sensors.pb.h"

namespace apollo {
namespace localization {

using apollo::sensors::Pose;

class Localization : public Component<Pose> {
 public:
  bool Init() override;
  bool Proc(const std::shared<Pose>& pose) override;
};

CYBER_REGISTER_COMPONENT(LocalizationComponent);
}  // namespace localization
}  // namespace apollo
