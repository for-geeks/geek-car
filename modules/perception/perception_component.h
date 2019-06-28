
#pragma once
#include "cyber/component/component.h"
#include "moduels/sensors/proto/sensors.pb.h"

namespace apollo {
namespace perception {

using apollo::sensors::Image;

class PerceptionComponent : public Component<Image> {
 public:
  bool Init() override;
  bool Proc(const std::shared<Image>& image) override;
};

CYBER_REGISTER_COMPONENT(PerceptionComponent)
}  // namespace perception
}  // namespace apollo
