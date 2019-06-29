#pragma once

#include <memory>
#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "modules/sensors/proto/sensors.pb.h"

namespace apollo {
namespace perception {

using apollo::cyber::Component;
using apollo::sensors::Image;

class PerceptionComponent : public Component<Image> {
 public:
  bool Init() override;
  bool Proc(const std::shared_ptr<Image>& image) override;
};

CYBER_REGISTER_COMPONENT(PerceptionComponent)
}  // namespace perception
}  // namespace apollo
