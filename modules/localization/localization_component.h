#pragma once
#include <memory>

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"

namespace apollo {
namespace localization {

using apollo::cyber::Component;
using apollo::cyber::Reader;
using apollo::cyber::Writer;

class LocalizationComponent : public Component<> {
 public:
  LocalizationComponent() = default;
  ~LocalizationComponent();

  bool Init() override;

 private:
};
CYBER_REGISTER_COMPONENT(LocalizationComponent);
}  // namespace localization
}  // namespace apollo
