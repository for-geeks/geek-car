#include "modules/localization/localization_component.h"

namespace apollo {
namespace localization {
bool LocalizationComponent::Init() {}

bool LocalizationComponent::Proc(const std::shared<Pose>& pose) {
  ADEBUG << pose->DebugString();
}

}  // namespace localization
}  // namespace apollo
