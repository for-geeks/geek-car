#include "modules/planning/planning_component.h"

namespace apollo {
namespace planning {
bool Planning_Component::Init() {
  // init

  writer_ = node_->CreateWriter<Trajectory>("/planning");
}

}  // namespace planning
}  // namespace apollo
