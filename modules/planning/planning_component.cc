#include "modules/planning/planning_component.h"

namespace apollo {
namespace planning {

bool PlanningComponent::Init() {
  // init

  writer_ = node_->CreateWriter<Trajectory>("/planning");

  return true;
}

PlanningComponent::~PlanningComponent() {}

}  // namespace planning
}  // namespace apollo
