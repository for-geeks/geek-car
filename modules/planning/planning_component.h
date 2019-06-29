#include <memory>

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "modules/planning/proto/planning.pb.h"

namespace apollo {
namespace planning {

using apollo::cyber::Component;
using apollo::planning::Tracjectory;

class PlanningComponent : public Component<> {
 public:
  ~PlanningComponent();
  bool Init() override;

 private:
  std::shared_ptr<Writer<Tracjectory>> writer_ = nullptr;
}

CYBER_REGISTER_COMPONENT(PlanningComponent)
}  // namespace planning
}  // namespace apollo
