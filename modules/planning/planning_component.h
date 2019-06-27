#include <memory>
#include "cyber/component/component.h"
#include "modules/planning/proto/planning.pb.h"
namespace apollo {
namespace planning {

using apollo::planning::Tracjectory;

class Planning_Component : public Component<> {
 public:
  Planning_Component();
  ~Plannning_Component();
  bool Init();

 private:
  std::shared_ptr<Writer<Tracjectory>> writer_ = nullptr;
}

CYBER_REGISTER_COMPONENT(Planning_Component)
}  // namespace planning
}  // namespace apollo
