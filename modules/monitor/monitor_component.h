

#include "cyber/component/component.h"

namespace apollo {
namespace monitor {

// Tell other device, we are ok or not
// (devices, include Arduino, Realsense)
class MonitorComponent : public Component<> {
public:
  void Init() override;
  void Action();
}
} // namespace monitor
} // namespace apollo