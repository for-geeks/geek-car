#pragma once

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"

#include "modules/sensors/proto/nooploop.pb.h"

namespace apollo {
namespace sensors {

using apollo::cyber::Component;
using apollo::sensors::proto::TagFrame;

class NooploopComponent : public Component<> {
 public:
  NooploopComponennt() = default;
  ~NooploopComponent() = default;

  bool Init() override;

 private:
};
CYBER_REGISTER_COMPONENT(NooploopComponent)
}  // namespace sensors
}  // namespace apollo
