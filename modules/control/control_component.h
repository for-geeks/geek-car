#pragma once

#include <memory>

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "modules/chassis/proto/chassis.pb.h"

namespace apollo {
namespace chassis {

using apollo::chassis::Chassis;
using apollo::cyber::Component;
using apollo::cyber::Writer;

class Control_Component : public Component<> {
 public:
  bool Init() override;

  Control_Component();
  ~Control_Component();

  void Chassis();
  void GenerateCommand();
  void TestCommand(Control_Command cmd);

 private:
  std::shared_ptr<Writer<Chassis>> chassis_writer_ = nullptr;
  std::shared_ptr<Writer<Control_Command>> control_writer_ = nullptr;

  Uart arduino("ttyACM0");

  std::future<void> chassis_feedback_;
}

CYBER_REGISTER_COMPONENT(Control_Component)
}  // namespace chassis
}  // namespace apollo
