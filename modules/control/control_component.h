#pragma once

#include <memory>

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "modules/control/Uart.h"
#include "modules/control/control_gflags.h"
#include "modules/control/proto/chassis.pb.h"
#include "modules/control/proto/control.pb.h"

namespace apollo {
namespace control {

using apollo::control::Chassis;
using apollo::control::Control_Command;
using apollo::cyber::Component;
using apollo::cyber::Writer;

class ControlComponent : public Component<> {
 public:
  bool Init() override;

  void OnChassis();
  void GenerateCommand();
  void Action(const Control_Command& cmd);
  void TestCommand();
  ~ControlComponent();

 private:
  Uart arduino_ = Uart(FLAGS_device_name);

  std::shared_ptr<Writer<Chassis>> chassis_writer_ = nullptr;
  std::shared_ptr<Writer<Control_Command>> control_writer_ = nullptr;

  std::future<void> async_action_;
  std::future<void> async_feedback_;

  // atomic flag for action
  std::atomic<bool> action_ready_ = {false};
};

CYBER_REGISTER_COMPONENT(ControlComponent)
}  // namespace control
}  // namespace apollo
