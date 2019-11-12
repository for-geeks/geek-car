/******************************************************************************
 * MIT License

 * Copyright (c) 2019 Geekstyle

 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
******************************************************************************/
#pragma once
#include <memory>

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "modules/common/global_gflags.h"
#include "modules/common/Uart.h"
#include "modules/control/proto/chassis.pb.h"
#include "modules/control/proto/control.pb.h"

namespace apollo {
namespace control {

using apollo::control::Chassis;
using apollo::control::Control_Command;
using apollo::cyber::Component;
using apollo::cyber::Reader;
using apollo::cyber::Writer;

class ChassisComponent : public Component<> {
 public:
  bool Init() override;
  void Action();
  void OnChassis();
  ~ChassisComponent();

 private:
  Uart arduino_ = Uart(FLAGS_device_name.c_str());
  std::shared_ptr<Reader<Control_Command>> control_reader_ = nullptr;
  std::shared_ptr<Writer<Chassis>> chassis_writer_ = nullptr;

  Control_Command cmd_;
  uint32_t message_wait_ = 200;

  std::future<void> async_action_;
  std::future<void> async_feedback_;

  // atomic flag for action
  std::atomic<bool> action_ready_ = {false};
};

CYBER_REGISTER_COMPONENT(ChassisComponent)

}  // namespace control
}  // namespace apollo
