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
#include "modules/control/proto/chassis.pb.h"
#include "modules/control/proto/control.pb.h"
#include "modules/sensors/proto/sensors.pb.h"
namespace apollo {
namespace control {

using apollo::control::Chassis;
using apollo::control::Control_Command;
using apollo::cyber::Component;
using apollo::cyber::Writer;
using apollo::sensors::Pose;

class ControlComponent : public Component<> {
 public:
  bool Init() override;

  void OnChassis();
  void GenerateCommand();
  void Action(const Control_Command& cmd);
  void TestCommand();
  ~ControlComponent();

 private:
  std::shared_ptr<Reader<Chassis>> chassis_reader_ = nullptr;
  std::shared_ptr<Writer<Control_Command>> control_writer_ = nullptr;
  std::shared_ptr<Reader<Control_Reference>> control_refs_reader_ = nullptr;
  std::shared_ptr<Reader<Pose>> pose_reader_ = nullptr;
  std::future<void> async_action_;
  Control_Reference refs_;
  Chassis chassis_;
  Pose pose_;
};

CYBER_REGISTER_COMPONENT(ControlComponent)
}  // namespace control
}  // namespace apollo
