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
#include <fstream>
#include <iostream>
#include <memory>

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "cyber/cyber.h"
#include "modules/control/proto/chassis.pb.h"
#include "modules/control/proto/control.pb.h"
#include "modules/sensors/proto/sensors.pb.h"

namespace apollo {
namespace control {

using apollo::control::Chassis;
using apollo::control::Control_Command;
using apollo::cyber::Component;
using apollo::sensors::Pose;

/**
 * @brief record prefused data from pose chassis and control command
 *
 */
class CalibrationComponent : public Component<Pose, Chassis, Control_Command> {
 public:
  bool Init() override;
  bool Proc(const std::shared_ptr<Pose>& pose,
            const std::shared_ptr<Chassis>& chassis,
            const std::shared_ptr<Control_Command>& cmd) override;

 private:
  std::ofstream file;
  std::ofstream out_put;
};

CYBER_REGISTER_COMPONENT(CalibrationComponent)
}  // namespace control
}  // namespace apollo
