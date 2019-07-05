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
#include "modules/control/control_component.h"

#include <string>
#include "cyber/cyber.h"
#include "modules/control/Uart.h"
#include "modules/control/proto/chassis.pb.h"
#include "modules/control/proto/control.pb.h"

namespace apollo {
namespace control {

using apollo::control::Chassis;
using apollo::control::Control_Command;
using apollo::cyber::Time;

bool ControlComponent::Init() {
  chassis_reader_ = node_->CreateReader<Chassis>(
      FLAGS_chassis_channel, [this](const std::shared_ptr<Chassis>& chassis) {
        chassis_.CopyFrom(*chassis);
      });
  control_writer_ = node_->CreateWriter<Control_Command>(FLAGS_control_channel);

  GenerateCommand();

  // async_action_ = cyber::Async(&ControlComponent::TestCommand, this);
  return true;
}

// write to channel
void ControlComponent::GenerateCommand() {
  auto cmd = std::make_shared<Control_Command>();
  double t = 0.0;
  while (true) {
    cmd->set_steer_angle(static_cast<float>(40 * sin(t)));
    cmd->set_throttle(static_cast<float>(12 * cos(t)));

    control_writer_->Write(cmd);

    t += 0.5;
    cyber::SleepFor(std::chrono::microseconds(50));
  }
}

void ControlComponent::TestCommand() {
  double t = 0.0;
  while (1) {
    float steer_angle = static_cast<float>(40 * sin(t));
    float steer_throttle = static_cast<float>(20 * cos(t));
    ADEBUG << "control message, times: " << t << " steer_angle:" << steer_angle
           << " steer_throttle:" << steer_throttle;

    // tell OnChassis() you can receive message now
    action_ready_ = true;
    char protoco_buf[10];
    std::memcpy(protoco_buf, &steer_angle, 4);
    std::memcpy(protoco_buf + 4, &steer_throttle, 4);
    protoco_buf[8] = 0x0d;
    protoco_buf[9] = 0x0a;
    arduino_.Write(protoco_buf, 10);

    t += 0.05;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

ControlComponent::~ControlComponent() {
  if (action_ready_.exchange(true)) {
    // close arduino
    // back chassis handle
    async_feedback_.wait();
  }
}

}  // namespace control
}  // namespace apollo
