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
#include "modules/control/chassis_component.h"

namespace apollo {
namespace control {

typedef struct _vehicle_info_s {
  float steerangle;
  float throttle;
  float speed_now;
} vehicle_info_s;

bool chassis_flag = false;

bool ChassisComponent::Init() {
  arduino_.SetOpt(9600, 8, 'N', 1);

  // Read and copy control message
  control_reader_ = node_->CreateReader<Control_Command>(
      FLAGS_control_channel,
      [this](const std::shared_ptr<Control_Command>& cmd) {
        ADEBUG << "Received Control message. run callback.";
        cmd_.Clear();
        cmd_.CopyFrom(*cmd);
        chassis_flag = true;
      });

  chassis_writer_ = node_->CreateWriter<Chassis>(FLAGS_chassis_channel);

  // control message action
  async_action_ = cyber::Async(&ChassisComponent::Action, this);

  // thread to publish chassis message
  std::thread(&ChassisComponent::OnChassis, this).detach();
  return true;
}

/**
 * @brief
 *
 * @param cmd
 */
void ChassisComponent::Action() {
  while (!cyber::IsShutdown()) {
    if (!cmd_.has_steer_angle() || !cmd_.has_throttle()) {
      AINFO << "control message is not ready";
      // cyber::SleepFor(std::chrono::milliseconds(message_wait_));
      continue;
    } else {
      ADEBUG << "Message Origin: " << cmd_.DebugString();

      // tell OnChassis() you can receive message now
      action_ready_.exchange(true);

      float steer_angle = cmd_.steer_angle();
      float steer_throttle = cmd_.throttle();

      cmd_.Clear();
      ADEBUG << "control message, times: "
             << " steer_angle:" << steer_angle
             << " steer_throttle:" << steer_throttle;
      char protoco_buf[10];
      std::memcpy(protoco_buf, &steer_angle, 4);
      std::memcpy(protoco_buf + 4, &steer_throttle, 4);
      protoco_buf[8] = 0x0d;
      protoco_buf[9] = 0x0a;
      int result = arduino_.Write(protoco_buf, 10);
      ADEBUG << "Arduino action result is :" << result;
      chassis_flag = false;
    }
  }
}

void ChassisComponent::OnChassis() {
  int count = 0;
  static char buffer[100];
  static char buf;
  vehicle_info_s vehicle_info;
  while (!cyber::IsShutdown()) {
    count = 0;
    std::memset(buffer, 0, 100);
    std::memset(&vehicle_info, 0, sizeof(vehicle_info));
    while (1) {
      int ret = arduino_.Read(&buf, 1);
      if (ret == 1) {
        // ADEBUG << "Arduino return state:" << ret;
        if (buf == 0x0A) {
          break;
        }
        buffer[count] = buf;
        count++;
      }
    }
    if (count == 12) {
      std::memcpy(&vehicle_info, buffer, 12);
      ADEBUG << "chassis feedback , steer_angle: " << vehicle_info.steerangle
             << " throttle:" << vehicle_info.throttle
             << " speed: " << vehicle_info.speed_now;
      auto proto_chassis = std::make_shared<Chassis>();
      proto_chassis->set_steer_angle(vehicle_info.steerangle);
      proto_chassis->set_throttle(vehicle_info.throttle);
      proto_chassis->set_speed(vehicle_info.speed_now / 5544 *
                               (float)FLAGS_speed_feedback);
      chassis_writer_->Write(proto_chassis);
    }
  }
}

ChassisComponent::~ChassisComponent() {
  if (action_ready_.load()) {
    // close arduino
    // back chassis handle
    async_action_.wait();
    async_feedback_.wait();
  }
}

}  // namespace control
}  // namespace apollo
