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
#include "cyber/time/rate.h"
#include "math.h"
#include "modules/control/proto/chassis.pb.h"
#include "modules/control/proto/control.pb.h"
#include "modules/sensors/proto/sensors.pb.h"

namespace apollo {
namespace control {

using apollo::control::Chassis;
using apollo::control::Control_Command;
using apollo::control::Control_Reference;
using apollo::cyber::Rate;
using apollo::cyber::Time;
using apollo::sensors::Pose;

bool ControlComponent::Init() {
  // Reader
  chassis_reader_ = node_->CreateReader<Chassis>(
      FLAGS_chassis_channel, [this](const std::shared_ptr<Chassis>& chassis) {
        chassis_.CopyFrom(*chassis);
      });
  // TODO(zongbao) need to depend on the sensors
  pose_reader_ = node_->CreateReader<Pose>(
      FLAGS_pose_channel,
      [this](const std::shared_ptr<Pose>& Pose) { pose_.CopyFrom(*Pose); });
  control_refs_reader_ = node_->CreateReader<Control_Reference>(
      FLAGS_control_ref_channel,
      [this](const std::shared_ptr<Control_Reference>& refs) {
        refs_.Clear();
        refs_.CopyFrom(*refs);
      });

  // create Writer
  control_writer_ = node_->CreateWriter<Control_Command>(FLAGS_control_channel);

  // compute control message in aysnc
  async_action_ = cyber::Async(&ControlComponent::GenerateCommand, this);
  // async_action_.get();

  return true;
}

// write to channel
void ControlComponent::GenerateCommand() {
  auto cmd = std::make_shared<Control_Command>();
  double t = 0.0;
  Rate rate(20.0);
  float error_sum = 0;
  float error_yawrate_sum = 0;
  while (true) {
    /*PID core*/
    float speed_ref = refs_.vehicle_speed();
    float angular_speed_ref = refs_.angular_speed();
    float speed_now = chassis_.speed();
    float error = speed_ref - speed_now;
    error_sum += static_cast<float>(error * 0.05);
    cmd->set_throttle(static_cast<float>(
        speed_ref * FLAGS_longitude_ff + error * FLAGS_longitude_kp +
        error_sum * FLAGS_longitude_ki + FLAGS_offset));
    float error_yawrate =
        angular_speed_ref - static_cast<float>(pose_.angular_velocity().y());
    error_yawrate_sum += static_cast<float>(error_yawrate * 0.05);
    cmd->set_steer_angle(static_cast<float>(0));

    control_writer_->Write(cmd);

    t += 0.05;
    rate.Sleep();
  }
}

ControlComponent::~ControlComponent() {
  // back chassis handle
  async_action_.wait();
}

}  // namespace control
}  // namespace apollo
