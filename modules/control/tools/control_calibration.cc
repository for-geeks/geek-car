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
#include "modules/control/tools/control_calibration.h"
#include <string>

#include "cyber/cyber.h"
#include "modules/control/proto/chassis.pb.h"
#include "modules/control/proto/control.pb.h"
#include "modules/sensors/proto/sensors.pb.h"

namespace apollo {
namespace control {

using apollo::control::Chassis;
using apollo::control::Control_Command;
using apollo::sensors::Pose;

bool CalibrationComponent::Init() { return true; }

bool CalibrationComponent::Proc(const std::shared_ptr<Pose>& pose,
                                const std::shared_ptr<Chassis>& chassis,
                                const std::shared_ptr<Control_Command>& cmd) {
  ADEBUG << "Pose msg: " << pose->DebugString();
  ADEBUG << "Chassis msg: " << chassis->DebugString();

#if 0
  auto chassis_speed = chassis->speed();
  auto sensor_speed = pose->angular_velocity();
  auto control_speed = cmd->throttle();

  ADEBUG << control_speed << chassis_speed << sensor_speed
         << chssis_speed / sensor_speed;

  std::string calibration_name =
      "/home/raosiyue/apollo_lite/data/calibration.csv";

#endif
  return true;
}
}  // namespace control
}  // namespace apollo
