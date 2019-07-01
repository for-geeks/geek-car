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

typedef struct _vehicle_info_s {
  float steerangle;
  float throttle;
  float speed_now;
} vehicle_info_s;

bool ControlComponent::Init() {
  arduino_.SetOpt(9600, 8, 'N', 1);  //, int bits, char event, int stop);

  chassis_writer_ = node_->CreateWriter<Chassis>("/chassis");
  control_writer_ = node_->CreateWriter<Control_Command>("/control");

  GenerateCommand();

  // async_action_ = cyber::Async(&ControlComponent::TestCommand, this);

  // chassis feedback
  async_feedback_ = cyber::Async(&ControlComponent::OnChassis, this);
  return true;
}

/**
 * @brief read arduino and write to chassis channel
 *
 */
void ControlComponent::OnChassis() {
  int count = 0;
  static char buffer[100];
  static char buf;
  vehicle_info_s vehicle_info;
  while (!cyber::IsShutdown() && action_ready_.load()) {
    count = 0;
    memset(buffer, 0, 100);
    memset(&vehicle_info, 0, sizeof(vehicle_info));
    static char buffer[100];
    memset(buffer, 0, 100);
    while (1) {
      int ret = arduino_.Read(&buf, 1);
      if (ret == 1) {
        ADEBUG << "Arduino return state:" << ret;
        if (buf == 0x0A) {
          break;
        }
        buffer[count] = buf;
        count++;
      }
    }
    if (count == 12) {
      memcpy(&vehicle_info, buffer, 12);
      ADEBUG << "chassis feedback , steer_angle: " << vehicle_info.steerangle
             << " throttle:" << vehicle_info.throttle
             << " speed: " << vehicle_info.speed_now;
      auto proto_chassis = std::make_shared<Chassis>();
      proto_chassis->set_steer_angle(vehicle_info.steerangle);
      proto_chassis->set_throttle(vehicle_info.throttle);
      proto_chassis->set_speed(vehicle_info.speed_now);
      chassis_writer_->Write(proto_chassis);
    }
  }
}

// write to channel
void ControlComponent::GenerateCommand() {
  auto cmd = std::make_shared<Control_Command>();
  double t = 0.0;
  while (true) {
    cmd->set_steer_angle(static_cast<float>(40 * sin(t)));
    cmd->set_throttle(static_cast<float>(12 * cos(t)));

    // atomic operation, when action failed
    control_writer_->Write(cmd);
    // yield cmd;
    // send and action
    // Action(cmd);

    t += 0.5;
    cyber::SleepFor(std::chrono::microseconds(50));
  }
}

/**
 * @brief action method for control command
 * TODO(fengzongbao) async cmd
 * @param cmd
 */
void ControlComponent::Action(const Control_Command& cmd) {
  while (!cyber::IsShutdown()) {
    if (!cmd.has_steer_angle() || !cmd.has_throttle()) {
      continue;
      // cyber::SleepFor(std::chrono::milliseconds());
    }

    // tell OnChassis() you can receive message now
    action_ready_ = true;

    float steer_angle = cmd.steer_angle();
    float steer_throttle = cmd.throttle();
    ADEBUG << "control message, times: "
           << " steer_angle:" << steer_angle
           << " steer_throttle:" << steer_throttle;
    char protoco_buf[10];
    memcpy(protoco_buf, &steer_angle, 4);
    memcpy(protoco_buf + 4, &steer_throttle, 4);
    protoco_buf[8] = 0x0d;
    protoco_buf[9] = 0x0a;
    int result = arduino_.Write(protoco_buf, 10);
    ADEBUG << "Arduino action result is :" << result;
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
    memcpy(protoco_buf, &steer_angle, 4);
    memcpy(protoco_buf + 4, &steer_throttle, 4);
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
