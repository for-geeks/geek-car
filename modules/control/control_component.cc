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

typedef union FLOAT_CONV {
  float f;
  char c[4];
} float_conv;

float BLEndianFloat(float value) {
  float_conv d1, d2;
  d1.f = value;
  d2.c[0] = d1.c[3];
  d2.c[1] = d1.c[2];
  d2.c[2] = d1.c[1];
  d2.c[3] = d1.c[0];
  return d2.f;
}

bool ControlComponent::Init() {
  arduino_.SetOpt(9600, 8, 'N', 1);  //, int bits, char event, int stop);

  chassis_writer_ = node_->CreateWriter<Chassis>("/chassis");
  control_writer_ = node_->CreateWriter<Control_Command>("/control");

  TestCommand();
  // async method wait for control message
  // async_action_ = cyber::Async(&ControlComponent::Action, this);

  // chassis feedback
  async_feedback_ = cyber::Async(&ControlComponent::OnChassis, this);
  return true;
}

/**
 * @brief read arduino and write to chassis channel
 *
 */
void ControlComponent::OnChassis() {
  while (!cyber::IsShutdown() && action_ready_.load()) {
    static char buffer[100];
    memset(buffer, 0, 100);
    int ret = arduino_.Read(buffer, 100);
    if (ret != -1) {
      std::string s = buffer;
      AINFO << "chassis feedback : " << s;
      // std::cout << "Arduino says: " << std::endl << s << std::endl;

      //   auto proto_chassis = make_shared<Chassis>();
      //   proto_chassis->set_steer_angle();
      //   proto_chassis->set_throttle();
      //   proto_chassis->set_speed();
      //   chassis_writer_->Writer(proto_chassis);
    }
  }
}

void ControlComponent::GenerateCommand() {
  // write to channel
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
    arduino_.Write(protoco_buf, 10);
  }
}

void ControlComponent::TestCommand() {
  double t = 0.0;
  while (1) {
    float steer_angle = static_cast<float>(40 * sin(t));
    float steer_throttle = static_cast<float>(20 * cos(t));
    ADEBUG << "control message, times: " << t << " steer_angle:" << steer_angle
           << " steer_throttle:" << steer_throttle;
    char protoco_buf[10];
    // float steer = BLEndianFloat(steer_angle);
    // float throttle = BLEndianFloat(steer_throttle);
    memcpy(protoco_buf, &steer_angle, 4);
    memcpy(protoco_buf + 4, &steer_throttle, 4);
    protoco_buf[8] = 0x0d;
    protoco_buf[9] = 0x0a;
    arduino_.Write(protoco_buf, 10);

    t += 0.05;
    // std::this_thread::sleep_for(std::chrono::milliseconds(50000));
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
