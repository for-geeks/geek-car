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

bool Control_Component::Init() {
  arduino_.reset(Uart("ttyACM0"));

  arduino_.SetOpt(9600, 8, 'N', 1);  //, int bits, char event, int stop);

  chassis_writer_ = node_->CreateWriter<Chassis>("/chassis");
  control_writer_ = node_->CreateWriter<Control_Command>("/control");

  // collect chassis feedback
  chassis_feedback_ = cyber::Async(&Control_Component::ChassisFeedback, this);
  // chassis_feedback_ = std::aysnc(std::launch::async,
  // &Control_Component::Chassis, this);
  return true;
}

/**
 * @brief read arduino and write to chassis channel
 *
 */
void Control_Component::ChassisFeedback() {
  while (!cyber::IsShutdown()) {
    static char buffer[100];
    memset(buffer, 0, 100);
    int ret = arduino_.Read(buffer, 100);
    if (ret != -1) {
      std::string s = buffer;
      std::cout << s;
      // std::cout << "Arduino says: " << std::endl << s << std::endl;

      //   auto proto_chassis = make_shared<Chassis>();
      //   proto_chassis->set_steer_angle();
      //   proto_chassis->set_throttle();
      //   proto_chassis->set_speed();
      //   chassis_writer_->Writer(proto_chassis);
    }
  }
}

void Control_Component::GenerateCommand() {
  // write to channel
}

void Control_Component::TestCommand() {
  float t = 0.0;
  auto cmd = Control_Command();
  while (1) {
    cmd.steer_angle = 40 * sin(t);
    cmd.throttle = 20 * cos(t);
    char protoco_buf[10];
    float steer = BLEndianFloat(cmd.steerangle);
    float throttle = BLEndianFloat(cmd.throttle);
    memcpy(protoco_buf, &cmd.steerangle, 4);
    memcpy(protoco_buf + 4, &cmd.throttle, 4);
    protoco_buf[8] = 0x0d;
    protoco_buf[9] = 0x0a;
    arduino_.Write(protoco_buf, 10);

    std::string log_control = protoco_buf;
    AINFO << protoco_buf;

    t += 0.05;
    std::this_thread::sleep_for(std::chrono::milliseconds(50000));
  }
}

}  // namespace control
}  // namespace apollo
