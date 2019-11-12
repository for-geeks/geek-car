
#include "modules/sensors/nooploop_component.h"

#include "modules/common/global_gflags.h"
#include "modules/sensors/nooploop/ncommon.h"

namespace apollo {
namespace sensors {


bool NooploopComponent::Init() {
  device_.SetOpt(921600, 8, 'N', 1);

  if (FLAGS_publish_acc) {
    acc_writer_ = node_->CreateWriter<Acc>(FLAGS_acc_channel);
  }

  if (FLAGS_publish_gyro) {
    gyro_writer_ = node_->CreateWriter<Gyro>(FLAGS_gyro_channel);
  }

  if (FLAGS_publish_tagframe) {
    tagframe_writer_ = node_->CreateWriter<TagFrame>(FLAGS_tagframe_channel);
  }

  cyber::Async(&NooploopComponent::Run, this);
  return true;
}

void NooploopComponent::Run() {
  uint8_t buffer[512];
  while (!stop_) {
    std::memset(buffer, 0, 512);
    device_.Read((char*)buffer, 128);
    if ((buffer[0] == 0x55) && (buffer[1] == 0x01)) {
      AINFO << "Get data from Nooploop";
      unpackTagFrame0Data(buffer);
      // tagFrame0Data_;
      TagFrame0Data data = tagFrame0Data_;

      auto proto_tag = std::make_shared<TagFrame>();
      proto_tag->set_tag_id(data.frame.id);
      proto_tag->set_network_system_time(data.frame.systemTime);
      proto_tag->mutable_pos()->set_x(data.pos[0]);
      proto_tag->mutable_pos()->set_y(data.pos[1]);
      proto_tag->mutable_pos()->set_z(data.pos[2]);

      proto_tag->mutable_eop()->set_x(data.eop[0]);
      proto_tag->mutable_eop()->set_y(data.eop[1]);
      proto_tag->mutable_eop()->set_z(data.eop[2]);

      proto_tag->mutable_vel()->set_x(data.vel[0]);
      proto_tag->mutable_vel()->set_y(data.vel[1]);
      proto_tag->mutable_vel()->set_z(data.vel[2]);

      proto_tag->mutable_angle()->set_x(data.angle[0]);
      proto_tag->mutable_angle()->set_y(data.angle[1]);
      proto_tag->mutable_angle()->set_z(data.angle[2]);

      proto_tag->mutable_rotation()->set_qx(data.frame.q[0]);
      proto_tag->mutable_rotation()->set_qy(data.frame.q[1]);
      proto_tag->mutable_rotation()->set_qz(data.frame.q[2]);
      proto_tag->mutable_rotation()->set_qw(data.frame.q[3]);

      proto_tag->set_supply_voltage(data.supplyVoltage);

      for (size_t i = 0; i < 8; i++)
      {
        if(data.dis[i] != 1) {
          proto_tag->add_distance(data.dis[i]);
        }
      }
      tagframe_writer_->Write(proto_tag);

      OnAcc(data.frame.acc);
      OnGyro(data.frame.gyro);
    }
  }
}

void NooploopComponent::OnAcc(float acc[3]) {
  auto proto_accel = std::make_shared<Acc>();
  proto_accel->mutable_acc()->set_x(acc[0]);
  proto_accel->mutable_acc()->set_y(acc[1]);
  proto_accel->mutable_acc()->set_z(acc[2]);

  acc_writer_->Write(proto_accel);
}

void NooploopComponent::OnGyro(float gyro[3]) {
  auto proto_gyro = std::make_shared<Gyro>();
  proto_gyro->mutable_gyro()->set_x(gyro[0]);
  proto_gyro->mutable_gyro()->set_y(gyro[1]);
  proto_gyro->mutable_gyro()->set_z(gyro[2]);

  gyro_writer_->Write(proto_gyro);
}

NooploopComponent::~NooploopComponent(){
  AINFO << "Destructor from NooploopComponent";

  if (!stop_.load()) {
    stop_.exchange(true);
  }
}

}  // namespace sensors
}  // namespace apollo
