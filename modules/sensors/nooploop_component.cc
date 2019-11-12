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
#include "modules/sensors/nooploop_component.h"

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
    tagframe_writer_ = node_->CreateWriter<TagFrame>(FLAGS_uwb_pose_channel);
  }

  cyber::Async(&NooploopComponent::Run, this);
  return true;
}

void NooploopComponent::Run() {
  uint8_t buffer[512];
  while (!stop_) {
    std::memset(buffer, 0, 512);
    char id = 0;
    char func_mark = 0;

    device_.Read(&id, 1);

    if (id == 0x55){
      device_.Read(&func_mark, 1);
      if(func_mark == 0x01 ){
        buffer[0] = 0x55;
        buffer[1] = 0x01;
        for (int i = 0; i < 126; ++i)
        {
          device_.Read((char*)(buffer + i +2), 1);
        }

        auto veri = verifyTagFrame0Data(buffer);
        if(veri){
          unpackTagFrame0Data(buffer);
          // tagFrame0Data_;
          // TagFrame0Data data = tagFrame0Data_;
          auto proto_tag = std::make_shared<TagFrame>();
          proto_tag->set_tag_id(tagFrame0Data_.frame.id);
          proto_tag->set_network_system_time(tagFrame0Data_.frame.systemTime);
          proto_tag->mutable_pos()->set_x(tagFrame0Data_.pos[0]);
          proto_tag->mutable_pos()->set_y(tagFrame0Data_.pos[1]);
          proto_tag->mutable_pos()->set_z(tagFrame0Data_.pos[2]);

          proto_tag->mutable_eop()->set_x(tagFrame0Data_.eop[0]);
          proto_tag->mutable_eop()->set_y(tagFrame0Data_.eop[1]);
          proto_tag->mutable_eop()->set_z(tagFrame0Data_.eop[2]);

          proto_tag->mutable_vel()->set_x(tagFrame0Data_.vel[0]);
          proto_tag->mutable_vel()->set_y(tagFrame0Data_.vel[1]);
          proto_tag->mutable_vel()->set_z(tagFrame0Data_.vel[2]);

          proto_tag->mutable_angle()->set_x(tagFrame0Data_.angle[0]);
          proto_tag->mutable_angle()->set_y(tagFrame0Data_.angle[1]);
          proto_tag->mutable_angle()->set_z(tagFrame0Data_.angle[2]);

          proto_tag->mutable_rotation()->set_qx(tagFrame0Data_.frame.q[0]);
          proto_tag->mutable_rotation()->set_qy(tagFrame0Data_.frame.q[1]);
          proto_tag->mutable_rotation()->set_qz(tagFrame0Data_.frame.q[2]);
          proto_tag->mutable_rotation()->set_qw(tagFrame0Data_.frame.q[3]);

          proto_tag->set_supply_voltage(tagFrame0Data_.supplyVoltage);

          for (size_t i = 0; i < 8; i++) {
            DistanceAnchor2Tag da2t;
            da2t.set_distance(tagFrame0Data_.dis[i]);
            auto next_da2t = proto_tag->add_dis();
            next_da2t->CopyFrom(da2t);
          }

          tagframe_writer_->Write(proto_tag);

          OnAcc(tagFrame0Data_.frame.acc);
          OnGyro(tagFrame0Data_.frame.gyro);
        }

      }
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

NooploopComponent::~NooploopComponent() {
  AINFO << "Destructor from NooploopComponent";

  if (!stop_.load()) {
    stop_.exchange(true);
  }
}

}  // namespace sensors
}  // namespace apollo
