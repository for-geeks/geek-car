#pragma once

#include <memory>

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "cyber/cyber.h"
#include "cyber/node/node.h"
#include "cyber/node/writer.h"

#include "modules/common/Uart.h"
#include "modules/sensors/nooploop/nlink_linktrack_tag_frame0.h"
#include "modules/sensors/proto/nooploop.pb.h"
#include "modules/sensors/proto/sensors.pb.h"

namespace apollo {
namespace sensors {

using apollo::cyber::Component;
using apollo::cyber::Writer;
using apollo::sensors::Acc;
using apollo::sensors::Gyro;
using apollo::sensors::TagFrame;

class NooploopComponent : public Component<> {
 public:
  NooploopComponent() = default;
  ~NooploopComponent();

  bool Init() override;

  void Run();

  void OnAcc(float acc[3]);
  void OnGyro(float gyro[3]);

 private:
  Uart device_ = Uart("ttyUSB0");

  std::shared_ptr<Writer<Acc>> acc_writer_ = nullptr;
  std::shared_ptr<Writer<Gyro>> gyro_writer_ = nullptr;

  std::shared_ptr<Writer<TagFrame>> tagframe_writer_ = nullptr;

  std::atomic<bool> stop_ = {false};
};
CYBER_REGISTER_COMPONENT(NooploopComponent)
}  // namespace sensors
}  // namespace apollo
