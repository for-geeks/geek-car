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
#pragma once

#include <memory>

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "cyber/cyber.h"
#include "cyber/node/node.h"
#include "cyber/node/writer.h"

#include "modules/common/global_gflags.h"
#include "modules/common/uart.h"
#include "modules/sensors/nooploop/nlink_linktrack_tag_frame0.h"
#include "modules/sensors/proto/nooploop.pb.h"
#include "modules/sensors/proto/sensors.pb.h"

namespace apollo {
namespace sensors {

using apollo::cyber::Component;
using apollo::cyber::Writer;
using apollo::sensors::Acc;
using apollo::sensors::DistanceAnchor2Tag;
using apollo::sensors::Gyro;
using apollo::sensors::TagFrame;

class NooploopComponent : public Component<> {
 public:
  NooploopComponent() = default;
  ~NooploopComponent();

  bool Init() override;

  void Run();
  void OnFrame();
  void OnAcc(float acc[3]);
  void OnGyro(float gyro[3]);

 private:
  Uart device_ = Uart(FLAGS_nooploop_device_name.c_str());

  std::shared_ptr<Writer<Acc>> acc_writer_ = nullptr;
  std::shared_ptr<Writer<Gyro>> gyro_writer_ = nullptr;

  std::shared_ptr<Writer<TagFrame>> tagframe_writer_ = nullptr;

  std::atomic<bool> stop_ = {false};
};
CYBER_REGISTER_COMPONENT(NooploopComponent)
}  // namespace sensors
}  // namespace apollo
