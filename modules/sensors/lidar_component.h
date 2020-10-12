/******************************************************************************
 * MIT License

 * Copyright (c) 2020 Geekstyle

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

#include <map>
#include <memory>
#include <string>

#include "CYdLidar.h"
#include "core/common/ydlidar_datatype.h"

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "cyber/node/node.h"
#include "cyber/time/rate.h"

#include "modules/common/global_gflags.h"
#include "modules/sensors/proto/laserscan.pb.h"
#include "modules/sensors/proto/ydlidar_config.pb.h"

namespace apollo {
namespace sensors {

using apollo::cyber::Component;
using apollo::cyber::Rate;
using apollo::cyber::Writer;
using apollo::sensors::Scan;
using apollo::sensors::YDlidarDeviceConf;

std::vector<float> split(const std::string &s, char delim) {
  std::vector<float> elems;
  std::stringstream ss(s);
  std::string number;
  while (std::getline(ss, number, delim)) {
    elems.push_back(static_cast<float>(std::atof(number.c_str())));
  }
  return elems;
}

class LidarComponent : public Component<> {
 public:
  bool Init() override;
  void InitSensor();
  void Run();
  ~LidarComponent();

 private:
  // lidar device
  CYdLidar laser;

  YDlidarDeviceConf device_conf_;

  std::shared_ptr<Writer<Scan>> scan_writer = nullptr;
};

CYBER_REGISTER_COMPONENT(LidarComponent)
}  // namespace sensors
}  // namespace apollo
