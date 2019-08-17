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
#include <cstdlib>
#include "cyber/cyber.h"
#include "modules/common/global_gflags.h"
#include "modules/sensors/proto/sensors.pb.h"

void PoseCallback(const std::shared_ptr<apollo::sensors::Pose> &pose) {
  if (std::to_string(pose->translation().x()) == "nan") {
    // restart realsense_component
    std::string cmd = "/apollo/scripts/realsense.sh restart";

    AWARN << "realsense T265 return nan, waitting for respawn";
    const int ret = std::system(cmd.c_str());
    if (ret == 0) {
      AINFO << "SUCCESS: " << cmd;
    } else {
      AERROR << "FAILED(" << ret << "): " << cmd;
    }
  }
}

int main() {
  apollo::cyber::Init("realsense_monitor");
  auto node = apollo::cyber::CreateNode("realsense_monitor");
  auto reader = node->CreateReader<apollo::sensors::Pose>(FLAGS_pose_channel,
                                                          PoseCallback);

  apollo::cyber::WaitForShutdown();

  return 0;
}
