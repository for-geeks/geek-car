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
#include <fstream>
#include "cyber/cyber.h"
#include "modules/common/global_gflags.h"
#include "modules/sensors/proto/sensors.pb.h"
#define MAX_LEN 1000000
using apollo::sensors::Pose;

struct Point {
  double x, z, y;
};
char buffer[MAX_LEN];
std::ofstream file("/home/raosiyue/pose.dat", std::ios::out | std::ios::binary);
std::vector<Point> container;

void MessageCallback(const std::shared_ptr<Pose>& msg) {
  static int time_counter = 0;
  static int counter = 0;
  if (time_counter % 1 == 0) {
    Point p{msg->translation().x(), msg->translation().z(),
            msg->rotation().y()};
    memcpy(buffer + counter * 24, &p, 24);
    counter++;
    container.push_back(p);
  }
  time_counter++;
}

int main(int argc, char* argv[]) {
  // init cyber framework
  apollo::cyber::Init(argv[0]);
  // create listener node
  auto listener_node = apollo::cyber::CreateNode("pose listener");
  // create listener
  auto listener =
      listener_node->CreateReader<Pose>(FLAGS_pose_channel, MessageCallback);
  apollo::cyber::WaitForShutdown();
  if (apollo::cyber::IsShutdown()) {
    std::cout << sizeof(container) << std::endl;
    file.write(buffer, sizeof(Point) * container.size());
    file.close();
  }
  return 0;
}
