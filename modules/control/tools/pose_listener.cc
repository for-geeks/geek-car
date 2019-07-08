/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include <fstream>
#include "cyber/cyber.h"
#include "modules/sensors/proto/sensors.pb.h"
#define MAX_LEN 1000000
using apollo::sensors::Pose;

struct Point {
  double x,z,y;
};
char buffer[MAX_LEN];
std::ofstream file("/home/raosiyue/pose.dat", std::ios::out | std::ios::binary);
std::vector<Point> container;

void MessageCallback(const std::shared_ptr<Pose>& msg) {
  static int time_counter = 0;
  static int counter = 0;
  if (time_counter % 1 == 0){
	Point p{msg->translation().x(), msg->translation().z(), msg->rotation().y()};
        memcpy(buffer + counter * 24, &p, 24);
	counter++;
	container.push_back(p);
  }
  //
  time_counter++;
}

int main(int argc, char* argv[]) {
  // init cyber framework
  apollo::cyber::Init(argv[0]);
  // create listener node
  auto listener_node = apollo::cyber::CreateNode("pose listener");
  // create listener
  auto listener =
      listener_node->CreateReader<Pose>("/realsense/pose", MessageCallback);
  apollo::cyber::WaitForShutdown();
  if(apollo::cyber::IsShutdown()){
    std::cout << sizeof(container) << std::endl; 
    file.write(buffer, sizeof(Point) * container.size());
    file.close();
  }
  return 0;
}
