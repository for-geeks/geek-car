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
#include <malloc.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <cstring>
#include <fstream>
#include <iostream>
#include <iterator>
#include <memory>
#include <string>
#include <vector>

#include "cyber/cyber.h"
#include "cyber/time/rate.h"
#include "modules/common/curve_fitting.h"
#include "modules/common/global_gflags.h"
#include "modules/control/proto/control.pb.h"
#include "modules/sensors/proto/sensors.pb.h"

using apollo::control::Coefficient;
using apollo::control::Control_Reference;
using apollo::cyber::Rate;
using apollo::cyber::Time;
using apollo::cyber::common::GetAbsolutePath;
using apollo::sensors::Pose;
#if 0
struct Point {
  double x, z, y;
};

std::vector<Point> Points;

int trajectory_reader() {
  char buffer[100000];

  // std::string work_root = GetCyberWorkRoot();
  // std::string config_file =
  //     GetAbsolutePath(options.root_dir, options.conf_file);
  // config_file = GetAbsolutePath(work_root, config_file);
  std::string file_name = "/home/raosiyue/pose.dat";

  std::ifstream input_file(file_name, std::ios::in | std::ios::binary);
  input_file.seekg(0, std::ios_base::end);
  auto size_v = input_file.tellg();
  input_file.seekg(0, std::ios_base::beg);
  Point temp_vec;

  // buffer = (char*)malloc(size_v);
  std::cout << "length is " << size_v << std::endl;
  input_file.read(buffer, size_v);
  input_file.close();
  for (unsigned int i = 0; i < size_v; i += 24) {
    double test_double = 0;
    std::memcpy(&test_double, buffer, 8);
    if ((*buffer + i) > 100000) {
      break;
    }
    std::memcpy(&temp_vec, buffer + i, 24);
    Points.push_back(temp_vec);
  }

  ADEBUG << "Read Points size from Pose:" << Points.size();

  // for (const auto& v : Points) {
  //   std::cout << v.x << ", " << v.z << " , " << v.y << std::endl;
  // }
  return 0;
}

int near_pt_find(double x, double y) {
  int ret = -1;
  double min_dist = 100;

  for (unsigned int i = 0; i < Points.size(); i++) {
    double dist = std::fabs(Points.at(i).x - x) + std::fabs(Points.at(i).z - y);
    if ((dist < min_dist) && (dist < 1.5)) {
      min_dist = dist;
      ret = i;
    }
  }
  return ret;
}

std::vector<Point> points_select(int index) {
  // 1、差分去掉变化为0的值；2、距离大于20cm 3、20个点；
  std::vector<Point> selected;
  auto pointNum = static_cast<int>(Points.size());
  for (auto i = index, j = index + 1; j < pointNum; i++) {
    signed int kSelectedPointNum = 20;
    auto left = pointNum - index;
    auto selectedSize = static_cast<int>(selected.size());
    // TODO(fengzongbao) add condition points size less 20;
    if (selectedSize >= 20 || pointNum - index < 20) {
      break;
    }
    // point distance diff
    double kMaxDiff = 0.05;
    if (distance2Points(Points[i], [Points[j]) >= kMaxDiff) {
      selected.push_back(Points[i]);
    }
  }

  auto selectedPointSize = selected.size();
  auto selectedEndPoint = selected[selectedPointSize - 1];
  double selectedTotalLength = distance2Points(selectedEndPoint, selected[0]);
  if (selectedTotalLength < 0.1) {
    ADEBUG << "Close to destination";
    return std::vector<Point>();
  }

  return selected;
}
void fitting(std::vector<Point>& selected, double coefficient[]) {
  if (selected.empty()) {
    ADEBUG << "selected points is empty.";
    return;
  }
  auto selectedPointSize = selected.size();

  // double coefficient[5];
  std::memset(coefficient, 0, sizeof(double) * 5);
  std::vector<double> vx, vy;
  for (unsigned int i = 0; i < selectedPointSize; i++) {
    vx.push_back(selected[i].x);
    vy.push_back(selected[i].z);
  }
  ADEBUG << "selected point vector size:" << selectedPointSize;
  for (auto i = 0; i < selectedPointSize; i++) {
    ADEBUG << " selected point index:" << i << " x:" << selected[i].x
           << " z:" << selected[i].z;
  }
  apollo::common::EMatrix(vx, vy, static_cast<int>(selectedPointSize), 3,
                          coefficient);
  printf("拟合方程为：y = %lf + %lfx + %lfx^2 \n", coefficient[1],
         coefficient[2], coefficient[3]);
}

// distance of two pose point
double distance2Points(Point a, Point b) {
  return sqrt(pow((a.x - b.x), 2) + pow((a.z - b.z), 2));
}

#endif
int main(int argc, char* argv[]) {
#if 0
  trajectory_reader();
  apollo::cyber::Init("control ref gen");
  auto node_ = apollo::cyber::CreateNode("control_ref_gen");
  auto control_refs_writer_ =
      node_->CreateWriter<Control_Reference>(FLAGS_control_ref_channel);
  auto control_coefficient_writer_ =
      node_->CreateWriter<Coefficient>(FLAGS_control_coefficient);

  Pose pose_;
  auto pose_reader_ = node_->CreateReader<Pose>(
      FLAGS_pose_channel,
      [&](const std::shared_ptr<Pose>& pose) { pose_.CopyFrom(*pose); });

  Rate rate(20.0);
  double t = 0;
  auto cmd = std::make_shared<Control_Reference>();
  auto coefficient = std::make_shared<Coefficient>();
  while (apollo::cyber::OK()) {
    // find nearest point and linear fitting
    int index = near_pt_find(pose_.translation().x(), pose_.translation().z());

    double path_remain =
        distance2Points(Points[Points.size() - 1], Point[index]);

    if (path_remain < 0.1) {
      cmd->set_angular_speed(0);
      cmd->set_vehicle_speed(0);
      ADEBUG << "Stop at recorded end point";
    } else {
      auto selected = points_select(index);
      double coefficient_[5];
      fitting(selected, coefficient_);

      coefficient->set_a(coefficient_[3]);
      coefficient->set_b(coefficient_[2]);
      coefficient->set_c(coefficient_[1]);

      control_coefficient_writer_->Write(coefficient);

      // todo update theta
      // double theta = coefficient_[2] * 0 + coefficient_[3] * 1;

      // keep cruise speed 0.5 before get end point
      double speed = FLAGS_cruise_speed;

      cmd->set_angular_speed(static_cast<float>(sin(t / 2.0)));
      cmd->set_vehicle_speed(static_cast<float>(speed));
    }

    t += 0.05;
    control_refs_writer_->Write(cmd);
    rate.Sleep();
  }
#endif
  return 0;
}
