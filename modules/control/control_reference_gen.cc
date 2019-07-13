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

using apollo::control::Control_Reference;
using apollo::cyber::Rate;
using apollo::cyber::Time;
using apollo::cyber::common::GetAbsolutePath;
using apollo::sensors::Pose;

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

  for (const auto& v : Points) {
    std::cout << v.x << ", " << v.z << " , " << v.y << std::endl;
  }
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
  auto point_num = static_cast<int>(Points.size());
  for (auto i = index, j = index + 1;j < point_num; i++) {
    signed int kSelectedPointNum = 20;
    auto left = point_num - index;
    auto selected_size = static_cast<int>(selected.size());
    if (selected_size >= kSelectedPointNum ||
        left > kSelectedPointNum) {
      break;
    }
    // point distance diff
    double kMaxDiff = 0.05;
    if (((Points[j].x - Points[i].x) + (Points[j].z - Points[i].z)) >= kMaxDiff) {
      selected.push_back(Points[i]);
    }
  }

  auto selectedPointSize = selected.size();
  auto lastPoint = selected[selectedPointSize - 1];
  // TODO distance
  double total_length = lastPoint.x - selected[0].x;
  if (total_length < 0.2) {
    return std::vector<Point>();
  }

  return selected;
}

double fitting(std::vector<Point>& selected) {
  auto selectedPointSize = selected.size();
  //double target_line = selected[selectedPointSize - 1] - selected[0];

  double coefficient[5];
  std::memset(coefficient, 0, sizeof(double) * 5);
  std::vector<double> vx, vy;
  for (unsigned int i = 0; i < selectedPointSize; i++) {
    vx.push_back(selected[i].x);
    vy.push_back(selected[i].z);
  }
  apollo::common::EMatrix(vx, vy, static_cast<int>(selectedPointSize), 3, coefficient);
  ADEBUG << "拟合方程为：y = " << coefficient[1] << " + " << coefficient[2]
         << "x + " << coefficient[3] << "x^2";

  return *coefficient;
}

int main(int argc, char* argv[]) {
  Pose pose_;
  trajectory_reader();
  apollo::cyber::Init("control ref gen");
  // create talker node
  auto node_ = apollo::cyber::CreateNode("control ref gen");
  // create talker
  auto control_refs_writer_ =
      node_->CreateWriter<Control_Reference>(FLAGS_control_ref_channel);
  auto pose_reader_ = node_->CreateReader<Pose>(
      FLAGS_pose_channel,
      [&](const std::shared_ptr<Pose>& pose) { pose_.CopyFrom(*pose); });
  Rate rate(20.0);
  double t = 0;
  auto cmd = std::make_shared<Control_Reference>();
  while (apollo::cyber::OK()) {
    int index = near_pt_find(pose_.translation().x(), pose_.translation().z());
    auto selected = points_select(index);
    // auto coefficient = fitting(selected);
    //double c = coefficient[1];
    //double theta = atan(coefficient[2]);
    
    cmd->set_angular_speed(static_cast<float>(sin(t / 2.0)));
    cmd->set_vehicle_speed(static_cast<float>(0.4));
    t += 0.05;
    control_refs_writer_->Write(cmd);
    rate.Sleep();
  }
  return 0;
}
