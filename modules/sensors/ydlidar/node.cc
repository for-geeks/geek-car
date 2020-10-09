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

/*
 *  YDLIDAR SYSTEM
 *  YDLIDAR ROS Node Client
 *
 *  Copyright 2015 - 2018 EAI TEAM
 *  http://www.ydlidar.com
 *
 */

#include <signal.h>
#include <iostream>
#include <string>
#include <vector>

#include "CYdLidar.h"

#include "cyber/common/log.h"
#include "cyber/cyber.h"

#include "modules/sensors/proto/laserscan.pb.h"
#include "modules/sensors/proto/ydlidar_config.pb.h"

namespace apollo {
namespace sensors {
namespace ydlidar {

using namespace ydlidar;

std::vector<float> split(const std::string &s, char delim) {
  std::vector<float> elems;
  std::stringstream ss(s);
  std::string number;
  while (std::getline(ss, number, delim)) {
    elems.push_back(atof(number.c_str()));
  }
  return elems;
}

int main(int argc, char *argv[]) {
  apollo::cyber::Init(argv[0]);

  bool reversion, resolution_fixed;
  double angle_max, angle_min;
  result_t op_result;
  std::string list;
  std::vector<float> ignore_array;
  double max_range, min_range;
  double frequency;

  std::shared_ptr<apollo::cyber::Node> node_ =
      apollo::cyber::CreateNode("ydlidar_node");
  std::shared_ptr<apollo::cyber::Writer<apollo::sensors::LaserScan>> scan_pub =
      node_->CreateWriter<apollo::sensors::LaserScan>("scan");

  apollo::sensors::YDlidarDeviceConf device_conf_;
  std::string config_path = "/apollo/modules/sensors/conf/ydlidar.pb.conf";
  if (!common::GetProtoFromFile(config_path, device_conf_)) {
    AERROR << "Unable to load conf file: " << config_path;
    return false;
  }

  ADEBUG << "Device conf:" << device_conf_.ShortDebugString();

  ignore_array = split(list, ',');
  if (ignore_array.size() % 2) {
        AERROR("ignore array is odd need be even";
  }

  for (uint16_t i = 0; i < ignore_array.size(); i++) {
    if (ignore_array[i] < -180 && ignore_array[i] > 180) {
      AERROR << "ignore array should be between 0 and 360";
    }
  }

  CYdLidar laser;
  if (frequency < 3) {
    frequency = 7.0;
  }
  if (frequency > 15.7) {
    frequency = 15.7;
  }
  if (angle_max < angle_min) {
    double temp = angle_max;
    angle_max = angle_min;
    angle_min = temp;
  }

  laser.setSerialPort(device_conf_.port);
  laser.setSerialBaudrate(device_conf_.baudrate);
  laser.setMaxRange(device_conf_.max_range);
  laser.setMinRange(device_conf_.min_range);
  laser.setMaxAngle(device_conf_.angle_max);
  laser.setMinAngle(device_conf_.angle_min);
  laser.setReversion(device_conf_.reversion);
  laser.setFixedResolution(device_conf_.resolution_fixed);
  laser.setAutoReconnect(device_conf_.auto_reconnect);
  laser.setScanFrequency(device_conf_.frequency);
  laser.setIgnoreArray(device_conf_.ignore_array);
  laser.setSampleRate(device_conf_.samp_rate);
  laser.setInverted(device_conf_.in_verted);
  laser.setSingleChannel(device_conf_.isSingleChannel);
  laser.setLidarType(device_conf_.isTOFLidar ? TYPE_TOF : TYPE_TRIANGLE);
  bool ret = laser.initialize();
  if (ret) {
    ret = laser.turnOn();
    if (!ret) {
      AERROR << "Failed to start scan mode!!!";
    }
  } else {
    AERROR << "Error initializing YDLIDAR Comms and Status!!!";
  }
  apollo::cyber::Rate rate(20);

  while (ret && !apollo::cyber::IsShutdown()) {
    bool hardError;
    LaserScan scan;
    if (laser.doProcessSimple(scan, hardError)) {
      apollo::sensors::LaserScan scan_msg;
      //   apollo::cyber::Time start_scan_time;
      //   start_scan_time.sec = scan.stamp / 1000000000ul;
      //   start_scan_time.nsec = scan.stamp % 1000000000ul;
      scan_msg.header.timestamp_sec = apollo::cyber::Time::Now().ToNanosecond();
      scan_msg.header.frame_id = frame_id;
      scan_msg.angle_min = (scan.config.min_angle);
      scan_msg.angle_max = (scan.config.max_angle);
      scan_msg.angle_increment = (scan.config.angle_increment);
      scan_msg.scan_time = scan.config.scan_time;
      scan_msg.time_increment = scan.config.time_increment;
      scan_msg.range_min = (scan.config.min_range);
      scan_msg.range_max = (scan.config.max_range);
      int size = (scan.config.max_angle - scan.config.min_angle) /
                     scan.config.angle_increment +
                 1;
      scan_msg.ranges.resize(size);
      scan_msg.intensities.resize(size);
      for (int i = 0; i < scan.points.size(); i++) {
        int index = std::ceil((scan.points[i].angle - scan.config.min_angle) /
                              scan.config.angle_increment);
        if (index >= 0 && index < size) {
          scan_msg.ranges[index] = scan.points[i].range;
          scan_msg.intensities[index] = scan.points[i].intensity;
        }
      }
      scan_pub.Write(scan_msg);
    }
    rate.sleep();
  }

  laser.turnOff();
  AINFO << "[YDLIDAR INFO] Now YDLIDAR is stopping .......";
  laser.disconnecting();
  return 0;
}

}  // namespace ydlidar
}  // namespace sensors
}  // namespace apollo
