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

#include "modules/sensors/lidar_component.h"

namespace apollo {
namespace sensors {

using namespace ydlidar;

bool LidarComponent::Init() {
  // Init by config device
  InitSensor();
  AINFO << "Init Lidar device successfuly";
  cyber::Async(&LidarComponent::Run, this);
  return true;
}

void LidarComponent::InitSensor() {
  double angle_max, angle_min;
  std::string list;
  std::vector<float> ignore_array;
  double frequency;

  std::shared_ptr<Writer<Scan>> scan_writer = node_->CreateWriter<Scan>("scan");

  if (!GetProtoConfig(&device_conf_)) {
    AERROR << "Unable to load conf file: " << ConfigFilePath();
    return;
  }

  ADEBUG << "Device conf:" << device_conf_.ShortDebugString();

  ignore_array = split(list, ',');
  if (ignore_array.size() % 2) {
    AERROR << "ignore array is odd need be even";
  }

  for (uint16_t i = 0; i < ignore_array.size(); i++) {
    if (ignore_array[i] < -180 && ignore_array[i] > 180) {
      AERROR << "ignore array should be between 0 and 360";
    }
  }

  if (device_conf_.frequency() < 3) {
    frequency = 7.0;
  }
  if (device_conf_.frequency() > 15.7) {
    frequency = 15.7;
  }
  angle_max = device_conf_.angle_max();
  angle_min = device_conf_.angle_min();
  if (angle_max < angle_min) {
    std::swap(angle_max, angle_min);
  }

  laser.setlidaropt(LidarPropSerialPort, device_conf_.port().c_str(),
                    sizeof(std::string));
  // laser.setlidaropt(LidarPropSerialBaudrate, device_conf_.baudrate(),
  //                   sizeof(int32_t));

  // laser.setMaxRange(device_conf_.range_max());
  // laser.setMinRange(device_conf_.range_min());
  // laser.setMaxAngle(device_conf_.angle_max());
  // laser.setMinAngle(device_conf_.angle_min());
  laser.setlidaropt(LidarPropMaxAngle, &angle_max, sizeof(float));
  laser.setlidaropt(LidarPropMinAngle, &angle_min, sizeof(float));
  // laser.setReversion(device_conf_.reversion());
  // laser.setFixedResolution(device_conf_.resolution_fixed());
  // laser.setAutoReconnect(device_conf_.auto_reconnect());
  laser.setlidaropt(LidarPropScanFrequency, &frequency, sizeof(float));

  // laser.setIgnoreArray(device_conf_.ignore_array());
  // laser.setSampleRate(device_conf_.samp_rate());
  // laser.setInverted(device_conf_.in_verted());
  // laser.setSingleChannel(device_conf_.isSingleChannel());
  // laser.setLidarType(device_conf_.isTOFLidar() ? TYPE_TOF : TYPE_TRIANGLE);

  bool ret = laser.initialize();
  if (ret) {
    ret = laser.turnOn();
    if (!ret) {
      AERROR << "Failed to start scan mode!!!";
    }
  } else {
    AERROR << "Error initializing YDLIDAR Comms and Status!!!";
  }
}

void LidarComponent::Run() {
  apollo::cyber::Rate rate(20.);

  while (!apollo::cyber::IsShutdown()) {
    LaserScan scan;
    if (laser.doProcessSimple(scan)) {
      apollo::sensors::Scan scan_msg;
      //   apollo::cyber::Time start_scan_time;
      //   start_scan_time.sec = scan.stamp / 1000000000ul;
      //   start_scan_time.nsec = scan.stamp % 1000000000ul;
      scan_msg.mutable_header()->set_timestamp_sec(
          apollo::cyber::Time::Now().ToSecond());
      scan_msg.mutable_header()->set_frame_id(device_conf_.frame_id());
      scan_msg.set_angle_min(scan.config.min_angle);
      scan_msg.set_angle_max(scan.config.max_angle);
      scan_msg.set_angle_increment(scan.config.angle_increment);
      scan_msg.set_scan_time(scan.config.scan_time);
      scan_msg.set_time_increment(scan.config.time_increment);
      scan_msg.set_range_min(scan.config.min_range);
      scan_msg.set_range_max(scan.config.max_range);
      int size =
          static_cast<int>((scan.config.max_angle - scan.config.min_angle) /
                               scan.config.angle_increment +
                           1);
      // scan_msg.ranges().resize(size);
      // scan_msg.intensities().resize(size);
      for (uint i = 0; i < scan.points.size(); i++) {
        int index = static_cast<int>(
            std::ceil((scan.points[i].angle - scan.config.min_angle) /
                      scan.config.angle_increment));
        if (index >= 0 && index < size) {
          scan_msg.set_ranges(index, scan.points[i].range);
          // no intensity for sensor X4 laser
          scan_msg.set_intensities(index, scan.points[i].intensity);
        }
      }
      scan_writer->Write(scan_msg);
    }
    rate.Sleep();
  }
}

LidarComponent::~LidarComponent() {
  laser.turnOff();
  AINFO << "[YDLIDAR INFO] Now YDLIDAR is stopping .......";
  laser.disconnecting();
}

}  // namespace sensors
}  // namespace apollo
