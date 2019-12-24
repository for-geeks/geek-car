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
#include <thread>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "cyber/base/concurrent_object_pool.h"
#include "cyber/node/node.h"
#include "cyber/node/writer.h"

#include "modules/sensors/proto/sensor_image.pb.h"
#include "modules/sensors/proto/sensors.pb.h"
#include "modules/sensors/realsense/device_base.h"

namespace apollo {
namespace sensors {
namespace realsense {

using apollo::cyber::Node;
using apollo::cyber::Time;
using apollo::cyber::Writer;
using apollo::cyber::base::CCObjectPool;
using apollo::sensors::Acc;
using apollo::sensors::CompressedImage;
using apollo::sensors::Gyro;
using apollo::sensors::Image;
using apollo::sensors::PointCloud;
using apollo::sensors::realsense::DeviceBase;

class D435I : public DeviceBase {
 public:
  D435I(){};
  ~D435I();

  bool Init(std::shared_ptr<Node> node_) override;
  void DeviceConfig() override;
  void InitChannelWriter(std::shared_ptr<Node> node_) override;

 private:
  void Run();
  void OnColorImage(const rs2::frame &f);
  void OnDepthImage(const rs2::frame &f);
  void OnPointCloud(rs2::frame depth_frame);
  void PublishPointCloud();
  std::shared_ptr<Writer<Image>> color_image_writer_ = nullptr;
  std::shared_ptr<Writer<Image>> depth_image_writer_ = nullptr;
  std::shared_ptr<Writer<PointCloud>> point_cloud_writer_ = nullptr;

  std::shared_ptr<CCObjectPool<PointCloud>> point_cloud_pool_ = nullptr;

  // filtered point cloud frame
  rs2::frame_queue filtered_data_;

  const int pool_size_ = 8;
  const int point_size_ = 10000;

  std::thread realsense_t1;
  std::thread realsense_t2;

  std::atomic<bool> stop_ = {false};
};
}  // namespace realsense
}  // namespace sensors
}  // namespace apollo
