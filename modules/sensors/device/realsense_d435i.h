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
#include "modules/sensors/device/device_base.h"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <memory>

#include "cyber/base/concurrent_object_pool.h"

namespace apollo {
namespace sensors {
namespace device {

using apollo::cyber::Time;
using apollo::cyber::base::CCObjectPool;
using apollo::sensors::Acc;
using apollo::sensors::Gyro;
using apollo::sensors::PointCloud;

class D435I : public DeviceBase {
 public:
  D435I();
  ~D435I();

  bool Init(std::shared_ptr<Node> node_);
  void DeviceConfig();
  void InitChannelWriter(std::shared_ptr<Node> node_);

  void Run();

  void OnColorImage(const rs2::frame &f);
  void OnCompressedImage(const rs2::frame &f, cv::Mat raw_image);
  void OnPointCloud(rs2::frame depth_frame);
  void PublishPointCloud();

 private:
  std::shared_ptr<Writer<Image>> color_image_writer_ = nullptr;
  std::shared_ptr<Writer<CompressedImage>> compressed_image_writer_ = nullptr;

  std::shared_ptr<Writer<PointCloud>> point_cloud_writer_ = nullptr;

  std::shared_ptr<Writer<Acc>> acc_writer_ = nullptr;
  std::shared_ptr<Writer<Gyro>> gyro_writer_ = nullptr;

  std::shared_ptr<CCObjectPool<PointCloud>> point_cloud_pool_ = nullptr;

  // Contruct a pipeline which abstracts the device
  rs2::pipeline pipe;

  // Configuring the pipeline with a non default profile
  rs2::config cfg;

  // filtered point cloud frame
  rs2::frame_queue filtered_data;

  const int pool_size_ = 8;
  const int point_size_ = 210000;

  // Declare object that handles camera pose calculations
  rotation_estimator algo_;
  Eigen::Matrix4f transform;
};
}  // namespace device
}  // namespace sensors
}  // namespace apollo
