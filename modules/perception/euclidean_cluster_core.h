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

#include <chrono>
#include <ctime>
#include <memory>
#include <vector>

#include <pcl/common/common.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/organized.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "cyber/base/concurrent_object_pool.h"
#include "cyber/common/log.h"
#include "cyber/node/node.h"
#include "cyber/node/writer.h"

#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/sensors/proto/pointcloud.pb.h"

namespace apollo {
namespace perception {

using apollo::cyber::Node;
using apollo::cyber::Writer;
using apollo::cyber::base::CCObjectPool;
using apollo::perception::PerceptionObstacle;
using apollo::perception::PerceptionObstacles;
using apollo::sensors::PointCloud;
using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

class EuClusterCore {
 public:
  EuClusterCore(std::shared_ptr<Node> node_);
  ~EuClusterCore();

  void Proc(std::shared_ptr<PointCloud> in_cloud_ptr);

 private:
  std::shared_ptr<Writer<PerceptionObstacles>> obstacles_writer_ = nullptr;

  // 欧几里德聚类最重要的参数是聚类半径阈值，为了达到更好的聚类效果，
  // 我们在不同距离的区域使用不同的聚类半径阈值
  std::vector<double> seg_distance_ = {0.5, 0.9, 1.3, 1.7};
  std::vector<double> cluster_distance_ = {0.05, 0.055, 0.060, 0.065, 0.07};

  std::shared_ptr<CCObjectPool<pcl::PointCloud<pcl::PointXYZ>>>
      point_cloud_pool_ = nullptr;

  const int pool_size_ = 8;
  const int point_size_ = 1000;

  void VoxelGridFilter(pcl_ptr in, pcl_ptr out);

  void CropBoxFilter(pcl_ptr in, pcl_ptr out);

  void ClusterByDistance(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> in_pc,
                         std::shared_ptr<PerceptionObstacles> obstacles);

  void ClusterSegment(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> in_pc,
                      double cluster_distance,
                      std::shared_ptr<PerceptionObstacles> obstacles);
};

}  // namespace perception
}  // namespace apollo
