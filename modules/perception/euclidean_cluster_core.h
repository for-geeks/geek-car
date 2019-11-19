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

#include "cyber/common/log.h"
#include "cyber/node/node.h"

#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/sensors/proto/pointcloud.pb.h"

#define MIN_CLUSTER_SIZE 15
#define MAX_CLUSTER_SIZE 1500

namespace apollo {
namespace perception {

using apollo::cyber::Node;
using apollo::cyber::Writer;
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
  std::shared_ptr<Writer<PerceptionObstacles>> obstacle_writer_ = nullptr;

  // 欧几里德聚类最重要的参数是聚类半径阈值，为了达到更好的聚类效果，
  // 我们在不同距离的区域使用不同的聚类半径阈值
  std::vector<double> seg_distance_ = {0.5, 1.0, 1.5, 2.0};
  std::vector<double> cluster_distance_ = {0.05, 0.1, 0.125, 0.15, 0.20};

  // 聚类是一个费时运算，为了减少计算量，我们通常先进行降采样
  void VoxelGridFilter(pcl_ptr in, pcl_ptr out);

  void CropBoxFilter(pcl_ptr in, pcl_ptr out);

  void ClusterByDistance(pcl_ptr in_pc,
                         std::shared_ptr<PerceptionObstacles> obstacles);

  void ClusterSegment(pcl_ptr in_pc, double in_max_cluster_distance,
                      std::shared_ptr<PerceptionObstacles> obstacles);
};

}  // namespace perception
}  // namespace apollo
