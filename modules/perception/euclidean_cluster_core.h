#pragma once

#include <chrono>
#include <ctime>
#include <memory>
#include <vector>

#include <pcl/conversions.h>
#include <pcl/point_types.h>
//#include <pcl_conversions/pcl_conversions.h>
//#include <pcl_ros/transforms.h>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/segmentation/conditional_euclidean_clustering.h>

#include <pcl/common/common.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/search/kdtree.h>
#include <pcl/search/organized.h>

#include "cyber/common/log.h"
#include "cyber/node/node.h"

#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/sensors/proto/pointcloud.pb.h"

// 定义降采样的leaf
// size，聚类是一个费时运算，为了减少计算量，我们通常先进行降采样
#define LEAF_SIZE 0.001f

#define MIN_CLUSTER_SIZE 15
#define MAX_CLUSTER_SIZE 1500

namespace apollo {
namespace perception {

using apollo::cyber::Node;
using apollo::cyber::Writer;
using apollo::perception::PerceptionObstacle;
using apollo::perception::PerceptionObstacles;
using apollo::sensors::PointCloud;

class EuClusterCore {
 public:
  EuClusterCore(std::shared_ptr<Node> node_);
  ~EuClusterCore();

  void Proc(std::shared_ptr<PointCloud> in_cloud_ptr);

 private:
  std::shared_ptr<Writer<PerceptionObstacles>> obstacle_writer_ = nullptr;
  std::vector<double> seg_distance_, cluster_distance_;

  void VoxelGridFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr in,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr out,
                       float leaf_size);

  void CropBoxFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr in,
                     pcl::PointCloud<pcl::PointXYZ>::Ptr out);

  void ClusterByDistance(pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc,
                         std::vector<PerceptionObstacle> &obj_list);

  void ClusterSegment(pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc,
                      double in_max_cluster_distance,
                      const std::vector<PerceptionObstacle> &obj_list);
};

}  // namespace perception
}  // namespace apollo
