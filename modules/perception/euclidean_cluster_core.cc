#include "euclidean_cluster_core.h"

#include "modules/common/global_gflags.h"

namespace apollo {
namespace perception {
EuClusterCore::EuClusterCore(std::shared_ptr<Node> node_)
    : cluster_distance_({0.05, 0.1, 0.125, 0.15, 0.20}),
                        seg_distance_({0.5, 1.0, 1.5, 2.0})) {
  // 欧几里德聚类最重要的参数是聚类半径阈值，为了达到更好的聚类效果，
  // 我们在不同距离的区域使用不同的聚类半径阈值
  // seg_distance_ = {0.5, 1.0, 1.5, 2.0};
  // cluster_distance_ = {0.05, 0.1, 0.125, 0.15, 0.20};

  obstacle_writer_ =
      node_->CreateWriter<PerceptionObstacle>(FLAGS_obstacle_channel);
}

void EuClusterCore::VoxelGridFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr in,
                                    pcl::PointCloud<pcl::PointXYZ>::Ptr out,
                                    double leaf_size) {
  pcl::VoxelGrid<pcl::PointXYZ> filter;
  filter.setInputCloud(in);
  filter.setLeafSize(leaf_size, leaf_size, leaf_size);
  filter.filter(*out);
}

void EuClusterCore::CropBoxFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr in,
                                  pcl::PointCloud<pcl::PointXYZ>::Ptr out) {
  pcl::CropBox<pcl::PointXYZ> region(true);
  region.setMin(Eigen::Vector4f(0, -1.5, -0.6, 1));
  region.setMax(Eigen::Vector4f(80, 3, 2, 1));
  region.setInputCloud(in);
  region.filter(*out);
}

void EuClusterCore::ClusterSegment(pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc,
                                   double in_max_cluster_distance,
                                   std::vector<PerceptionObstacle> &obj_list) {
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>);

  // create 2d pc
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2d(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(*in_pc, *cloud_2d);
  // make it flat
  for (size_t i = 0; i < cloud_2d->points.size(); i++) {
    cloud_2d->points[i].z = 0;
  }

  if (cloud_2d->points.size() > 0) {
    tree->setInputCloud(cloud_2d);
  }

  std::vector<pcl::PointIndices> local_indices;

  pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclid;
  euclid.setInputCloud(cloud_2d);
  euclid.setClusterTolerance(in_max_cluster_distance);
  euclid.setMinClusterSize(MIN_CLUSTER_SIZE);
  euclid.setMaxClusterSize(MAX_CLUSTER_SIZE);
  euclid.setSearchMethod(tree);
  euclid.extract(local_indices);

  for (size_t i = 0; i < local_indices.size(); i++) {
    // the structure to save one detected object
    PerceptionObstacle *obj_info;

    float min_x = std::numeric_limits<float>::max();  //编译器允许的最大值
    float max_x = -std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float max_y = -std::numeric_limits<float>::max();
    float min_z = std::numeric_limits<float>::max();
    float max_z = -std::numeric_limits<float>::max();

    for (auto pit = local_indices[i].indices.begin();
         pit != local_indices[i].indices.end(); ++pit) {
      double sum = 0;

      // fill new colored cluster point by point
      pcl::PointXYZ p;
      p.x = in_pc->points[*pit].x;
      p.y = in_pc->points[*pit].y;
      p.z = in_pc->points[*pit].z;

      obj_info->centroid.x += p.x;
      obj_info->centroid.y += p.y;
      obj_info->centroid.z += p.z;

      if (p.x < min_x) min_x = p.x;
      if (p.y < min_y) min_y = p.y;
      if (p.z < min_z) min_z = p.z;
      if (p.x > max_x) max_x = p.x;
      if (p.y > max_y) max_y = p.y;
      if (p.z > max_z) max_z = p.z;
    }

    obj_info->set_id(i);

    // min, max points
    auto bbox_3d = obj_info->mutable_bbox3d();
    bbox3d->set_xmin(min_x);
    bbox3d->set_ymin(min_y);
    bbox3d->set_zmin(min_z);
    bbox3d->set_xmax(max_x);
    bbox3d->set_ymax(max_y);
    bbox3d->set_zmax(max_z);

    // calculate centroid, average
    if (local_indices[i].indices.size() > 0) {
      obj_info->centroid.x /= local_indices[i].indices.size();
      obj_info->centroid.y /= local_indices[i].indices.size();
      obj_info->centroid.z /= local_indices[i].indices.size();
    }

    // Position
    auto position = obj_info->mutable_position();
    position->set_x(obj_info->bbox3d.zmin + length_ / 2);
    position->set_y(obj_info->bbox3d.ymin + width_ / 2);
    position->set_z(obj_info->bbox3d.zmin + height_ / 2);

    // calculate bounding box
    double length_ = obj_info->bbox3d.xmax - obj_info->bbox3d.xmin;
    double width_ = obj_info->bbox3d.ymax - obj_info->bbox3d.ymin;
    double height_ = obj_info->bbox3d.zmax - obj_info->bbox3d.zmin;

    obj_info->set_length((length_ < 0) ? -1 * length_ : length_);
    obj_info->set_width((width_ < 0) ? -1 * width_ : width_);
    obj_info->set_height((height_ < 0) ? -1 * height_ : height_);
    // Length < 5, width < 5, height > 0.2 for now
    if (obj_info->length < 5 && obj_info->position.y < 8 &&
        obj_info->position.y > -5 && obj_info->width < 5 &&
        obj_info->position.z > 0.2 && obj_info->height > 0.2) {
      obj_list.push_back(obj_info);
    } else {
      AINFO << "OBSTACLE AFTER CLUSTER MABYE NOT OBSTACLE. SKIP";
    }
  }
}

void EuClusterCore::ClusterByDistance(
    pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc,
    const std::vector<PerceptionObstacle> &obj_list) {
  // cluster the pointcloud according to the distance of the points using
  // different thresholds (not only one for the entire pc) in this way, the
  // points farther in the pc will also be clustered

  // 0 => 0-15m d=0.5
  // 1 => 15-30 d=1
  // 2 => 30-45 d=1.6
  // 3 => 45-60 d=2.1
  // 4 => >60   d=2.6

  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> segment_pc_array(5);

  for (size_t i = 0; i < segment_pc_array.size(); i++) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
    segment_pc_array[i] = tmp;
  }

  for (size_t i = 0; i < in_pc->points.size(); i++) {
    pcl::PointXYZ current_point;
    current_point.x = in_pc->points[i].x;
    current_point.y = in_pc->points[i].y;
    current_point.z = in_pc->points[i].z;

    float origin_distance =
        std::sqrt(std::pow(current_point.x, 2) + std::pow(current_point.y, 2));

    // 如果点的距离大于5m, 忽略该点
    if (origin_distance >= 5) {
      continue;
    }

    if (origin_distance < seg_distance_[0]) {
      segment_pc_array[0]->points.push_back(current_point);
    } else if (origin_distance < seg_distance_[1]) {
      segment_pc_array[1]->points.push_back(current_point);
    } else if (origin_distance < seg_distance_[2]) {
      segment_pc_array[2]->points.push_back(current_point);
    } else if (origin_distance < seg_distance_[3]) {
      segment_pc_array[3]->points.push_back(current_point);
    } else {
      segment_pc_array[4]->points.push_back(current_point);
    }
  }

  for (size_t i = 0; i < segment_pc_array.size(); i++) {
    ClusterSegment(segment_pc_array[i], cluster_distance_[i], obj_list);
  }
}

void EuClusterCore::Proc(std::shared<PointCloud> in_cloud_ptr) {
  auto startTime = std::chrono::steady_clock::now();
  pcl::PointCloud<pcl::PointXYZ>::Ptr current_pc_ptr(
      new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_grid_filtered_pc_ptr(new
  // pcl::PointCloud<pcl::PointXYZ>); pcl::PointCloud<pcl::PointXYZ>::Ptr
  // CropBox_filtered_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);

  // point_cloud_header_ = in_cloud_ptr->header;

  pcl::fromROSMsg(*in_cloud_ptr, *current_pc_ptr);
  // down sampling the point cloud before cluster
  // VoxelGridFilter(current_pc_ptr, current_pc_ptr, LEAF_SIZE);
  // CropBoxFilter(current_pc_ptr,current_pc_ptr);

  std::vector<PerceptionObstacle> global_obj_list;
  // ClusterByDistance(filtered_pc_ptr, global_obj_list);
  ClusterByDistance(current_pc_ptr, global_obj_list);
  // ClusterByDistance(CropBox_filtered_pc_ptr, global_obj_list);

  PerceptionObstacles obstacles;

  for (size_t i = 0; i < global_obj_list.size(); i++) {
    // bbox_array.boxes.push_back(global_obj_list[i].bounding_box_);
    auto next_obstacle = obstacles->add_obstacle();
    next_obstacle->CopyFrom(global_obj_list[i]);
  }

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  AINFO << "cluster took " << elapsedTime.count() << "milliseconds";

  obstacle_writer_->Write(obstacles);
}

EuClusterCore::~EuClusterCore() { AINFO << "Destructor from EuClusterCore"; }

}  // namespace perception
}  // namespace apollo
