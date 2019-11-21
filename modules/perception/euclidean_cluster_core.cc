#include "modules/perception/euclidean_cluster_core.h"

#include "modules/common/global_gflags.h"

namespace apollo {
namespace perception {
using apollo::cyber::Time;

void PbMsg2PointCloud(std::shared_ptr<PointCloud> pb_cloud, pcl_ptr pcl_pc) {
  pcl_pc->width = pb_cloud->width();
  pcl_pc->height = pb_cloud->height();
  pcl_pc->is_dense = false;
  pcl_pc->points.resize(pb_cloud->point_size());
  auto ptr = pb_cloud->mutable_point(0);
  for (auto &p : pcl_pc->points) {
    p.x = ptr->x();
    p.y = ptr->y();
    p.z = ptr->z();
    ptr++;
  }
}

EuClusterCore::EuClusterCore(std::shared_ptr<Node> node_) {
  obstacles_writer_ =
      node_->CreateWriter<PerceptionObstacles>(FLAGS_obstacle_channel);
}

void EuClusterCore::VoxelGridFilter(pcl_ptr in, pcl_ptr out) {
  pcl::VoxelGrid<pcl::PointXYZ> filter;
  filter.setInputCloud(in);
  filter.setLeafSize(float(FLAGS_leaf_size), float(FLAGS_leaf_size),
                     float(FLAGS_leaf_size));
  filter.filter(*out);
  AINFO << "AFTER VOXEL GRIDFILTER, POINT SIZE IS: " << out->points.size();
}

void EuClusterCore::CropBoxFilter(pcl_ptr in, pcl_ptr out) {
  pcl::CropBox<pcl::PointXYZ> region(true);
  region.setMin(Eigen::Vector4f(0, -1.5f, -0.6f, 1.f));
  region.setMax(Eigen::Vector4f(80, 3, 2, 1));
  region.setInputCloud(in);
  region.filter(*out);
  AINFO << "AFTER CROP BOX FILTER, POINT SIZE IS: " << out->points.size();
}

void EuClusterCore::ClusterSegment(
    pcl_ptr in_pc, double in_max_cluster_distance,
    std::shared_ptr<PerceptionObstacles> obstacles) {
  auto t1 = Time::Now().ToSecond();
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>);

  // create 2d pc
  pcl_ptr cloud_2d(new pcl::PointCloud<pcl::PointXYZ>);
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
  AWARN << "AFTER EUCLIDEAN CLUSTER EXTRACTION, INDICES IS: " << local_indices.size();
  auto t2 = Time::Now().ToSecond();
  AWARN << "Time for Euclidean Cluster Extraction:" << t2 - t1;
  /**
   *
   * mainboard: external/pcl/pcl/kdtree/impl/kdtree_flann.hpp:172: 
   * int pcl::KdTreeFLANN<PointT, Dist>::radiusSearch(const PointT&, double, std::vector<int>&, std::vector<float>&, unsigned int) const [with PointT = pcl::PointXYZ; Dist = flann::L2_Simple<float>]: Assertion `point_representation_->isValid (point) && "Invalid (NaN, Inf) point coordinates given to radiusSearch!"' failed.
   **/
  for (size_t i = 0; i < local_indices.size(); i++) {
    // the structure to save one detected object
    PerceptionObstacle *obj_info = nullptr;

    float min_x = std::numeric_limits<float>::min();  //编译器允许的最大值
    float max_x = std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::min();
    float max_y = std::numeric_limits<float>::max();
    float min_z = std::numeric_limits<float>::min();
    float max_z = std::numeric_limits<float>::max();

    for (auto pit = local_indices[i].indices.begin();
         pit != local_indices[i].indices.end(); ++pit) {
      // fill new colored cluster point by point
      pcl::PointXYZ p;
      p.x = in_pc->points[*pit].x;
      p.y = in_pc->points[*pit].y;
      p.z = in_pc->points[*pit].z;

      // auto centroid = obj_info->mutable_centroid();
      // centroid->set_x(centroid->x() + p.x);
      // centroid->set_y(centroid->y() + p.y);
      // centroid->set_z(centroid->z() + p.z);

      if (p.x < min_x) min_x = p.x;
      if (p.y < min_y) min_y = p.y;
      if (p.z < min_z) min_z = p.z;
      if (p.x > max_x) max_x = p.x;
      if (p.y > max_y) max_y = p.y;
      if (p.z > max_z) max_z = p.z;
    }

    AWARN << "MIN_X:" << min_x << " MAX_X: " << max_x;

    obj_info->set_id(static_cast<int32_t>(i));

    // min, max points
    auto bbox3d = obj_info->mutable_bbox3d();
    bbox3d->set_xmin(min_x);
    bbox3d->set_ymin(min_y);
    bbox3d->set_zmin(min_z);
    bbox3d->set_xmax(max_x);
    bbox3d->set_ymax(max_y);
    bbox3d->set_zmax(max_z);

    // calculate centroid, average
    // if (local_indices[i].indices.size() > 0) {
    //   centroid->set_x(centroid.x() / local_indices[i].indices.size());
    //   obj_info->centroid.y /= local_indices[i].indices.size();
    //   obj_info->centroid.z /= local_indices[i].indices.size();
    // }

        // calculate bounding box
    double length_ = obj_info->mutable_bbox3d()->xmax() - obj_info->mutable_bbox3d()->xmin();
    double width_ = obj_info->mutable_bbox3d()->ymax() - obj_info->mutable_bbox3d()->ymin();
    double height_ = obj_info->mutable_bbox3d()->zmax() - obj_info->mutable_bbox3d()->zmin();

    // Position
    auto position = obj_info->mutable_position();
    position->set_x(obj_info->mutable_bbox3d()->zmin() + length_ / 2);
    position->set_y(obj_info->mutable_bbox3d()->ymin() + width_ / 2);
    position->set_z(obj_info->mutable_bbox3d()->zmin() + height_ / 2);


    obj_info->set_length((length_ < 0) ? -1 * length_ : length_);
    obj_info->set_width((width_ < 0) ? -1 * width_ : width_);
    obj_info->set_height((height_ < 0) ? -1 * height_ : height_);
    AWARN << "Obj_info debug info :" << obj_info->DebugString();
    // Length < 5, width < 5, height > 0.2 for now
    if (obj_info->length() < 5 && obj_info->mutable_position()->y() < 8 &&
        obj_info->mutable_position()->y() > -5 && obj_info->width() < 5 &&
        obj_info->mutable_position()->z() > 0.2 && obj_info->height() > 0.2) {
      auto next_obstacle = obstacles->add_perception_obstacle();
      next_obstacle->CopyFrom(*obj_info);
    } else {
      AINFO
          << "OBSTACLE AFTER CLUSTER MABYE NOT OBSTACLE. SKIP FOR THIS CLUSTER";
    }
  }
  auto te = Time::Now().ToSecond();
  AWARN << "Time for DISTANCE segmentation:" << te - t1;
}

void EuClusterCore::ClusterByDistance(
    pcl_ptr in_pc, std::shared_ptr<PerceptionObstacles> obstacles) {
  // cluster the pointcloud according to the distance of the points using
  // different thresholds (not only one for the entire pc) in this way, the
  // points farther in the pc will also be clustered
  std::vector<pcl_ptr> segment_pc_array(5);
  // pcl_ptr pc_array(new pcl::PointCloud<pcl::PointXYZ>);

  for (size_t i = 0; i < segment_pc_array.size(); i++) {
    pcl_ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
    segment_pc_array[i] = tmp;
  }

  for (size_t i = 0; i < in_pc->points.size(); i++) {
    pcl::PointXYZ current_point;
    current_point.x = in_pc->points[i].x;
    current_point.y = in_pc->points[i].y;
    current_point.z = in_pc->points[i].z;

    double origin_distance =
        std::sqrt(std::pow(current_point.x, 2) + std::pow(current_point.y, 2));
    // AINFO << "point: " <<  std::to_string(i) << " origin_distance: " << origin_distance;

    // 如果点的距离大于3m, 忽略该点
    if (origin_distance >= 3 || origin_distance <= 0) {
      continue;
    }
    // pc_array->points.push_back(current_point);

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
  // AINFO << "PC_ARRAY POINT SIZE: " <<  pc_array->points.size();
  // ClusterSegment(segment_pc_array[0], cluster_distance_[0], obstacles);
  // AWARN << "RADIUS : " << seg_distance_[0]<< " CLUSTER DISTANCE:" 
  //       << cluster_distance_[0] << " POINT SIZE IS :" 
  //       <<  segment_pc_array[0]->points.size();

  for (size_t i = 0; i < segment_pc_array.size(); i++) {
    AINFO << "RADIUS : " << seg_distance_[i]<< " CLUSTER DISTANCE:" 
          << cluster_distance_[i] << " POINT SIZE IS :" 
          <<  segment_pc_array[i]->points.size();
    ClusterSegment(segment_pc_array[i], cluster_distance_[i], obstacles);
  }
}

void EuClusterCore::Proc(std::shared_ptr<PointCloud> in_cloud_ptr) {
  auto startTime = std::chrono::steady_clock::now();
  pcl_ptr current_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  // pcl_ptr voxel_grid_filtered_pc_ptr(new
  // pcl::PointCloud<pcl::PointXYZ>); pcl_ptr
  // CropBox_filtered_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);

  AINFO << "FLAGS LEAF_SIZE:" << FLAGS_leaf_size;

  AINFO << "RECEIVED POINT SIZE IS : " << in_cloud_ptr->point_size();

  PbMsg2PointCloud(in_cloud_ptr, current_pc_ptr);
  // down sampling the point cloud before cluster
  // VoxelGridFilter(current_pc_ptr, current_pc_ptr);
  // CropBoxFilter(current_pc_ptr, current_pc_ptr);
  AINFO << "BEFORE CLUSTER, POINT SIZE IS : " << current_pc_ptr->points.size();

  auto obstacles = std::make_shared<PerceptionObstacles>();
  // ClusterByDistance(filtered_pc_ptr, obstacles);
  ClusterByDistance(current_pc_ptr, obstacles);
  // ClusterByDistance(CropBox_filtered_pc_ptr, obstacles);

  AINFO << "OBSTACLES OUTPUT DEBUG MSG:" << obstacles->DebugString();

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  AINFO << "cluster took " << elapsedTime.count() << "milliseconds";

  obstacles_writer_->Write(obstacles);
}

EuClusterCore::~EuClusterCore() { 
  AINFO << "Destructor from EuClusterCore";
}

}  // namespace perception
}  // namespace apollo
