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
#include "modules/perception/euclidean_cluster_core.h"

#include <math.h>
#include "modules/common/global_gflags.h"
namespace apollo {
namespace perception {
using apollo::cyber::Time;

void PbMsg2PointCloud(std::shared_ptr<PointCloud> pb_cloud,
                      std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pcl_pc) {
  if (pb_cloud->point_size() <= 0) {
    return;
  }
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

  //  Concurrent object pool for point cloud
  point_cloud_pool_.reset(
      new CCObjectPool<pcl::PointCloud<pcl::PointXYZ>>(pool_size_));
  point_cloud_pool_->ConstructAll();
  for (int i = 0; i < pool_size_; i++) {
    auto point_cloud = point_cloud_pool_->GetObject();
    if (point_cloud == nullptr) {
      AERROR << "Failed to Getobject, i: " << i;
    }
    point_cloud->points.resize(point_size_);
  }
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
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> in_pc,
    double cluster_distance, std::shared_ptr<PerceptionObstacles> obstacles) {
  auto t1 = Time::Now().ToSecond();

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>);

  // create 2d pc
  pcl_ptr cloud_2d(new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::copyPointCloud(*in_pc, *cloud_2d);
  *cloud_2d = *in_pc;
  // make it flat
  for (size_t i = 0; i < in_pc->points.size(); i++) {
    if (std::isnan(cloud_2d->points[i].y) ||
        std::isinf(cloud_2d->points[i].y)) {
      cloud_2d->points[i].y = -1;
    }
    if (std::isnan(cloud_2d->points[i].x) ||
        std::isinf(cloud_2d->points[i].x)) {
      cloud_2d->points[i].x = -1;
    }
    if (std::isnan(cloud_2d->points[i].z) ||
        std::isinf(cloud_2d->points[i].z)) {
      cloud_2d->points[i].z = -1;
    }
    cloud_2d->points[i].y = 0;
  }

  if (cloud_2d->points.size() > 0) {
    tree->setInputCloud(cloud_2d);
  } else {
    return;
  }

  std::vector<pcl::PointIndices> local_indices;

  pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclid;
  euclid.setInputCloud(cloud_2d);
  euclid.setClusterTolerance(cluster_distance);
  euclid.setMinClusterSize(FLAGS_min_cluster_size);
  euclid.setMaxClusterSize(FLAGS_max_cluster_size);
  euclid.setSearchMethod(tree);
  euclid.extract(local_indices);
  AWARN << "AFTER EUCLIDEAN CLUSTER EXTRACTION, INDICES IS: "
        << local_indices.size();
  auto t2 = Time::Now().ToSecond();
  AWARN << "Time for Euclidean Cluster Extraction:" << t2 - t1;

  for (size_t i = 0; i < local_indices.size(); i++) {
    // the structure to save one detected object
    // PerceptionObstacle obj_info;

    float min_x = std::numeric_limits<float>::max();  //编译器允许的最大值
    float max_x = -std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float max_y = -std::numeric_limits<float>::max();
    float min_z = std::numeric_limits<float>::max();
    float max_z = -std::numeric_limits<float>::max();

    for (auto pit = local_indices[i].indices.begin();
         pit != local_indices[i].indices.end(); ++pit) {
      // fill new colored cluster point by point
      pcl::PointXYZ p;
      p.x = in_pc->points[*pit].x;
      p.y = in_pc->points[*pit].y;
      p.z = in_pc->points[*pit].z;

      // auto centroid = obj_info.mutable_centroid();
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

    // min, max points

    // calculate centroid, average
    // if (local_indices[i].indices.size() > 0) {
    //   centroid->set_x(centroid.x() / local_indices[i].indices.size());
    //   obj_info->centroid.y /= local_indices[i].indices.size();
    //   obj_info->centroid.z /= local_indices[i].indices.size();
    // }

    // calculate bounding box by REALSENSE coordinate system
    double length_ = max_z - min_z;
    double width_ = max_x - min_x;
    double height_ = max_y - min_y;

    // // Position
    // auto position = obj_info.mutable_position();
    auto position_x = min_x + length_ / 2;
    auto position_y = min_y + height_ / 2;
    auto position_z = min_z + width_ / 2;

    auto real_length = (length_ < 0) ? -1 * length_ : length_;
    auto real_width = (width_ < 0) ? -1 * width_ : width_;
    // auto real_height = (height_ < 0) ? -1 * height_ : height_;

    // Length < 5, width < 5 for now
    if (real_length > 0.03&& real_length < 5  &&  real_width > 0.03 && real_width < 5) {
      auto next_obstacle = obstacles->add_perception_obstacle();
      next_obstacle->set_id(static_cast<int32_t>(i));
      next_obstacle->mutable_bbox2d()->set_xmin(min_x);
      next_obstacle->mutable_bbox2d()->set_xmax(max_x);  
      next_obstacle->mutable_bbox2d()->set_zmin(min_z);
      next_obstacle->mutable_bbox2d()->set_zmax(max_z);

      next_obstacle->set_length((length_ < 0) ? -1 * length_ : length_);
      next_obstacle->set_width((width_ < 0) ? -1 * width_ : width_);
      next_obstacle->set_height((height_ < 0) ? -1 * height_ : height_);

      next_obstacle->mutable_position()->set_x(position_x);
      next_obstacle->mutable_position()->set_y(position_y);
      next_obstacle->mutable_position()->set_z(position_z);
    } else {
      // AINFO << "OBSTACLE SKIPPED :" << obj_info.DebugString();
      AINFO
          << "OBSTACLE AFTER CLUSTER MABYE NOT OBSTACLE. SKIP FOR THIS CLUSTER";
    }
  }
  auto te = Time::Now().ToSecond();
  AWARN << "Time for DISTANCE segmentation:" << te - t1;
}

void EuClusterCore::ClusterByDistance(
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> in_pc,
    std::shared_ptr<PerceptionObstacles> obstacles) {
  // cluster the pointcloud according to the distance of the points using
  // different thresholds (not only one for the entire pc) in this way, the
  // points farther in the pc will also be clustered
  std::vector<pcl_ptr> segment_pc_array(5);
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pc_array(
      new pcl::PointCloud<pcl::PointXYZ>);

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
        std::sqrt(std::pow(current_point.x, 2) + std::pow(current_point.z, 2));
    // AINFO << "point: " <<  std::to_string(i) << " origin_distance: " <<
    // origin_distance;

    // 如果点的距离大于3m, 忽略该点
    if (origin_distance >= 3) {
      continue;
    }
    pc_array->points.push_back(current_point);

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
  AINFO << "PC_ARRAY POINT SIZE: " << pc_array->points.size();
  ClusterSegment(pc_array, cluster_distance_[0], obstacles);
  AWARN << "RADIUS : " << seg_distance_[0]
        << " CLUSTER DISTANCE:" << cluster_distance_[0]
        << " POINT SIZE IS :" << pc_array->points.size();

  // Euclidean Cluster by different distance
  // for (size_t i = 0; i < segment_pc_array.size(); i++) {
  //   if(segment_pc_array[i]->points.size() <= 0) {
  //     continue;
  //   }
  //   AINFO << " CLUSTER DISTANCE:"
  //         << cluster_distance_[i] << " POINT SIZE IS :"
  //         <<  segment_pc_array[i]->points.size();
  //   ClusterSegment(segment_pc_array[i], cluster_distance_[i], obstacles);
  // }
}

void EuClusterCore::Proc(std::shared_ptr<PointCloud> in_cloud_ptr) {
  auto startTime = std::chrono::steady_clock::now();
  AINFO << "RECEIVED POINT SIZE IS : " << in_cloud_ptr->point_size();
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> point_cloud_in_ptr =
      point_cloud_pool_->GetObject();
  if (point_cloud_in_ptr == nullptr) {
    AWARN << "Point cloud pool return nullptr, will be create new.";
    point_cloud_in_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    point_cloud_in_ptr->points.resize(point_size_);
  }
  if (point_cloud_in_ptr == nullptr) {
    AWARN << "Point cloud out is nullptr";
    return;
  }
  point_cloud_in_ptr->clear();
  // pcl_ptr current_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  // pcl_ptr voxel_grid_filtered_pc_ptr(new
  // pcl::PointCloud<pcl::PointXYZ>); pcl_ptr
  // CropBox_filtered_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);

  PbMsg2PointCloud(in_cloud_ptr, point_cloud_in_ptr);
  // down sampling the point cloud before cluster
  // VoxelGridFilter(current_pc_ptr, current_pc_ptr);
  // CropBoxFilter(current_pc_ptr, current_pc_ptr);
  AINFO << "BEFORE CLUSTER, POINT SIZE IS : "
        << point_cloud_in_ptr->points.size();

  auto obstacles = std::make_shared<PerceptionObstacles>();
  // ClusterByDistance(filtered_pc_ptr, obstacles);
  ClusterByDistance(point_cloud_in_ptr, obstacles);
  // ClusterByDistance(CropBox_filtered_pc_ptr, obstacles);

  AINFO << "OBSTACLES OUTPUT DEBUG MSG:" << obstacles->DebugString();

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  AINFO << "cluster took " << elapsedTime.count() << " milliseconds";

  obstacles_writer_->Write(obstacles);
}

EuClusterCore::~EuClusterCore() { AINFO << "Destructor from EuClusterCore"; }

}  // namespace perception
}  // namespace apollo
