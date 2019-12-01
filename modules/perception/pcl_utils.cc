
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
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

auto t5 = Time::Now().ToSecond();
// 下采样，体素叶子大小为0.01
pcl::VoxelGrid<pcl::PointXYZ> vg;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(
    new pcl::PointCloud<pcl::PointXYZ>);
vg.setInputCloud(cloud_);
vg.setLeafSize(float(FLAGS_leaf_size), float(FLAGS_leaf_size),
               float(FLAGS_leaf_size));
vg.filter(*cloud_filtered);
std::cout << "PointCloud after Voxel Grid filtering has: "
          << cloud_filtered->points.size() << " data points." << std::endl;
auto t6 = Time::Now().ToSecond();
AWARN << "Time for voxel grid filter:" << t6 - t5;

auto t7 = Time::Now().ToSecond();
//直通滤波
pcl::PassThrough<pcl::PointXYZ> pass_y;  //设置滤波器对象

//参数设置
pass_y.setInputCloud(cloud_);
pass_y.setFilterFieldName("y");
// y轴区间设置
pass_y.setFilterLimits(float(FLAGS_passthrough_y_min),
                       float(FLAGS_passthrough_y_max));
pass_y.setFilterLimitsNegative(false);
pass_y.filter(*cloud_);
auto t8 = Time::Now().ToSecond();
AWARN << "Time for Y PASSTHROUGH :" << t8 - t7;

if (FLAGS_pcl_visualization) {
  // PCL VISUALIZATION
  std::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
      new pcl::visualization::PCLVisualizer("3D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(
      cloud_, 255, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ>(cloud_, target_color, "target cloud");
  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target cloud");

  // Starting Visualizer
  viewer->addCoordinateSystem(1.0, "global");
  viewer->initCameraParameters();
  // Wait until visualizer window is closed.
  while (!viewer->wasStopped()) {
    viewer->updatePointCloud<pcl::PointXYZ>(cloud_, target_color,
                                            "new point cloud");
    viewer->spinOnce(100);
    // std::this_thread::sleep_for(std::chrono::microseconds(100000));
  }
}