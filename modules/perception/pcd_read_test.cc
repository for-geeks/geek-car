#include <pcl/io/pcd_io.h>
#include <iostream>

pcl::PointCloud<pcl::PointXYZ>::Ptr read_cloud_point(
    std::string const &file_path) {
  // Loading first scan.
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_path, *cloud) == -1) {
    PCL_ERROR("Couldn't read the pcd file\n");
    return nullptr;
  }
  return cloud;
}

int main(int argc, char **argv) {
  auto target_cloud = read_cloud_point(argv[1]);
  std::cout << "Loaded " << target_cloud->size() << " data points from "
            << argv[1] << std::endl;

  std::cout << "Loaded " << target_cloud->width * target_cloud->height
            << " data points from test_pcd.pcd with the following fields: "
            << std::endl;
  for (size_t i = 0; i < target_cloud->points.size(); ++i)
    std::cout << "    " << target_cloud->points[i].x << " "
              << target_cloud->points[i].y << " " << target_cloud->points[i].z
              << std::endl;

  return 0;
}