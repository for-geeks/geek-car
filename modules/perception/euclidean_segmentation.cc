
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr
read_cloud_point(std::string const &file_path) {
  // Loading first scan.
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_path, *cloud) == -1) {
    PCL_ERROR("Couldn't read the pcd file\n");
    return nullptr;
  }
  return cloud;
}

/******************************************************************************
 打开点云数据，并对点云进行滤波重采样预处理，然后采用平面分割模型对点云进行分割处理
 提取出点云中所有在平面上的点集，并将其存盘
******************************************************************************/
int main(int argc, char **argv) {
  // 读取文件
  pcl::PointCloud<pcl::PointXYZ>::Ptr add_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(
      new pcl::PointCloud<pcl::PointXYZ>);

  auto cloud = read_cloud_point(argv[1]);

  std::cout << "PointCloud before filtering has: " << cloud->points.size()
            << " data points." << std::endl;

  // 下采样，体素叶子大小为0.01
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(
      new pcl::PointCloud<pcl::PointXYZ>);
  vg.setInputCloud(cloud);
  vg.setLeafSize(0.01f, 0.01f, 0.01f);
  vg.filter(*cloud_filtered);
  std::cout << "PointCloud after filtering has: "
            << cloud_filtered->points.size() << " data points." << std::endl;

  //创建平面模型分割的对象并设置参数
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices); //设置聚类的内点索引
  pcl::ModelCoefficients::Ptr coefficients(
      new pcl::ModelCoefficients); //平面模型的因子
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(
      new pcl::PointCloud<pcl::PointXYZ>());

//   pcl::PCDWriter writer;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE); //分割模型
  seg.setMethodType(pcl::SAC_RANSAC);    //随机参数估计方法
  seg.setMaxIterations(100);             //最大的迭代的次数
  seg.setDistanceThreshold(0.02);        //设置阀值

  int nr_points = (int)cloud_filtered->points.size(); //剩余点云的数量
  while (cloud_filtered->points.size() > 0.3 * nr_points) {
    // 从剩余点云中再分割出最大的平面分量
    // （因为我们要处理的点云的数据是两个平面的存在的）
    seg.setInputCloud(cloud_filtered);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0) //如果内点的数量已经等于0，就说明没有
    {
      std::cout << "Could not estimate a planar model for the given dataset."
                << std::endl;
      break;
    }

    // 从输入的点云中提取平面模型的内点
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers); //提取内点的索引并存储在其中
    extract.setNegative(false);

    // 得到与平面表面相关联的点云数据
    extract.filter(*cloud_plane);
    std::cout << "PointCloud representing the planar component: "
              << cloud_plane->points.size() << " data points." << std::endl;

    //  // 移去平面局内点，提取剩余点云
    extract.setNegative(true);
    extract.filter(*cloud_f);
    *cloud_filtered = *cloud_f;
  }

  // 创建用于提取搜索方法的kdtree树对象
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec; //欧式聚类对象
  ec.setClusterTolerance(0.02); // 设置近邻搜索的搜索半径为2cm
  ec.setMinClusterSize(100); //设置一个聚类需要的最少的点数目为100
  ec.setMaxClusterSize(25000); //设置一个聚类需要的最大点数目为25000
  ec.setSearchMethod(tree);    //设置点云的搜索机制
  ec.setInputCloud(cloud_filtered);
  ec.extract(cluster_indices);
  //从点云中提取聚类，并将点云索引保存在cluster_indices中
  //迭代访问点云索引cluster_indices,直到分割处所有聚类
  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it =
           cluster_indices.begin();
       it != cluster_indices.end(); ++it) {
    //迭代容器中的点云的索引，并且分开保存索引的点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(
        new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin();
         pit != it->indices.end(); ++pit) {
      //设置保存点云的属性问题
      cloud_cluster->points.push_back(cloud_filtered->points[*pit]);
    }
    cloud_cluster->width = static_cast<uint32_t>(cloud_filtered->width);
    cloud_cluster->height = static_cast<uint32_t>(cloud_cluster->points.size() /
                                                  cloud_cluster->width);
    cloud_cluster->is_dense = true;

    std::cout << "PointCloud representing the Cluster: "
              << cloud_cluster->points.size() << " data points." << std::endl;
    std::stringstream ss;
    ss << "cloud_cluster_" << j << ".pcd";
    // writer.write<pcl::PointXYZ>(ss.str(), *cloud_cluster, false);

    //————————————以上就是实现所有的聚类的步骤，并且保存了————————————————————————————//
    //以下就是我为了回答网友提问解决可视化除了平面以后的可视化的代码也就两行
    j++;
    *add_cloud += *cloud_cluster;
    // pcl::io::savePCDFileASCII("add_cloud.pcd", *add_cloud);
  }

  return (0);
}