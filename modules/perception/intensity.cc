#include <iostream>

#include "pcl/common/common.h"
#include "pcl/common/transforms.h"
#include "pcl/io/pcd_io.h"
#include "pcl/io/png_io.h"
#include "pcl/filters/passthrough.h"
#include "pcl/visualization/cloud_viewer.h"
#include "pcl/visualization/pcl_visualizer.h"

#include "Eigen/Geometry"
#include "opencv2/opencv.hpp"

pcl::PointCloud<pcl::PointXYZI>::Ptr read_cloud_point(
    std::string const &file_path) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  if (pcl::io::loadPCDFile<pcl::PointXYZI>(file_path, *cloud) == -1) {
    PCL_ERROR("Couldn't read the pcd file\n");
    return nullptr;
  }
  return cloud;
}

void GetIntensityImg(cv::Mat *intensity_img, int cols_, int rows_,
                     pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
  *intensity_img = cv::Mat(cv::Size(cols_, rows_), CV_8UC1);

  // pcl::PointCloud<pcl::PointXYZI>::Ptr org_cloud(new
  // pcl::PointCloud<pcl::PointXYZI>); org_cloud->height = 64; org_cloud->width
  // = 4500; org_cloud->resize(64 * 4500);

  for (size_t i = 0; i < cloud->points.size(); ++i) {
    if (cloud->points[i].intensity > 0) {
      std::cout << "intensity value: " << cloud->points[i].intensity
                << std::endl;
    };
  }

  // for (unsigned int y = 0; y < rows_; ++y) {
  //   for (unsigned int x = 0; x < cols_; ++x) {
  //     unsigned int id = y * cols_ + x;
  //     if((*cloud)[id].intensity > 0) {
  //       std::cout << "intensity value: " << (*cloud)[id].intensity <<
  //       std::endl;
  //     }
  //     // intensity_img->at<unsigned char>(y, x) =
  //     //     (unsigned char)((*cloud)[id].intensity);
  //   }
  // }
}

// void Unorganised2Organ(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud){
//       for (unsigned int h = 0; h < cloud->height; ++h) {
//         for (unsigned int w = 0; w < cloud->width; ++w) {
//           double x = cloud->at(w, h).x;
//           double y = cloud->at(w, h).y;
//           double z = cloud->at(w, h).z;
//           Eigen::Vector3d pt3d(x, y, z);
//           if (pt3d[0] == pt3d[0] && pt3d[1] == pt3d[1] && pt3d[2] == pt3d[2])
//           {
//             Eigen::Vector3d pt3d_local;
//             // if (is_global) {
//               // pt3d_local = pose_inv * pt3d;
//             // } else {
//               pt3d_local = pt3d;
//             // }
//             unsigned char intensity =
//                 static_cast<unsigned char>(cloud->at(w, h).intensity);
//             // pt3ds->push_back(pt3d_local);
//             // intensities->push_back(intensity);
//           }
//         }
//       }
//       bool success =
//       cv::imwrite("/apollo/modules/perception/testdata/intensity.png",
//       image);
// }

pcl::PointCloud<pcl::PointXYZI>::Ptr passThroughFilter1D(
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, std::string dimention, float min,
    float max) {
  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZI> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName(dimention);
  pass.setFilterLimits(min, max);
  // pass.setFilterLimitsNegative (true);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
  pass.filter(*cloud_filtered);

  // // PCL VISUALIZATION
  // std::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
  //     new pcl::visualization::PCLVisualizer("3D Viewer"));
  // viewer->setBackgroundColor(0, 0, 0);
  // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> target_color(
  //     cloud_filtered, 255, 0, 0);
  // viewer->addPointCloud<pcl::PointXYZI>(cloud_filtered, target_color,
  //                                       "target cloud");
  // viewer->setPointCloudRenderingProperties(
  //     pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target cloud");

  // // Starting Visualizer
  // viewer->addCoordinateSystem(1.0, "global");
  // viewer->initCameraParameters();
  // // Wait until visualizer window is closed.
  // while (!viewer->wasStopped()) {
  //   viewer->updatePointCloud<pcl::PointXYZI>(cloud_filtered, target_color,
  //                                            "new point cloud");
  //   viewer->spinOnce(100);
  //   // std::this_thread::sleep_for(std::chrono::microseconds(100000));
  // }

  if(cloud_filtered != nullptr) {
      // pcl::io::savePCDFileASCII("slice_" + dimention +"_"+ std::to_string(min) +" _" +std::to_string(max)+ ".pcd", *cloud_filtered);
  }
  return cloud_filtered;
}

cv::Mat makeImageFromPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                                std::string dimensionToRemove, float stepSize1,
                                float stepSize2) {
  pcl::PointXYZI cloudMin, cloudMax;
  pcl::getMinMax3D(*cloud, cloudMin, cloudMax);

  std::string dimen1, dimen2;
  float dimen1Max, dimen1Min, dimen2Min, dimen2Max;
  if (dimensionToRemove == "x") {
    dimen1 = "y";
    dimen2 = "z";
    dimen1Min = cloudMin.y;
    dimen1Max = cloudMax.y;
    dimen2Min = cloudMin.z;
    dimen2Max = cloudMax.z;
  } else if (dimensionToRemove == "y") {
    dimen1 = "x";
    dimen2 = "z";
    dimen1Min = cloudMin.x;
    dimen1Max = cloudMax.x;
    dimen2Min = cloudMin.z;
    dimen2Max = cloudMax.z;
  } else if (dimensionToRemove == "z") {
    dimen1 = "x";
    dimen2 = "y";
    dimen1Min = cloudMin.x;
    dimen1Max = cloudMax.x;
    dimen2Min = cloudMin.y;
    dimen2Max = cloudMax.y;
  }

  std::vector<std::vector<int>> pointCountGrid;
  int maxPoints = 0;

  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> grid;

  for (float i = dimen1Min; i < dimen1Max; i += stepSize1) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr slice =
        passThroughFilter1D(cloud, dimen1, i, i + stepSize1);
    grid.push_back(slice);

    std::vector<int> slicePointCount;

    for (float j = dimen2Min; j < dimen2Max; j += stepSize2) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr grid_cell =
          passThroughFilter1D(slice, dimen2, j, j + stepSize2);

      int gridSize = static_cast<int>(grid_cell->size());
      slicePointCount.push_back(gridSize);

      if (gridSize > maxPoints) {
        maxPoints = gridSize;
      }
    }
    pointCountGrid.push_back(slicePointCount);
  }
  // cv::Mat mat;
  cv::Mat mat(static_cast<int>(pointCountGrid.size()),
              static_cast<int>(pointCountGrid.at(0).size()), CV_8UC1);
  mat = cv::Scalar(0);

  for (int i = 0; i < mat.rows; ++i) {
    for (int j = 0; j < mat.cols; ++j) {
      int pointCount = pointCountGrid.at(i).at(j);
      float percentOfMax =
          static_cast<float>((pointCount + 0.0) / (maxPoints + 0.0));
      int intensity = static_cast<int>(percentOfMax * 255);

      mat.at<uchar>(i, j) = static_cast<uchar>(intensity);
    }
  }

  return mat;
}

int main(int argc, char **argv) {
  auto target_cloud = read_cloud_point(argv[1]);
  std::cout << "Loaded " << target_cloud->size() << " data points from "
            << argv[1] << std::endl;


  // std::cout << "Loaded " << target_cloud->width * target_cloud->height
  // << " data points from FILE with the following fields." << std::endl;

  // pcl::io::savePNGFile("/apollo/modules/perception/testdata/curvature.png",
  //                      *target_cloud, "curvature");

  cv::Mat image;
  // GetIntensityImg(&image, 3356, 64, target_cloud);
  // bool success =
  // cv::imwrite("/apollo/modules/perception/testdata/intensity.png", image);

  // for (size_t i = 0; i < target_cloud->points.size(); ++i){
  //   if(target_cloud->points[i].intensity > 0) {
  //     std::cout << "    " << target_cloud->points[i].x << " "
  //             << target_cloud->points[i].y << " " <<
  //             target_cloud->points[i].z  << " intensity:" <<
  //             target_cloud->points[i].intensity
  //             << std::endl;
  //   }
  // }
  // pcl::PointCloud<pcl::PointXYZI>::Ptr slice = passThroughFilter1D(target_cloud, "x", 0.1f, 0.5f);

  image = makeImageFromPointCloud(target_cloud, "z", 0.125f, 0.125f);
  bool success =
      cv::imwrite("/apollo/modules/perception/testdata/"+ std::string(argv[1]) +".png", image);

  return 0;
}