// point cloud to image
// audit by
// https://stackoverflow.com/questions/49731101/generate-image-from-an-unorganized-point-cloud-in-pcl

#include <iostream>
#include <numeric>

#include "pcl/io/pcd_io.h"
#include "pcl/io/png_io.h"
#include "pcl/point_types.h"
#include "pcl/search/kdtree.h"
#include "pcl/search/organized.h"

#include "Eigen/Geometry"

#include "opencv2/opencv.hpp"

using cv::Mat;

pcl::PointCloud<pcl::PointXYZINormal>::Ptr ProjectToPlane(
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud, Eigen::Vector3f origin,
    Eigen::Vector3f axis_x, Eigen::Vector3f axis_y) {
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr aux_cloud(
      new pcl::PointCloud<pcl::PointXYZINormal>);
  copyPointCloud(*cloud, *aux_cloud);

  auto normal = axis_x.cross(axis_y);
  Eigen::Hyperplane<float, 3> plane(normal, origin);

  for (auto itPoint = aux_cloud->begin(); itPoint != aux_cloud->end();
       itPoint++) {
    // project point to plane
    auto proj = plane.projection(itPoint->getVector3fMap());
    itPoint->getVector3fMap() = proj;
    // optional: save the reconstruction information as normals in the projected
    // cloud
    itPoint->getNormalVector3fMap() = itPoint->getVector3fMap() - proj;
  }
  return aux_cloud;
}

pcl::PointCloud<pcl::PointXYZINormal>::Ptr GenerateGrid(Eigen::Vector3f origin,
                                                        Eigen::Vector3f axis_x,
                                                        Eigen::Vector3f axis_y,
                                                        float length,
                                                        int image_size) {
  auto step = length / image_size;

  pcl::PointCloud<pcl::PointXYZINormal>::Ptr image_cloud(
      new pcl::PointCloud<pcl::PointXYZINormal>(image_size, image_size));
  for (auto i = 0; i < image_size; i++)
    for (auto j = 0; j < image_size; j++) {
      int x = i - int(image_size / 2);
      int y = j - int(image_size / 2);
      image_cloud->at(i, j).getVector3fMap() =
          //   center + (x * step * axisx) + (y * step * axisy); origin codes
          origin + (x * step * axis_x) + (y * step * axis_y);
    }

  return image_cloud;
}

void InterpolateToGrid(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud,
                       pcl::PointCloud<pcl::PointXYZINormal>::Ptr grid,
                       float max_resolution, int max_nn_to_consider) {
  pcl::search::KdTree<pcl::PointXYZINormal>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZINormal>);
  tree->setInputCloud(cloud);

  for (auto idx = 0; idx < grid->points.size(); idx++) {
    std::vector<int> indices;
    std::vector<float> distances;
    if (tree->radiusSearch(grid->points[idx], max_resolution, indices,
                           distances, max_nn_to_consider) > 0) {
      // Linear Interpolation of:
      //      Intensity
      //      Normals- residual vector to inflate(recondtruct) the surface
      float intensity(0);
      Eigen::Vector3f n(0, 0, 0);
      float weight_factor =
          1.0F / std::accumulate(distances.begin(), distances.end(), 0.0F);
      for (auto i = 0; i < indices.size(); i++) {
        float w = weight_factor * distances[i];
        intensity += w * cloud->points[indices[i]].intensity;
        auto res = cloud->points[indices[i]].getVector3fMap() -
                   grid->points[idx].getVector3fMap();
        n += w * res;
      }
      grid->points[idx].intensity = intensity;
      grid->points[idx].getNormalVector3fMap() = n;
      grid->points[idx].curvature = 1;
    } else {
      grid->points[idx].intensity = 0;
      grid->points[idx].curvature = 0;
      grid->points[idx].getNormalVector3fMap() = Eigen::Vector3f(0, 0, 0);
    }
  }
}

// Convert an Organized cloud to cv::Mat (an image and a mask)
//      point Intensity is used for the image
//          if as_float is true => take the raw intensity (image is CV_32F)
//          if as_float is false => assume intensity is in range [0, 255] and
//          round it (image is CV_8U)
//      point Curvature is used for the mask (assume 1 or 0)
std::pair<cv::Mat, cv::Mat> ConvertGridToImage(
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr grid, bool as_float) {
  int rows = grid->height;
  int cols = grid->width;

  if ((rows <= 0) || (cols <= 0)) return std::pair<Mat, Mat>(Mat(), Mat());

  // Initialize

  Mat image = Mat(rows, cols, as_float ? CV_32F : CV_8U);
  Mat mask = Mat(rows, cols, CV_8U);

  if (as_float) {
    for (int y = 0; y < image.rows; y++) {
      for (int x = 0; x < image.cols; x++) {
        image.at<float>(y, x) = grid->at(x, image.rows - y - 1).intensity;
        mask.at<uchar>(y, x) = 255 * grid->at(x, image.rows - y - 1).curvature;
      }
    }
  } else {
    for (int y = 0; y < image.rows; y++) {
      for (int x = 0; x < image.cols; x++) {
        image.at<uchar>(y, x) =
            (int)round(grid->at(x, image.rows - y - 1).intensity);
        mask.at<uchar>(y, x) = 255 * grid->at(x, image.rows - y - 1).curvature;
      }
    }
  }

  return std::pair<Mat, Mat>(image, mask);
}

// project image to cloud (using the grid data)
// organized - whether the resulting cloud should be an organized cloud
pcl::PointCloud<pcl::PointXYZI>::Ptr BackProjectImage(
    cv::Mat image, pcl::PointCloud<pcl::PointXYZINormal>::Ptr grid,
    bool organized) {
  if ((image.size().height != grid->height) ||
      (image.size().width != grid->width)) {
    assert(false);
    throw;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  cloud->reserve(grid->height * grid->width);

  // order of iteration is critical for organized target cloud
  for (auto r = image.size().height - 1; r >= 0; r--) {
    for (auto c = 0; c < image.size().width; c++) {
      pcl::PointXYZI point;
      auto mask_value = mask.at<uchar>(image.rows - r - 1, c);
      if (mask_value > 0)  // valid pixel
      {
        point.intensity = mask_value;
        point.getVector3fMap() = grid->at(c, r).getVector3fMap() +
                                 grid->at(c, r).getNormalVector3fMap();
      } else  // invalid pixel
      {
        if (organized) {
          point.intensity = 0;
          point.x = std::numeric_limits<float>::quiet_NaN();
          point.y = std::numeric_limits<float>::quiet_NaN();
          point.z = std::numeric_limits<float>::quiet_NaN();
        } else {
          continue;
        }
      }

      cloud->push_back(point);
    }
  }

  if (organized) {
    cloud->width = grid->width;
    cloud->height = grid->height;
  }

  return cloud;
}

int main(int argc, char **argv) {
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr original_cloud(
      new pcl::PointCloud<pcl::PointXYZINormal>);
  if (pcl::io::loadPCDFile<pcl::PointXYZINormal>(argv[1], *original_cloud) ==
      -1) {
    PCL_ERROR("Couldn't read the pcd file\n");
    return;
  }

  // reference frame for the projection
  // e.g. take XZ plane around 0,0,0 of length 100 and map to 128*128
  // image
  Eigen::Vector3f origin = Eigen::Vector3f(0, 0, 0);
  Eigen::Vector3f axis_x = Eigen::Vector3f(1, 0, 0);
  Eigen::Vector3f axis_y = Eigen::Vector3f(0, 0, 1);
  float length = 100;
  int image_size = 128;

  auto aux_cloud = ProjectToPlane(original_cloud, origin, axis_x, axis_y);
  // aux_cloud now contains the points of original_cloud, with:
  //      xyz coordinates projected to XZ plane
  //      color (intensity) of the original_cloud (remains unchanged)
  //      normals - we lose the normal information, as we use this field
  //      to save the projection information. if you wish to keep the
  //      normal data, you should define a custom PointType.
  // note: for the sake of projection, the origin is only used to define
  // the plane, so any arbitrary point on the plane can be used

  auto grid = GenerateGrid(origin, axis_x, axis_y, length, image_size);
  // organized cloud that can be trivially mapped to an image

  float max_resolution = 2 * length / image_size;
  int max_nn_to_consider = 16;
  InterpolateToGrid(aux_cloud, grid, max_resolution, max_nn_to_consider);
  // Now you have a grid (an organized cloud), which you can easily map
  // to an image. Any changes you make to the images, you can map back
  // to the grid, and use the normals to project back to your original
  // point cloud.
  return 0;
}