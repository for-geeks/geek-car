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

#include "modules/sensors/realsense/realsense_d435i.h"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "pcl/io/pcd_io.h"
#include "pcl/visualization/cloud_viewer.h"
#include "pcl/visualization/pcl_visualizer.h"

#include "modules/common/global_gflags.h"
#include "modules/sensors/realsense.h"

namespace apollo {
namespace sensors {
namespace realsense {

using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

bool D435I::Init(std::shared_ptr<Node> node_) {
  // 1. Init Device
  DeviceConfig();
  // 2. Channel Writer Config
  InitChannelWriter(node_);

  // 3. Concurrent object pool for point cloud
  point_cloud_pool_.reset(new CCObjectPool<PointCloud>(pool_size_));
  point_cloud_pool_->ConstructAll();
  for (int i = 0; i < pool_size_; i++) {
    auto point_cloud = point_cloud_pool_->GetObject();
    if (point_cloud == nullptr) {
      AERROR << "Failed to Getobject, i: " << i;
      return false;
    }
    point_cloud->mutable_point()->Reserve(point_size_);
  }

  // 4.1 Thread to handle frames
  realsense_t1 = std::thread(&D435I::Run, this);

  // 4.2 Thread to get point cloud from frame queue, and publish
  realsense_t2 = std::thread(&D435I::PublishPointCloud, this);

  AINFO << "Realsense Device D435I Init Successfully";
  return true;
}

void D435I::DeviceConfig() {
  // Add desired streams to configuration
  cfg.enable_stream(RS2_STREAM_COLOR, FLAGS_color_image_width,
                    FLAGS_color_image_height, RS2_FORMAT_BGR8,
                    FLAGS_color_image_frequency);
  // Use a configuration object to request only depth from the pipeline
  cfg.enable_stream(RS2_STREAM_DEPTH, FLAGS_depth_image_width,
                    FLAGS_depth_image_height, RS2_FORMAT_Z16,
                    FLAGS_depth_image_frequency);
  // Add streams of gyro and accelerometer to configuration
  cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
  cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);

  // Instruct pipeline to start streaming with the requested configuration
  auto profile = pipe.start(cfg);
  auto sensor = profile.get_device().first<rs2::depth_sensor>();

  // Set the device to High Accuracy preset of the D400 stereoscopic
  if (sensor && sensor.is<rs2::depth_stereo_sensor>()) {
    sensor.set_option(RS2_OPTION_VISUAL_PRESET, RS2_RS400_VISUAL_PRESET_HAND);
  }
}

void D435I::InitChannelWriter(std::shared_ptr<Node> node_) {
  // Channel Writer flags
  if (FLAGS_publish_color_image) {
    color_image_writer_ = node_->CreateWriter<Image>(FLAGS_color_image_channel);
  }

  if (FLAGS_publish_depth_image) {
    depth_image_writer_ = node_->CreateWriter<Image>(FLAGS_depth_image_channel);
  }

  // Point cloud channel
  if (FLAGS_publish_point_cloud) {
    point_cloud_writer_ =
        node_->CreateWriter<PointCloud>(FLAGS_point_cloud_channel);
  }

  // acc channel
  if (FLAGS_publish_realsense_acc) {
    acc_writer_ = node_->CreateWriter<Acc>(FLAGS_realsense_acc_channel);
  }

  // gyro channel
  if (FLAGS_publish_realsense_gyro) {
    gyro_writer_ = node_->CreateWriter<Gyro>(FLAGS_realsense_gyro_channel);
  }

  // compreessed image channel
  if (FLAGS_publish_compressed_color_image) {
    compressed_image_writer_ =
        node_->CreateWriter<Image>(FLAGS_compressed_color_image_channel);
  }
}

void D435I::Run() {
  while (!stop_) {
    rs2::frameset frames;
    // Wait for all configured streams to produce a frame
    frames = pipe.wait_for_frames();

    rs2::frame color_frame = frames.get_color_frame();
    OnColorImage(color_frame);

    rs2::frame depth_frame = frames.get_depth_frame();
    if (FLAGS_publish_depth_image) {
      OnDepthImage(depth_frame);
    }

    if (FLAGS_publish_point_cloud) {
      OnPointCloud(depth_frame);
    }

    if (FLAGS_publish_realsense_acc) {
      rs2::motion_frame accel_frame = frames.first_or_default(RS2_STREAM_ACCEL);
      OnAcc(accel_frame);
    }

    if (FLAGS_publish_realsense_gyro) {
      rs2::motion_frame gyro_frame = frames.first_or_default(RS2_STREAM_GYRO);
      OnGyro(gyro_frame);
    }
  }
}

void D435I::OnColorImage(const rs2::frame &color_frame) {
  AINFO << "COLOR FRAME NUMBER:" << color_frame.get_frame_number();
  auto mat = frame_to_mat(color_frame);

  if (FLAGS_publish_color_image) {
    auto image_proto = std::make_shared<Image>();
    image_proto->set_frame_no(color_frame.get_frame_number());
    image_proto->set_height(mat.rows);
    image_proto->set_width(mat.cols);
    image_proto->set_encoding(
        rs2_format_to_string(color_frame.get_profile().format()));

    image_proto->set_measurement_time(Time::Now().ToSecond());
    auto m_size = mat.rows * mat.cols * mat.elemSize();
    image_proto->set_data(mat.data, m_size);
    color_image_writer_->Write(image_proto);
  }

  if (FLAGS_publish_compressed_color_image) {
    OnCompressedImage(color_frame, mat);
  }
}

void D435I::OnDepthImage(const rs2::frame &f) {
  AINFO << "DEPTH FRAME NUMBER:" << f.get_frame_number();
  auto mat = frame_to_mat(f);

  if (FLAGS_publish_depth_image) {
    auto image_proto = std::make_shared<Image>();
    image_proto->set_frame_no(f.get_frame_number());
    image_proto->set_height(mat.rows);
    image_proto->set_width(mat.cols);
    // encoding 16-bit linear depth values.
    // The depth is meters is equal to depth scale * pixel value.
    image_proto->set_encoding(rs2_format_to_string(f.get_profile().format()));

    image_proto->set_measurement_time(Time::Now().ToSecond());
    auto m_size = mat.rows * mat.cols * mat.elemSize();
    image_proto->set_data(mat.data, m_size);
    depth_image_writer_->Write(image_proto);
  }
}

void D435I::OnPointCloud(rs2::frame depth_frame) {
  // Post processing
  rs2::threshold_filter thr_filter(
      static_cast<float>(FLAGS_point_cloud_min_distance),
      static_cast<float>(FLAGS_point_cloud_max_distance));
  depth_frame = thr_filter.process(depth_frame);

  rs2::temporal_filter temp_filter(static_cast<float>(FLAGS_temp_filter_alpha),
                                   static_cast<float>(FLAGS_temp_filter_delta),
                                   0);
  depth_frame = temp_filter.process(depth_frame);

  filtered_data_.enqueue(depth_frame);
}

void D435I::PublishPointCloud() {
  while (!stop_) {
    // Declare pointcloud object, for calculating pointclouds
    rs2::pointcloud pc;
    // We want the points object to be persistent so we can display the last
    // cloud when a frame drops
    rs2::points points;

    rs2::frame depth_frame;
    filtered_data_.poll_for_frame(&depth_frame);
    if (!depth_frame) {
      AINFO << "POINT CLOUD FRAME QUEUE IS EMPTY, WAIT FOR ENQUEUE;";
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }
    // Generate the pointcloud and texture mappings
    points = pc.calculate(depth_frame);

    auto t1 = Time::Now().ToSecond();
    auto cloud_ = points_to_pcl(points);
    auto t2 = Time::Now().ToSecond();
    AWARN << "Time for realsense point to point cloud:" << t2 - t1;

    // Point Cloud save
    pcl_ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_out->clear();

    if (FLAGS_pcl_visualization) {
      // PCL VISUALIZATION
      std::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
          new pcl::visualization::PCLVisualizer("3D Viewer"));
      viewer->setBackgroundColor(0, 0, 0);
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
          target_color(cloud_, 255, 0, 0);
      viewer->addPointCloud<pcl::PointXYZ>(cloud_, target_color,
                                           "target cloud");
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

    std::shared_ptr<PointCloud> point_cloud_out =
        point_cloud_pool_->GetObject();
    if (point_cloud_out == nullptr) {
      AWARN << "Point cloud pool return nullptr, will be create new.";
      point_cloud_out = std::make_shared<PointCloud>();
      point_cloud_out->mutable_point()->Reserve(point_size_);
    }
    if (point_cloud_out == nullptr) {
      AWARN << "Point cloud out is nullptr";
      return;
    }
    point_cloud_out->Clear();
    point_cloud_out->set_is_dense(false);
    point_cloud_out->set_measurement_time(Time::Now().ToSecond());
    int counter = 0;
    AWARN << "POINT CLOUD SIZE :" << (*cloud_).size();
    for (size_t i = 0; i < (*cloud_).size(); i++) {
      if ((*cloud_)[i].z) {
        ::Eigen::Vector4d pos((*cloud_)[i].x, (*cloud_)[i].y, (*cloud_)[i].z,
                              1);
        auto pos_trans = transform * pos;
        if ((pos_trans[1] > -0.05) && (pos_trans[1] < 0.1) &&
            (pos_trans[2] < 2)) {
          if ((counter % 40) == 0) {
            apollo::sensors::PointXYZIT *p = point_cloud_out->add_point();
            if (FLAGS_save_pcd) {
              cloud_out->push_back(
                  pcl::PointXYZ(static_cast<float>(pos_trans[0]),
                                static_cast<float>(pos_trans[1]),
                                static_cast<float>(pos_trans[2])));
            }
            p->set_x(static_cast<float>(pos_trans[0]));
            p->set_y(static_cast<float>(pos_trans[1]));
            p->set_z(static_cast<float>(pos_trans[2]));
          }
          counter++;
        }
      }
    }
    point_cloud_out->set_width(point_cloud_out->point_size());
    point_cloud_out->set_height(1);

    auto tt = Time::Now().ToSecond();
    AINFO << "Time for point cloud from collect to publish :" << tt - t1;

    if (FLAGS_save_pcd && cloud_out->points.size() > 0) {
      pcl::io::savePCDFile("/apollo/data/" + std::to_string(t1) + ".pcd",
                           *cloud_out);
    }

    point_cloud_writer_->Write(point_cloud_out);
  }
}

D435I::~D435I() {
  AINFO << "Destructor from D435I";

  if (!stop_.load()) {
    stop_.exchange(true);
    realsense_t1.join();
    realsense_t2.join();
  }
}

}  // namespace realsense
}  // namespace sensors
}  // namespace apollo
