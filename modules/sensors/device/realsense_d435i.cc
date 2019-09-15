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

#include "modules/sensors/device/realsense_d435i.h"

#include <memory>

bool D435I::Init() {
  // Add desired streams to configuration
  cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
  // Use a configuration object to request only depth from the pipeline
  cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
  // Add streams of gyro and accelerometer to configuration
  cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
  cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);

  // Instruct pipeline to start streaming with the requested configuration
  pipe.start(cfg);
  return true;
}

void D435I::OnDepthImage(cv::Mat mat, uint64 frame_no) {
  if (FLAGS_publish_depth_image) {
    auto image_proto = std::make_shared<Image>();
    image_proto->set_frame_no(frame_no);
    image_proto->set_height(mat.rows);
    image_proto->set_width(mat.cols);
    // encoding /**< 16-bit linear depth values. The depth is meters is equal to
    // depth scale * pixel value. */
    image_proto->set_encoding(rs2_format_to_string(RS2_FORMAT_Z16));

    image_proto->set_measurement_time(Time::Now().ToSecond());
    auto m_size = mat.rows * mat.cols * mat.elemSize();
    image_proto->set_data(mat.data, m_size);
    image_writer_->Write(image_proto);
  }
}

void D435I::OnPointCloud(rs2::frame f) {
  auto depth_frame = f.as<rs2::depth_frame>();
  // Declare pointcloud object, for calculating pointclouds and texture mappings
  rs2::pointcloud pc;
  // We want the points object to be persistent so we can display the last cloud
  // when a frame drops
  rs2::points points;

  // auto depth = f.get_depth_frame();

  // Generate the pointcloud and texture mappings
  points = pc.calculate(depth_frame);

  auto pcl_points = points_to_pcl(points);

  pcl_ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(pcl_points);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.0, 1.0);
  pass.filter(*cloud_filtered);

  // Apply an affine transform defined by an Eigen Transform.
  // pcl::transformPointCloud(*source_cloud, *transformed_cloud, transform_1);

  std::vector<pcl_ptr> layers;
  layers.push_back(pcl_points);
  layers.push_back(cloud_filtered);

#if 0
  auto sp = points.get_profile().as<rs2::video_stream_profile>();

  apollo::sensors::PointCloud point_cloud_proto;
  point_cloud_proto->set_frame_id(f.get_frame_number());
  point_cloud_proto->set_is_dense(false);
  point_cloud_proto->set_measurement_time(Time::Now().ToSecond());
  point_cloud_proto->set_width(sp.width());
  point_cloud_proto->set_height(sp.height());

  for (int i = 0; i < points.size(); i++) {
    apollo::sensor::Point p;

    p->set_x(nan);
    p->set_y(nan);
    p->set_z(nan);
    p->set_timestamp(timestamp);
    p->set_intensity(0);

    auto next_point = point_cloud_proto->add_point();
    next_point->CopyFrom(p);
  }

  point_cloud_writer_->Write(point_cloud_proto);
#endif
}
