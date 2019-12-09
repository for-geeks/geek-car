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
#include "modules/sensors/realsense.h"

#include "cyber/cyber.h"

namespace apollo {
namespace sensors {

rs2::device first_connected_device() {
  rs2::context ctx;
  auto list = ctx.query_devices();
  // Get a snapshot of currently connected devices
  if (list.size() == 0) {
    AWARN << "No device detected. Is it plugged in?";
  }

  return list.front();
}
// Convert rs2::frame to cv::Mat
cv::Mat frame_to_mat(rs2::frame f) {
  auto vf = f.as<rs2::video_frame>();
  const int w = vf.get_width();
  const int h = vf.get_height();

  if (f.get_profile().format() == RS2_FORMAT_BGR8) {
    return cv::Mat(cv::Size(w, h), CV_8UC3, const_cast<void*>(f.get_data()),
                   cv::Mat::AUTO_STEP);
  } else if (f.get_profile().format() == RS2_FORMAT_RGB8) {
    auto r = cv::Mat(cv::Size(w, h), CV_8UC3, const_cast<void*>(f.get_data()),
                     cv::Mat::AUTO_STEP);
    cv::cvtColor(r, r, cv::COLOR_RGB2BGR);
    return r;
  } else if (f.get_profile().format() == RS2_FORMAT_Z16) {
    return cv::Mat(cv::Size(w, h), CV_16UC1, const_cast<void*>(f.get_data()),
                   cv::Mat::AUTO_STEP);
  } else if (f.get_profile().format() == RS2_FORMAT_Y8) {
    return cv::Mat(cv::Size(w, h), CV_8UC1, const_cast<void*>(f.get_data()),
                   cv::Mat::AUTO_STEP);
  } else if (f.get_profile().format() == RS2_FORMAT_DISPARITY32) {
    return cv::Mat(cv::Size(w, h), CV_32FC1, const_cast<void*>(f.get_data()),
                   cv::Mat::AUTO_STEP);
  }

  cv::Mat empty;

  AWARN << "Frame format is not supported yet!";
  return empty;
}

// Converts depth frame to a matrix of doubles with distances in meters
cv::Mat depth_frame_to_meters(const rs2::pipeline& pipe,
                              const rs2::depth_frame& f) {
  cv::Mat dm = frame_to_mat(f);
  dm.convertTo(dm, CV_64F);
  auto depth_scale = pipe.get_active_profile()
                         .get_device()
                         .first<rs2::depth_sensor>()
                         .get_depth_scale();
  dm = dm * depth_scale;
  return dm;
}

pcl_ptr points_to_pcl(const rs2::points& points) {
  pcl_ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  auto sp = points.get_profile().as<rs2::video_stream_profile>();
  cloud->width = sp.width();
  cloud->height = sp.height();
  cloud->is_dense = false;
  cloud->points.resize(points.size());
  auto ptr = points.get_vertices();
  for (auto& p : cloud->points) {
    p.x = ptr->x;
    p.y = ptr->y;
    p.z = ptr->z;
    ptr++;
  }

  return cloud;
}
}  // namespace sensors
}  // namespace apollo
