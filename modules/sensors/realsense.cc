#include <chrono>
#include <iomanip>
#include <iostream>
#include <librealsense2/rs.hpp>
#include <mutex>
#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "cyber/common/log.h"
#include "cyber/cyber.h"
#include "cyber/time/rate.h"
#include "cyber/time/time.h"
#include "modules/sensors/proto/sensors.pb.h"

using apollo::cyber::Rate;
using apollo::cyber::Time;
using apollo::sensors::Image;

int main(int argc, char* argv[]) try {
  // Declare RealSense pipeline, encapsulating the actual device and sensors
  rs2::pipeline pipe;
  // Create a configuration for configuring the pipeline with a non default
  // profile
  rs2::config cfg;
  cv::Mat intrinsicsL;
  cv::Mat distCoeffsL;
  cv::Mat map1;
  cv::Mat map2;
  // Add pose stream
  cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
  // Enable both image streams
  // Note: It is not currently possible to enable only one
  cfg.enable_stream(RS2_STREAM_FISHEYE, 1, RS2_FORMAT_Y8);
  cfg.enable_stream(RS2_STREAM_FISHEYE, 2, RS2_FORMAT_Y8);

  // Define frame callback

  // The callback is executed on a sensor thread and can be called
  // simultaneously from multiple sensors Therefore any modification to common
  // memory should be done under lock
  std::mutex data_mutex;
  uint64_t pose_counter = 0;
  uint64_t frame_counter = 0;
  bool first_data = true;
  auto last_print = std::chrono::system_clock::now();
  auto callback = [&](const rs2::frame& frame) {
    std::lock_guard<std::mutex> lock(data_mutex);
    // Only start measuring time elapsed once we have received the
    // first piece of data
    if (first_data) {
      first_data = false;
      last_print = std::chrono::system_clock::now();
    }

    if (auto fp = frame.as<rs2::pose_frame>()) {
      rs2_pose posdata = fp.get_pose_data();
      pose_counter++;
    } else if (auto fs = frame.as<rs2::frameset>()) {
      cv::Mat image(cv::Size(fs.get_fisheye_frame(1).get_width(),
                             fs.get_fisheye_frame(1).get_height()),
                    CV_8U, (void*)fs.get_fisheye_frame(1).get_data(),
                    cv::Mat::AUTO_STEP);
      cv::Mat dst;
      /*
      cv::fisheye::undistortImage(
              image,
              dst,
              intrinsicsL,
              distCoeffsL
        );*/
      cv::remap(image, dst, map1, map2, cv::INTER_LINEAR);
      /*------------- write image channel ------------------*/
      apollo::cyber::Init(argv[0]);

      auto image_node = apollo::cyber::CreateNode("realsense");
      auto image_writer =
          image_node->CreateWriter<Image>("/realsense/raw_image");
      Rate rate(10.0);

      while (apollo::cyber::OK()) {
        auto image_proto = std::make_shared<Image>();
        image_proto->set_frame_id(frame_counter);
        image_proto->set_measurement_time(Time::Now().ToSecond());
        auto m_size = dst.rows * dst.cols * dst.elemSize();
        image_proto->set_data(dst.data, m_size);
        image_writer->Write(image_proto);
        rate.Sleep();
      }

      frame_counter++;
    }

    // Print the approximate pose and image rates once per second
    auto now = std::chrono::system_clock::now();
    if (now - last_print >= std::chrono::seconds(1)) {
      AINFO << "\r" << std::setprecision(0) << std::fixed
            << "Pose rate: " << pose_counter << " "
            << "Image rate: " << frame_counter;
      pose_counter = 0;
      frame_counter = 0;
      last_print = now;
    }
  };

  // Start streaming through the callback
  rs2::pipeline_profile profiles = pipe.start(cfg, callback);
  rs2::stream_profile fisheye_stream =
      profiles.get_stream(RS2_STREAM_FISHEYE, 1);
  rs2_intrinsics intrinsicsleft =
      fisheye_stream.as<rs2::video_stream_profile>().get_intrinsics();
  intrinsicsL =
      (cv::Mat_<double>(3, 3) << intrinsicsleft.fx, 0, intrinsicsleft.ppx, 0,
       intrinsicsleft.fy, intrinsicsleft.ppy, 0, 0, 1);
  distCoeffsL = cv::Mat(1, 4, CV_32F, intrinsicsleft.coeffs);
  cv::Mat R = cv::Mat::eye(3, 3, CV_32F);
  cv::Mat P =
      (cv::Mat_<double>(3, 4) << intrinsicsleft.fx, 0, intrinsicsleft.ppx, 0, 0,
       intrinsicsleft.fy, intrinsicsleft.ppy, 0, 0, 0, 1, 0);

  cv::fisheye::initUndistortRectifyMap(
      intrinsicsL, distCoeffsL, R, P, cv::Size(848, 816), CV_16SC2, map1, map2);

  // Sleep this thread until we are done
  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  return EXIT_SUCCESS;
} catch (const rs2::error& e) {
  std::cerr << "RealSense error calling " << e.get_failed_function() << "("
            << e.get_failed_args() << "):\n    " << e.what() << std::endl;
  return EXIT_FAILURE;
} catch (const std::exception& e) {
  std::cerr << e.what() << std::endl;
  return EXIT_FAILURE;
}