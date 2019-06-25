#include <iomanip>
#include <iostream>
#include <librealsense2/rs.hpp>
#include <string>
#include <thread>

/*
 * T265 connect check tool
 */

void print_info(rs2::device& dev);

int main() {
  rs2::device dev = [] {
    rs2::context ctx;
    std::cout << "waiting for device..." << std::endl;
    while (true) {
      for (auto&& dev : ctx.query_devices()) return dev;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }();
  print_info(dev);

  rs2::pipeline pipe;
  rs2::config cfg;
  std::string serial_number = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
  std::cout << "opening pipeline for " << serial_number << std::endl;
  cfg.enable_device(serial_number);
  cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
  if (!pipe.start(cfg)) return 1;

  while (true) {
    auto frames = pipe.wait_for_frames();
    auto f = frames.first_or_default(RS2_STREAM_POSE);
    auto pose_data = f.as<rs2::pose_frame>().get_pose_data();
    std::cout << "\r"
              << "Device Position: " << std::setprecision(3) << std::fixed
              << pose_data.translation.x << " " << pose_data.translation.y
              << " " << pose_data.translation.z << " (meters)";
  }
}

void print_info(rs2::device& dev) {
  std::cout << "device found:" << std::endl;
  std::cout << dev.get_info(RS2_CAMERA_INFO_NAME) << " "
            << dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << " "
            << dev.get_info(RS2_CAMERA_INFO_PRODUCT_ID) << std::endl;

  auto sensors = dev.query_sensors();
  for (rs2::sensor& sensor : sensors) {
    std::cout << "sensor " << sensor.get_info(RS2_CAMERA_INFO_NAME)
              << std::endl;
    for (rs2::stream_profile& profile : sensor.get_stream_profiles()) {
      std::cout << "  stream " << profile.stream_name() << std::endl;
    }
  }
}