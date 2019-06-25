#include <cstring>
#include <iomanip>
#include <iostream>
#include <librealsense2/rs.hpp>
#include <string>
#include <thread>

rs2::device get_device(const std::string& serial_number = "") {
  rs2::context ctx;
  while (true) {
    for (auto&& dev : ctx.query_devices()) {
      if (((serial_number.empty() &&
            std::strstr(dev.get_info(RS2_CAMERA_INFO_NAME), "T265")) ||
           std::strcmp(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER),
                       serial_number.c_str()) == 0))
        return dev;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

int main() {
  // get the device given the serial number
  std::cout << "Waiting for device..." << std::endl;
  std::string serial_number = "908412111198";  // serial number or empty
  auto device = get_device(serial_number);

  std::cout << "Device with serial number "
            << device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << " got"
            << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(1));

  // open the profiles you need, or all of them
  auto sensor = device.first<rs2::sensor>();
  sensor.open(sensor.get_stream_profiles());

  // start the sensor providing a callback to get the frame
  sensor.start([](rs2::frame f) {
    if (f.get_profile().stream_type() == RS2_STREAM_POSE) {
      auto pose_frame = f.as<rs2::pose_frame>();
      auto pose_data = pose_frame.get_pose_data();
      std::cout << "pose " << pose_data.translation << std::endl;
    } else if (f.get_profile().stream_type() == RS2_STREAM_FISHEYE) {
      // this is one of the fisheye imagers
      auto fisheye_frame = f.as<rs2::video_frame>();
      std::cout << "fisheye " << f.get_profile().stream_index() << ", "
                << fisheye_frame.get_width() << "x"
                << fisheye_frame.get_height() << std::endl;
    }
  });

  std::this_thread::sleep_for(std::chrono::seconds(10));

  // and stop
  sensor.stop();
}