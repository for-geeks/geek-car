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
#include <chrono>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <thread>

#include <fstream>
#include <vector>

#include "librealsense2/rs.hpp"
#include "librealsense2/rsutil.h"

/**
 * @brief  RealSense T265 Map TEST
 *
 * @return double
 */
double timestamp() {
  auto now = std::chrono::system_clock::now().time_since_epoch();
  return std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
}

void s_sleep(const int x, std::string info = "") {
  std::cout << info << " - "
            << "s_sleep for " << std::setprecision(5) << float(x) / 1000
            << " seconds" << std::endl;
  std::this_thread::sleep_for(std::chrono::milliseconds(x));
}

void bin_file_from_bytes(const std::string& filename,
                         const std::vector<uint8_t>& bytes) {
  std::ofstream file(filename, std::ios::binary | std::ios::trunc);
  if (!file.good())
    // throw std::runtime_error(to_string() << "Invalid binary file specified "
    // << filename << " verify the target path and location permissions");
    std::cerr << "Invalid binary file specified " << filename
              << " verify the target path and location permissions"
              << std::endl;
  file.write((char*)bytes.data(), bytes.size());
}

std::vector<uint8_t> bytes_from_bin_file(const std::string& filename) {
  std::ifstream file(filename.c_str(), std::ios::binary);
  if (!file.good())
    std::cerr << "Invalid binary file specified " << filename
              << " verify the source path and location permissions"
              << std::endl;
  // throw std::runtime_error(to_string() << "Invalid binary file specified " <<
  // filename << " verify the source path and location permissions");

  // Determine the file length
  file.seekg(0, std::ios_base::end);
  std::size_t size = file.tellg();
  if (!size)
    std::cerr << "Invalid binary file " << filename << " provided  - zero-size "
              << std::endl;
  // throw std::runtime_error(to_string() << "Invalid binary file " << filename
  // <<  " provided  - zero-size ");
  file.seekg(0, std::ios_base::beg);

  // Create a vector to store the data
  std::vector<uint8_t> v(size);

  // Load the data
  file.read((char*)&v[0], size);

  return v;
}

int main(int argc, char* argv[]) {
  std::cout << "Realsense api: " << RS2_API_VERSION_STR << std::endl;

  rs2::context ctx;
  rs2::pipeline pipe(ctx);

  rs2::sensor tracking_sensor;
  rs2::stream_profile pose_stream;

  rs2_vector offset;
  rs2_quaternion offsetQ;

  for (auto& sensor : ctx.query_all_sensors()) {
    std::cout << sensor.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
    if (std::string(sensor.get_info(RS2_CAMERA_INFO_NAME)) ==
        "Tracking Module") {
      tracking_sensor = sensor;
      for (auto& stream_profile : sensor.get_stream_profiles()) {
        std::cout << "\t" << stream_profile.stream_name() << std::endl;
        if (stream_profile.stream_name() == "Pose") {
          pose_stream = stream_profile;
        }
      }
    }
  }
  // exit(0);

  auto callback = [&](const rs2::frame& frame) {
    if (rs2::pose_frame fp = frame.as<rs2::pose_frame>()) {
      rs2_pose pose_data = fp.get_pose_data();
      std::cout << "\r \t \t \t \t \t \t \t" << std::flush;
      std::cout << "\r" << std::setprecision(3) << pose_data.translation.x * 100
                << ", " << pose_data.translation.z * 100 << ", "
                << pose_data.translation.z * 100;
      std::cout << " - " << pose_data.tracker_confidence << ", "
                << pose_data.mapper_confidence << std::flush;
    }
  };

  std::vector<uint8_t> map;

  try {
    tracking_sensor.open(pose_stream);

    tracking_sensor.start(callback);
    s_sleep(1000 * 20, "tracking_sensor.start(callback);");

    while (!tracking_sensor.as<rs2::pose_sensor>().set_static_node(
        "origin", rs2_vector({0, 0, 0}), rs2_quaternion({0, 0, 0, 1}))) {
      s_sleep(100, "tracking_sensor.set_static_node() failed");
    }
    s_sleep(100, "tracking_sensor.set_static_node() success");

    tracking_sensor.stop();
    s_sleep(1000 * 5, "tracking_sensor.stop();");
    map = tracking_sensor.as<rs2::pose_sensor>().export_localization_map();
    s_sleep(1000, "export_localization_map()");

    std::string filename = "data.dat";
    // std::string filename = "/home/mmoyes/Desktop/test.raw";
    // bin_file_from_bytes(filename, map);

    map = bytes_from_bin_file(filename);

    bool importBool =
        tracking_sensor.as<rs2::pose_sensor>().import_localization_map(map);
    std::cout << "importBool: " << importBool << std::endl;
    s_sleep(2000, "import_localization_map(map)");
    tracking_sensor.start(callback);

    while (!tracking_sensor.as<rs2::pose_sensor>().get_static_node(
        "origin", offset, offsetQ)) {
      s_sleep(100, "get_static_node");
    }

    s_sleep(1000 * 20, "running a bit");
    tracking_sensor.stop();
    tracking_sensor.close();
    std::cout << "Tracking stopped and closed" << std::endl;

    std::cout << "Origin: " << offset.x << ", " << offset.y << ", " << offset.z
              << std::endl;

  } catch (const rs2::error& e) {
    std::cerr << "RealSense error calling " << e.get_failed_function() << "("
              << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    // return EXIT_FAILURE;
  } catch (const std::exception& e) {
    std::cerr << "EXCEPTION: " << e.what() << std::endl;
    // return EXIT_FAILURE;
  }

  exit(EXIT_SUCCESS);
}
