#pragma once

#include <iomanip>
#include <iostream>
#include <librealsense2/rs.hpp>
#include <map>
#include <utility>
#include <vector>

// 

/**
 * https://github.com/IntelRealSense/librealsense/blob/master/examples/sensor-control/api_how_to.h
 * The RealSense class provides several functions for common usages of thesensor API
*/
class RealSense {
 public:
  static rs2::device get_device();

  static void print_device_information(const rs2::device& dev);

  static std::string get_device_name(const rs2::device& dev);

  static std::string get_sensor_name(const rs2::sensor& sensor);

  static rs2::sensor get_a_sensor_from_a_device(const rs2::device& dev);

  static rs2_option get_sensor_option(const rs2::sensor& sensor);

  static float get_depth_units(const rs2::sensor& sensor);

  static void get_field_of_view(const rs2::stream_profile& stream);

  static void get_extrinsics(const rs2::stream_profile& from_stream,
                             const rs2::stream_profile& to_stream);

  static void change_sensor_option(const rs2::sensor& sensor,
                                   rs2_option option_type);

  static rs2::stream_profile choose_a_streaming_profile(
      const rs2::sensor& sensor);

  static void start_streaming_a_profile(
      const rs2::sensor& sensor, const rs2::stream_profile& stream_profile);
};