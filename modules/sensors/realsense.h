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
 * The RealSense class provides several functions for common usages of thesensor
 * API
 */
class RealSense {
 public:
  static rs2::device getDevice();

  static void printDeviceInformation(const rs2::device& dev);

  static std::string getDeviceName(const rs2::device& dev);

  static std::string getSensorName(const rs2::sensor& sensor);

  static rs2::sensor getSensorFromDevice(const rs2::device& dev);

  static rs2_option getSensorOption(const rs2::sensor& sensor);

  static float getDepthUnits(const rs2::sensor& sensor);

  static void getFieldOfView(const rs2::stream_profile& stream);

  static void getExtrinsics(const rs2::stream_profile& from_stream,
                            const rs2::stream_profile& to_stream);

  static void changeSensorOption(const rs2::sensor& sensor,
                                 rs2_option option_type);

  static rs2::stream_profile chooseStreamingProfile(const rs2::sensor& sensor);

  static void startStreamingProfile(const rs2::sensor& sensor,
                                    const rs2::stream_profile& stream_profile);
};