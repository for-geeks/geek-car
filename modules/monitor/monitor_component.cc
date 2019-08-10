#include "modules/monitor/monitor_component.h"

#include "librealsense2/rs.hpp"

namespace apollo {
namespace monitor {

rs2::device get_device(const std::string &serial_number = "") {
  rs2::context ctx;
  while (true) {
    for (auto &&dev : ctx.query_devices()) {
      if (((serial_number.empty() &&
            std::strstr(dev.get_info(RS2_CAMERA_INFO_NAME),
                        FLAGS_device_model.c_str())) ||
           std::strcmp(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER),
                       serial_number.c_str()) == 0))
        return dev;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void MonitorComponent::Init() {}

void MonitorComponent::Action() { rs2::device device = get_device(); }

} // namespace monitor
} // namespace apollo
