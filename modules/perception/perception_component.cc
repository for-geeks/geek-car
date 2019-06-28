#include "modules/perception/perception_component.h"

namespace apollo {
namespace perception {
bool PerceptionComponent::Init() {
  // maybe some config here

  return true;
}

bool PerceptionComponent::Proc(const std::shared<Image>& image) {
  // print image fields like defied in modules/sensors/proto/sensors.proto

  //   message Image {
  //     optional uint64 frame_no = 2;
  //     optional double measurement_time = 3;

  //     optional uint32 height = 4;  // image height, that is, number of rows
  //     optional uint32 width = 5;   // image width, that is, number of columns

  //     optional string encoding = 6;
  //     optional uint32 step = 7;  // Full row length in bytes
  //     optional bytes data = 8;   // actual matrix data, size is (step * rows)
  //   }
  ADEBUG << image->DebugString();
}

}  // namespace perception
}  // namespace apollo
