#include "nlink_linktrack_anchor_frame0.h"

AnchorFrame0Data anchorFrame0Data_;
static const int kFrameLength = 896;
void unpackAnchorFrame0Data(uint8_t *byteArray) {
  //    assert(sizeof(anchorFrame0Data_.frame) == kFrameLength);
  memcpy(&anchorFrame0Data_.frame, byteArray, kFrameLength);

  for (size_t i = 0, count = sizeof(anchorFrame0Data_.tag) /
                             sizeof(anchorFrame0Data_.tag[0]);
       i < count; ++i) {
    anchorFrame0Data_.tag[i].id = anchorFrame0Data_.frame.tagTemp[i].id;
    int24ToFloat(anchorFrame0Data_.frame.tagTemp[i].posTemp,
                 anchorFrame0Data_.tag[i].pos, 3, kPosMultiply_);

    for (size_t j = 0, count = sizeof(anchorFrame0Data_.tag[0].group) /
                               sizeof(anchorFrame0Data_.tag[0].group[0]);
         j < count; ++j) {
      anchorFrame0Data_.tag[i].group[j].id =
          anchorFrame0Data_.frame.tagTemp[i].group[j].id;
      anchorFrame0Data_.tag[i].group[j].dis =
          static_cast<float>(
              int24Value(anchorFrame0Data_.frame.tagTemp[i].group[j].disTemp)) /
          kDisMultiply_;
    }
  }

  anchorFrame0Data_.supplyVoltage =
      anchorFrame0Data_.frame.supplyVoltageTemp / kVoltageMultiply_;

  //    AnchorFrame0Data data = anchorFrame0Data_;
}

uint8_t verifyAnchorFrame0Data(uint8_t *byteArray) {
  if (byteArray[0] != 0x55 || byteArray[1] != 0x00 ||
      byteArray[kFrameLength - 1] != 0xeeu)
    return 0;
  return 1;
}
