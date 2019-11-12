#include "nlink_linktrack_tag_frame0.h"
#include <assert.h>
#include <string.h>

TagFrame0Data tagFrame0Data_;
static const int kFrameLength = 128;
void unpackTagFrame0Data(uint8_t *byteArray) {
  //    assert(sizeof(tagFrame0Data_.frame) == kFrameLength);
  memcpy(&tagFrame0Data_.frame, byteArray, kFrameLength);

  int24ToFloat(tagFrame0Data_.frame.posTemp, tagFrame0Data_.pos,
               static_cast<int>(sizeof(tagFrame0Data_.pos) /
                                sizeof(tagFrame0Data_.pos[0])),
               kPosMultiply_);

  int24ToFloat(tagFrame0Data_.frame.velTemp, tagFrame0Data_.vel,
               static_cast<int>(sizeof(tagFrame0Data_.vel) /
                                sizeof(tagFrame0Data_.vel[0])),
               kVelMultiply_);

  int24ToFloat(tagFrame0Data_.frame.disTemp, tagFrame0Data_.dis,
               static_cast<int>(sizeof(tagFrame0Data_.dis) /
                                sizeof(tagFrame0Data_.dis[0])),
               kDisMultiply_);

  int16ToFloat(tagFrame0Data_.frame.angleTemp, tagFrame0Data_.angle,
               static_cast<int>(sizeof(tagFrame0Data_.angle) /
                                sizeof(tagFrame0Data_.angle[0])),
               kAngleMultiply_);
  uint8ToFloat(tagFrame0Data_.frame.eopTemp, tagFrame0Data_.eop,
               static_cast<int>(sizeof(tagFrame0Data_.eop) /
                                sizeof(tagFrame0Data_.eop[0])),
               kEopMultiply_);
  tagFrame0Data_.supplyVoltage =
      tagFrame0Data_.frame.supplyVoltageTemp / kVoltageMultiply_;

  //      TagFrame0Data data = tagFrame0Data_;
}

uint8_t verifyTagFrame0Data(uint8_t *byteArray) {
  if (byteArray[0] != 0x55 || byteArray[1] != 0x01) return 0;
  return verifyCheckSum(byteArray, kFrameLength);
}
