#include <string.h>
#include "ncommon.h"

const float kVoltageMultiply_ = 1000.0f;
const float kPosMultiply_ = 1000.0f;
const float kDisMultiply_ = 1000.0f;
const float kVelMultiply_ = 10000.0f;
const float kAngleMultiply_ = 100.0f;
const float kRssiMultiply_ = -2.0f;
const float kEopMultiply_ = 100.0f;

int stringToHex(char *str, uint8_t *out) {
  int outLength = 0;
  int cnt = 0;
  uint8_t high = 0, low = 0;
  char current = 0;
  uint8_t value = 0;
  uint8_t isHighValid = 0;
  while ((current = str[cnt])) {
    ++cnt;
    if (current >= '0' && current <= '9') {
      value = static_cast<uint8_t>(current - '0');
    } else if (current >= 'a' && current <= 'f') {
      value = static_cast<uint8_t>(current - 'a' + 10);
    } else if (current >= 'A' && current <= 'F') {
      value = static_cast<uint8_t>(current - 'A' + 10);
    } else {
      continue;
    }

    if (!isHighValid) {
      high = value;

      isHighValid = 1;
    } else {
      low = value;

      out[outLength] = static_cast<uint8_t>(high << 4) | low;
      ++outLength;
      isHighValid = 0;
    }
  }

  return outLength;
}

int32_t int24Value(NInt24 data) {
  uint8_t *byte = (uint8_t *)(&data);
  return (int32_t)(byte[0] << 8 | byte[1] << 16 | byte[2] << 24) / 256;
}

void int24ToFloat(NInt24 src[], float dst[], int arrayLength, float multiply) {
  for (int i = 0; i < arrayLength; ++i) {
    dst[i] = static_cast<float>(int24Value(src[i])) / multiply;
  }
}

void int16ToFloat(int16_t src[], float dst[], int arrayLength, float multiply) {
  for (int i = 0; i < arrayLength; ++i) {
    dst[i] = src[i] / multiply;
  }
}

uint8_t verifyCheckSum(uint8_t *data, int32_t length) {
  uint8_t sum = 0;
  for (int32_t i = 0; i < length - 1; ++i) {
    sum = static_cast<uint8_t>(sum + data[i]);
  }
  return sum == data[length - 1];
}

void uint8ToFloat(uint8_t src[], float dst[], int arrayLength, float multiply) {
  for (int i = 0; i < arrayLength; ++i) {
    dst[i] = src[i] / multiply;
  }
}
