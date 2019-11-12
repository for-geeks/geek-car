#ifndef NLINK_LINKTRACK_TAG_FRAME0_H
#define NLINK_LINKTRACK_TAG_FRAME0_H

#include "ncommon.h"

// typedef void (*Method)();

#pragma pack(1)

typedef struct {
  uint8_t header[2];  // 55 01
  uint8_t id;
  uint8_t reserved0[1];
  NInt24 posTemp[3];
  NInt24 velTemp[3];
  NInt24 disTemp[8];
  float gyro[3];
  float acc[3];
  uint8_t reserved1[12];
  int16_t angleTemp[3];
  float q[4];
  uint8_t reserved2[8];
  uint32_t systemTime;
  struct {
    uint8_t : 4;
    uint8_t accWorkNormal : 1;
    uint8_t accHardwareIsOnline : 1;
    uint8_t gyroWorkNormal : 1;
    uint8_t gyroHardwareIsOnline : 1;
  } sensorStatus;
  uint8_t eopTemp[3];
  uint16_t supplyVoltageTemp;
  uint8_t reserved3[5];
  uint8_t checkSum;
} NLink_LinkTrack_Tag_Frame0;
#pragma pack()

typedef struct {
  NLink_LinkTrack_Tag_Frame0 frame;
  //协议结构体中部分变量需要经过二次转化，如pos，vel，dis，angle
  float pos[3];
  float eop[3];  //位置估计精度
  float vel[3];
  float dis[8];
  float angle[3];

  float supplyVoltage;

} TagFrame0Data;

extern TagFrame0Data tagFrame0Data_;

//解析传入的 byteArray(已校验)
void unpackTagFrame0Data(uint8_t *byteArray);

//校验传入的 byteArray
uint8_t verifyTagFrame0Data(uint8_t *byteArray);

#endif  // NLINK_LINKTRACK_TAG_FRAME0_H
