#ifndef NLINK_LINKTRACK_NODE_FRAME2_H
#define NLINK_LINKTRACK_NODE_FRAME2_H

#include "ncommon.h"
#pragma pack(1)
typedef struct {
  uint8_t role;
  uint8_t id;
  NInt24 disTemp;
  uint8_t fpRssiTemp;
  uint8_t rxRssiTemp;
  uint32_t systemTime;
  uint8_t reserved[2];
} RawNode2_t;

typedef struct {
  uint8_t header[2];  // 55 04
  uint16_t frameLength;
  uint8_t role;
  uint8_t id;
  uint32_t systemTime;
  uint8_t eopTemp[3];
  NInt24 posTemp[3];
  NInt24 velTemp[3];
  uint8_t reserved1[9];
  float imuGyro[3];
  float imuAcc[3];
  uint8_t reserved2[12];
  int16_t angleTemp[3];
  float q[4];
  uint8_t reserved3[18];
  uint16_t supplyVoltageTemp;
  uint8_t validNodeQuantity;
  //... nodes
  //        uint8_t checkSum;
} NLink_LinkTrack_Node_Frame2_Part;
#pragma pack()

typedef struct {
  uint8_t role;
  uint8_t id;
  float dis;
  float fpRssi;
  float rxRssi;
  uint32_t systemTime;
} Node2_t;

typedef struct {
  //由于是变长协议，这里仅表示前面固定部分
  NLink_LinkTrack_Node_Frame2_Part framePart;
  //协议结构体中部分变量需要经过二次转化
  float pos[3];
  float eop[3];
  float vel[3];
  float angle[3];
  float supplyVoltage;
  Node2_t *node[256];

} NodeFrame2Data;

extern NodeFrame2Data nodeFrame2Data_;

//解析传入的 byteArray(已校验)
void unpackNodeFrame2Data(uint8_t *byteArray);

//校验传入的 byteArray
uint8_t verifyNodeFrame2Data(uint8_t *byteArray, int32_t length);

#endif  // NLINK_LINKTRACK_NODE_FRAME2_H
