#ifndef NLINK_LINKTRACK_NODE_FRAME1_H
#define NLINK_LINKTRACK_NODE_FRAME1_H

#include "ncommon.h"

#pragma pack(1)

typedef struct {
  uint8_t role;
  uint8_t id;
  NInt24 posTemp[3];
  uint8_t reserved[9];
} RawNode1_t;

typedef struct {
  uint8_t header[2];  // 55 03
  uint16_t frameLength;
  uint8_t role;
  uint8_t id;
  uint32_t systemTime;
  uint8_t reserved0[14];
  uint16_t supplyVoltageTemp;

  uint8_t validNodeQuantity;  //当前协议帧所包含的有效节点数量
                              //········变长组
  //    uint8_t sumCheck;
} NLink_LinkTrack_Node_Frame1_Part;
#pragma pack()

typedef struct {
  uint8_t role;
  uint8_t id;
  float pos[3];
} Node1_t;

typedef struct {
  //由于是变长协议，这里仅表示前面固定部分
  NLink_LinkTrack_Node_Frame1_Part framePart;
  //协议结构体中部分变量需要经过二次转化
  Node1_t *node[256];
  float supplyVoltage;
} NodeFrame1Data;

extern NodeFrame1Data nodeFrame1Data_;

//解析传入的 byteArray(已校验)
void unpackNodeFrame1Data(uint8_t *byteArray);

//校验传入的 byteArray
uint8_t verifyNodeFrame1Data(uint8_t *byteArray, int32_t length);

#endif  // NLINK_LINKTRACK_NODE_FRAME1_H
