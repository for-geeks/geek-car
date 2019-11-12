#ifndef NLINK_LINKTRACK_NODE_FRAME0_H
#define NLINK_LINKTRACK_NODE_FRAME0_H

#include "ncommon.h"
#pragma pack(1)

typedef struct {
  uint8_t header[2];  // 55 02
  uint16_t frameLength;
  uint8_t role;
  uint8_t id;
  uint8_t reserved[4];
  uint8_t validNodeQuantity;  //当前协议帧所包含的有效节点数量
                              //········变长组
                              //    uint8_t sumCheck;
} NLink_LinkTrack_Node_Frame0_Part;
#pragma pack()

typedef struct {
  uint8_t role;
  uint8_t id;
  uint16_t dataLength;
  // WARNING 请根据自身情况设置单个节点单次最大数传长度，上限参考 LinkTrack
  // Datasheet
  uint8_t data[100];
} Node0_t;

typedef struct {
  //由于是变长协议，这里仅表示前面固定部分
  NLink_LinkTrack_Node_Frame0_Part framePart;
  //协议结构体中部分变量需要经过二次转化
  Node0_t *node[256];
} NodeFrame0Data;

extern NodeFrame0Data nodeFrame0Data_;

//解析传入的 byteArray(已校验)
void unpackNodeFrame0Data(uint8_t *byteArray);

//校验传入的 byteArray
uint8_t verifyNodeFrame0Data(uint8_t *byteArray, int32_t length);

#endif  // NLINK_LINKTRACK_NODE_FRAME0_H
