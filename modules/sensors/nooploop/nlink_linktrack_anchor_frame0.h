#ifndef NLINK_LINKTRACK_ANCHOR_FRAME0_H
#define NLINK_LINKTRACK_ANCHOR_FRAME0_H

#include "ncommon.h"

#pragma pack(1)

typedef struct {
  uint8_t id;  //表示 anchor id
  NInt24 disTemp;
} RawDisTag2Anchor_t;

typedef struct {
  uint8_t id;  //表示tag id
  uint8_t reserved;
  NInt24 posTemp[3];
  RawDisTag2Anchor_t group[4];  //表示当前tag到各基站距离
} RawTag_t;

typedef struct {
  uint8_t header[2];  // 55 00
  RawTag_t tagTemp[30];
  uint8_t reserved0[75];
  uint16_t supplyVoltageTemp;
  uint32_t systemTime;
  uint8_t id;  //表示当前anchor的 id
  uint8_t reserved1[1];
  uint8_t fixCheck;  // 0xee
} NLink_LinkTrack_Anchor_Frame0;
#pragma pack()

typedef struct {
  uint8_t id;  //表示 anchor id
  float dis;
} DisTag2Anchor_t;

typedef struct {
  uint8_t id;  //表示tag id
  float pos[3];
  DisTag2Anchor_t group[4];  //表示当前tag到各基站距离
} Tag_t;

typedef struct {
  NLink_LinkTrack_Anchor_Frame0 frame;
  //协议结构体中部分变量需要经过二次转化
  Tag_t tag[30];

  float supplyVoltage;

} AnchorFrame0Data;

extern AnchorFrame0Data anchorFrame0Data_;

//解析传入的 byteArray(已校验)
void unpackAnchorFrame0Data(uint8_t *byteArray);

//校验传入的 byteArray
uint8_t verifyAnchorFrame0Data(uint8_t *byteArray);

#endif  // NLINK_LINKTRACK_ANCHOR_FRAME0_H
