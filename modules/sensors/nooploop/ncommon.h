#ifndef NCOMMON_H
#define NCOMMON_H

#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

enum LPSRole {
  kRoleNode,      //!< 节点，通常代表所有类型
  kRoleAnchor,    //!< 基站
  kRoleTag,       //!< 标签
  kRoleConsole,   //!< 监视器
  kRoleDTMaster,  //!< DT 主机
  kRoleDTSlave,   //!< DT 从机
};

extern const float kVoltageMultiply_;
extern const float kPosMultiply_;
extern const float kDisMultiply_;
extern const float kVelMultiply_;
extern const float kAngleMultiply_;
extern const float kRssiMultiply_;
extern const float kEopMultiply_;

typedef struct {
  uint8_t byteArray[3];
} NInt24;

// 16进制字符串转数组，0-9 a-f A-F
// 以外字符会直接跳过，前后两个字符组成一个字节，末尾不足一个字节时忽略
int stringToHex(char *str, uint8_t *out);

// int24 类型转换为 int32标准类型
int32_t int24Value(NInt24 data);

void int24ToFloat(NInt24 src[], float dst[], int arrayLength, float multiply);

void int16ToFloat(int16_t src[], float dst[], int arrayLength, float multiply);

void uint8ToFloat(uint8_t src[], float dst[], int arrayLength, float multiply);

//根据传入的数据 data 及 总长度
// length，进行和校验（校验和在最后一字节），成功返回1，否则0
uint8_t verifyCheckSum(uint8_t *data, int32_t length);

#endif  // NCOMMON_H
