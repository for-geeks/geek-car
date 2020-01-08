#include "nlink_linktrack_node_frame1.h"

NodeFrame1Data nodeFrame1Data_;

void unpackNodeFrame1Data(uint8_t *byteArray) {
  const int framePartLength = 27;
  static uint8_t initNeeded = 1;
  if (initNeeded) {
    memset(nodeFrame1Data_.node, 0, sizeof(nodeFrame1Data_.node));
    initNeeded = 0;
    assert(sizeof(nodeFrame1Data_.framePart) == framePartLength);
  }

  memcpy(&nodeFrame1Data_.framePart, byteArray, framePartLength);

  for (int i = 0; i < nodeFrame1Data_.framePart.validNodeQuantity; ++i) {
    RawNode1_t rawNode;
    memcpy(&rawNode, byteArray + framePartLength + i * sizeof(RawNode1_t),
           sizeof(RawNode1_t));

    if (!nodeFrame1Data_.node[rawNode.id]) {
      nodeFrame1Data_.node[rawNode.id] = (Node1_t*)malloc(sizeof(Node1_t));
    }
    nodeFrame1Data_.node[rawNode.id]->role = rawNode.role;
    nodeFrame1Data_.node[rawNode.id]->id = rawNode.id;
    int24ToFloat(rawNode.posTemp, nodeFrame1Data_.node[rawNode.id]->pos, 3,
                 kPosMultiply_);
  }

  nodeFrame1Data_.supplyVoltage =
      nodeFrame1Data_.framePart.supplyVoltageTemp / kVoltageMultiply_;

  //    NodeFrame1Data data = nodeFrame1Data_;
}

uint8_t verifyNodeFrame1Data(uint8_t *byteArray, int32_t length) {
  if (byteArray[0] != 0x55 || byteArray[1] != 0x03) return 0;
  return verifyCheckSum(byteArray, length);
}
