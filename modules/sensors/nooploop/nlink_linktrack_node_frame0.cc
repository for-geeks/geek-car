#include "nlink_linktrack_node_frame0.h"

NodeFrame0Data nodeFrame0Data_;

void unpackNodeFrame0Data(uint8_t *byteArray) {
  const int framePartLength = 11;
  static uint8_t initNeeded = 1;
  if (initNeeded) {
    memset(nodeFrame0Data_.node, 0, sizeof(nodeFrame0Data_.node));
    initNeeded = 0;

    assert(sizeof(nodeFrame0Data_.framePart) == framePartLength);
  }

  memcpy(&nodeFrame0Data_.framePart, byteArray, framePartLength);

  for (int i = 0, address = framePartLength;
       i < nodeFrame0Data_.framePart.validNodeQuantity; ++i) {
    uint8_t id = byteArray[address + 1];
    uint16_t dataLength;
    memcpy(&dataLength, byteArray + address + 2, sizeof(dataLength));
    const int currentNodeSize = 4 + dataLength;

    if (!nodeFrame0Data_.node[id]) {
      nodeFrame0Data_.node[id] = (Node0_t *)malloc(sizeof(Node0_t));
    }
    memcpy(nodeFrame0Data_.node[id], byteArray + address, currentNodeSize);

    address += currentNodeSize;
  }

  //    NodeFrame0Data data = nodeFrame0Data_;
}

uint8_t verifyNodeFrame0Data(uint8_t *byteArray, int32_t length) {
  if (byteArray[0] != 0x55 || byteArray[1] != 0x02) return 0;
  return verifyCheckSum(byteArray, length);
}
