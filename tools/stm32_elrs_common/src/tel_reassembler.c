#include "tel_reassembler.h"

void tel_reassembler_reset(TelReassembler *r) {
  for (uint16_t i = 0; i < TEL_REASSEMBLER_BUF_SIZE; i++) r->buf[i] = '\0';
  r->receivedMask = 0;
  r->expectedCount = 0;
  r->haveExpectedCount = false;
  r->lastChunkDataLen = 0;
}

bool tel_reassembler_push(TelReassembler *r, uint8_t chunkIndex,
                           uint8_t chunkCount, const uint8_t *data,
                           uint8_t dataLen) {
  if (chunkCount == 0 || chunkCount > TEL_REASSEMBLER_MAX_CHUNKS ||
      chunkIndex >= chunkCount || dataLen > CRSF_TEL_CHUNK_DATA_MAX) {
    return false;
  }

  /* A different chunkCount than the round in progress means a new line
     started; do not let bytes from two different lines mix in one buffer. */
  if (!r->haveExpectedCount || chunkCount != r->expectedCount) {
    tel_reassembler_reset(r);
    r->expectedCount = chunkCount;
    r->haveExpectedCount = true;
  }

  uint16_t offset = (uint16_t)chunkIndex * CRSF_TEL_CHUNK_DATA_MAX;
  if (offset + dataLen >= TEL_REASSEMBLER_BUF_SIZE) {
    return false; /* would overflow the '\0' slot; malformed input */
  }
  for (uint8_t i = 0; i < dataLen; i++) {
    r->buf[offset + i] = (char)data[i];
  }
  r->receivedMask |= (uint8_t)(1u << chunkIndex);
  if (chunkIndex == (uint8_t)(chunkCount - 1)) {
    r->lastChunkDataLen = dataLen;
  }

  uint8_t completeMask = (uint8_t)((1u << chunkCount) - 1u);
  if ((r->receivedMask & completeMask) != completeMask) {
    return false;
  }

  uint16_t totalLen = (uint16_t)(chunkCount - 1) * CRSF_TEL_CHUNK_DATA_MAX +
                       r->lastChunkDataLen;
  r->buf[totalLen] = '\0';
  return true;
}
