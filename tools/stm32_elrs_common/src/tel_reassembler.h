#ifndef TEL_REASSEMBLER_H
#define TEL_REASSEMBLER_H

/* Reassembles the Mega's ASCII "TEL,..." dashboard telemetry line from the
   CRSF_FRAME_TEL_CHUNK pieces crsf_build_tel_chunk_frame() produces (see
   crsf.h). Pure data handling, no UART/registers, so it is host-testable
   the same way crsf.c is. */

#include <stdint.h>
#include <stdbool.h>

#include "crsf.h" /* CRSF_TEL_CHUNK_DATA_MAX: per-chunk payload size this depends on */

#define TEL_REASSEMBLER_MAX_CHUNKS 8u
#define TEL_REASSEMBLER_BUF_SIZE \
  (TEL_REASSEMBLER_MAX_CHUNKS * CRSF_TEL_CHUNK_DATA_MAX + 1u) /* +1 for '\0' */

typedef struct {
  char buf[TEL_REASSEMBLER_BUF_SIZE];
  uint8_t receivedMask;    /* bit i set once chunk i has been stored */
  uint8_t expectedCount;
  bool haveExpectedCount;
  uint8_t lastChunkDataLen; /* dataLen recorded for chunk (expectedCount-1);
                                every earlier chunk is assumed full-size, so
                                this is what the final string length
                                requires once every bit is set */
} TelReassembler;

void tel_reassembler_reset(TelReassembler *r);

/* Stores one chunk. If chunkCount differs from the count already in
   progress, any partially-reassembled data is discarded first (a new round
   started; do not silently mix bytes from two different lines). Returns
   true exactly when this push supplied the last still-missing chunk, in
   which case r->buf holds the full, '\0'-terminated line. chunkIndex/
   chunkCount >= TEL_REASSEMBLER_MAX_CHUNKS, or dataLen that would overflow
   the buffer, are rejected (returns false, no state change). */
bool tel_reassembler_push(TelReassembler *r, uint8_t chunkIndex,
                           uint8_t chunkCount, const uint8_t *data,
                           uint8_t dataLen);

#endif
