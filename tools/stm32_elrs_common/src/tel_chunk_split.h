#ifndef TEL_CHUNK_SPLIT_H
#define TEL_CHUNK_SPLIT_H

/* Splits a '\0'-terminated ASCII line (the Mega's "TEL,..." dashboard
   string) into CRSF_TEL_CHUNK_DATA_MAX-sized pieces for
   crsf_build_tel_chunk_frame(). Pure data handling, no UART/registers;
   this firmware links -nostdlib, so tel_cstrlen() replaces strlen(). */

#include <stdint.h>

#include "crsf.h" /* CRSF_TEL_CHUNK_DATA_MAX */

uint16_t tel_cstrlen(const char *s);

/* Number of chunks needed for a line of this length. A zero-length line
   still counts as one (empty) chunk, so a round always completes. */
uint8_t tel_chunk_count(uint16_t lineLen);

/* Chunk `index` of `line` (length lineLen), 0-based. Caller must ensure
   index < tel_chunk_count(lineLen); out of range yields an empty slice. */
void tel_chunk_slice(const char *line, uint16_t lineLen, uint8_t index,
                      const char **outPtr, uint8_t *outLen);

#endif
