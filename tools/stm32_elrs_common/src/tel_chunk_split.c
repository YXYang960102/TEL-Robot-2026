#include "tel_chunk_split.h"

uint16_t tel_cstrlen(const char *s) {
  uint16_t len = 0;
  while (s[len] != '\0') len++;
  return len;
}

uint8_t tel_chunk_count(uint16_t lineLen) {
  uint16_t count = (uint16_t)((lineLen + CRSF_TEL_CHUNK_DATA_MAX - 1u) / CRSF_TEL_CHUNK_DATA_MAX);
  if (count == 0) count = 1;
  return (uint8_t)count;
}

void tel_chunk_slice(const char *line, uint16_t lineLen, uint8_t index,
                      const char **outPtr, uint8_t *outLen) {
  uint16_t offset = (uint16_t)index * CRSF_TEL_CHUNK_DATA_MAX;
  if (offset >= lineLen) {
    *outPtr = line + lineLen;
    *outLen = 0;
    return;
  }
  uint16_t remaining = (uint16_t)(lineLen - offset);
  *outPtr = line + offset;
  *outLen = (uint8_t)(remaining < CRSF_TEL_CHUNK_DATA_MAX ? remaining : CRSF_TEL_CHUNK_DATA_MAX);
}
