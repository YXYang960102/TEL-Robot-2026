/* Host-side unit test for tel_chunk_split.c/h. */
#include <stdio.h>
#include <string.h>
#include "tel_chunk_split.h"

static int failures = 0;

#define CHECK(cond, msg) do { \
  if (!(cond)) { \
    printf("FAIL: %s (%s:%d)\n", msg, __FILE__, __LINE__); \
    failures++; \
  } \
} while (0)

static void test_cstrlen_matches_strlen(void) {
  CHECK(tel_cstrlen("") == 0, "empty string length");
  CHECK(tel_cstrlen("TEL,1") == 5, "short string length");
  const char *line = "TEL,1000,0.10,0.20,900.00,1,1,1500,1500,1500,1500,1500,1,3,0";
  CHECK(tel_cstrlen(line) == (uint16_t)strlen(line), "tel_cstrlen matches libc strlen");
}

static void test_chunk_count_boundaries(void) {
  CHECK(tel_chunk_count(0) == 1, "an empty line still counts as one chunk");
  CHECK(tel_chunk_count(1) == 1, "a 1-byte line fits in one chunk");
  CHECK(tel_chunk_count(CRSF_TEL_CHUNK_DATA_MAX) == 1, "exactly one chunk's worth fits in one chunk");
  CHECK(tel_chunk_count((uint16_t)(CRSF_TEL_CHUNK_DATA_MAX + 1)) == 2,
        "one byte over a chunk boundary needs a second chunk");
  CHECK(tel_chunk_count((uint16_t)(CRSF_TEL_CHUNK_DATA_MAX * 3)) == 3,
        "an exact multiple needs exactly that many chunks");
}

static void test_slice_reconstructs_the_line(void) {
  const char *line = "TEL,2000,-0.5,0.5,1200.00,2,0,1000,1200,1500,1800,2000,0,0,1";
  uint16_t len = tel_cstrlen(line);
  uint8_t count = tel_chunk_count(len);
  CHECK(count >= 2, "test line spans multiple chunks");

  char rebuilt[256];
  uint16_t pos = 0;
  for (uint8_t i = 0; i < count; i++) {
    const char *ptr;
    uint8_t chunkLen;
    tel_chunk_slice(line, len, i, &ptr, &chunkLen);
    CHECK(chunkLen > 0, "every in-range chunk is non-empty for this line");
    CHECK(chunkLen <= CRSF_TEL_CHUNK_DATA_MAX, "no chunk exceeds the CRSF payload cap");
    for (uint8_t j = 0; j < chunkLen; j++) rebuilt[pos++] = ptr[j];
  }
  rebuilt[pos] = '\0';
  CHECK(strcmp(rebuilt, line) == 0, "concatenating every slice reproduces the original line");
}

static void test_slice_out_of_range_is_empty(void) {
  const char *line = "TEL,1";
  uint16_t len = tel_cstrlen(line);
  uint8_t count = tel_chunk_count(len);
  const char *ptr;
  uint8_t chunkLen;
  tel_chunk_slice(line, len, count, &ptr, &chunkLen); /* one past the last valid index */
  CHECK(chunkLen == 0, "a one-past-the-end chunk index yields an empty slice, not garbage");
}

int main(void) {
  test_cstrlen_matches_strlen();
  test_chunk_count_boundaries();
  test_slice_reconstructs_the_line();
  test_slice_out_of_range_is_empty();

  if (failures == 0) {
    printf("All tel_chunk_split host tests passed.\n");
    return 0;
  }
  printf("%d tel_chunk_split host test failure(s).\n", failures);
  return 1;
}
