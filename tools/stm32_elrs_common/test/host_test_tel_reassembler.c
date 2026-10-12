/* Host-side unit test for tel_reassembler.c/h. Native compiler, no hardware
   or CRSF framing/UART involved -- this only exercises the chunk-reassembly
   logic itself. */
#include <stdio.h>
#include <string.h>
#include "tel_reassembler.h"
#include "tel_chunk_split.h"

static int failures = 0;

#define CHECK(cond, msg) do { \
  if (!(cond)) { \
    printf("FAIL: %s (%s:%d)\n", msg, __FILE__, __LINE__); \
    failures++; \
  } \
} while (0)

/* Splits `line` into CRSF_TEL_CHUNK_DATA_MAX-sized pieces using the same
   tel_chunk_split helpers the real robot-bridge firmware calls, so this
   test exercises the actual sender-side logic rather than a parallel
   reimplementation of it. */
static uint8_t split_into_chunks(const char *line, uint8_t chunks[][CRSF_TEL_CHUNK_DATA_MAX],
                                  uint8_t lens[], uint8_t *outCount) {
  uint16_t len = tel_cstrlen(line);
  uint8_t count = tel_chunk_count(len);
  for (uint8_t c = 0; c < count; c++) {
    const char *ptr;
    uint8_t thisLen;
    tel_chunk_slice(line, len, c, &ptr, &thisLen);
    for (uint8_t i = 0; i < thisLen; i++) chunks[c][i] = (uint8_t)ptr[i];
    lens[c] = thisLen;
  }
  *outCount = count;
  return count;
}

static void test_in_order_reassembles(void) {
  const char *line = "TEL,1000,0.10,0.20,900.00,1,1,1500,1500,1500,1500,1500,1,3,0";
  uint8_t chunks[TEL_REASSEMBLER_MAX_CHUNKS][CRSF_TEL_CHUNK_DATA_MAX];
  uint8_t lens[TEL_REASSEMBLER_MAX_CHUNKS];
  uint8_t count;
  split_into_chunks(line, chunks, lens, &count);

  TelReassembler r;
  tel_reassembler_reset(&r);
  bool done = false;
  for (uint8_t i = 0; i < count; i++) {
    done = tel_reassembler_push(&r, i, count, chunks[i], lens[i]);
  }
  CHECK(done, "pushing the final in-order chunk reports completion");
  CHECK(strcmp(r.buf, line) == 0, "in-order reassembly reproduces the original line exactly");
}

static void test_out_of_order_reassembles(void) {
  const char *line = "TEL,2000,-0.5,0.5,1200.00,2,0,1000,1200,1500,1800,2000,0,0,1";
  uint8_t chunks[TEL_REASSEMBLER_MAX_CHUNKS][CRSF_TEL_CHUNK_DATA_MAX];
  uint8_t lens[TEL_REASSEMBLER_MAX_CHUNKS];
  uint8_t count;
  split_into_chunks(line, chunks, lens, &count);
  CHECK(count >= 2, "test line is long enough to span multiple chunks");

  TelReassembler r;
  tel_reassembler_reset(&r);
  bool done = false;
  /* push the last chunk first, then the rest forwards */
  done = tel_reassembler_push(&r, (uint8_t)(count - 1), count, chunks[count - 1], lens[count - 1]);
  CHECK(!done, "a single out-of-order chunk alone is never complete");
  for (uint8_t i = 0; i < (uint8_t)(count - 1); i++) {
    done = tel_reassembler_push(&r, i, count, chunks[i], lens[i]);
  }
  CHECK(done, "completion fires once every chunk has arrived, any order");
  CHECK(strcmp(r.buf, line) == 0, "out-of-order reassembly reproduces the original line exactly");
}

static void test_missing_chunk_never_completes(void) {
  const char *line = "TEL,3000,0,0,0,0,0,1500,1500,1500,1500,1500,0,0,1";
  uint8_t chunks[TEL_REASSEMBLER_MAX_CHUNKS][CRSF_TEL_CHUNK_DATA_MAX];
  uint8_t lens[TEL_REASSEMBLER_MAX_CHUNKS];
  uint8_t count;
  split_into_chunks(line, chunks, lens, &count);
  CHECK(count >= 2, "test line is long enough to span multiple chunks");

  TelReassembler r;
  tel_reassembler_reset(&r);
  bool done = false;
  for (uint8_t i = 0; i < (uint8_t)(count - 1); i++) { /* skip the last chunk */
    done = tel_reassembler_push(&r, i, count, chunks[i], lens[i]);
  }
  CHECK(!done, "a round missing one chunk never reports completion");
}

static void test_new_chunk_count_discards_stale_round(void) {
  const char *shortLine = "TEL,1";
  const char *longLine = "TEL,4000,1.1,2.2,3.3,4,1,1500,1500,1500,1500,1500,1,2,0";
  uint8_t shortChunks[TEL_REASSEMBLER_MAX_CHUNKS][CRSF_TEL_CHUNK_DATA_MAX];
  uint8_t shortLens[TEL_REASSEMBLER_MAX_CHUNKS];
  uint8_t shortCount;
  split_into_chunks(shortLine, shortChunks, shortLens, &shortCount);
  CHECK(shortCount == 1, "the short test line fits in exactly one chunk");

  uint8_t longChunks[TEL_REASSEMBLER_MAX_CHUNKS][CRSF_TEL_CHUNK_DATA_MAX];
  uint8_t longLens[TEL_REASSEMBLER_MAX_CHUNKS];
  uint8_t longCount;
  split_into_chunks(longLine, longChunks, longLens, &longCount);
  CHECK(longCount > 1, "the long test line spans multiple chunks");

  TelReassembler r;
  tel_reassembler_reset(&r);
  /* start a long round, push only its first chunk */
  bool done = tel_reassembler_push(&r, 0, longCount, longChunks[0], longLens[0]);
  CHECK(!done, "the first chunk of a multi-chunk round is never complete alone");

  /* a new round with a different chunkCount must discard that partial state,
     not report completion from a mix of old+new bytes */
  done = tel_reassembler_push(&r, 0, shortCount, shortChunks[0], shortLens[0]);
  CHECK(done, "the new round's own single chunk completes it");
  CHECK(strcmp(r.buf, shortLine) == 0, "the new round's line is clean, not mixed with the stale one");
}

int main(void) {
  test_in_order_reassembles();
  test_out_of_order_reassembles();
  test_missing_chunk_never_completes();
  test_new_chunk_count_discards_stale_round();

  if (failures == 0) {
    printf("All tel_reassembler host tests passed.\n");
    return 0;
  }
  printf("%d tel_reassembler host test failure(s).\n", failures);
  return 1;
}
