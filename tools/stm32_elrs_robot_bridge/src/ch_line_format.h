#ifndef CH_LINE_FORMAT_H
#define CH_LINE_FORMAT_H

/* Pure text helpers for the bridge's UART link to the Mega (Pins.h
   MECH_LINK_SERIAL_PORT). No UART/registers here, and no libc (the firmware
   links -nostdlib), so they run unchanged in the host test.

   Downlink: "RC,c1,...,c10*HH\n" -- the first RCMAP_CHANNEL_COUNT CRSF
   channels in microseconds, table in src/IO/RcControlMap.h.
   Uplink:   "ST,flags,seq,code*HH" from the Mega, packed by the main loop
   into a CRSF Battery frame for the ground station.
   "*HH" is RcControlMap's line check (2026-10-27): the Mega drops an RC line
   whose check fails, and this board drops such an ST line. */

#include <stdbool.h>
#include <stdint.h>

#include "RcControlMap.h"

/* "RC," + N signed ints (each up to 6 chars incl. sign) + N-1 commas +
   "*HH" + '\n' + '\0', rounded up with margin. */
#define CH_LINE_MAX_CHARS (4u + 7u * RCMAP_CHANNEL_COUNT + RCMAP_LINE_CHECK_CHARS)

/* Writes a '\0'-terminated "RC,...\n" into buf and returns its length
   excluding the '\0'. Returns 0 (empty string) if bufSize is too small. */
uint16_t format_rc_line(char *buf, uint16_t bufSize, const int ch_us[16]);

/* Whether channels may be forwarded to the Mega right now: only while a
   validated RC_CHANNELS frame from the receiver has arrived within
   stale_ms. When RF is lost the bridge goes silent instead of repeating the
   last channels, so the Mega's MechLink line timeout reports signalLost and
   its failsafe runs (a bridge that kept talking would hide the outage). */
bool ch_line_should_forward(bool ever_received, uint32_t now_ms,
                            uint32_t last_rc_rx_ms, uint32_t stale_ms);

/* Parses "ST,<flags 0-255>,<seq 0-16777215>,<code 0-65535>*HH" (no '\n').
   Returns false and leaves out untouched on a missing / wrong check or any
   malformed field. */
bool parse_status_line(const char *line, uint16_t len, rcmap_status *out);

#endif
