#ifndef CH_LINE_FORMAT_H
#define CH_LINE_FORMAT_H

/* Formats decoded CRSF channel values into the ASCII line this firmware
   sends to the Mega over its dedicated UART link (Pins.h
   MECH_LINK_SERIAL_PORT), replacing the channel values the Mega used to
   read directly off SBUS. Pure string formatting, no UART/registers --
   this firmware links -nostdlib, so no snprintf()/itoa() from libc. */

#include <stdint.h>

/* "CH," + 6 signed ints (each up to 6 chars incl. sign) + 5 commas + '\n' +
   '\0', rounded up with margin. */
#define CH_LINE_MAX_CHARS 48u

/* ch_us must have at least as many entries as the highest TEL_CH_* index
   (tel_channel_map.h) plus one; values are the project's existing
   pulse-microsecond convention (same range SBUS used to produce). Writes
   a '\0'-terminated "CH,fwd,turn,mech,aux,fire,mode\n" into buf (capacity
   bufSize) and returns the formatted length excluding the '\0'. Returns 0
   and writes an empty string if bufSize is too small for CH_LINE_MAX_CHARS. */
uint16_t format_ch_line(char *buf, uint16_t bufSize, const int ch_us[16]);

#endif
