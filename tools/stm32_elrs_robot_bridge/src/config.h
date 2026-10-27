#pragma once

/* Robot-side board is a fresh WeAct F401CCU6 with no WeAct HID bootloader
   (2026-10-11), flashed through the chip's ROM USB DFU -- so the image
   starts at the beginning of flash, not 0x08004000. */
#define APP_BASE 0x08000000u
#define HSI_HZ 16000000u
#define LED_PIN 13u

/* USART2: PA2 TX / PA3 RX, full-duplex -> ES900RX RX/TX. Same wiring role
   and baud as the stm32_elrs_bidirectional_test bench's USART2 (retired 2026-10-14, still in git history) (that pairing
   is hardware-confirmed working over real ELRS RF, see
   docs/codex-handoff.md) -- unchanged by splitting the bench loopback rig
   into separate robot/ground firmwares. */
#define USART2_BAUD 420000u
#define USART2_BRR_VALUE 0x26u

/* USART1: PA9 TX / PA10 RX, full-duplex -> Mega hardware serial port
   (Pins.h MECH_LINK_SERIAL_PORT). NOT the bench's half-duplex single-wire
   ES900TX setup: this board has no TX module, so USART1 is repurposed for
   an ordinary two-wire link to the Mega, and PA10 (unused by the bench)
   is brought up as USART1_RX.
   250000 since 2026-10-27 (was 115200): USARTDIV = 16000000/(16*250000)
   = 4.0 exactly -> BRR=0x40, and the Mega's 16 MHz U2X hits 250000
   exactly too (UBRR 7). At 115200 the Mega ran at 117647 (+2.1%) against
   this board's 115108 (and its HSI is ~0.6% fast): digits flipped on the
   wire. Must match src/IO/MechLink.cpp LINK_BAUD; both ends also check
   every line's "*HH" (src/IO/RcControlMap.h). */
#define USART1_BAUD 250000u
#define USART1_BRR_VALUE 0x40u

#define RC_FORWARD_PERIOD_MS 20u      /* how often a "CH,..." line goes to the Mega */

/* No fresh validated RC_CHANNELS frame from ES900RX for this long = RF
   lost: the LED drops to slow blink AND "CH,..." forwarding to the Mega
   stops (see ch_line_should_forward). With MechLink's 200 ms line timeout
   the Mega reports signalLost within ~700 ms of the RF loss. */
#define DOWNLINK_STALE_MS 500u

/* Mega status uplink (Battery frame to the receiver). The Mega reports
   every 100 ms; a status older than STATUS_STALE_MS is not forwarded. */
#define STATUS_SEND_PERIOD_MS 100u
#define STATUS_STALE_MS 500u

#define LED_SLOW_PERIOD_MS 1000u /* downlink not fresh */
#define LED_FAST_PERIOD_MS 200u  /* downlink fresh (solid once Mega-link send activity starts) */
