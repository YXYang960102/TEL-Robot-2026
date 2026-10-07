#pragma once

#define APP_BASE 0x08004000u
#define HSI_HZ 16000000u
#define LED_PIN 13u

/* USART2: PA2 TX / PA3 RX, full-duplex -> ES900RX RX/TX. Same wiring role
   and baud as tools/stm32_elrs_bidirectional_test's USART2 (that pairing
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
   USARTDIV = 16000000/(16*115200) = 8.680556; mantissa=8,
   fraction=round(0.680556*16)=11 -> BRR=0x8B. Actual baud =
   16000000/(16*8+11) = 115107.9 Hz, -0.08% vs nominal -- tighter than the
   already-accepted +0.25% on USART2 above. */
#define USART1_BAUD 115200u
#define USART1_BRR_VALUE 0x8Bu

#define RC_FORWARD_PERIOD_MS 20u      /* how often a "CH,..." line goes to the Mega */
#define TEL_CHUNK_SEND_PERIOD_MS 20u  /* how often one TEL_CHUNK frame goes to ES900RX */

#define DOWNLINK_STALE_MS 500u /* no fresh validated RC_CHANNELS frame from ES900RX */

#define LED_SLOW_PERIOD_MS 1000u /* downlink not fresh */
#define LED_FAST_PERIOD_MS 200u  /* downlink fresh (solid once Mega-link send activity starts) */
