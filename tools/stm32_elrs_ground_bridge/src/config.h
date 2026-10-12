#pragma once

#define APP_BASE 0x08004000u
#define HSI_HZ 16000000u
#define LED_PIN 13u

/* USART1: PA9 half-duplex single wire -> ES900TX JR Pin 5 Signal/Data.
   Unchanged from tools/stm32_elrs_bidirectional_test (hardware-confirmed
   working over real ELRS RF, see docs/codex-handoff.md). */
#define USART1_BAUD 400000u
#define USART1_BRR_VALUE 0x28u

/* USART2: PA2 TX / PA3 RX, full-duplex. The bench rig used this pair for
   ES900RX; this board has no RX module, so it is repurposed to drive an
   external USB-UART adapter feeding the laptop running tools/dashboard/.
   The adapter's exact pin header is TBD until the real board/enclosure is
   built -- PA2/PA3 is a software-only decision (reusing the already
   hardware-proven USART2 register setup), not a claim about final wiring.
   115200 baud matches tools/dashboard/README.md's existing Mega-USB-Serial
   convention, so the dashboard side needs no changes. USARTDIV =
   16000000/(16*115200) = 8.680556; mantissa=8, fraction=11 -> BRR=0x8B
   (same derivation as stm32_elrs_robot_bridge's Mega-link USART1). */
#define USART2_BAUD 115200u
#define USART2_BRR_VALUE 0x8Bu

#define RC_SEND_PERIOD_MS 10u /* 100 Hz RC uplink, same cadence as the bench rig */

#define DOWNLINK_STALE_MS 500u   /* no confirmed RC echo from ES900TX */
#define BACKLINK_STALE_MS 1500u  /* no valid TEL_CHUNK frame from ES900TX */
#define CHANNEL_MATCH_TOLERANCE_US 20

#define LED_SLOW_PERIOD_MS 1000u /* neither link confirmed */
#define LED_FAST_PERIOD_MS 200u  /* downlink confirmed, backlink not */

/* ---- Provisional control-input wiring (GPIO/ADC), see README.md ----
   The real multi-function keypad does not exist yet; these pins are
   placeholders so this firmware has *something* concrete to read and
   build against, not a claim about final hardware. Re-measure/re-wire
   once the keypad is built. */
#define ADC_PIN_DRIVE_FORWARD 0u /* PA0 = ADC1_IN0 */
#define ADC_PIN_DRIVE_TURN    1u /* PA1 = ADC1_IN1 */
/* Digital switches, all active-low (pressed/thrown = 0V, pull-up idle). */
#define GPIO_PIN_MODE_BIT0        4u  /* PA4 */
#define GPIO_PIN_MODE_BIT1        5u  /* PA5 */
#define GPIO_PIN_FIRE_BUTTON      6u  /* PA6 */
#define GPIO_PIN_STARTING_SIDE    7u  /* PA7; LEFT=0(idle), RIGHT=1(active) */
