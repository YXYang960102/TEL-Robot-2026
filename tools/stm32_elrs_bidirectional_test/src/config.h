#pragma once

#define APP_BASE 0x08004000u
#define HSI_HZ 16000000u
#define LED_PIN 13u

/* USART1: PA9 half-duplex single wire -> ES900TX JR Pin 5 Signal/Data. */
#define USART1_BAUD 400000u
/* USARTDIV = HSI_HZ / (16 * baud) = 2.5 exact at 400000 baud, 16 MHz.
   mantissa=2, fraction=8 (8/16 = 0.5). */
#define USART1_BRR_VALUE 0x28u

/* USART2: PA2 TX / PA3 RX full-duplex -> ES900RX RX/TX. */
#define USART2_BAUD 420000u
/* USARTDIV = 16000000 / (16*420000) = 2.375. mantissa=2, fraction=6 (6/16).
   Actual baud = 16000000/(16*2.375) = 421052.6 Hz, +0.25% vs nominal. */
#define USART2_BRR_VALUE 0x26u

#define RC_SEND_PERIOD_MS 10u
#define DEMO_CH1_PERIOD_MS 2000u
#define BATTERY_SEND_PERIOD_MS 1000u

#define DOWNLINK_STALE_MS 500u
#define BACKLINK_STALE_MS 1500u
#define CHANNEL_MATCH_TOLERANCE_US 20

/* LED status: slow blink = downlink not confirmed, fast blink = downlink
   confirmed but backlink not, solid on = both confirmed. */
#define LED_SLOW_PERIOD_MS 1000u
#define LED_FAST_PERIOD_MS 200u
