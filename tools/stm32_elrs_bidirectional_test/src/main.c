#include <stdint.h>
#include <stdbool.h>
#include "config.h"
#include "crsf.h"

/* STM32F401 RM0368 register map + Cortex-M4 core registers. Bare-metal,
   register-level only (no HAL) so the vector table stays fixed at
   APP_BASE and coexists with the WeAct HID bootloader occupying
   0x08000000-0x08003FFF. See tools/stm32_hid_led_test for the same
   pattern applied to a single GPIO output; this extends it to two USARTs
   plus a millisecond SysTick. */
#define REG32(address) (*(volatile uint32_t *)(address))

#define RCC_CR         REG32(0x40023800u)
#define RCC_CFGR       REG32(0x40023808u)
#define RCC_AHB1RSTR   REG32(0x40023810u)
#define RCC_AHB2RSTR   REG32(0x40023814u)
#define RCC_AHB1ENR    REG32(0x40023830u)
#define RCC_APB1ENR    REG32(0x40023840u)
#define RCC_APB2ENR    REG32(0x40023844u)

#define GPIOA_MODER    REG32(0x40020000u)
#define GPIOA_OTYPER   REG32(0x40020004u)
#define GPIOA_OSPEEDR  REG32(0x40020008u)
#define GPIOA_PUPDR    REG32(0x4002000cu)
#define GPIOA_AFRL     REG32(0x40020020u)
#define GPIOA_AFRH     REG32(0x40020024u)

#define GPIOC_MODER    REG32(0x40020800u)
#define GPIOC_OTYPER   REG32(0x40020804u)
#define GPIOC_OSPEEDR  REG32(0x40020808u)
#define GPIOC_PUPDR    REG32(0x4002080cu)
#define GPIOC_BSRR     REG32(0x40020818u)

#define USART1_SR      REG32(0x40011000u)
#define USART1_DR      REG32(0x40011004u)
#define USART1_BRR     REG32(0x40011008u)
#define USART1_CR1     REG32(0x4001100cu)
#define USART1_CR2     REG32(0x40011010u)
#define USART1_CR3     REG32(0x40011014u)

#define USART2_SR      REG32(0x40004400u)
#define USART2_DR      REG32(0x40004404u)
#define USART2_BRR     REG32(0x40004408u)
#define USART2_CR1     REG32(0x4000440cu)
#define USART2_CR2     REG32(0x40004410u)
#define USART2_CR3     REG32(0x40004414u)

#define NVIC_ISER1     REG32(0xe000e104u)

#define SYST_CSR       REG32(0xe000e010u)
#define SYST_RVR       REG32(0xe000e014u)
#define SYST_CVR       REG32(0xe000e018u)
#define SCB_VTOR       REG32(0xe000ed08u)

#define USART_SR_ORE   (1u << 3)
#define USART_SR_RXNE  (1u << 5)
#define USART_SR_TC    (1u << 6)
#define USART_SR_TXE   (1u << 7)
#define USART_CR1_RE   (1u << 2)
#define USART_CR1_TE   (1u << 3)
#define USART_CR1_RXNEIE (1u << 5)
#define USART_CR1_UE   (1u << 13)
#define USART_CR3_HDSEL (1u << 3)

static volatile uint32_t g_millis = 0;

/* Ground-station demo channels, expressed as PWM-style microseconds before
   conversion to CRSF ticks. CH1 cycles for a visible downlink test signal;
   the rest sit at safe fixed values (no arm/fire channel is ever driven
   active by this bench firmware). */
static int tx_ch_us[16] = {
  1500, 1500, 1500, 1000, 1000, 1000, 1500, 1000,
  1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500
};

static uint16_t last_rx_ch_tick[16];
static int last_rx_ch_us[16];

static CrsfParser parser_tx_module; /* bytes arriving on USART1 from ES900TX */
static CrsfParser parser_receiver;  /* bytes arriving on USART2 from ES900RX */

static volatile bool downlink_ok = false;
static volatile uint32_t last_rc_rx_ms = 0;

/* Backlink freshness must come from a frame that is actually a validated
   Battery Sensor payload (correct type, correct length, plausible voltage),
   not from "any valid CRSF frame arrived on USART1" - the TX module also
   emits its own link-statistics/telemetry frames on that same wire, and
   using their arrival to keep backlink_now true would let the LED report
   a stale/fake "still receiving battery echoes" state. See
   USART1_IRQHandler() and update_status_led(). */
static volatile uint32_t last_valid_battery_frame_ms = 0;

/* TEMPORARY bring-up diagnostic, added 2026-09-14 to distinguish "USART1
   has never received anything typed as a Battery Sensor frame at all"
   from "it received one but the frame failed length/voltage validation".
   Latches permanently once true (until power-cycle) so a single occurrence
   is not missed. Remove once the real backlink path is confirmed working
   and this question is answered - it is not part of the intended
   three-state LED design. See update_status_led(). */
static volatile bool battery_type_ever_seen = false;

/* Evidence counters. Not currently readable from a running board (no host
   link beyond this USB-HID bootloader/flash path), but kept cheap and
   correct so a future SWD peek or added UART can report real counts rather
   than only "it compiled". */
static volatile uint32_t rc_sent_count = 0;
static volatile uint32_t rc_downlink_match_count = 0;
static volatile uint32_t battery_sent_count = 0;
static volatile uint32_t battery_valid_backlink_count = 0;
static volatile uint32_t battery_rejected_count = 0; /* wrong type/length/range */

/* ------------------------------------------------------------------ */
/* Hardware bring-up                                                   */
/* ------------------------------------------------------------------ */

static void prepare_hardware(void) {
  /* Do not inherit the HID bootloader's IRQ, SysTick, or USB state. */
  SYST_CSR = 0;
  for (unsigned i = 0; i < 8; ++i) {
    REG32(0xe000e180u + 4u * i) = 0xffffffffu; /* NVIC ICER */
    REG32(0xe000e280u + 4u * i) = 0xffffffffu; /* NVIC ICPR */
  }
  REG32(0xe000ed04u) = (1u << 25) | (1u << 27); /* clear pending SysTick/PendSV */
  SCB_VTOR = APP_BASE;
  __asm volatile ("dsb\nisb" ::: "memory");

  /* Switch from any inherited PLL configuration to internal HSI 16 MHz.
     16 MHz is enough for exact/near-exact 400000 and 420000 baud (see
     config.h), so no PLL is needed for this bench firmware. */
  RCC_CR |= 1u;
  while (!(RCC_CR & (1u << 1))) {}
  RCC_CFGR &= ~3u;
  while (RCC_CFGR & (3u << 2)) {}
  RCC_CFGR &= ~((15u << 4) | (7u << 10) | (7u << 13));
  RCC_CR &= ~(1u << 24); /* PLL off after HSI is selected */
  while (RCC_CR & (1u << 25)) {}

  /* End inherited USB operation; reset A/B/C to remove inherited pin modes
     (this also clears whatever the bootloader left on PA9/PA2/PA3/PA11/12).
     GPIO reset preserves the chip's default SWD alternate function. */
  RCC_AHB2RSTR |= 1u << 7;
  RCC_AHB2RSTR &= ~(1u << 7);
  RCC_AHB1RSTR |= 7u;
  RCC_AHB1RSTR &= ~7u;

  /* GPIOA (USART1/2 pins) + GPIOC (status LED) clocks. */
  RCC_AHB1ENR |= (1u << 0) | (1u << 2);
  (void)RCC_AHB1ENR;

  GPIOC_BSRR = 1u << LED_PIN; /* active-low LED starts off */
  GPIOC_OTYPER &= ~(1u << LED_PIN);
  GPIOC_OSPEEDR &= ~(3u << (2u * LED_PIN));
  GPIOC_PUPDR &= ~(3u << (2u * LED_PIN));
  GPIOC_MODER = (GPIOC_MODER & ~(3u << (2u * LED_PIN))) | (1u << (2u * LED_PIN));

  /* Millisecond tick, interrupt-driven (PRIMASK still set here; enabled
     globally only after every peripheral below is configured). */
  SYST_RVR = HSI_HZ / 1000u - 1u; /* 15999: nominal 1 ms */
  SYST_CVR = 0;
  SYST_CSR = 7u; /* ENABLE | TICKINT | CLKSOURCE(core clock) */
}

static void usart_gpio_init(void) {
  /* PA9 = USART1_TX (AF7), half-duplex single wire.
     PA2 = USART2_TX (AF7), PA3 = USART2_RX (AF7), full-duplex. */
  GPIOA_MODER &= ~((3u << (2u * 9)) | (3u << (2u * 2)) | (3u << (2u * 3)));
  GPIOA_MODER |= (2u << (2u * 9)) | (2u << (2u * 2)) | (2u << (2u * 3));

  GPIOA_OTYPER &= ~((1u << 9) | (1u << 2) | (1u << 3));
  GPIOA_OSPEEDR |= (3u << (2u * 9)) | (3u << (2u * 2)) | (3u << (2u * 3));
  GPIOA_PUPDR &= ~((3u << (2u * 9)) | (3u << (2u * 2)) | (3u << (2u * 3)));
  GPIOA_PUPDR |= (1u << (2u * 9)) | (1u << (2u * 2)) | (1u << (2u * 3));

  GPIOA_AFRH = (GPIOA_AFRH & ~(0xfu << (4u * (9u - 8u)))) | (7u << (4u * (9u - 8u)));
  GPIOA_AFRL = (GPIOA_AFRL & ~((0xfu << (4u * 2u)) | (0xfu << (4u * 3u)))) |
               (7u << (4u * 2u)) | (7u << (4u * 3u));
}

static void usart1_init(void) {
  RCC_APB2ENR |= (1u << 4); /* USART1EN */
  (void)RCC_APB2ENR;

  USART1_BRR = USART1_BRR_VALUE;
  USART1_CR2 = 0;
  USART1_CR3 = USART_CR3_HDSEL;
  /* Start in receive direction; send_rc_to_tx_module() toggles TE/RE. */
  USART1_CR1 = USART_CR1_UE | USART_CR1_RE | USART_CR1_RXNEIE;
}

static void usart2_init(void) {
  RCC_APB1ENR |= (1u << 17); /* USART2EN */
  (void)RCC_APB1ENR;

  USART2_BRR = USART2_BRR_VALUE;
  USART2_CR2 = 0;
  USART2_CR3 = 0;
  USART2_CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
}

static void nvic_enable_usarts(void) {
  NVIC_ISER1 = (1u << (37u - 32u)) | (1u << (38u - 32u)); /* USART1, USART2 */
}

/* ------------------------------------------------------------------ */
/* USART1 half-duplex direction control                                */
/* ------------------------------------------------------------------ */

static void usart1_switch_to_tx(void) {
  USART1_CR1 = (USART1_CR1 & ~USART_CR1_RE) | USART_CR1_TE;
}

static void usart1_switch_to_rx(void) {
  USART1_CR1 = (USART1_CR1 & ~USART_CR1_TE) | USART_CR1_RE;
}

/* ------------------------------------------------------------------ */
/* CRSF send/receive                                                    */
/* ------------------------------------------------------------------ */

static void send_rc_to_tx_module(void) {
  uint8_t frame[CRSF_RC_FRAME_SIZE];
  crsf_build_rc_frame(frame, tx_ch_us);

  usart1_switch_to_tx();
  for (uint8_t i = 0; i < CRSF_RC_FRAME_SIZE; i++) {
    while (!(USART1_SR & USART_SR_TXE)) {}
    USART1_DR = frame[i];
  }
  while (!(USART1_SR & USART_SR_TC)) {}
  usart1_switch_to_rx();
  rc_sent_count++;
}

static void send_battery_to_receiver(void) {
  uint8_t frame[16];
  static uint16_t fake_voltage_01v = 120; /* 12.0V, ramps for visibility */
  fake_voltage_01v++;
  if (fake_voltage_01v > 126) fake_voltage_01v = 120;

  uint8_t len = crsf_build_battery_frame(frame, fake_voltage_01v, 5, 100, 90);
  for (uint8_t i = 0; i < len; i++) {
    while (!(USART2_SR & USART_SR_TXE)) {}
    USART2_DR = frame[i];
  }
  battery_sent_count++;
}

static bool channels_match_sent(const int rx_us[16], const int sent_us[16]) {
  for (uint8_t i = 0; i < 8; i++) {
    int diff = rx_us[i] - sent_us[i];
    if (diff < 0) diff = -diff;
    if (diff > CHANNEL_MATCH_TOLERANCE_US) return false;
  }
  return true;
}

static void update_demo_channels(void) {
  static uint8_t state = 0;
  state = (uint8_t)((state + 1) % 3);

  if (state == 0) tx_ch_us[0] = 1000;
  if (state == 1) tx_ch_us[0] = 1500;
  if (state == 2) tx_ch_us[0] = 2000;

  tx_ch_us[1] = 1500;
  tx_ch_us[2] = 1500;
  tx_ch_us[3] = 1000; /* Fire off */
  tx_ch_us[4] = 1000; /* Arm off */
  tx_ch_us[5] = 1000; /* Manual */
  tx_ch_us[6] = 1500; /* Target center */
  tx_ch_us[7] = 1000; /* Emergency off */
}

/* ------------------------------------------------------------------ */
/* Status LED: reflects CURRENT link health, not "ever confirmed". If  */
/* backlink drops while downlink keeps working, the LED must fall back */
/* from solid to fast blink; if downlink also drops, to slow blink;    */
/* recovery must bring it back up the same way. Nothing here latches,  */
/* EXCEPT the TEMPORARY diagnostic long-pulse state below.             */
/* slow blink = downlink not currently fresh.                          */
/* fast blink = downlink fresh, backlink not currently fresh, and      */
/*              USART1 has never seen a Battery-typed frame at all.    */
/* long pulse = downlink/backlink not both fresh, but USART1 HAS seen  */
/*              at least one Battery-typed frame (length/content may   */
/*              still be failing validation) - TEMPORARY, see          */
/*              battery_type_ever_seen.                                */
/* solid on   = both currently fresh (the real target state).          */
/* ------------------------------------------------------------------ */

static void led_on(void)  { GPIOC_BSRR = 1u << (LED_PIN + 16u); }
static void led_off(void) { GPIOC_BSRR = 1u << LED_PIN; }

static void update_status_led(void) {
  uint32_t now = g_millis;
  bool downlink_now = (now - last_rc_rx_ms) < DOWNLINK_STALE_MS && downlink_ok;
  bool backlink_now = (now - last_valid_battery_frame_ms) < BACKLINK_STALE_MS;

  if (downlink_now && backlink_now) {
    led_on();
    return;
  }

  if (battery_type_ever_seen) {
    /* TEMPORARY diagnostic pattern: 1000ms on, 500ms off, repeat - clearly
       different from both the 500/500 slow blink and the 100/100 fast
       blink below. */
    uint32_t phase = now % 1500u;
    if (phase < 1000u) led_on(); else led_off();
    return;
  }

  uint32_t period = downlink_now ? LED_FAST_PERIOD_MS : LED_SLOW_PERIOD_MS;
  uint32_t phase = now % period;
  if (phase < period / 2u) led_on(); else led_off();
}

/* ------------------------------------------------------------------ */
/* Interrupt handlers                                                  */
/* ------------------------------------------------------------------ */

void SysTick_Handler(void) {
  g_millis++;
}

void USART1_IRQHandler(void) {
  uint32_t sr = USART1_SR;
  if (sr & USART_SR_RXNE) {
    uint8_t b = (uint8_t)USART1_DR;
    CrsfFrame frame;
    if (crsf_parser_push(&parser_tx_module, b, &frame)) {
      /* The TX module also emits its own frames on this wire (e.g. link
         statistics) that are NOT our injected battery telemetry echoing
         back. Only a frame that is actually type+length+content-valid as
         our Battery Sensor payload counts as backlink evidence; anything
         else parsing cleanly is deliberately ignored here rather than
         treated as "the backlink is alive". */
      if (frame.type == CRSF_FRAME_BATTERY) {
        battery_type_ever_seen = true; /* TEMPORARY diagnostic, see declaration */
      }
      if (frame.type == CRSF_FRAME_BATTERY && frame.payload_len == 8) {
        uint16_t voltage_01v = (uint16_t)((frame.payload[0] << 8) | frame.payload[1]);
        if (voltage_01v >= BATTERY_VOLTAGE_MIN_01V &&
            voltage_01v <= BATTERY_VOLTAGE_MAX_01V) {
          last_valid_battery_frame_ms = g_millis;
          battery_valid_backlink_count++;
        } else {
          battery_rejected_count++;
        }
      } else if (frame.type == CRSF_FRAME_BATTERY) {
        battery_rejected_count++; /* wrong length for a battery payload */
      }
    }
  } else if (sr & USART_SR_ORE) {
    (void)USART1_DR;
  }
}

void USART2_IRQHandler(void) {
  uint32_t sr = USART2_SR;
  if (sr & USART_SR_RXNE) {
    uint8_t b = (uint8_t)USART2_DR;
    CrsfFrame frame;
    if (crsf_parser_push(&parser_receiver, b, &frame)) {
      if (frame.type == CRSF_FRAME_RC_CHANNELS &&
          frame.payload_len == CRSF_RC_PAYLOAD_SIZE) {
        crsf_unpack_channels(frame.payload, last_rx_ch_tick);
        for (uint8_t i = 0; i < 16; i++) {
          last_rx_ch_us[i] = crsf_tick_to_us(last_rx_ch_tick[i]);
        }
        downlink_ok = channels_match_sent(last_rx_ch_us, tx_ch_us);
        if (downlink_ok) rc_downlink_match_count++;
        last_rc_rx_ms = g_millis;
      }
    }
  } else if (sr & USART_SR_ORE) {
    (void)USART2_DR;
  }
}

/* ------------------------------------------------------------------ */
/* Main                                                                 */
/* ------------------------------------------------------------------ */

int main(void) {
  prepare_hardware();
  usart_gpio_init();
  usart1_init();
  usart2_init();
  crsf_parser_init(&parser_tx_module);
  crsf_parser_init(&parser_receiver);
  nvic_enable_usarts();

  __asm volatile ("cpsie i" ::: "memory"); /* last step: unmask interrupts */

  uint32_t last_rc_send_ms = 0;
  uint32_t last_batt_send_ms = 0;
  uint32_t last_demo_ms = 0;

  for (;;) {
    uint32_t now = g_millis;

    if ((now - last_demo_ms) >= DEMO_CH1_PERIOD_MS) {
      last_demo_ms = now;
      update_demo_channels();
    }

    if ((now - last_rc_send_ms) >= RC_SEND_PERIOD_MS) {
      last_rc_send_ms = now;
      send_rc_to_tx_module();
    }

    if ((now - last_batt_send_ms) >= BATTERY_SEND_PERIOD_MS) {
      last_batt_send_ms = now;
      send_battery_to_receiver();
    }

    update_status_led();
  }
}
