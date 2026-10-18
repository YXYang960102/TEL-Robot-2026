#include <stdint.h>
#include <stdbool.h>
#include "config.h"
#include "crsf.h"
#include "ch_line_format.h"

/* STM32F401 RM0368 register map + Cortex-M4 core registers. Bare-metal,
   register-level only (no HAL), adapted from the hardware-confirmed
   stm32_elrs_bidirectional_test bench rig (retired 2026-10-14, still in git history) (see docs/codex-handoff.md
   for its RF confirmation). That rig put both ES900TX and ES900RX on one
   board for a desk loopback test; this is the real robot-side half of the
   split deployment: USART2/PA2-PA3 still talks to ES900RX exactly as
   before, but USART1/PA9 is repurposed from "half-duplex to ES900TX" to
   "full-duplex to the Mega" (PA10 added as USART1_RX, which the bench
   never used). */
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
#define USART_SR_TXE   (1u << 7)
#define USART_CR1_RE   (1u << 2)
#define USART_CR1_TE   (1u << 3)
#define USART_CR1_RXNEIE (1u << 5)
#define USART_CR1_UE   (1u << 13)

#define MEGA_LINE_BUF_SIZE 160u /* an "ST,..." line is < 30 chars; margin kept */

static volatile uint32_t g_millis = 0;

static uint16_t last_rx_ch_tick[16];
static int last_rx_ch_us[16];

static CrsfParser parser_receiver; /* bytes arriving on USART2 from ES900RX */

static volatile bool downlink_ok = false;
static volatile uint32_t last_rc_rx_ms = 0;

/* Mega -> this board, over USART1: a plain ASCII line (no CRSF framing),
   accumulated byte-by-byte in the IRQ until '\n'. Only "ST,..." status
   lines are used; anything else is dropped (2026-10-17). The old bench
   path relayed any other line as TEL_CHUNK frames every 20 ms, forever --
   so one garbled line (a Mega reset / upload mid-line) kept ~40 custom
   frames a second competing with the status Battery frames for ELRS's few
   telemetry slots, delaying status by seconds. The production Mega never
   sends TEL here. */
static char mega_rx_line[MEGA_LINE_BUF_SIZE];
static uint16_t mega_rx_line_len = 0;

/* Latest "ST,..." status line from the Mega (ManualControl), copied out by
   the IRQ and parsed in the main loop. It goes to the ground station as a
   CRSF Battery frame (rcmap_status_to_battery), the telemetry type already
   proven across this ELRS link. */
static char status_line_ready[MEGA_LINE_BUF_SIZE];
static volatile uint16_t status_line_ready_len = 0;
static volatile bool status_line_pending = false;
static rcmap_status latest_status;
static bool have_status = false;
static uint32_t last_status_ms = 0;
static uint8_t status_rolling = 0;

/* ------------------------------------------------------------------ */
/* Hardware bring-up                                                   */
/* ------------------------------------------------------------------ */

static void prepare_hardware(void) {
  SYST_CSR = 0;
  for (unsigned i = 0; i < 8; ++i) {
    REG32(0xe000e180u + 4u * i) = 0xffffffffu; /* NVIC ICER */
    REG32(0xe000e280u + 4u * i) = 0xffffffffu; /* NVIC ICPR */
  }
  REG32(0xe000ed04u) = (1u << 25) | (1u << 27); /* clear pending SysTick/PendSV */
  SCB_VTOR = APP_BASE;
  __asm volatile ("dsb\nisb" ::: "memory");

  RCC_CR |= 1u;
  while (!(RCC_CR & (1u << 1))) {}
  RCC_CFGR &= ~3u;
  while (RCC_CFGR & (3u << 2)) {}
  RCC_CFGR &= ~((15u << 4) | (7u << 10) | (7u << 13));
  RCC_CR &= ~(1u << 24);
  while (RCC_CR & (1u << 25)) {}

  RCC_AHB2RSTR |= 1u << 7;
  RCC_AHB2RSTR &= ~(1u << 7);
  RCC_AHB1RSTR |= 7u;
  RCC_AHB1RSTR &= ~7u;

  RCC_AHB1ENR |= (1u << 0) | (1u << 2); /* GPIOA, GPIOC */
  (void)RCC_AHB1ENR;

  GPIOC_BSRR = 1u << LED_PIN;
  GPIOC_OTYPER &= ~(1u << LED_PIN);
  GPIOC_OSPEEDR &= ~(3u << (2u * LED_PIN));
  GPIOC_PUPDR &= ~(3u << (2u * LED_PIN));
  GPIOC_MODER = (GPIOC_MODER & ~(3u << (2u * LED_PIN))) | (1u << (2u * LED_PIN));

  SYST_RVR = HSI_HZ / 1000u - 1u;
  SYST_CVR = 0;
  SYST_CSR = 7u;
}

static void usart_gpio_init(void) {
  /* PA9=USART1_TX, PA10=USART1_RX (both AF7, full-duplex to the Mega).
     PA2=USART2_TX, PA3=USART2_RX (AF7, full-duplex to ES900RX, unchanged
     from the bench). */
  const uint32_t pins_9_10_2_3 =
      (3u << (2u * 9)) | (3u << (2u * 10)) | (3u << (2u * 2)) | (3u << (2u * 3));
  GPIOA_MODER &= ~pins_9_10_2_3;
  GPIOA_MODER |= (2u << (2u * 9)) | (2u << (2u * 10)) | (2u << (2u * 2)) | (2u << (2u * 3));

  GPIOA_OTYPER &= ~((1u << 9) | (1u << 10) | (1u << 2) | (1u << 3));
  GPIOA_OSPEEDR |= (3u << (2u * 9)) | (3u << (2u * 10)) | (3u << (2u * 2)) | (3u << (2u * 3));
  GPIOA_PUPDR &= ~pins_9_10_2_3;
  GPIOA_PUPDR |= (1u << (2u * 9)) | (1u << (2u * 10)) | (1u << (2u * 2)) | (1u << (2u * 3));

  GPIOA_AFRH = (GPIOA_AFRH & ~((0xfu << (4u * (9u - 8u))) | (0xfu << (4u * (10u - 8u))))) |
               (7u << (4u * (9u - 8u))) | (7u << (4u * (10u - 8u)));
  GPIOA_AFRL = (GPIOA_AFRL & ~((0xfu << (4u * 2u)) | (0xfu << (4u * 3u)))) |
               (7u << (4u * 2u)) | (7u << (4u * 3u));
}

static void usart1_init(void) {
  RCC_APB2ENR |= (1u << 4);
  (void)RCC_APB2ENR;

  USART1_BRR = USART1_BRR_VALUE;
  USART1_CR2 = 0;
  USART1_CR3 = 0; /* full-duplex: no HDSEL, unlike the bench's ES900TX wiring */
  USART1_CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
}

static void usart2_init(void) {
  RCC_APB1ENR |= (1u << 17);
  (void)RCC_APB1ENR;

  USART2_BRR = USART2_BRR_VALUE;
  USART2_CR2 = 0;
  USART2_CR3 = 0;
  USART2_CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
}

static void nvic_enable_usarts(void) {
  NVIC_ISER1 = (1u << (37u - 32u)) | (1u << (38u - 32u));
}

/* ------------------------------------------------------------------ */
/* Mega link (USART1): send decoded channels, receive the ST line       */
/* ------------------------------------------------------------------ */

static void send_ch_line_to_mega(void) {
  char line[CH_LINE_MAX_CHARS];
  uint16_t len = format_rc_line(line, sizeof(line), last_rx_ch_us);
  for (uint16_t i = 0; i < len; i++) {
    while (!(USART1_SR & USART_SR_TXE)) {}
    USART1_DR = (uint8_t)line[i];
  }
}

static void take_status_line(uint32_t now) {
  if (!status_line_pending) return;
  status_line_pending = false;
  rcmap_status parsed;
  if (parse_status_line(status_line_ready, status_line_ready_len, &parsed)) {
    latest_status = parsed;
    have_status = true;
    last_status_ms = now;
  }
}

/* One Battery frame carrying the Mega's latest status, only while that
   status is fresh: if the Mega stops reporting, the ground sees the status
   go stale instead of a frozen "armed". */
static void send_status_frame(uint32_t now) {
  if (!have_status || (uint32_t)(now - last_status_ms) > STATUS_STALE_MS) return;
  rcmap_status s = latest_status;
  s.rolling = status_rolling++;
  uint16_t voltage, current;
  uint32_t capacity;
  uint8_t remaining;
  rcmap_status_to_battery(&s, &voltage, &current, &capacity, &remaining);
  uint8_t frame[12];
  uint8_t len = crsf_build_battery_frame(frame, voltage, current, capacity, remaining);
  for (uint8_t i = 0; i < len; i++) {
    while (!(USART2_SR & USART_SR_TXE)) {}
    USART2_DR = frame[i];
  }
}

/* ------------------------------------------------------------------ */
/* Status LED: solid while the ES900RX downlink is fresh, slow blink    */
/* once it goes stale. No backlink-health state here (unlike the bench  */
/* rig's 3-state LED): this board cannot observe whether its own        */
/* status frames are actually reaching the ground station without an   */
/* ack scheme this firmware does not implement.                         */
/* ------------------------------------------------------------------ */

static void led_on(void)  { GPIOC_BSRR = 1u << (LED_PIN + 16u); }
static void led_off(void) { GPIOC_BSRR = 1u << LED_PIN; }

static void update_status_led(void) {
  uint32_t now = g_millis;
  bool downlink_now = downlink_ok && (now - last_rc_rx_ms) < DOWNLINK_STALE_MS;

  if (downlink_now) {
    led_on();
    return;
  }
  uint32_t phase = now % LED_SLOW_PERIOD_MS;
  if (phase < LED_SLOW_PERIOD_MS / 2u) led_on(); else led_off();
}

/* ------------------------------------------------------------------ */
/* Interrupt handlers                                                   */
/* ------------------------------------------------------------------ */

void SysTick_Handler(void) {
  g_millis++;
}

void USART1_IRQHandler(void) {
  uint32_t sr = USART1_SR;
  if (sr & USART_SR_RXNE) {
    uint8_t b = (uint8_t)USART1_DR;
    if (b == (uint8_t)'\n') {
      uint16_t len = mega_rx_line_len;
      if (len >= MEGA_LINE_BUF_SIZE) len = MEGA_LINE_BUF_SIZE - 1u;
      if (len >= 3u && mega_rx_line[0] == 'S' && mega_rx_line[1] == 'T' &&
          mega_rx_line[2] == ',') {
        for (uint16_t i = 0; i < len; i++) status_line_ready[i] = mega_rx_line[i];
        status_line_ready_len = len;
        status_line_pending = true;
      } /* any other line: dropped (see mega_rx_line above) */
      mega_rx_line_len = 0;
    } else if (b != (uint8_t)'\r') {
      if (mega_rx_line_len < MEGA_LINE_BUF_SIZE) {
        mega_rx_line[mega_rx_line_len++] = (char)b;
      } else {
        mega_rx_line_len = 0; /* oversize line: drop it and resync on the next '\n' */
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
        downlink_ok = true;
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
  crsf_parser_init(&parser_receiver);
  nvic_enable_usarts();

  for (uint8_t i = 0; i < 16; i++) last_rx_ch_us[i] = 1500;

  __asm volatile ("cpsie i" ::: "memory");

  uint32_t last_ch_send_ms = 0;
  uint32_t last_status_send_ms = 0;

  for (;;) {
    uint32_t now = g_millis;

    if ((now - last_ch_send_ms) >= RC_FORWARD_PERIOD_MS) {
      last_ch_send_ms = now;
      if (ch_line_should_forward(downlink_ok, now, last_rc_rx_ms, DOWNLINK_STALE_MS)) {
        send_ch_line_to_mega();
      }
    }

    take_status_line(now);
    if ((now - last_status_send_ms) >= STATUS_SEND_PERIOD_MS) {
      last_status_send_ms = now;
      send_status_frame(now);
    }

    update_status_led();
  }
}
