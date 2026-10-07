#include <stdint.h>
#include <stdbool.h>
#include "config.h"
#include "crsf.h"
#include "tel_chunk_split.h"
#include "tel_channel_map.h"
#include "ch_line_format.h"

/* STM32F401 RM0368 register map + Cortex-M4 core registers. Bare-metal,
   register-level only (no HAL), adapted from the hardware-confirmed
   tools/stm32_elrs_bidirectional_test bench rig (see docs/codex-handoff.md
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

#define MEGA_LINE_BUF_SIZE 160u /* Mega's "TEL,..." line is ~100-120 chars; margin kept */

static volatile uint32_t g_millis = 0;

static uint16_t last_rx_ch_tick[16];
static int last_rx_ch_us[16];

static CrsfParser parser_receiver; /* bytes arriving on USART2 from ES900RX */

static volatile bool downlink_ok = false;
static volatile uint32_t last_rc_rx_ms = 0;

/* Mega -> this board, over USART1: a plain ASCII line (no CRSF framing),
   accumulated byte-by-byte in the IRQ until '\n'. No lock/double-buffer
   between the IRQ and the main loop's chunk sender below reading
   tel_line_ready -- same accepted simplification the bench firmware
   documents for its own tx_ch_us (see that file's comment); a new line
   arriving mid-chunk-cycle simply restarts the cycle from chunk 0. */
static char mega_rx_line[MEGA_LINE_BUF_SIZE];
static uint16_t mega_rx_line_len = 0;
static char tel_line_ready[MEGA_LINE_BUF_SIZE];
static uint16_t tel_line_ready_len = 0;
static volatile bool tel_line_pending_reset = false;

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
/* Mega link (USART1): send decoded channels, receive the TEL line      */
/* ------------------------------------------------------------------ */

static void send_ch_line_to_mega(void) {
  char line[CH_LINE_MAX_CHARS];
  uint16_t len = format_ch_line(line, sizeof(line), last_rx_ch_us);
  for (uint16_t i = 0; i < len; i++) {
    while (!(USART1_SR & USART_SR_TXE)) {}
    USART1_DR = (uint8_t)line[i];
  }
}

/* ------------------------------------------------------------------ */
/* ES900RX link (USART2): receive RC, send one TEL_CHUNK frame at a time */
/* ------------------------------------------------------------------ */

static void send_next_tel_chunk(void) {
  static uint8_t chunk_cursor = 0;

  if (tel_line_pending_reset) {
    tel_line_pending_reset = false;
    chunk_cursor = 0;
  }
  if (tel_line_ready_len == 0) {
    return; /* nothing received from the Mega yet */
  }

  uint8_t count = tel_chunk_count(tel_line_ready_len);
  if (chunk_cursor >= count) chunk_cursor = 0;

  const char *ptr;
  uint8_t chunkLen;
  tel_chunk_slice(tel_line_ready, tel_line_ready_len, chunk_cursor, &ptr, &chunkLen);

  uint8_t frame[CRSF_TEL_CHUNK_FRAME_MAX];
  uint8_t frameLen = crsf_build_tel_chunk_frame(frame, chunk_cursor, count,
                                                 (const uint8_t *)ptr, chunkLen);
  for (uint8_t i = 0; i < frameLen; i++) {
    while (!(USART2_SR & USART_SR_TXE)) {}
    USART2_DR = frame[i];
  }

  chunk_cursor = (uint8_t)((chunk_cursor + 1u) % count);
}

/* ------------------------------------------------------------------ */
/* Status LED: solid while the ES900RX downlink is fresh, slow blink    */
/* once it goes stale. No backlink-health state here (unlike the bench  */
/* rig's 3-state LED): this board cannot observe whether its own        */
/* TEL_CHUNK frames are actually reaching the ground bridge without an  */
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
      for (uint16_t i = 0; i < len; i++) tel_line_ready[i] = mega_rx_line[i];
      tel_line_ready_len = len;
      tel_line_pending_reset = true;
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
  uint32_t last_tel_send_ms = 0;

  for (;;) {
    uint32_t now = g_millis;

    if ((now - last_ch_send_ms) >= RC_FORWARD_PERIOD_MS) {
      last_ch_send_ms = now;
      send_ch_line_to_mega();
    }

    if ((now - last_tel_send_ms) >= TEL_CHUNK_SEND_PERIOD_MS) {
      last_tel_send_ms = now;
      send_next_tel_chunk();
    }

    update_status_led();
  }
}
