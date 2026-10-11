#include <stdbool.h>
#include <stdint.h>

#include "config.h"
#include "ulink.h"

/* STM32 side of the standalone Mega <-> STM32 UART link test. Register
 * level, no HAL. Clock/GPIO/USART1 bring-up is taken from
 * tools/stm32_elrs_robot_bridge/src/main.c (same PA9/PA10 USART1, same
 * 115200 BRR); USART2/ES900RX is not used here. Protocol: ulink.h, role
 * 'S'. This board has no USB output of its own; all results are read on
 * the Mega's USB port (the Mega relays this board's status lines). */

#define REG32(address) (*(volatile uint32_t *)(address))

#define RCC_CR         REG32(0x40023800u)
#define RCC_CFGR       REG32(0x40023808u)
#define RCC_AHB1RSTR   REG32(0x40023810u)
#define RCC_AHB2RSTR   REG32(0x40023814u)
#define RCC_AHB1ENR    REG32(0x40023830u)
#define RCC_APB2ENR    REG32(0x40023844u)

#define GPIOA_MODER    REG32(0x40020000u)
#define GPIOA_OTYPER   REG32(0x40020004u)
#define GPIOA_OSPEEDR  REG32(0x40020008u)
#define GPIOA_PUPDR    REG32(0x4002000cu)
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

#define NVIC_ISER1     REG32(0xe000e104u)
#define SYST_CSR       REG32(0xe000e010u)
#define SYST_RVR       REG32(0xe000e014u)
#define SYST_CVR       REG32(0xe000e018u)
#define SCB_VTOR       REG32(0xe000ed08u)

#define USART_SR_ORE     (1u << 3)
#define USART_SR_NE      (1u << 2)
#define USART_SR_FE      (1u << 1)
#define USART_SR_RXNE    (1u << 5)
#define USART_SR_TXE     (1u << 7)
#define USART_CR1_RE     (1u << 2)
#define USART_CR1_TE     (1u << 3)
#define USART_CR1_RXNEIE (1u << 5)
#define USART_CR1_UE     (1u << 13)

static volatile uint32_t g_millis = 0;

/* USART1 RX ring: written only by the IRQ, read only by the main loop. */
static volatile char rx_ring[RX_RING_SIZE];
static volatile uint16_t rx_head = 0;
static volatile uint16_t rx_tail = 0;
/* Bytes lost below the protocol (hardware overrun/framing/noise, or ring
   full). Folded into the protocol's "bad" counter so the Mega sees them. */
static volatile uint32_t rx_hw_errors = 0;

static ulink_endpoint link;

static void prepare_hardware(void) {
  SYST_CSR = 0;
  for (unsigned i = 0; i < 8; ++i) {
    REG32(0xe000e180u + 4u * i) = 0xffffffffu; /* NVIC ICER */
    REG32(0xe000e280u + 4u * i) = 0xffffffffu; /* NVIC ICPR */
  }
  REG32(0xe000ed04u) = (1u << 25) | (1u << 27);
  SCB_VTOR = APP_BASE;
  __asm volatile ("dsb\nisb" ::: "memory");

  /* Back to plain 16 MHz HSI with the PLL off, whatever the ROM DFU
     bootloader left configured (it runs USB from HSE/PLL). */
  RCC_CR |= 1u;
  while (!(RCC_CR & (1u << 1))) {}
  RCC_CFGR &= ~3u;
  while (RCC_CFGR & (3u << 2)) {}
  RCC_CFGR &= ~((15u << 4) | (7u << 10) | (7u << 13));
  RCC_CR &= ~(1u << 24);
  while (RCC_CR & (1u << 25)) {}

  RCC_AHB2RSTR |= 1u << 7; /* USB OTG FS reset (left running by DFU) */
  RCC_AHB2RSTR &= ~(1u << 7);
  RCC_AHB1RSTR |= 5u;      /* GPIOA, GPIOC */
  RCC_AHB1RSTR &= ~5u;

  RCC_AHB1ENR |= (1u << 0) | (1u << 2);
  (void)RCC_AHB1ENR;

  GPIOC_BSRR = 1u << LED_PIN; /* LED off (active low) */
  GPIOC_OTYPER &= ~(1u << LED_PIN);
  GPIOC_OSPEEDR &= ~(3u << (2u * LED_PIN));
  GPIOC_PUPDR &= ~(3u << (2u * LED_PIN));
  GPIOC_MODER = (GPIOC_MODER & ~(3u << (2u * LED_PIN))) | (1u << (2u * LED_PIN));

  SYST_RVR = HSI_HZ / 1000u - 1u;
  SYST_CVR = 0;
  SYST_CSR = 7u;
}

static void usart1_init(void) {
  /* PA9 = USART1_TX, PA10 = USART1_RX, AF7, pull-ups (idle-high line). */
  const uint32_t pins = (3u << (2u * 9)) | (3u << (2u * 10));
  GPIOA_MODER = (GPIOA_MODER & ~pins) | (2u << (2u * 9)) | (2u << (2u * 10));
  GPIOA_OTYPER &= ~((1u << 9) | (1u << 10));
  GPIOA_OSPEEDR |= pins;
  GPIOA_PUPDR = (GPIOA_PUPDR & ~pins) | (1u << (2u * 9)) | (1u << (2u * 10));
  GPIOA_AFRH = (GPIOA_AFRH & ~((0xfu << 4) | (0xfu << 8))) | (7u << 4) | (7u << 8);

  RCC_APB2ENR |= (1u << 4);
  (void)RCC_APB2ENR;
  USART1_BRR = USART1_BRR_VALUE;
  USART1_CR2 = 0;
  USART1_CR3 = 0;
  USART1_CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
  NVIC_ISER1 = 1u << (37u - 32u);
}

static void usart1_write(void *ctx, const char *data, uint16_t len) {
  (void)ctx;
  for (uint16_t i = 0; i < len; i++) {
    while (!(USART1_SR & USART_SR_TXE)) {}
    USART1_DR = (uint8_t)data[i];
  }
}

static void led(bool on) {
  GPIOC_BSRR = on ? (1u << (LED_PIN + 16u)) : (1u << LED_PIN);
}

/* LED is a convenience only, never pass/fail evidence: slow blink = no run,
   solid = run active and ACKs coming back, fast blink = run active but the
   link is down. */
static void update_led(uint32_t now) {
  if (!link.sending) {
    led((now % LED_IDLE_PERIOD_MS) < LED_IDLE_PERIOD_MS / 2u);
  } else if (link.link_up) {
    led(true);
  } else {
    led((now % LED_LINK_DOWN_PERIOD_MS) < LED_LINK_DOWN_PERIOD_MS / 2u);
  }
}

void SysTick_Handler(void) {
  g_millis++;
}

void USART1_IRQHandler(void) {
  const uint32_t sr = USART1_SR;
  if (sr & (USART_SR_ORE | USART_SR_NE | USART_SR_FE)) {
    rx_hw_errors++;
  }
  if (sr & (USART_SR_RXNE | USART_SR_ORE)) {
    const char c = (char)USART1_DR; /* reading DR also clears ORE/NE/FE */
    const uint16_t next = (uint16_t)((rx_head + 1u) & (RX_RING_SIZE - 1u));
    if (next == rx_tail) {
      rx_hw_errors++; /* ring full: byte dropped */
    } else {
      rx_ring[rx_head] = c;
      rx_head = next;
    }
  }
}

int main(void) {
  prepare_hardware();
  usart1_init();
  ulink_init(&link, 'S', usart1_write, 0, g_millis);
  __asm volatile ("cpsie i" ::: "memory");

  uint32_t hw_errors_seen = 0;
  for (;;) {
    const uint32_t now = g_millis;

    while (rx_tail != rx_head) {
      const char c = rx_ring[rx_tail];
      rx_tail = (uint16_t)((rx_tail + 1u) & (RX_RING_SIZE - 1u));
      ulink_rx_byte(&link, c, now);
    }

    const uint32_t hw_errors = rx_hw_errors;
    if (hw_errors != hw_errors_seen) {
      link.st.bad += hw_errors - hw_errors_seen;
      hw_errors_seen = hw_errors;
    }

    ulink_poll(&link, now);
    update_led(now);
  }
}
