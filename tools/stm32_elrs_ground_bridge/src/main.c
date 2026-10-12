#include <stdint.h>
#include <stdbool.h>
#include "config.h"
#include "crsf.h"
#include "tel_reassembler.h"
#include "control_input_scaling.h"

/* STM32F401 RM0368 register map + Cortex-M4 core registers. Bare-metal,
   register-level only (no HAL), adapted from the hardware-confirmed
   tools/stm32_elrs_bidirectional_test bench rig (see docs/codex-handoff.md).
   That rig put both ES900TX and ES900RX on one board for a desk loopback
   test; this is the real ground-station half of the split deployment:
   USART1/PA9 still talks to ES900TX exactly as before (half-duplex,
   confirmed over real RF), but USART2/PA2-PA3 is repurposed from "talks to
   ES900RX" to "drives an external USB-UART adapter feeding the dashboard
   laptop" -- this board has no RX module. GPIOA PA0/PA1 (ADC) and
   PA4-PA7 (digital) are new: the bench rig never read any control input,
   it only ever sent synthetic demo channel values. */
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
#define GPIOA_IDR      REG32(0x40020010u)
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

#define ADC1_SR        REG32(0x40012000u)
#define ADC1_CR1       REG32(0x40012004u)
#define ADC1_CR2       REG32(0x40012008u)
#define ADC1_SMPR2     REG32(0x40012010u)
#define ADC1_SQR3      REG32(0x40012034u)
#define ADC1_DR        REG32(0x4001204cu)

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

#define ADC_SR_EOC     (1u << 1)
#define ADC_CR2_ADON   (1u << 0)
#define ADC_CR2_SWSTART (1u << 30)

static volatile uint32_t g_millis = 0;

/* Channel order/meaning: tools/stm32_elrs_common/src/tel_channel_map.h. */
static int tx_ch_us[16] = {
  1500, 1500, 1500, 1000, 1000, 1500, 1500, 1500,
  1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500
};

static CrsfParser parser_tx_module; /* bytes arriving on USART1 from ES900TX */

static volatile uint32_t last_valid_tel_chunk_ms = 0;
static TelReassembler tel_reassembler;

/* ------------------------------------------------------------------ */
/* Hardware bring-up                                                   */
/* ------------------------------------------------------------------ */

static void prepare_hardware(void) {
  SYST_CSR = 0;
  for (unsigned i = 0; i < 8; ++i) {
    REG32(0xe000e180u + 4u * i) = 0xffffffffu;
    REG32(0xe000e280u + 4u * i) = 0xffffffffu;
  }
  REG32(0xe000ed04u) = (1u << 25) | (1u << 27);
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
  /* PA9 = USART1_TX (AF7), half-duplex single wire to ES900TX, unchanged
     from the bench. PA2/PA3 = USART2_TX/RX (AF7), full-duplex to the
     USB-UART adapter. */
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

static void control_input_gpio_init(void) {
  /* PA0/PA1 analog mode (MODER=11) for ADC1_IN0/IN1. PA4-PA7 input with
     pull-up (MODER=00, PUPDR=01), active-low switches (see config.h). */
  const uint32_t analog_pins = (3u << (2u * ADC_PIN_DRIVE_FORWARD)) | (3u << (2u * ADC_PIN_DRIVE_TURN));
  GPIOA_MODER |= analog_pins;

  const uint32_t digital_pins =
      (3u << (2u * GPIO_PIN_MODE_BIT0)) | (3u << (2u * GPIO_PIN_MODE_BIT1)) |
      (3u << (2u * GPIO_PIN_FIRE_BUTTON)) | (3u << (2u * GPIO_PIN_STARTING_SIDE));
  GPIOA_MODER &= ~digital_pins; /* 00 = input */
  GPIOA_PUPDR &= ~digital_pins;
  GPIOA_PUPDR |= (1u << (2u * GPIO_PIN_MODE_BIT0)) | (1u << (2u * GPIO_PIN_MODE_BIT1)) |
                 (1u << (2u * GPIO_PIN_FIRE_BUTTON)) | (1u << (2u * GPIO_PIN_STARTING_SIDE));
}

static void adc1_init(void) {
  RCC_APB2ENR |= (1u << 8); /* ADC1EN */
  (void)RCC_APB2ENR;
  ADC1_CR2 = ADC_CR2_ADON; /* power on; single-conversion, software-triggered */
  ADC1_CR1 = 0;
}

static void usart1_init(void) {
  RCC_APB2ENR |= (1u << 4);
  (void)RCC_APB2ENR;

  USART1_BRR = USART1_BRR_VALUE;
  USART1_CR2 = 0;
  USART1_CR3 = USART_CR3_HDSEL;
  USART1_CR1 = USART_CR1_UE | USART_CR1_RE | USART_CR1_RXNEIE;
}

static void usart2_init(void) {
  RCC_APB1ENR |= (1u << 17);
  (void)RCC_APB1ENR;

  USART2_BRR = USART2_BRR_VALUE;
  USART2_CR2 = 0;
  USART2_CR3 = 0;
  USART2_CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}

static void nvic_enable_usarts(void) {
  NVIC_ISER1 = 1u << (37u - 32u); /* USART1 only: USART2 is polled, not IRQ-driven */
}

/* ------------------------------------------------------------------ */
/* USART1 half-duplex direction control (unchanged from the bench)      */
/* ------------------------------------------------------------------ */

static void usart1_switch_to_tx(void) {
  USART1_CR1 = (USART1_CR1 & ~USART_CR1_RE) | USART_CR1_TE;
}

static void usart1_switch_to_rx(void) {
  USART1_CR1 = (USART1_CR1 & ~USART_CR1_TE) | USART_CR1_RE;
}

/* ------------------------------------------------------------------ */
/* Control inputs -> tx_ch_us                                          */
/* ------------------------------------------------------------------ */

static uint16_t read_adc_channel(uint8_t channel) {
  ADC1_SQR3 = channel;
  ADC1_CR2 |= ADC_CR2_SWSTART;
  while (!(ADC1_SR & ADC_SR_EOC)) {}
  uint16_t value = (uint16_t)(ADC1_DR & 0x0FFFu); /* reading DR clears EOC */
  return value;
}

static bool read_digital_active_low(uint8_t pin) {
  return (GPIOA_IDR & (1u << pin)) == 0u;
}

static void update_control_inputs(void) {
  tx_ch_us[0] = (int)adc_axis_to_pulse_us(read_adc_channel(ADC_PIN_DRIVE_FORWARD));
  tx_ch_us[1] = (int)adc_axis_to_pulse_us(read_adc_channel(ADC_PIN_DRIVE_TURN));
  tx_ch_us[2] = 1500; /* mechanism: no input assigned yet */
  tx_ch_us[3] = (int)digital_switch_to_pulse_us(read_digital_active_low(GPIO_PIN_STARTING_SIDE));
  tx_ch_us[4] = (int)digital_switch_to_pulse_us(read_digital_active_low(GPIO_PIN_FIRE_BUTTON));
  tx_ch_us[5] = (int)mode_switch_to_pulse_us(read_digital_active_low(GPIO_PIN_MODE_BIT0),
                                              read_digital_active_low(GPIO_PIN_MODE_BIT1));
}

/* ------------------------------------------------------------------ */
/* CRSF send (RC to ES900TX) / receive (TEL_CHUNK from ES900TX)         */
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
}

/* Polled (not IRQ-driven): this board has no other USART2 traffic to
   interleave with, so a blocking write from the main loop is simplest. */
static void send_line_to_dashboard(const char *line, uint16_t len) {
  for (uint16_t i = 0; i < len; i++) {
    while (!(USART2_SR & USART_SR_TXE)) {}
    USART2_DR = (uint8_t)line[i];
  }
  while (!(USART2_SR & USART_SR_TXE)) {}
  USART2_DR = (uint8_t)'\n';
}

/* ------------------------------------------------------------------ */
/* Status LED: solid once a valid TEL_CHUNK frame has been seen          */
/* recently (the only backlink-health signal this firmware has, see      */
/* README.md); slow blink once that goes stale. No separate downlink-    */
/* confirmed state: unlike the bench rig's desk loopback, nothing in a   */
/* real deployment echoes the ground station's own RC channels back to  */
/* it, so there is nothing equivalent to compare against.                */
/* ------------------------------------------------------------------ */

static void led_on(void)  { GPIOC_BSRR = 1u << (LED_PIN + 16u); }
static void led_off(void) { GPIOC_BSRR = 1u << LED_PIN; }

static void update_status_led(void) {
  uint32_t now = g_millis;
  bool backlink_now = (now - last_valid_tel_chunk_ms) < BACKLINK_STALE_MS;

  if (backlink_now) {
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
    CrsfFrame frame;
    if (crsf_parser_push(&parser_tx_module, b, &frame)) {
      /* The TX module also emits its own frames on this wire (e.g. link
         statistics); only our own custom type counts as backlink
         evidence here, same discipline the bench rig's Battery-frame
         check used. */
      if (frame.type == CRSF_FRAME_TEL_CHUNK && frame.payload_len >= 2) {
        last_valid_tel_chunk_ms = g_millis;
        uint8_t chunkIndex = frame.payload[0];
        uint8_t chunkCount = frame.payload[1];
        uint8_t dataLen = (uint8_t)(frame.payload_len - 2);
        if (tel_reassembler_push(&tel_reassembler, chunkIndex, chunkCount,
                                  &frame.payload[2], dataLen)) {
          /* Reassembly complete; the main loop picks this up via
             tel_reassembler.buf on its next pass (polled, not pushed from
             here, since USART2 sends are blocking and this is an IRQ). */
        }
      }
    }
  } else if (sr & USART_SR_ORE) {
    (void)USART1_DR;
  }
}

/* startup.S's vector table (shared, unmodified, with
   tools/stm32_elrs_bidirectional_test/tools/stm32_elrs_robot_bridge)
   references USART2_IRQHandler by name regardless of whether this
   firmware enables that interrupt. USART2 is polled here (see
   send_line_to_dashboard()), not IRQ-driven, so this is never actually
   called -- it exists only so the link succeeds. */
void USART2_IRQHandler(void) {}

/* ------------------------------------------------------------------ */
/* Main                                                                 */
/* ------------------------------------------------------------------ */

int main(void) {
  prepare_hardware();
  usart_gpio_init();
  control_input_gpio_init();
  adc1_init();
  usart1_init();
  usart2_init();
  crsf_parser_init(&parser_tx_module);
  tel_reassembler_reset(&tel_reassembler);
  nvic_enable_usarts();

  __asm volatile ("cpsie i" ::: "memory");

  uint32_t last_rc_send_ms = 0;
  uint8_t last_forwarded_mask = 0; /* tel_reassembler.receivedMask value already sent out */

  for (;;) {
    uint32_t now = g_millis;

    update_control_inputs();

    if ((now - last_rc_send_ms) >= RC_SEND_PERIOD_MS) {
      last_rc_send_ms = now;
      send_rc_to_tx_module();
    }

    /* A completed reassembly leaves tel_reassembler.haveExpectedCount/
       receivedMask showing "all bits set"; forward it once, then wait for
       the mask to look different (a new round started) before forwarding
       again, so the same line is not repeated every loop iteration. */
    if (tel_reassembler.haveExpectedCount && tel_reassembler.receivedMask != last_forwarded_mask) {
      uint8_t completeMask = (uint8_t)((1u << tel_reassembler.expectedCount) - 1u);
      if ((tel_reassembler.receivedMask & completeMask) == completeMask) {
        uint16_t len = 0;
        while (tel_reassembler.buf[len] != '\0') len++;
        send_line_to_dashboard(tel_reassembler.buf, len);
        last_forwarded_mask = tel_reassembler.receivedMask;
      }
    }

    update_status_led();
  }
}
