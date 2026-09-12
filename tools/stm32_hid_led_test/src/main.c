#include <stdint.h>
#include "config.h"

/* STM32F401 RM0368: RCC, GPIOC. Cortex-M4: SysTick, SCB, NVIC. */
#define REG32(address) (*(volatile uint32_t *)(address))
#define RCC_CR REG32(0x40023800u)
#define RCC_CFGR REG32(0x40023808u)
#define RCC_AHB1RSTR REG32(0x40023810u)
#define RCC_AHB2RSTR REG32(0x40023814u)
#define RCC_AHB1ENR REG32(0x40023830u)
#define GPIOC_MODER REG32(0x40020800u)
#define GPIOC_OTYPER REG32(0x40020804u)
#define GPIOC_OSPEEDR REG32(0x40020808u)
#define GPIOC_PUPDR REG32(0x4002080cu)
#define GPIOC_BSRR REG32(0x40020818u)
#define SYST_CSR REG32(0xe000e010u)
#define SYST_RVR REG32(0xe000e014u)
#define SYST_CVR REG32(0xe000e018u)
#define SCB_VTOR REG32(0xe000ed08u)

static void prepare_hardware(void) {
    /* Do not inherit the HID bootloader's IRQ or SysTick state. */
    SYST_CSR = 0;
    for (unsigned i = 0; i < 8; ++i) {
        REG32(0xe000e180u + 4u * i) = 0xffffffffu; /* NVIC ICER */
        REG32(0xe000e280u + 4u * i) = 0xffffffffu; /* NVIC ICPR */
    }
    REG32(0xe000ed04u) = (1u << 25) | (1u << 27); /* clear pending SysTick/PendSV */
    SCB_VTOR = APP_BASE;
    __asm volatile ("dsb\nisb" ::: "memory");

    /* Switch from any inherited PLL configuration to internal HSI 16 MHz. */
    RCC_CR |= 1u;
    while (!(RCC_CR & (1u << 1))) {}
    RCC_CFGR &= ~3u;
    while (RCC_CFGR & (3u << 2)) {}
    RCC_CFGR &= ~((15u << 4) | (7u << 10) | (7u << 13));
    RCC_CR &= ~(1u << 24); /* PLL off after HSI is selected */
    while (RCC_CR & (1u << 25)) {}

    /* End inherited USB operation; reset A/B/C to remove inherited pin modes.
       GPIO reset preserves the chip's default SWD alternate function. */
    RCC_AHB2RSTR |= 1u << 7;
    RCC_AHB2RSTR &= ~(1u << 7);
    RCC_AHB1RSTR |= 7u;
    RCC_AHB1RSTR &= ~7u;
    RCC_AHB1ENR |= 1u << 2;
    (void)RCC_AHB1ENR;
    GPIOC_BSRR = 1u << LED_PIN; /* active-low LED starts off */
    GPIOC_OTYPER &= ~(1u << LED_PIN);
    GPIOC_OSPEEDR &= ~(3u << (2u * LED_PIN));
    GPIOC_PUPDR &= ~(3u << (2u * LED_PIN));
    GPIOC_MODER = (GPIOC_MODER & ~(3u << (2u * LED_PIN))) |
                  (1u << (2u * LED_PIN));

    SYST_RVR = HSI_HZ / 1000u - 1u; /* 15999: nominal 1 ms */
    SYST_CVR = 0;
    SYST_CSR = 5u; /* core clock + enable, no interrupt */
}

static void delay_ms(uint32_t duration) {
    /* Start at a fresh counter boundary, then poll one rollover per ms. */
    SYST_CVR = 0;
    while (duration--) {
        while (!(SYST_CSR & (1u << 16))) {}
    }
}

int main(void) {
    prepare_hardware();
    for (;;) {
        for (unsigned pulse = 0; pulse < PULSES_PER_GROUP; ++pulse) {
            GPIOC_BSRR = 1u << (LED_PIN + 16u);
            delay_ms(PULSE_MS);
            GPIOC_BSRR = 1u << LED_PIN;
            if (pulse + 1u < PULSES_PER_GROUP) delay_ms(BETWEEN_PULSES_MS);
        }
        delay_ms(GROUP_PAUSE_MS);
    }
}
