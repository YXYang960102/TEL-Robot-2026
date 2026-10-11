#pragma once

/* Fresh, never-flashed WeAct STM32F401CCU6 (2026-10-11): no WeAct HID
   bootloader is assumed, so the image links at the start of flash and is
   loaded with the chip's built-in ROM USB DFU bootloader (BOOT0). See
   README.md. */
#define APP_BASE 0x08000000u
#define HSI_HZ 16000000u
#define LED_PIN 13u /* PC13, active low on the WeAct board */

/* USART1: PA9 TX -> Mega D17 (RX2), PA10 RX <- Mega D16 (TX2), 115200 8N1
   full duplex. Same pins, baud and BRR as tools/stm32_elrs_robot_bridge.
   USARTDIV = 16000000/(16*115200) = 8.6806 -> mantissa 8, fraction 11 ->
   BRR 0x8B -> 115107.9 baud (-0.08%). */
#define USART1_BRR_VALUE 0x8Bu

#define RX_RING_SIZE 512u /* power of two */

#define LED_IDLE_PERIOD_MS 1000u
#define LED_LINK_DOWN_PERIOD_MS 200u
