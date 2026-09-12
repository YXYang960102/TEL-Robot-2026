.syntax unified
.cpu cortex-m4
.thumb

/* Vector table layout (word index from the start of this table):
   0  = initial stack pointer
   1  = Reset_Handler
   2-14  = NMI..PendSV (13 core exceptions we don't use) -> Default_Handler
   15 = SysTick_Handler (drives the millisecond tick used for scheduling)
   16-52 = external IRQ0..IRQ36, none used here -> Default_Handler
   53 = IRQ37 = USART1_IRQHandler (verified against the STMicroelectronics
        cmsis_device_f4 stm32f401xc.h IRQn_Type enum: USART1_IRQn = 37)
   54 = IRQ38 = USART2_IRQHandler (USART2_IRQn = 38 in the same header)
   55-101 = remaining IRQs, unused here -> Default_Handler (pads the table to
        the same 100-word-after-reset length as the LED test build). */
.section .isr_vector,"a",%progbits
.global vectors
vectors:
  .word _stack_top
  .word Reset_Handler
  .rept 13
  .word Default_Handler
  .endr
  .word SysTick_Handler
  .rept 37
  .word Default_Handler
  .endr
  .word USART1_IRQHandler
  .word USART2_IRQHandler
  .rept 47
  .word Default_Handler
  .endr

.section .text.Reset_Handler,"ax",%progbits
.global Reset_Handler
.type Reset_Handler,%function
.thumb_func
Reset_Handler:
  cpsid i
  ldr r0, =_stack_top
  msr msp, r0
  movs r0, #0
  msr control, r0
  isb
  ldr r0, =_sidata
  ldr r1, =_sdata
  ldr r2, =_edata
1:
  cmp r1, r2
  bcs 2f
  ldr r3, [r0], #4
  str r3, [r1], #4
  b 1b
2:
  ldr r1, =_sbss
  ldr r2, =_ebss
  movs r3, #0
3:
  cmp r1, r2
  bcs 4f
  str r3, [r1], #4
  b 3b
4:
  bl main
  b Default_Handler
.size Reset_Handler, .-Reset_Handler

.section .text.Default_Handler,"ax",%progbits
.global Default_Handler
.type Default_Handler,%function
.thumb_func
Default_Handler:
  cpsid i
  b .
