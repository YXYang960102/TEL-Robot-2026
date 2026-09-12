.syntax unified
.cpu cortex-m4
.thumb
.section .isr_vector,"a",%progbits
.global vectors
vectors:
  .word _stack_top
  .word Reset_Handler
  /* All remaining core and external vectors stop in Default_Handler. */
  .rept 100
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
