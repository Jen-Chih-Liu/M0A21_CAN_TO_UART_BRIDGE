;/**************************************************************************//**
; * @file     startup_M0A21.s
; * @version  V2.00
; * $Revision: 4 $
; * $Date: 20/05/22 7:57p $
; * @brief    M0A21 Series Startup Source File for KEIL Platform
; *
; * @note
; * SPDX-License-Identifier: Apache-2.0  
; * Copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
; *
; ******************************************************************************/
    IF :LNOT: :DEF: Stack_Size
Stack_Size      EQU     0x0000200
    ENDIF

                AREA    STACK, NOINIT, READWRITE, ALIGN=3
Stack_Mem       SPACE   Stack_Size
__initial_sp


; <h> Heap Configuration
;   <o>  Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>
    IF :LNOT: :DEF: Heap_Size
Heap_Size       EQU     0x00000000
    ENDIF
                AREA    HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base
Heap_Mem        SPACE   Heap_Size
__heap_limit


                PRESERVE8
                THUMB


; Vector Table Mapped to Address 0 at Reset
                AREA    RESET, DATA, READONLY
                EXPORT  __Vectors

__Vectors       DCD     __initial_sp              ; Top of Stack
                DCD     Reset_Handler             ; Reset Handler
                DCD     NMI_Handler               ; NMI Handler
                DCD     HardFault_Handler         ; Hard Fault Handler
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     SVC_Handler               ; SVCall Handler
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     PendSV_Handler            ; PendSV Handler
                DCD     SysTick_Handler           ; SysTick Handler

                ; External Interrupts
                                                  ; maximum of 32 External Interrupts are possible
                DCD     BOD_IRQHandler
                DCD     WDT_IRQHandler
                DCD     EINT024_IRQHandler
                DCD     EINT135_IRQHandler
                DCD     GPAB_IRQHandler
                DCD     GPCD_IRQHandler
                DCD     PWM0_IRQHandler
                DCD     Default_Handler
                DCD     TMR0_IRQHandler
                DCD     TMR1_IRQHandler
                DCD     TMR2_IRQHandler
                DCD     TMR3_IRQHandler
                DCD     UART0_IRQHandler
                DCD     UART1_IRQHandler
                DCD     Default_Handler
                DCD     CAN0_IRQHandler
                DCD     Default_Handler
                DCD     Default_Handler
                DCD     Default_Handler
                DCD     Default_Handler
                DCD     Default_Handler
                DCD     Default_Handler
                DCD     USCI0_IRQHandler
                DCD     Default_Handler
                DCD     DAC0_IRQHandler
                DCD     ACMP01_IRQHandler
                DCD     PDMA_IRQHandler
                DCD     USCI1_IRQHandler
                DCD     PWRWU_IRQHandler
                DCD     ADC_IRQHandler
                DCD     CKFAIL_IRQHandler
                DCD     Default_Handler

                AREA    |.text|, CODE, READONLY

; Reset Handler

                ENTRY

Reset_Handler   PROC
                EXPORT  Reset_Handler             [WEAK]
                IMPORT  SystemInit
                IMPORT  __main

                LDR     R0, =SystemInit
                BLX     R0
                LDR     R0, =__main
                BX      R0
                ENDP



; Dummy Exception Handlers (infinite loops which can be modified)

NMI_Handler     PROC
                EXPORT  NMI_Handler               [WEAK]
                B       .
                ENDP
HardFault_Handler\
                PROC
                IMPORT  ProcessHardFault
                EXPORT  HardFault_Handler         [WEAK]
                MOV     R0, LR
                MRS     R1, MSP
                MRS     R2, PSP
                LDR     R3, =ProcessHardFault
                BLX     R3
                BX      R0
                ENDP
SVC_Handler     PROC
                EXPORT  SVC_Handler               [WEAK]
                B       .
                ENDP
PendSV_Handler  PROC
                EXPORT  PendSV_Handler            [WEAK]
                B       .
                ENDP
SysTick_Handler PROC
                EXPORT  SysTick_Handler           [WEAK]
                B       .
                ENDP

Default_Handler PROC

                EXPORT  BOD_IRQHandler            [WEAK]
                EXPORT  WDT_IRQHandler            [WEAK]
                EXPORT  EINT024_IRQHandler        [WEAK]
                EXPORT  EINT135_IRQHandler        [WEAK]
                EXPORT  GPAB_IRQHandler           [WEAK]
                EXPORT  GPCD_IRQHandler           [WEAK]
                EXPORT  PWM0_IRQHandler           [WEAK]
                EXPORT  TMR0_IRQHandler           [WEAK]
                EXPORT  TMR1_IRQHandler           [WEAK]
                EXPORT  TMR2_IRQHandler           [WEAK]
                EXPORT  TMR3_IRQHandler           [WEAK]
				EXPORT  UART0_IRQHandler          [WEAK]
				EXPORT  UART1_IRQHandler          [WEAK]
                EXPORT  CAN0_IRQHandler           [WEAK]
                EXPORT  USCI0_IRQHandler          [WEAK]
                EXPORT  DAC0_IRQHandler           [WEAK]
                EXPORT  ACMP01_IRQHandler         [WEAK]
                EXPORT  PDMA_IRQHandler           [WEAK]
                EXPORT  USCI1_IRQHandler          [WEAK]
                EXPORT  PWRWU_IRQHandler          [WEAK]
                EXPORT  ADC_IRQHandler            [WEAK]
                EXPORT  CKFAIL_IRQHandler        [WEAK]

BOD_IRQHandler
WDT_IRQHandler
EINT024_IRQHandler
EINT135_IRQHandler
GPAB_IRQHandler
GPCD_IRQHandler
PWM0_IRQHandler
TMR0_IRQHandler
TMR1_IRQHandler
TMR2_IRQHandler
TMR3_IRQHandler
UART0_IRQHandler
UART1_IRQHandler
CAN0_IRQHandler
USCI0_IRQHandler
DAC0_IRQHandler
ACMP01_IRQHandler
PDMA_IRQHandler
USCI1_IRQHandler
PWRWU_IRQHandler
ADC_IRQHandler
CKFAIL_IRQHandler

                B       .
                ENDP

                ALIGN

; User Initial Stack & Heap

                IF      :DEF:__MICROLIB

                EXPORT  __initial_sp
                EXPORT  __heap_base
                EXPORT  __heap_limit

                ELSE

                IMPORT  __use_two_region_memory
                EXPORT  __user_initial_stackheap
__user_initial_stackheap

                LDR     R0, =  Heap_Mem
                LDR     R1, = (Stack_Mem + Stack_Size)
                LDR     R2, = (Heap_Mem +  Heap_Size)
                LDR     R3, = Stack_Mem
                BX      LR

                ALIGN

                ENDIF

;int32_t SH_DoCommand(int32_t n32In_R0, int32_t n32In_R1, int32_t *pn32Out_R0)
SH_DoCommand    PROC

                EXPORT      SH_DoCommand
                IMPORT      SH_Return

                BKPT   0xAB                ; Wait ICE or HardFault
                LDR    R3, =SH_Return
                PUSH   {R3 ,lr}
                BLX    R3                  ; Call SH_Return. The return value is in R0
                POP    {R3 ,PC}            ; Return value = R0

                ENDP
                
__PC            PROC
                EXPORT      __PC

                MOV     r0, lr
                BLX     lr
                ALIGN

                ENDP
					
                END
