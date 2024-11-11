;/**************************************************************************//**
; * @file     startup_M0A21.s
; * @version  V2.00
; * $Revision: 4 $
; * $Date: 20/06/08 3:56p $
; * @brief    M0A21 Series Startup Source File for IAR Platform
; *
; * @note
; * SPDX-License-Identifier: Apache-2.0  
; * Copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
; *
; ******************************************************************************/

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


    MODULE  ?cstartup

    ;; Forward declaration of sections.
    SECTION CSTACK:DATA:NOROOT(3) ;; 8 bytes alignment

    SECTION .intvec:CODE:NOROOT(2);; 4 bytes alignment

    EXTERN  SystemInit
    EXTERN  __iar_program_start
    EXTERN  ProcessHardFault       
    PUBLIC  __vector_table

    DATA
__vector_table
    DCD     sfe(CSTACK)
    DCD     Reset_Handler
    DCD     NMI_Handler
    DCD     HardFault_Handler
    DCD     0
    DCD     0
    DCD     0
    DCD     0
    DCD     0
    DCD     0
    DCD     0
    DCD     SVC_Handler
    DCD     0
    DCD     0
    DCD     PendSV_Handler
    DCD     SysTick_Handler

    ; External Interrupts
    DCD     BOD_IRQHandler              ; Brownout low voltage detected interrupt
    DCD     WDT_IRQHandler              ; Watch Dog Timer interrupt
    DCD     EINT024_IRQHandler          ; External interrupt from group 0/2/4
    DCD     EINT135_IRQHandler          ; External interrupt from group 1/3/5
    DCD     GPAB_IRQHandler             ; External interrupt from PA[15:0]/PB[15:0]
    DCD     GPCD_IRQHandler             ; External interrupt from PC[15:0]/PD[15:0]
    DCD     PWM0_IRQHandler             ; PWM0 interrupt
    DCD     Default_Handler             ; Reserved
    DCD     TMR0_IRQHandler             ; Timer 0 interrupt
    DCD     TMR1_IRQHandler             ; Timer 1 interrupt
    DCD     TMR2_IRQHandler             ; Timer 2 interrupt
    DCD     TMR3_IRQHandler             ; Timer 3 interrupt
    DCD     UART0_IRQHandler            ; UART0 interrupt
    DCD     UART1_IRQHandler            ; UART1 interrupt
    DCD     Default_Handler             ; Reserved
    DCD     CAN0_IRQHandler             ; CAN0 interrupt
    DCD     Default_Handler             ; Reserved
    DCD     Default_Handler             ; Reserved
    DCD     Default_Handler             ; Reserved
    DCD     Default_Handler             ; Reserved
    DCD     Default_Handler             ; Reserved
    DCD     Default_Handler             ; Reserved
    DCD     USCI0_IRQHandler            ; USCI0 interrupt
    DCD     Default_Handler             ; Reserved
    DCD     DAC0_IRQHandler             ; DAC0 interrupt
    DCD     ACMP01_IRQHandler           ; ACMP0/1 interrupt
    DCD     PDMA_IRQHandler             ; PDMA interrupt
    DCD     USCI1_IRQHandler            ; USCI1 interrupt
    DCD     PWRWU_IRQHandler            ; Clock controller interrupt for chip wake up from power-
    DCD     ADC_IRQHandler              ; ADC interrupt
    DCD     CLKFAIL_IRQHandler          ; Clock fail detect and IRC TRIM interrupt
    DCD     Default_Handler             ; Reserved

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;
;; Default interrupt handlers.
;;
    THUMB
    PUBWEAK Reset_Handler
    SECTION .text:CODE:REORDER:NOROOT(2)      ; 4 bytes alignment
Reset_Handler

        LDR      R0, =SystemInit
        BLX      R0
        LDR      R0, =__iar_program_start
        BX       R0

    PUBWEAK HardFault_Handler
    SECTION .text:CODE:REORDER:NOROOT(2)
HardFault_Handler
        MOV     R0, LR
        MRS     R1, MSP
        MRS     R2, PSP
        LDR     R3, =ProcessHardFault
        BLX     R3
        BX      R0
        
    PUBWEAK NMI_Handler
    PUBWEAK SVC_Handler
    PUBWEAK PendSV_Handler
    PUBWEAK SysTick_Handler
    PUBWEAK BOD_IRQHandler
    PUBWEAK WDT_IRQHandler
    PUBWEAK EINT024_IRQHandler
    PUBWEAK EINT135_IRQHandler
    PUBWEAK GPAB_IRQHandler
    PUBWEAK GPCD_IRQHandler
    PUBWEAK PWM0_IRQHandler
    PUBWEAK TMR0_IRQHandler
    PUBWEAK TMR1_IRQHandler
    PUBWEAK TMR2_IRQHandler
    PUBWEAK TMR3_IRQHandler
    PUBWEAK UART0_IRQHandler
    PUBWEAK UART1_IRQHandler
    PUBWEAK CAN0_IRQHandler
    PUBWEAK USCI0_IRQHandler
    PUBWEAK DAC0_IRQHandler
    PUBWEAK ACMP01_IRQHandler
    PUBWEAK PDMA_IRQHandler
    PUBWEAK USCI1_IRQHandler
	PUBWEAK PWRWU_IRQHandler
    PUBWEAK ADC_IRQHandler
	PUBWEAK CLKFAIL_IRQHandler
    SECTION .text:CODE:REORDER:NOROOT(2)

HardFault_Handler
NMI_Handler
SVC_Handler
PendSV_Handler
SysTick_Handler
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
CLKFAIL_IRQHandler
Default_Handler

    B Default_Handler

;int32_t SH_DoCommand(int32_t n32In_R0, int32_t n32In_R1, int32_t *pn32Out_R0)
          PUBWEAK SH_DoCommand
          SECTION .text:CODE:REORDER:ROOT(2)
SH_DoCommand
                IMPORT      SH_Return

                BKPT    0xAB                ; Wait ICE or HardFault
                LDR     R3, =SH_Return
		PUSH    {R3 ,lr}
                BLX     R3                  ; Call SH_Return. The return value is in R0
		POP     {R3 ,PC}            ; Return value = R0

    END
