;/**************************************************************************//**
; * @file     startup_LM3S.s
; * @brief    CMSIS Cortex-M3 Core Device Startup File for
; *           TI Stellaris
; * @version  V3.00
; * @date     19. December 2011
; *
; * @note
; * Copyright (C) 2011 ARM Limited. All rights reserved.
; *
; * @par
; * ARM Limited (ARM) is supplying this software for use with Cortex-M
; * processor based microcontrollers.  This file can be freely distributed
; * within development tools that are supporting such ARM based processors.
; *
; * @par
; * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
; * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
; * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
; * ARM SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
; * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
; *
; ******************************************************************************/


;
; The modules in this file are included in the libraries, and may be replaced
; by any user-defined modules that define the PUBLIC symbol _program_start or
; a user defined start symbol.
; To override the cstartup defined in the library, simply add your modified
; version to the workbench project.
;
; The vector table is normally located at address 0.
; When debugging in RAM, it can be located in RAM, aligned to at least 2^6.
; The name "__vector_table" has special meaning for C-SPY:
; it is where the SP start value is found, and the NVIC vector
; table register (VTOR) is initialized to this address if != 0.
;
; Cortex-M version
;

        MODULE  ?cstartup

        ;; Forward declaration of sections.
        SECTION CSTACK:DATA:NOROOT(3)

        SECTION .intvec:CODE:NOROOT(2)

        EXTERN  __iar_program_start
        EXTERN  SystemInit
        PUBLIC  __vector_table
        PUBLIC  __vector_table_0x1c
        PUBLIC  __Vectors
        PUBLIC  __Vectors_End
        PUBLIC  __Vectors_Size

        DATA

__vector_table
        DCD     sfe(CSTACK)
        DCD     Reset_Handler

        DCD     NMI_Handler
        DCD     HardFault_Handler
        DCD     MemManage_Handler
        DCD     BusFault_Handler
        DCD     UsageFault_Handler
__vector_table_0x1c
        DCD     0
        DCD     0
        DCD     0
        DCD     0
        DCD     SVC_Handler
        DCD     DebugMon_Handler
        DCD     0
        DCD     PendSV_Handler
        DCD     SysTick_Handler

        ; External Interrupts

        DCD     GPIOA_Handler             ; GPIO Port A
        DCD     GPIOB_Handler             ; GPIO Port B
        DCD     GPIOC_Handler             ; GPIO Port C
        DCD     GPIOD_Handler             ; GPIO Port D
        DCD     GPIOE_Handler             ; GPIO Port E
        DCD     UART0_Handler             ; UART0 Rx and Tx
        DCD     UART1_Handler             ; UART1 Rx and Tx
        DCD     SSI0_Handler              ; SSI0 Rx and Tx
        DCD     I2C0_Handler              ; I2C0 Master and Slave
        DCD     PMW0_FAULT_Handler        ; PWM Fault
        DCD     PWM0_0_Handler            ; PWM Generator 0
        DCD     PWM0_1_Handler            ; PWM Generator 1
        DCD     PWM0_2_Handler            ; PWM Generator 2
        DCD     QEI0_Handler              ; Quadrature Encoder 0
        DCD     ADC0SS0_Handler           ; ADC Sequence 0
        DCD     ADC0SS1_Handler           ; ADC Sequence 1
        DCD     ADC0SS2_Handler           ; ADC Sequence 2
        DCD     ADC0SS3_Handler           ; ADC Sequence 3
        DCD     WDT0_Handler              ; Watchdog timer
        DCD     TIMER0A_Handler           ; Timer 0 subtimer A
        DCD     TIMER0B_Handler           ; Timer 0 subtimer B
        DCD     TIMER1A_Handler           ; Timer 1 subtimer A
        DCD     TIMER1B_Handler           ; Timer 1 subtimer B
        DCD     TIMER2A_Handler           ; Timer 2 subtimer A
        DCD     TIMER2B_Handler           ; Timer 2 subtimer B
        DCD     COMP0_Handler             ; Analog Comparator 0
        DCD     COMP1_Handler             ; Analog Comparator 1
        DCD     COMP2_Handler             ; Analog Comparator 2
        DCD     SYSCTL_Handler            ; System Control (PLL, OSC, BO)
        DCD     FLASH_Handler             ; FLASH Control
        DCD     GPIOF_Handler             ; GPIO Port F
        DCD     GPIOG_Handler             ; GPIO Port G
        DCD     GPIOH_Handler             ; GPIO Port H
        DCD     UART2_Handler             ; UART2 Rx and Tx
        DCD     SSI1_Handler              ; SSI1 Rx and Tx
        DCD     TIMER3A_Handler           ; Timer 3 subtimer A
        DCD     TIMER3B_Handler           ; Timer 3 subtimer B
        DCD     I2C1_Handler              ; I2C1 Master and Slave
        DCD     QEI1_Handler              ; Quadrature Encoder 1
        DCD     CAN0_Handler              ; CAN0
        DCD     CAN1_Handler              ; CAN1
        DCD     CAN2_Handler              ; CAN2
        DCD     ETH0_Handler              ; Ethernet
        DCD     HIB_Handler               ; Hibernate
        DCD     USB0_Handler              ; USB0
        DCD     PWM0_3_Handler            ; PWM Generator 3
        DCD     UDMA_Handler              ; uDMA Software Transfer
        DCD     UDMAERR_Handler           ; uDMA Error
        DCD     ADC1SS0_Handler           ; ADC1 Sequence 0
        DCD     ADC1SS1_Handler           ; ADC1 Sequence 1
        DCD     ADC1SS2_Handler           ; ADC1 Sequence 2
        DCD     ADC1SS3_Handler           ; ADC1 Sequence 3
        DCD     I2S0_Handler              ; I2S0
        DCD     EPI0_Handler              ; External Bus Interface 0
        DCD     GPIOJ_Handler             ; GPIO Port J

        DCD     GPIOK_Handler             ; GPIO Port K
        DCD     GPIOL_Handler             ; GPIO Port L
        DCD     SSI2_Handler              ; SSI2 Rx and Tx
        DCD     SSI3_Handler              ; SSI3 Rx and Tx
        DCD     UART3_Handler             ; UART3 Rx and Tx
        DCD     UART4_Handler             ; UART4 Rx and Tx
        DCD     UART5_Handler             ; UART5 Rx and Tx
        DCD     UART6_Handler             ; UART6 Rx and Tx
        DCD     UART7_Handler             ; UART7 Rx and Tx
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     I2C2_Handler              ; I2C2 Master and Slave
        DCD     I2C3_Handler              ; I2C3 Master and Slave
        DCD     TIMER4A_Handler           ; Timer 4 subtimer A
        DCD     TIMER4B_Handler           ; Timer 4 subtimer B
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     TIMER5A_Handler           ; Timer 5 subtimer A
        DCD     TIMER5B_Handler           ; Timer 5 subtimer B
        DCD     WTIMER0A_Handler          ; Wide Timer 0 subtimer A
        DCD     WTIMER0B_Handler          ; Wide Timer 0 subtimer B
        DCD     WTIMER1A_Handler          ; Wide Timer 1 subtimer A
        DCD     WTIMER1B_Handler          ; Wide Timer 1 subtimer B
        DCD     WTIMER2A_Handler          ; Wide Timer 2 subtimer A
        DCD     WTIMER2B_Handler          ; Wide Timer 2 subtimer B
        DCD     WTIMER3A_Handler          ; Wide Timer 3 subtimer A
        DCD     WTIMER3B_Handler          ; Wide Timer 3 subtimer B
        DCD     WTIMER4A_Handler          ; Wide Timer 4 subtimer A
        DCD     WTIMER4B_Handler          ; Wide Timer 4 subtimer B
        DCD     WTIMER5A_Handler          ; Wide Timer 5 subtimer A
        DCD     WTIMER5B_Handler          ; Wide Timer 5 subtimer B
        DCD     FPU_Handler               ; FPU
        DCD     PECI0_Handler             ; PECI 0
        DCD     LPC0_Handler              ; LPC 0
        DCD     I2C4_Handler              ; I2C4 Master and Slave
        DCD     I2C5_Handler              ; I2C5 Master and Slave
        DCD     GPIOM_Handler             ; GPIO Port M
        DCD     GPION_Handler             ; GPIO Port N
        DCD     QEI2_Handler              ; Quadrature Encoder 2
        DCD     FAN0_Handler              ; Fan 0
        DCD     0                         ; Reserved
        DCD     GPIOP0_Handler            ; GPIO Port P (Summary or P0)
        DCD     GPIOP1_Handler            ; GPIO Port P1
        DCD     GPIOP2_Handler            ; GPIO Port P2
        DCD     GPIOP3_Handler            ; GPIO Port P3
        DCD     GPIOP4_Handler            ; GPIO Port P4
        DCD     GPIOP5_Handler            ; GPIO Port P5
        DCD     GPIOP6_Handler            ; GPIO Port P6
        DCD     GPIOP7_Handler            ; GPIO Port P7
        DCD     GPIOQ0_Handler            ; GPIO Port Q (Summary or Q0)
        DCD     GPIOQ1_Handler            ; GPIO Port Q1
        DCD     GPIOQ2_Handler            ; GPIO Port Q2
        DCD     GPIOQ3_Handler            ; GPIO Port Q3
        DCD     GPIOQ4_Handler            ; GPIO Port Q4
        DCD     GPIOQ5_Handler            ; GPIO Port Q5
        DCD     GPIOQ6_Handler            ; GPIO Port Q6
        DCD     GPIOQ7_Handler            ; GPIO Port Q7
        DCD     GPIOR_Handler             ; GPIO Port R
        DCD     GPIOS_Handler             ; GPIO Port S
        DCD     PMW1_0_Handler            ; PWM 1 Generator 0
        DCD     PWM1_1_Handler            ; PWM 1 Generator 1
        DCD     PWM1_2_Handler            ; PWM 1 Generator 2
        DCD     PWM1_3_Handler            ; PWM 1 Generator 3
        DCD     PWM1_FAULT_Handler        ; PWM 1 Fault

__Vectors_End

__Vectors       EQU   __vector_table
__Vectors_Size  EQU   __Vectors_End - __Vectors


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;
;; Default interrupt handlers.
;;
        THUMB

        PUBWEAK Reset_Handler
        SECTION .text:CODE:REORDER:NOROOT(2)
Reset_Handler
        LDR     R0, =SystemInit
        BLX     R0
        LDR     R0, =__iar_program_start
        BX      R0

        PUBWEAK NMI_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
NMI_Handler
        B NMI_Handler

        PUBWEAK HardFault_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
HardFault_Handler
        B HardFault_Handler

        PUBWEAK MemManage_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
MemManage_Handler
        B MemManage_Handler

        PUBWEAK BusFault_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
BusFault_Handler
        B BusFault_Handler

        PUBWEAK UsageFault_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
UsageFault_Handler
        B UsageFault_Handler

        PUBWEAK SVC_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
SVC_Handler
        B SVC_Handler

        PUBWEAK DebugMon_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
DebugMon_Handler
        B DebugMon_Handler

        PUBWEAK PendSV_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
PendSV_Handler
        B PendSV_Handler

        PUBWEAK SysTick_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
SysTick_Handler
        B SysTick_Handler


        PUBWEAK GPIOA_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIOA_Handler
        B GPIOA_Handler

        PUBWEAK GPIOB_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIOB_Handler
        B GPIOB_Handler

        PUBWEAK GPIOC_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIOC_Handler
        B GPIOC_Handler

        PUBWEAK GPIOD_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIOD_Handler
        B GPIOD_Handler

        PUBWEAK GPIOE_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIOE_Handler
        B GPIOE_Handler

        PUBWEAK UART0_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
UART0_Handler
        B UART0_Handler

        PUBWEAK UART1_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
UART1_Handler
        B UART1_Handler

        PUBWEAK SSI0_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
SSI0_Handler
        B SSI0_Handler

        PUBWEAK I2C0_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
I2C0_Handler
        B I2C0_Handler

        PUBWEAK PMW0_FAULT_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
PMW0_FAULT_Handler
        B PMW0_FAULT_Handler

        PUBWEAK PWM0_0_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
PWM0_0_Handler
        B PWM0_0_Handler

        PUBWEAK PWM0_1_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
PWM0_1_Handler
        B PWM0_1_Handler

        PUBWEAK PWM0_2_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
PWM0_2_Handler
        B PWM0_2_Handler

        PUBWEAK QEI0_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
QEI0_Handler
        B QEI0_Handler

        PUBWEAK ADC0SS0_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
ADC0SS0_Handler
        B ADC0SS0_Handler

        PUBWEAK ADC0SS1_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
ADC0SS1_Handler
        B ADC0SS1_Handler

        PUBWEAK ADC0SS2_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
ADC0SS2_Handler
        B ADC0SS2_Handler

        PUBWEAK ADC0SS3_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
ADC0SS3_Handler
        B ADC0SS3_Handler

        PUBWEAK WDT0_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
WDT0_Handler
        B WDT0_Handler

        PUBWEAK TIMER0A_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIMER0A_Handler
        B TIMER0A_Handler

        PUBWEAK TIMER0B_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIMER0B_Handler
        B TIMER0B_Handler

        PUBWEAK TIMER1A_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIMER1A_Handler
        B TIMER1A_Handler

        PUBWEAK TIMER1B_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIMER1B_Handler
        B TIMER1B_Handler

        PUBWEAK TIMER2A_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIMER2A_Handler
        B TIMER2A_Handler

        PUBWEAK TIMER2B_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIMER2B_Handler
        B TIMER2B_Handler

        PUBWEAK COMP0_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
COMP0_Handler
        B COMP0_Handler

        PUBWEAK COMP1_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
COMP1_Handler
        B COMP1_Handler

        PUBWEAK COMP2_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
COMP2_Handler
        B COMP2_Handler

        PUBWEAK SYSCTL_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
SYSCTL_Handler
        B SYSCTL_Handler

        PUBWEAK FLASH_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
FLASH_Handler
        B FLASH_Handler

        PUBWEAK GPIOF_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIOF_Handler
        B GPIOF_Handler

        PUBWEAK GPIOG_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIOG_Handler
        B GPIOG_Handler

        PUBWEAK GPIOH_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIOH_Handler
        B GPIOH_Handler

        PUBWEAK UART2_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
UART2_Handler
        B UART2_Handler

        PUBWEAK SSI1_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
SSI1_Handler
        B SSI1_Handler

        PUBWEAK TIMER3A_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIMER3A_Handler
        B TIMER3A_Handler

        PUBWEAK TIMER3B_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIMER3B_Handler
        B TIMER3B_Handler

        PUBWEAK I2C1_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
I2C1_Handler
        B I2C1_Handler

        PUBWEAK QEI1_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
QEI1_Handler
        B QEI1_Handler

        PUBWEAK CAN0_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
CAN0_Handler
        B CAN0_Handler

        PUBWEAK CAN1_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
CAN1_Handler
        B CAN1_Handler

        PUBWEAK CAN2_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
CAN2_Handler
        B CAN2_Handler

        PUBWEAK ETH0_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
ETH0_Handler
        B ETH0_Handler

        PUBWEAK HIB_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
HIB_Handler
        B HIB_Handler

        PUBWEAK USB0_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
USB0_Handler
        B USB0_Handler

        PUBWEAK PWM0_3_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
PWM0_3_Handler
        B PWM0_3_Handler

        PUBWEAK UDMA_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
UDMA_Handler
        B UDMA_Handler

        PUBWEAK UDMAERR_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
UDMAERR_Handler
        B UDMAERR_Handler

        PUBWEAK ADC1SS0_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
ADC1SS0_Handler
        B ADC1SS0_Handler

        PUBWEAK ADC1SS1_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
ADC1SS1_Handler
        B ADC1SS1_Handler

        PUBWEAK ADC1SS2_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
ADC1SS2_Handler
        B ADC1SS2_Handler

        PUBWEAK ADC1SS3_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
ADC1SS3_Handler
        B ADC1SS3_Handler

        PUBWEAK I2S0_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
I2S0_Handler
        B I2S0_Handler

        PUBWEAK EPI0_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
EPI0_Handler
        B EPI0_Handler

        PUBWEAK GPIOJ_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIOJ_Handler
        B GPIOJ_Handler


        PUBWEAK GPIOK_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIOK_Handler
        B GPIOK_Handler

        PUBWEAK GPIOL_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIOL_Handler
        B GPIOL_Handler

        PUBWEAK SSI2_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
SSI2_Handler
        B SSI2_Handler

        PUBWEAK SSI3_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
SSI3_Handler
        B SSI3_Handler

        PUBWEAK UART3_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
UART3_Handler
        B UART3_Handler

        PUBWEAK UART4_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
UART4_Handler
        B UART4_Handler

        PUBWEAK UART5_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
UART5_Handler
        B UART5_Handler

        PUBWEAK UART6_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
UART6_Handler
        B UART6_Handler

        PUBWEAK UART7_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
UART7_Handler
        B UART7_Handler

        PUBWEAK I2C2_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
I2C2_Handler
        B I2C2_Handler

        PUBWEAK I2C3_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
I2C3_Handler
        B I2C3_Handler

        PUBWEAK TIMER4A_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIMER4A_Handler
        B TIMER4A_Handler

        PUBWEAK TIMER4B_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIMER4B_Handler
        B TIMER4B_Handler

        PUBWEAK TIMER5A_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIMER5A_Handler
        B TIMER5A_Handler

        PUBWEAK TIMER5B_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIMER5B_Handler
        B TIMER5B_Handler

        PUBWEAK WTIMER0A_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
WTIMER0A_Handler
        B WTIMER0A_Handler

        PUBWEAK WTIMER0B_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
WTIMER0B_Handler
        B WTIMER0B_Handler

        PUBWEAK WTIMER1A_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
WTIMER1A_Handler
        B WTIMER1A_Handler

        PUBWEAK WTIMER1B_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
WTIMER1B_Handler
        B WTIMER1B_Handler

        PUBWEAK WTIMER2A_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
WTIMER2A_Handler
        B WTIMER2A_Handler

        PUBWEAK WTIMER2B_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
WTIMER2B_Handler
        B WTIMER2B_Handler

        PUBWEAK WTIMER3A_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
WTIMER3A_Handler
        B WTIMER3A_Handler

        PUBWEAK WTIMER3B_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
WTIMER3B_Handler
        B WTIMER3B_Handler

        PUBWEAK WTIMER4A_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
WTIMER4A_Handler
        B WTIMER4A_Handler

        PUBWEAK WTIMER4B_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
WTIMER4B_Handler
        B WTIMER4B_Handler

        PUBWEAK WTIMER5A_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
WTIMER5A_Handler
        B WTIMER5A_Handler

        PUBWEAK WTIMER5B_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
WTIMER5B_Handler
        B WTIMER5B_Handler

        PUBWEAK FPU_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
FPU_Handler
        B FPU_Handler

        PUBWEAK PECI0_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
PECI0_Handler
        B PECI0_Handler

        PUBWEAK LPC0_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
LPC0_Handler
        B LPC0_Handler

        PUBWEAK I2C4_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
I2C4_Handler
        B I2C4_Handler

        PUBWEAK I2C5_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
I2C5_Handler
        B I2C5_Handler

        PUBWEAK GPIOM_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIOM_Handler
        B GPIOM_Handler

        PUBWEAK GPION_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPION_Handler
        B GPION_Handler

        PUBWEAK QEI2_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
QEI2_Handler
        B QEI2_Handler

        PUBWEAK FAN0_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
FAN0_Handler
        B FAN0_Handler

        PUBWEAK GPIOP0_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIOP0_Handler
        B GPIOP0_Handler

        PUBWEAK GPIOP1_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIOP1_Handler
        B GPIOP1_Handler

        PUBWEAK GPIOP2_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIOP2_Handler
        B GPIOP2_Handler

        PUBWEAK GPIOP3_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIOP3_Handler
        B GPIOP3_Handler

        PUBWEAK GPIOP4_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIOP4_Handler
        B GPIOP4_Handler

        PUBWEAK GPIOP5_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIOP5_Handler
        B GPIOP5_Handler

        PUBWEAK GPIOP6_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIOP6_Handler
        B GPIOP6_Handler

        PUBWEAK GPIOP7_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIOP7_Handler
        B GPIOP7_Handler

        PUBWEAK GPIOQ0_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIOQ0_Handler
        B GPIOQ0_Handler

        PUBWEAK GPIOQ1_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIOQ1_Handler
        B GPIOQ1_Handler

        PUBWEAK GPIOQ2_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIOQ2_Handler
        B GPIOQ2_Handler

        PUBWEAK GPIOQ3_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIOQ3_Handler
        B GPIOQ3_Handler

        PUBWEAK GPIOQ4_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIOQ4_Handler
        B GPIOQ4_Handler

        PUBWEAK GPIOQ5_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIOQ5_Handler
        B GPIOQ5_Handler

        PUBWEAK GPIOQ6_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIOQ6_Handler
        B GPIOQ6_Handler

        PUBWEAK GPIOQ7_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIOQ7_Handler
        B GPIOQ7_Handler

        PUBWEAK GPIOR_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIOR_Handler
        B GPIOR_Handler

        PUBWEAK GPIOS_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIOS_Handler
        B GPIOS_Handler

        PUBWEAK PMW1_0_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
PMW1_0_Handler
        B PMW1_0_Handler

        PUBWEAK PWM1_1_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
PWM1_1_Handler
        B PWM1_1_Handler

        PUBWEAK PWM1_2_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
PWM1_2_Handler
        B PWM1_2_Handler

        PUBWEAK PWM1_3_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
PWM1_3_Handler
        B PWM1_3_Handler

        PUBWEAK PWM1_FAULT_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
PWM1_FAULT_Handler
        B PWM1_FAULT_Handler



        END
