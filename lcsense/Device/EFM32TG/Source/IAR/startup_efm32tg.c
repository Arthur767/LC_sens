/**************************************************************************//**
 * @file startup_efm32tg.c
 * @brief CMSIS Compatible EFM32TG startup file in C for IAR EWARM
 * @version 5.4.0
 ******************************************************************************
 * # License
 * <b>Copyright 2017 Silicon Laboratories, Inc. www.silabs.com</b>
 ******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.@n
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.@n
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Silicon Laboratories, Inc.
 * has no obligation to support this Software. Silicon Laboratories, Inc. is
 * providing the Software "AS IS", with no express or implied warranties of any
 * kind, including, but not limited to, any implied warranties of
 * merchantability or fitness for any particular purpose or warranties against
 * infringement of any proprietary rights of a third party.
 *
 * Silicon Laboratories, Inc. will not be liable for any consequential,
 * incidental, or special damages, or any other relief, or for any claim by
 * any third party, arising from your use of this Software.
 *
 *****************************************************************************/

#include "em_device.h"        /* The correct device header file. */
#include <stdbool.h>

#pragma language=extended
#pragma segment="CSTACK"

/* IAR start function */
extern void __iar_program_start(void);
/* CMSIS init function */
extern void SystemInit(void);

/* Auto defined by linker */
extern unsigned char CSTACK$$Limit;

__weak void Reset_Handler(void)
{
  SystemInit();
  __iar_program_start();
}

__weak void NMI_Handler(void)
{
  while (true) {
  }
}

__weak void HardFault_Handler(void)
{
  while (true) {
  }
}

__weak void MemManage_Handler(void)
{
  while (true) {
  }
}

__weak void BusFault_Handler(void)
{
  while (true) {
  }
}

__weak void UsageFault_Handler(void)
{
  while (true) {
  }
}

__weak void SVC_Handler(void)
{
  while (true) {
  }
}

__weak void DebugMon_Handler(void)
{
  while (true) {
  }
}

__weak void PendSV_Handler(void)
{
  while (true) {
  }
}

__weak void SysTick_Handler(void)
{
  while (true) {
  }
}

__weak void DMA_IRQHandler(void)
{
  while (true) {
  }
}

__weak void GPIO_EVEN_IRQHandler(void)
{
  while (true) {
  }
}

__weak void TIMER0_IRQHandler(void)
{
  while (true) {
  }
}

__weak void USART0_RX_IRQHandler(void)
{
  while (true) {
  }
}

__weak void USART0_TX_IRQHandler(void)
{
  while (true) {
  }
}

__weak void ACMP0_IRQHandler(void)
{
  while (true) {
  }
}

__weak void ADC0_IRQHandler(void)
{
  while (true) {
  }
}

__weak void DAC0_IRQHandler(void)
{
  while (true) {
  }
}

__weak void I2C0_IRQHandler(void)
{
  while (true) {
  }
}

__weak void GPIO_ODD_IRQHandler(void)
{
  while (true) {
  }
}

__weak void TIMER1_IRQHandler(void)
{
  while (true) {
  }
}

__weak void USART1_RX_IRQHandler(void)
{
  while (true) {
  }
}

__weak void USART1_TX_IRQHandler(void)
{
  while (true) {
  }
}

__weak void LESENSE_IRQHandler(void)
{
  while (true) {
  }
}

__weak void LEUART0_IRQHandler(void)
{
  while (true) {
  }
}

__weak void LETIMER0_IRQHandler(void)
{
  while (true) {
  }
}

__weak void PCNT0_IRQHandler(void)
{
  while (true) {
  }
}

__weak void RTC_IRQHandler(void)
{
  while (true) {
  }
}

__weak void CMU_IRQHandler(void)
{
  while (true) {
  }
}

__weak void VCMP_IRQHandler(void)
{
  while (true) {
  }
}

__weak void LCD_IRQHandler(void)
{
  while (true) {
  }
}

__weak void MSC_IRQHandler(void)
{
  while (true) {
  }
}

__weak void AES_IRQHandler(void)
{
  while (true) {
  }
}

typedef union {
  void (*pFunc)(void);
  void *topOfStack;
} tVectorEntry;

extern const tVectorEntry __vector_table[];

#pragma data_alignment=256
#pragma location = ".intvec"
const tVectorEntry __vector_table[] = {
  { .topOfStack = &CSTACK$$Limit },  /* With IAR, the CSTACK is defined via */
                                     /* project options settings */

  { Reset_Handler },
  { NMI_Handler },
  { HardFault_Handler },
  { MemManage_Handler },
  { BusFault_Handler },
  { UsageFault_Handler },
  { 0 },
  { 0 },
  { 0 },
  { 0 },
  { SVC_Handler },
  { DebugMon_Handler },
  { 0 },
  { PendSV_Handler },
  { SysTick_Handler },

  { DMA_IRQHandler },       /* 0 */
  { GPIO_EVEN_IRQHandler }, /* 1 */
  { TIMER0_IRQHandler },    /* 2 */
  { USART0_RX_IRQHandler }, /* 3 */
  { USART0_TX_IRQHandler }, /* 4 */
  { ACMP0_IRQHandler },     /* 5 */
  { ADC0_IRQHandler },      /* 6 */
  { DAC0_IRQHandler },      /* 7 */
  { I2C0_IRQHandler },      /* 8 */
  { GPIO_ODD_IRQHandler },  /* 9 */
  { TIMER1_IRQHandler },    /* 10 */
  { USART1_RX_IRQHandler }, /* 11 */
  { USART1_TX_IRQHandler }, /* 12 */
  { LESENSE_IRQHandler },   /* 13 */
  { LEUART0_IRQHandler },   /* 14 */
  { LETIMER0_IRQHandler },  /* 15 */
  { PCNT0_IRQHandler },     /* 16 */
  { RTC_IRQHandler },       /* 17 */
  { CMU_IRQHandler },       /* 18 */
  { VCMP_IRQHandler },      /* 19 */
  { LCD_IRQHandler },       /* 20 */
  { MSC_IRQHandler },       /* 21 */
  { AES_IRQHandler },       /* 22 */
};
