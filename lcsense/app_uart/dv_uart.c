/**
 *******************************************************************************
 @copyright SHENZHEN HAC TELECOM TECHNOLOGY CO.,LTD
 @file      dv_uart.c
 @author    MZX
 @mcu       EFM32TG210F32/EFM32G840F32
 @compiler  IAR EWARM 7.1
 @date      2015-12-17
 @brief     主程序文件

 @history   V1.00
 //==============================================================================

 *******************************************************************************/
#include <stdint.h>
#include <stdbool.h>

#include "em_cmu.h"
#include "dv_uart.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


/****************************************************************
 @brief : ${enclosing_method}
 @param : ${enclosing_type}
 @return :  ${return_type}
 @Note :${time}
 ****************************************************************/
//_Pragma("optimize=none")

static USART_TypeDef *selectUart = 0;


/****************************************************************
 @brief : 串口初始化
 @param : 
           uart - 要初始化的串口，如UART1
           pUartInit - 初始化参数
 @return :  ${return_type}
 @Note :${time}
 ****************************************************************/
void InitUsart(USART_TypeDef *uart,USART_InitAsync_TypeDef *pUartInit)
{
    selectUart = uart;
	CMU_ClockEnable(cmuClock_USART1, true);
	/* Reseting and initializing LEUART0 */
	USART_Reset(selectUart);
	USART_InitAsync(selectUart, pUartInit);
	
	//使能位置2的串口
	selectUart->ROUTE = USART_ROUTE_TXPEN | USART_ROUTE_RXPEN |
		USART_ROUTE_LOCATION_LOC2;
	
	selectUart->IEN |= USART_IEN_RXDATAV;
    
	NVIC_ClearPendingIRQ(USART1_RX_IRQn);
	NVIC_EnableIRQ(USART1_RX_IRQn);
	NVIC_ClearPendingIRQ(USART1_TX_IRQn);
	NVIC_EnableIRQ(USART1_TX_IRQn);
}


/****************************************************************
 @brief : 串口发送使能设置
 @param : 
           en - true 使能，false关闭
 @return :  ${return_type}
 @Note :${time}
 ****************************************************************/
void UartTxEnable(uint8_t en)
{
    if(en)
    {
        selectUart->IEN |= USART_IEN_TXBL;
    }
    else 
    {
        selectUart->IEN &=~USART_IEN_TXBL;
    }
}


/****************************************************************
 @brief : 串口接收使能设置
 @param : 
           en - true 使能，false关闭
 @return :  ${return_type}
 @Note :${time}
 ****************************************************************/
void UartRxEnable(uint8_t en)
{
    if(en)
    {
        selectUart->IEN |= USART_IEN_RXDATAV;
    }
    else 
    {
        selectUart->IEN &=~USART_IEN_RXDATAV;
    }
}


/****************************************************************
 @brief : 串口发送中断，用户需要在自己的串口应用程序中编写字节的处理程序
 @param : ${enclosing_type}
 @return :  ${return_type}
 @Note :${time}
 ****************************************************************/
void USART1_TX_IRQHandler(void)
{
    if((selectUart -> STATUS) & USART_STATUS_TXBL)		//发送BUF未满
    {
        Uart_TXBLHandler(selectUart);
    }
}


/****************************************************************
 @brief : 串口接收中断，用户需要在自己的串口应用程序中编写字节的处理程序
 @param : ${enclosing_type}
 @return :  ${return_type}
 @Note :${time}
 ****************************************************************/
void USART1_RX_IRQHandler()
{
    if((selectUart -> STATUS) & USART_STATUS_RXDATAV)
    {
        Uart_RXAVHandler(selectUart);
    }
}

