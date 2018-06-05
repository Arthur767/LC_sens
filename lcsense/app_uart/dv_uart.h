/**
 *******************************************************************************
 @copyright SHENZHEN HAC TELECOM TECHNOLOGY CO.,LTD
 @file      dv_uart.h
 @author    MZX
 @mcu       EFM32TG210F32/EFM32G840F32
 @compiler  IAR EWARM 7.1
 @date      2015-12-17
 @brief     主程序文件

 @history   V1.00
 //==============================================================================

 *******************************************************************************/
#ifndef __DV_UART_H__
#define __DV_UART_H__
#include "em_usart.h"
//加
//#include "efm32_usart.h"
//#include "efm32_cmu.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/****************************************************************
 @brief : 串口初始化
 @param : 
           uart - 要初始化的串口，如UART1
           pUartInit - 初始化参数
 @return :  ${return_type}
 @Note :${time}
 ****************************************************************/
//void InitUsart(USART_TypeDef *uart,USART_InitAsync_TypeDef *pUartInit);


/****************************************************************
 @brief : 串口发送使能设置
 @param : 
           en - true 使能，false关闭
 @return :  ${return_type}
 @Note :${time}
 ****************************************************************/
void UartTxEnable(uint8_t en);


/****************************************************************
 @brief : 串口接收使能设置
 @param : 
           en - true 使能，false关闭
 @return :  ${return_type}
 @Note :${time}
 ****************************************************************/
void UartRxEnable(uint8_t en);


/****************************************************************
 @brief : 串口发送处理
 @param : none
 @return :  ${return_type}
 @Note :${time}
 ****************************************************************/
void Uart_TXBLHandler(USART_TypeDef *uart);


/****************************************************************
 @brief : 串口接收处理
 @param : none
 @return :  ${return_type}
 @Note :${time}
 ****************************************************************/
void Uart_RXAVHandler(USART_TypeDef *uart);

/* Private functions ---------------------------------------------------------*/


void InitUsart(USART_TypeDef *uart,USART_InitAsync_TypeDef *pUartInit);
/****************************************************************
 @brief : ${enclosing_method}
 @param : ${enclosing_type}
 @return :  ${return_type}
 @Note :${time}
 ****************************************************************/
//_Pragma("optimize=none")

#endif