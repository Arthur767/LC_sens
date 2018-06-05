/**
 *******************************************************************************
 @copyright SHENZHEN HAC TELECOM TECHNOLOGY CO.,LTD
 @file      dv_uart.h
 @author    MZX
 @mcu       EFM32TG210F32/EFM32G840F32
 @compiler  IAR EWARM 7.1
 @date      2015-12-17
 @brief     �������ļ�

 @history   V1.00
 //==============================================================================

 *******************************************************************************/
#ifndef __DV_UART_H__
#define __DV_UART_H__
#include "em_usart.h"
//��
//#include "efm32_usart.h"
//#include "efm32_cmu.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/****************************************************************
 @brief : ���ڳ�ʼ��
 @param : 
           uart - Ҫ��ʼ���Ĵ��ڣ���UART1
           pUartInit - ��ʼ������
 @return :  ${return_type}
 @Note :${time}
 ****************************************************************/
//void InitUsart(USART_TypeDef *uart,USART_InitAsync_TypeDef *pUartInit);


/****************************************************************
 @brief : ���ڷ���ʹ������
 @param : 
           en - true ʹ�ܣ�false�ر�
 @return :  ${return_type}
 @Note :${time}
 ****************************************************************/
void UartTxEnable(uint8_t en);


/****************************************************************
 @brief : ���ڽ���ʹ������
 @param : 
           en - true ʹ�ܣ�false�ر�
 @return :  ${return_type}
 @Note :${time}
 ****************************************************************/
void UartRxEnable(uint8_t en);


/****************************************************************
 @brief : ���ڷ��ʹ���
 @param : none
 @return :  ${return_type}
 @Note :${time}
 ****************************************************************/
void Uart_TXBLHandler(USART_TypeDef *uart);


/****************************************************************
 @brief : ���ڽ��մ���
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