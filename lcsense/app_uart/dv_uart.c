/**
 *******************************************************************************
 @copyright SHENZHEN HAC TELECOM TECHNOLOGY CO.,LTD
 @file      dv_uart.c
 @author    MZX
 @mcu       EFM32TG210F32/EFM32G840F32
 @compiler  IAR EWARM 7.1
 @date      2015-12-17
 @brief     �������ļ�

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
 @brief : ���ڳ�ʼ��
 @param : 
           uart - Ҫ��ʼ���Ĵ��ڣ���UART1
           pUartInit - ��ʼ������
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
	
	//ʹ��λ��2�Ĵ���
	selectUart->ROUTE = USART_ROUTE_TXPEN | USART_ROUTE_RXPEN |
		USART_ROUTE_LOCATION_LOC2;
	
	selectUart->IEN |= USART_IEN_RXDATAV;
    
	NVIC_ClearPendingIRQ(USART1_RX_IRQn);
	NVIC_EnableIRQ(USART1_RX_IRQn);
	NVIC_ClearPendingIRQ(USART1_TX_IRQn);
	NVIC_EnableIRQ(USART1_TX_IRQn);
}


/****************************************************************
 @brief : ���ڷ���ʹ������
 @param : 
           en - true ʹ�ܣ�false�ر�
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
 @brief : ���ڽ���ʹ������
 @param : 
           en - true ʹ�ܣ�false�ر�
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
 @brief : ���ڷ����жϣ��û���Ҫ���Լ��Ĵ���Ӧ�ó����б�д�ֽڵĴ������
 @param : ${enclosing_type}
 @return :  ${return_type}
 @Note :${time}
 ****************************************************************/
void USART1_TX_IRQHandler(void)
{
    if((selectUart -> STATUS) & USART_STATUS_TXBL)		//����BUFδ��
    {
        Uart_TXBLHandler(selectUart);
    }
}


/****************************************************************
 @brief : ���ڽ����жϣ��û���Ҫ���Լ��Ĵ���Ӧ�ó����б�д�ֽڵĴ������
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

