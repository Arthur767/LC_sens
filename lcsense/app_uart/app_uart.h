/**
 *******************************************************************************
 @copyright SHENZHEN HAC TELECOM TECHNOLOGY CO.,LTD
 @file      app_uart.c
 @author    MZX
 @mcu       EFM32TG210F32/EFM32G840F32
 @compiler  IAR EWARM 7.1
 @date      2015-12-17
 @brief     �������ļ�

 @history   V1.00
 //==============================================================================

 *******************************************************************************/
#ifndef __APP_UART_H__
#define __APP_UART_H__

#include "dv_uart.h"


#define UART_BAUD_RATE    38400

#define UART_TXBUFF_SIZE    100//200
#define UART_RXBUFF_SIZE    100//200

#define user_uart	USART1		//��ʹ�õĴ���
#define T188_Frame_Head     0x68

/* Private typedef -----------------------------------------------------------*/
typedef enum
{
    uart_parity_none = USART_FRAME_PARITY_NONE,        // ��У��
    uart_parity_even = USART_FRAME_PARITY_EVEN,        // żУ��
    uart_parity_odd = USART_FRAME_PARITY_ODD          // ��У��
}UartParity_def;

/////////////////////////////////////////////////////////////////////////////////
//��������
typedef enum
{
  Func_PullsGet,
  Func_PullsSet,
  Func_Butn,
}FuncType;

//�������ݽṹ
typedef struct
{
  uint8_t Preamble[2];  //2���ֽ�FEH
  uint8_t Head;      //֡ͷ0x68
  uint8_t Type;     //����+���� ����|0x80
  uint8_t Err;      //����
  uint8_t Len;      //����
  uint8_t Cmd[6];      //ָ�����ݺ��ۼ�У��,������16H
}TxBuf_TypeDef;

#define Index_Head      0   //֡ͷλ��
#define Index_Typ       1   //֡���ͼӷ��� ����|0x80
#define Index_Err       2   //��������
#define Index_Len       3   //����λ��
#define Index_cnt       4   //����ֵλ��
#define Index_Max_Len   UART_RXBUFF_SIZE-5-2 //ȥ��Head,typ,Err,len,cs
#define Index_Min       4+1         //��Сָ��Ȳ���������
#define Index_Max       UART_RXBUFF_SIZE

#define ARRAY_SIZE( ARRAY ) (sizeof (ARRAY) / sizeof (ARRAY[0]))  


extern uint8_t uartTxBuffer[];
extern uint8_t uartRxBuffer[];

// ֡��ⳬʱ����
//extern uint16_t detectTimeOut;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/****************************************************************
 @brief : ���ڲ�������
 @param : rate - Ŀ�겨����
          parity - У�鷽ʽ
 @return :  ${return_type}
 @Note :${time}
 ****************************************************************/
void UartModeConfig(uint32_t rate,UartParity_def parity);

/****************************************************************
 @brief : ���ڳ�ʼ��
 @param : none
 @return :  ${return_type}
 @Note :${time}
 ****************************************************************/
void UartInit(void);


/****************************************************************
 @brief : ��յ�ǰ�Ĵ����շ���������
 @param : none
 @return :  none
 @Note :${time}
 ****************************************************************/
void UartFlushBuffer(void);


/****************************************************************
 @brief : ���ڷ���һ���ֽ�
 @param : 
          txbyte - Ҫ���͵��ֽ�
 @return :  1 - ���ͳɹ���0 - ����ʧ��
 @Note :${time}
 ****************************************************************/
uint8_t UartSendByte(uint8_t txbyte);


/****************************************************************
 @brief : ���ڷ�������
 @param : 
          pdata - Ҫ���͵��ֽ�
          len - ���͵������ֽ�
 @return :  ʵ��д����ֽ���
 @Note :
 1. ���д�����ݴ��ڻ��������пռ�Ĵ�С���������ֽ��ܾ�д�롣
 ****************************************************************/
uint16_t UartSendString(uint8_t *pdata,uint16_t len);


/****************************************************************
 @brief : ��ȡ���ڽ��ջ������е�������
 @param : none
 @return :  �������е����ݳ���
 @Note :${time}
 ****************************************************************/
uint16_t UartGetValidDataLength(void);


/****************************************************************
 @brief : �ӽ��ջ����е�����һ���ֽ�
 @param : none
 @return :  ���������ֽ�
 @Note :${time}
 ****************************************************************/
uint8_t UartPopFromBuff(void);


 /****************************************************************
 @brief : ȡ��Buffer�е����ݣ���ɾ��ȡ��������
 @param : pBuff - �������ݵ�buffer
          len - ���ݳ���
 @return :  ʵ��ȡ�������ݳ���
 @Note :${time}
 ****************************************************************/
uint16_t UartGetFromBuff(uint8_t *pBuff,uint16_t len);


//������ѯ
void ProcessUART_Task();
//�ط���ѯ����
void WakeNB_loop();
#endif
