/**
 *******************************************************************************
 @copyright SHENZHEN HAC TELECOM TECHNOLOGY CO.,LTD
 @file      app_uart.c
 @author    MZX
 @mcu       EFM32TG210F32/EFM32G840F32
 @compiler  IAR EWARM 7.1
 @date      2015-12-17
 @brief     主程序文件

 @history   V1.00
 //==============================================================================

 *******************************************************************************/
#ifndef __APP_UART_H__
#define __APP_UART_H__

#include "dv_uart.h"


#define UART_BAUD_RATE    38400

#define UART_TXBUFF_SIZE    100//200
#define UART_RXBUFF_SIZE    100//200

#define user_uart	USART1		//所使用的串口
#define T188_Frame_Head     0x68

/* Private typedef -----------------------------------------------------------*/
typedef enum
{
    uart_parity_none = USART_FRAME_PARITY_NONE,        // 无校验
    uart_parity_even = USART_FRAME_PARITY_EVEN,        // 偶校验
    uart_parity_odd = USART_FRAME_PARITY_ODD          // 奇校验
}UartParity_def;

/////////////////////////////////////////////////////////////////////////////////
//命令类型
typedef enum
{
  Func_PullsGet,
  Func_PullsSet,
  Func_Butn,
}FuncType;

//发送数据结构
typedef struct
{
  uint8_t Preamble[2];  //2个字节FEH
  uint8_t Head;      //帧头0x68
  uint8_t Type;     //类型+方向 下行|0x80
  uint8_t Err;      //错误
  uint8_t Len;      //长度
  uint8_t Cmd[6];      //指令内容和累加校验,结束符16H
}TxBuf_TypeDef;

#define Index_Head      0   //帧头位置
#define Index_Typ       1   //帧类型加方向 下行|0x80
#define Index_Err       2   //错误类型
#define Index_Len       3   //长度位置
#define Index_cnt       4   //计量值位置
#define Index_Max_Len   UART_RXBUFF_SIZE-5-2 //去除Head,typ,Err,len,cs
#define Index_Min       4+1         //最小指令长度不包括结束
#define Index_Max       UART_RXBUFF_SIZE

#define ARRAY_SIZE( ARRAY ) (sizeof (ARRAY) / sizeof (ARRAY[0]))  


extern uint8_t uartTxBuffer[];
extern uint8_t uartRxBuffer[];

// 帧检测超时参数
//extern uint16_t detectTimeOut;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/****************************************************************
 @brief : 串口参数配置
 @param : rate - 目标波特率
          parity - 校验方式
 @return :  ${return_type}
 @Note :${time}
 ****************************************************************/
void UartModeConfig(uint32_t rate,UartParity_def parity);

/****************************************************************
 @brief : 串口初始化
 @param : none
 @return :  ${return_type}
 @Note :${time}
 ****************************************************************/
void UartInit(void);


/****************************************************************
 @brief : 清空当前的串口收发缓存数据
 @param : none
 @return :  none
 @Note :${time}
 ****************************************************************/
void UartFlushBuffer(void);


/****************************************************************
 @brief : 串口发送一个字节
 @param : 
          txbyte - 要发送的字节
 @return :  1 - 发送成功，0 - 发送失败
 @Note :${time}
 ****************************************************************/
uint8_t UartSendByte(uint8_t txbyte);


/****************************************************************
 @brief : 串口发送数据
 @param : 
          pdata - 要发送的字节
          len - 发送的数据字节
 @return :  实际写入的字节数
 @Note :
 1. 如果写入数据大于缓存区空闲空间的大小，超出部分将拒绝写入。
 ****************************************************************/
uint16_t UartSendString(uint8_t *pdata,uint16_t len);


/****************************************************************
 @brief : 获取串口接收缓存区中的数据量
 @param : none
 @return :  缓存区中的数据长度
 @Note :${time}
 ****************************************************************/
uint16_t UartGetValidDataLength(void);


/****************************************************************
 @brief : 从接收缓存中弹出第一个字节
 @param : none
 @return :  弹出的首字节
 @Note :${time}
 ****************************************************************/
uint8_t UartPopFromBuff(void);


 /****************************************************************
 @brief : 取出Buffer中的数据，并删除取出的数据
 @param : pBuff - 接收数据的buffer
          len - 数据长度
 @return :  实际取出的数据长度
 @Note :${time}
 ****************************************************************/
uint16_t UartGetFromBuff(uint8_t *pBuff,uint16_t len);


//串口轮询
void ProcessUART_Task();
//重发轮询控制
void WakeNB_loop();
#endif
