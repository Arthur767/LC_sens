
#include <stdint.h>
#include <stdbool.h>

#include "sample.h"
#include "app_uart.h"
#include "Agreement.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static uint16_t txBufferIndex = 0;
static uint16_t rxBufferIndex = 0;
static uint16_t txBufferSize = 0;
static uint16_t rxBufferSize = 0;
uint8_t uartTxBuffer[UART_TXBUFF_SIZE];
uint8_t uartRxBuffer[UART_RXBUFF_SIZE];
static uint8_t ReadRxCnt = 0;  //串口查询控制，置1开始   周期由扫描周期决定
uint8_t ResendFlg = 0,ResendCnt = 0;    //唤醒重发控制

// printf定义
int putchar(int ch)  
{  
  UartSendByte((uint8_t)ch);  
  return ch;  
}   

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/****************************************************************
 @brief : 串口初始化 不开始
 @param : none
 @return :  ${return_type}
 @Note :${time}
 ****************************************************************/
void UartInit(void)
{
    txBufferIndex = 0;
    rxBufferIndex = 0;
    txBufferSize = 0;
    rxBufferSize = 0;
    
    UartModeConfig(UART_BAUD_RATE,USART_FRAME_PARITY_EVEN);       //偶校验
}


/****************************************************************
 @brief : 串口参数配置
 @param : rate - 目标波特率
          parity - 校验方式
 @return :  ${return_type}
 @Note :${time}
 ****************************************************************/
void UartModeConfig(uint32_t rate,UartParity_def parity)
{
    USART_InitAsync_TypeDef Usart1Init =
        {
        .enable         = usartEnable,          /* Activate data reception on LEUn_TX/rx pin. */        //04.30
        .refFreq        = 0,                    /* Inherit the clock frequenzy from the LEUART clock source */
        .baudrate       = rate,             /* Baudrate = 9600 bps */
        .oversampling   = usartOVS16,
        .databits       = usartDatabits8,       /* Each LEUART frame containes 8 databits */
        .parity         = (USART_Parity_TypeDef)parity,        /* No parity bits in use */
        .stopbits       = usartStopbits1,       /* Setting the number of stop bits in a frame to 1 bitperiods */
        };

    InitUsart(USART1,&Usart1Init);
}


/****************************************************************
 @brief : 串口发送处理
 @param : none
 @return :  ${return_type}
 @Note :${time}
 ****************************************************************/
void Uart_TXBLHandler(USART_TypeDef *uart)
{
    if(txBufferSize)
    {
        uart->TXDATA = uartTxBuffer[txBufferIndex];
        txBufferIndex++;
        txBufferIndex %= UART_TXBUFF_SIZE;
        txBufferSize--;
    }
    else
    {
        // 发送缓存区的数据已经发完，关闭串口空中断
        UartTxEnable(false);	//UDRIE0 关闭数据寄存器未满中断
    }
}


/****************************************************************
 @brief : 串口接收处理
 @param : none
 @return :  ${return_type}
 @Note :
 
 ****************************************************************/
void Uart_RXAVHandler(USART_TypeDef *uart)
{
    uint8_t rxdata;
    rxdata = uart->RXDATA;
    uartRxBuffer[(rxBufferIndex + rxBufferSize)%UART_RXBUFF_SIZE] = rxdata;	
    
    // 这里并不处理数据超出缓存区大小的意外，只要接收的数据大小超过了缓存区的大小，基本可以认为出错了!!!
    rxBufferSize++;
}


/****************************************************************
 @brief : 清空当前的串口收发缓存数据
 @param : none
 @return :  none
 @Note :${time}
 ****************************************************************/
void UartFlushBuffer(void)
{
    txBufferIndex = 0;
    rxBufferIndex = 0;
    txBufferSize = 0;
    rxBufferSize = 0;
}


/****************************************************************
 @brief : 串口发送一个字节
 @param : 
          txbyte - 要发送的字节
 @return :  1 - 发送成功，0 - 发送失败
 @Note :${time}
 ****************************************************************/
uint8_t UartSendByte(uint8_t txbyte)
{
    if( txBufferSize < UART_TXBUFF_SIZE)
    {
        uartTxBuffer[(txBufferIndex + txBufferSize) % UART_TXBUFF_SIZE] = txbyte;
        txBufferSize++;
        
        // 使能串口发送
        UartTxEnable(true);
        return 1;
    }
    else
    {
        return 0;
    }
}


/****************************************************************
 @brief : 串口发送数据
 @param : 
          pdata - 要发送的字节
          len - 发送的数据字节
 @return :  实际写入的字节数
 @Note :
 1. 如果写入数据大于缓存区空闲空间的大小，超出部分将拒绝写入。
 ****************************************************************/
uint16_t UartSendString(uint8_t *pdata,uint16_t len)
{
    uint16_t index;
    for(index = 0; index < len; index++ )
    {
        if(!UartSendByte(pdata[index]))
        {
            break;
        }
    }
    return index;
}


/****************************************************************
 @brief : 获取串口接收缓存区中的数据量
 @param : none
 @return :  缓存区中的数据长度
 @Note :${time}
 ****************************************************************/
uint16_t UartGetValidDataLength(void)
{
    return rxBufferSize;
}


/****************************************************************
 @brief : 从接收缓存中弹出第一个字节
 @param : none
 @return :  弹出的首字节
 @Note :${time}
 ****************************************************************/
uint8_t UartPopFromBuff(void)
{
    if( !rxBufferSize )
    {
        // 无数据可弹
        return 0;
    }
    
    uint8_t tmp = uartRxBuffer[rxBufferIndex];
    
    rxBufferIndex++;
    rxBufferIndex %= UART_RXBUFF_SIZE;
    rxBufferSize--;
    
    return tmp;
}

 
 /****************************************************************
	@brief : 取出Buffer中的数据，并删除取出的数据
	@param : pBuff - 接收数据的buffer
					 len - 数据长度
	@return :  实际取出的数据长度
	@Note :${time}
	****************************************************************/
 uint16_t UartGetFromBuff(uint8_t *pBuff,uint16_t len)
 {
		 uint16_t datalen = len;
		 if( datalen > rxBufferSize )
		 {
				 // 没那么多数据可取
				 datalen = rxBufferSize;
		 }
		 for(uint16_t index = 0; index < datalen; index++)
		 {
				 *pBuff++ = uartRxBuffer[rxBufferIndex % UART_RXBUFF_SIZE];
				 rxBufferIndex++;
				 rxBufferIndex = (rxBufferIndex % UART_RXBUFF_SIZE);
				 
		 }
		 rxBufferSize -= datalen;
		 return datalen;				
 }
 
 /****************************************************************
	@brief : 取出Buffer中的数据，但是不影响buf的结构
	@param : pBuff - 接收数据的buffer
					 len - 数据长度
	@return :  实际取出的数据长度
	@Note :${time}
	****************************************************************/
 uint16_t UartPreGetFromBuff(uint8_t *pBuff,uint16_t len)
 {
		 uint16_t datalen = len,rxTmpIndex = rxBufferIndex;
		 if( datalen > rxBufferSize )
		 {
				 // 没那么多数据可取
				 datalen = rxBufferSize;
		 }
		 for(uint16_t index = 0; index < datalen; index++)
		 {
				 *pBuff++ = uartRxBuffer[rxTmpIndex % UART_RXBUFF_SIZE];
				 rxTmpIndex++;
				 rxTmpIndex = (rxTmpIndex % UART_RXBUFF_SIZE);
				 
		 }
		 
		 return datalen;				
 }
 
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

TxBuf_TypeDef pTxBuf_TypeDef = {.Preamble = {0xfe,0xfe},  .Head = 0x68,  .Err = 0};

/***************** add here  ***********************/
//len为sum保存的位置
uint8_t get_sum_data(uint8_t buf[], uint8_t len)                     // len is the length of buf
{
  uint8_t i, sum = 0;
  for (i = 0; i < len - 1; i++)
  {
    sum += buf[i];
  }
  return sum;
}

#if 1   //是否做校验
uint8_t sum_check(uint8_t buf[], uint8_t len)
{
  uint8_t sum;
  sum = get_sum_data(buf, len);
  if (sum == buf[len - 1])
  {
    return 1;
  }
  return 0;
}
#else
uint8_t sum_check(uint8_t buf[], uint8_t len)
{
  return 1;
}
#endif

//找到帧头，否则丢掉一个字节
uint8_t Leuart_lookup_head_frame(uint8_t* pdata)
{
    uint16_t len=0;
    //uint8 buff[300];
    //memset(buff,0,300);
    len=UartGetValidDataLength();
    
    if(len>=Index_Min)
    {
        if(len>Index_Max)   //只取能处理的最大长度
        {
            len=Index_Max;
        }
        UartPreGetFromBuff(pdata,len); //读但是不删除
        
        if((pdata[Index_Head]==T188_Frame_Head)\
           &&(pdata[Index_Len]<Index_Max_Len))
        {
            if(len>=pdata[Index_Len]+Index_Min) //有足够长的数据再往下处理
            {
                len=pdata[Index_Len]+Index_Min;
                asm("nop");
                if(sum_check(pdata,len))
                {
                    //str_copy(pdata,buff,len);
                    UartGetFromBuff(pdata,len);
                    UartFlushBuffer();//清空当前的串口收发缓存数据
                    return (len);
                }
                else
                {
                    UartPopFromBuff(); //丢掉一个字节
                    return 0;
                }
            }
        }
        else
          UartPopFromBuff();

    }
    return  0;
}
  
//_Pragma("optimize=none")
uint8_t Do_ProcessUART_Data_Task()  //处理水表端串口收到的数据
{
  static
  uint8_t up_data[UART_RXBUFF_SIZE];   //水表发过来的数据
  uint32_t Pulse_cnt_tmp = 0;
  if (!UartGetValidDataLength())
    return 0;
      
  if(Leuart_lookup_head_frame(up_data))              //只留data数据长度
  {
    if(up_data[Index_Typ] & 0x80)  //下行命令
    {
      up_data[Index_Typ] &= ~(1<<7);
      switch(up_data[Index_Typ])  //帧类型
      {
        case Func_PullsGet:  //读脉冲 80
          pTxBuf_TypeDef.Type = up_data[Index_Typ];
          pTxBuf_TypeDef.Len = 4;
          Pulse_cnt_tmp = GetPulls();
          pTxBuf_TypeDef.Cmd[0] = Pulse_cnt_tmp >> 24;
          pTxBuf_TypeDef.Cmd[1] = Pulse_cnt_tmp >> 16;
          pTxBuf_TypeDef.Cmd[2] = Pulse_cnt_tmp >> 8;
          pTxBuf_TypeDef.Cmd[3] = Pulse_cnt_tmp;
          pTxBuf_TypeDef.Cmd[4] = get_sum_data((uint8_t*)(&pTxBuf_TypeDef.Head),pTxBuf_TypeDef.Len+5);
          pTxBuf_TypeDef.Cmd[5] = 0x16;     //结束符
          UartSendString((uint8_t*)(&pTxBuf_TypeDef),pTxBuf_TypeDef.Len+8); //缓冲发送
            break;
            
            
        case Func_PullsSet:  //写脉冲 81
          Pulse_cnt_tmp   = up_data[4];
          Pulse_cnt_tmp   = (Pulse_cnt_tmp << 8) | up_data[5];
          Pulse_cnt_tmp   = (Pulse_cnt_tmp << 8) | up_data[6];
          Pulse_cnt_tmp   = (Pulse_cnt_tmp << 8) | up_data[7];
          if (Pulse_cnt_tmp>Flash_Max_meter_cnt)
          {
            pTxBuf_TypeDef.Err = 0;
          }
          else
            pTxBuf_TypeDef.Err = Init_meter_cnt(Pulse_cnt_tmp);
          if(!pTxBuf_TypeDef.Err)
            Pulse_cnt = Pulse_cnt_tmp;
          pTxBuf_TypeDef.Type = up_data[Index_Typ];
          pTxBuf_TypeDef.Len = 0;
          pTxBuf_TypeDef.Cmd[0] = get_sum_data((uint8_t*)(&pTxBuf_TypeDef.Head),pTxBuf_TypeDef.Len+5);
          pTxBuf_TypeDef.Cmd[1] = 0x16;     //结束符
          UartSendString((uint8_t*)(&pTxBuf_TypeDef),pTxBuf_TypeDef.Len+8); //缓冲发送
            break;
      }
      return 1;
    }
    else                    //对方ack
    {
      if(up_data[Index_Typ] == Func_Butn)   //关闭重发
        ResendCnt = 0;
    }
  }
  return 0;
}

  
//唤醒NB模块，带重发
void WakeNB()
{
  pTxBuf_TypeDef.Type = Func_Butn;
  pTxBuf_TypeDef.Len = 0;
  pTxBuf_TypeDef.Cmd[0] = get_sum_data((uint8_t*)(&pTxBuf_TypeDef.Head),pTxBuf_TypeDef.Len+5);
  pTxBuf_TypeDef.Cmd[1] = 0x16;     //结束符
  UartSendString((uint8_t*)(&pTxBuf_TypeDef),pTxBuf_TypeDef.Len+8); //缓冲发送
  
  if(!ResendCnt)
  {
    ResendCnt = 5*3 -1;   //最多2次
  }
}


//重发轮询控制    周期由扫描周期决定
void WakeNB_loop()
{
  if(ResendCnt)
  {
    if(ResendCnt%5 == 0)
      WakeNB();
    ResendCnt--;
  }
}

//串口数据处理轮询
void ProcessUART_Task()
{
  if(ReadRxCnt)   //LE串口启用时使用 串口查询控制，置1开始   周期由扫描周期决定
  {
    if (ReadRxCnt >= 20)
    {
      ReadRxCnt = 0;
    } 	
  	else if(ReadRxCnt >= 2)	//2*60ms 弹4次
    {
      if(Do_ProcessUART_Data_Task())	//返回成功
      {
        ReadRxCnt = 0;
      }
  	}
	ReadRxCnt++;	
  }
}


/****************************************************************
 @brief : LE串口接收处理
 @param : none
 @return :  ${return_type}
 @Note :
 
 ****************************************************************/
void LEUart_RXAVHandler(LEUART_TypeDef *uart)
{
    uint8_t rxdata;
    rxdata = uart->RXDATA;
    uartRxBuffer[(rxBufferIndex + rxBufferSize)%UART_RXBUFF_SIZE] = rxdata;	
    rxBufferSize++;
}

__STATIC_INLINE void LEUART_IntClear(uint32_t flags)
{
      LEUART0 -> IFC = flags;
}
/****************************************************************
 @brief : LE串口接收处理
 ****************************************************************/
void LEUART0_IRQHandler()
{
    if((LEUART0 -> IF) & LEUART_IF_STARTF) //开始帧
    {
      ReadRxCnt = 1;
      LEUART_IntClear(LEUART_IF_STARTF);
    }
    if((LEUART0 -> IF) & LEUART_IF_RXDATAV) //USART_STATUS_RXDATAV
    {
      LEUart_RXAVHandler(LEUART0);
      LEUART_IntClear(LEUART_IF_RXDATAV);
    }
}
