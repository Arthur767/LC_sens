
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
static uint8_t ReadRxCnt = 0;  //���ڲ�ѯ���ƣ���1��ʼ   ������ɨ�����ھ���
uint8_t ResendFlg = 0,ResendCnt = 0;    //�����ط�����

// printf����
int putchar(int ch)  
{  
  UartSendByte((uint8_t)ch);  
  return ch;  
}   

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/****************************************************************
 @brief : ���ڳ�ʼ�� ����ʼ
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
    
    UartModeConfig(UART_BAUD_RATE,USART_FRAME_PARITY_EVEN);       //żУ��
}


/****************************************************************
 @brief : ���ڲ�������
 @param : rate - Ŀ�겨����
          parity - У�鷽ʽ
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
 @brief : ���ڷ��ʹ���
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
        // ���ͻ������������Ѿ����꣬�رմ��ڿ��ж�
        UartTxEnable(false);	//UDRIE0 �ر����ݼĴ���δ���ж�
    }
}


/****************************************************************
 @brief : ���ڽ��մ���
 @param : none
 @return :  ${return_type}
 @Note :
 
 ****************************************************************/
void Uart_RXAVHandler(USART_TypeDef *uart)
{
    uint8_t rxdata;
    rxdata = uart->RXDATA;
    uartRxBuffer[(rxBufferIndex + rxBufferSize)%UART_RXBUFF_SIZE] = rxdata;	
    
    // ���ﲢ���������ݳ�����������С�����⣬ֻҪ���յ����ݴ�С�����˻������Ĵ�С������������Ϊ������!!!
    rxBufferSize++;
}


/****************************************************************
 @brief : ��յ�ǰ�Ĵ����շ���������
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
 @brief : ���ڷ���һ���ֽ�
 @param : 
          txbyte - Ҫ���͵��ֽ�
 @return :  1 - ���ͳɹ���0 - ����ʧ��
 @Note :${time}
 ****************************************************************/
uint8_t UartSendByte(uint8_t txbyte)
{
    if( txBufferSize < UART_TXBUFF_SIZE)
    {
        uartTxBuffer[(txBufferIndex + txBufferSize) % UART_TXBUFF_SIZE] = txbyte;
        txBufferSize++;
        
        // ʹ�ܴ��ڷ���
        UartTxEnable(true);
        return 1;
    }
    else
    {
        return 0;
    }
}


/****************************************************************
 @brief : ���ڷ�������
 @param : 
          pdata - Ҫ���͵��ֽ�
          len - ���͵������ֽ�
 @return :  ʵ��д����ֽ���
 @Note :
 1. ���д�����ݴ��ڻ��������пռ�Ĵ�С���������ֽ��ܾ�д�롣
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
 @brief : ��ȡ���ڽ��ջ������е�������
 @param : none
 @return :  �������е����ݳ���
 @Note :${time}
 ****************************************************************/
uint16_t UartGetValidDataLength(void)
{
    return rxBufferSize;
}


/****************************************************************
 @brief : �ӽ��ջ����е�����һ���ֽ�
 @param : none
 @return :  ���������ֽ�
 @Note :${time}
 ****************************************************************/
uint8_t UartPopFromBuff(void)
{
    if( !rxBufferSize )
    {
        // �����ݿɵ�
        return 0;
    }
    
    uint8_t tmp = uartRxBuffer[rxBufferIndex];
    
    rxBufferIndex++;
    rxBufferIndex %= UART_RXBUFF_SIZE;
    rxBufferSize--;
    
    return tmp;
}

 
 /****************************************************************
	@brief : ȡ��Buffer�е����ݣ���ɾ��ȡ��������
	@param : pBuff - �������ݵ�buffer
					 len - ���ݳ���
	@return :  ʵ��ȡ�������ݳ���
	@Note :${time}
	****************************************************************/
 uint16_t UartGetFromBuff(uint8_t *pBuff,uint16_t len)
 {
		 uint16_t datalen = len;
		 if( datalen > rxBufferSize )
		 {
				 // û��ô�����ݿ�ȡ
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
	@brief : ȡ��Buffer�е����ݣ����ǲ�Ӱ��buf�Ľṹ
	@param : pBuff - �������ݵ�buffer
					 len - ���ݳ���
	@return :  ʵ��ȡ�������ݳ���
	@Note :${time}
	****************************************************************/
 uint16_t UartPreGetFromBuff(uint8_t *pBuff,uint16_t len)
 {
		 uint16_t datalen = len,rxTmpIndex = rxBufferIndex;
		 if( datalen > rxBufferSize )
		 {
				 // û��ô�����ݿ�ȡ
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
//lenΪsum�����λ��
uint8_t get_sum_data(uint8_t buf[], uint8_t len)                     // len is the length of buf
{
  uint8_t i, sum = 0;
  for (i = 0; i < len - 1; i++)
  {
    sum += buf[i];
  }
  return sum;
}

#if 1   //�Ƿ���У��
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

//�ҵ�֡ͷ�����򶪵�һ���ֽ�
uint8_t Leuart_lookup_head_frame(uint8_t* pdata)
{
    uint16_t len=0;
    //uint8 buff[300];
    //memset(buff,0,300);
    len=UartGetValidDataLength();
    
    if(len>=Index_Min)
    {
        if(len>Index_Max)   //ֻȡ�ܴ������󳤶�
        {
            len=Index_Max;
        }
        UartPreGetFromBuff(pdata,len); //�����ǲ�ɾ��
        
        if((pdata[Index_Head]==T188_Frame_Head)\
           &&(pdata[Index_Len]<Index_Max_Len))
        {
            if(len>=pdata[Index_Len]+Index_Min) //���㹻�������������´���
            {
                len=pdata[Index_Len]+Index_Min;
                asm("nop");
                if(sum_check(pdata,len))
                {
                    //str_copy(pdata,buff,len);
                    UartGetFromBuff(pdata,len);
                    UartFlushBuffer();//��յ�ǰ�Ĵ����շ���������
                    return (len);
                }
                else
                {
                    UartPopFromBuff(); //����һ���ֽ�
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
uint8_t Do_ProcessUART_Data_Task()  //����ˮ��˴����յ�������
{
  static
  uint8_t up_data[UART_RXBUFF_SIZE];   //ˮ������������
  uint32_t Pulse_cnt_tmp = 0;
  if (!UartGetValidDataLength())
    return 0;
      
  if(Leuart_lookup_head_frame(up_data))              //ֻ��data���ݳ���
  {
    if(up_data[Index_Typ] & 0x80)  //��������
    {
      up_data[Index_Typ] &= ~(1<<7);
      switch(up_data[Index_Typ])  //֡����
      {
        case Func_PullsGet:  //������ 80
          pTxBuf_TypeDef.Type = up_data[Index_Typ];
          pTxBuf_TypeDef.Len = 4;
          Pulse_cnt_tmp = GetPulls();
          pTxBuf_TypeDef.Cmd[0] = Pulse_cnt_tmp >> 24;
          pTxBuf_TypeDef.Cmd[1] = Pulse_cnt_tmp >> 16;
          pTxBuf_TypeDef.Cmd[2] = Pulse_cnt_tmp >> 8;
          pTxBuf_TypeDef.Cmd[3] = Pulse_cnt_tmp;
          pTxBuf_TypeDef.Cmd[4] = get_sum_data((uint8_t*)(&pTxBuf_TypeDef.Head),pTxBuf_TypeDef.Len+5);
          pTxBuf_TypeDef.Cmd[5] = 0x16;     //������
          UartSendString((uint8_t*)(&pTxBuf_TypeDef),pTxBuf_TypeDef.Len+8); //���巢��
            break;
            
            
        case Func_PullsSet:  //д���� 81
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
          pTxBuf_TypeDef.Cmd[1] = 0x16;     //������
          UartSendString((uint8_t*)(&pTxBuf_TypeDef),pTxBuf_TypeDef.Len+8); //���巢��
            break;
      }
      return 1;
    }
    else                    //�Է�ack
    {
      if(up_data[Index_Typ] == Func_Butn)   //�ر��ط�
        ResendCnt = 0;
    }
  }
  return 0;
}

  
//����NBģ�飬���ط�
void WakeNB()
{
  pTxBuf_TypeDef.Type = Func_Butn;
  pTxBuf_TypeDef.Len = 0;
  pTxBuf_TypeDef.Cmd[0] = get_sum_data((uint8_t*)(&pTxBuf_TypeDef.Head),pTxBuf_TypeDef.Len+5);
  pTxBuf_TypeDef.Cmd[1] = 0x16;     //������
  UartSendString((uint8_t*)(&pTxBuf_TypeDef),pTxBuf_TypeDef.Len+8); //���巢��
  
  if(!ResendCnt)
  {
    ResendCnt = 5*3 -1;   //���2��
  }
}


//�ط���ѯ����    ������ɨ�����ھ���
void WakeNB_loop()
{
  if(ResendCnt)
  {
    if(ResendCnt%5 == 0)
      WakeNB();
    ResendCnt--;
  }
}

//�������ݴ�����ѯ
void ProcessUART_Task()
{
  if(ReadRxCnt)   //LE��������ʱʹ�� ���ڲ�ѯ���ƣ���1��ʼ   ������ɨ�����ھ���
  {
    if (ReadRxCnt >= 20)
    {
      ReadRxCnt = 0;
    } 	
  	else if(ReadRxCnt >= 2)	//2*60ms ��4��
    {
      if(Do_ProcessUART_Data_Task())	//���سɹ�
      {
        ReadRxCnt = 0;
      }
  	}
	ReadRxCnt++;	
  }
}


/****************************************************************
 @brief : LE���ڽ��մ���
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
 @brief : LE���ڽ��մ���
 ****************************************************************/
void LEUART0_IRQHandler()
{
    if((LEUART0 -> IF) & LEUART_IF_STARTF) //��ʼ֡
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
