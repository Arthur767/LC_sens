/***************************************************************************//**
 *
 ******************************************************************************/

//#include <stdint.h>
//#include <stdbool.h>
//#include <stdio.h> 

//#include "em_device.h"
#include "em_acmp.h"
//#include "em_assert.h"
//#include "em_cmu.h"
#include "em_dac.h"
//#include "em_emu.h"
#include "em_gpio.h"
//#include "em_core.h"
#include "em_lesense.h"


#include "lcsense_conf.h"
#include "sample.h"
#include "Agreement.h"
   
/***************************************************************************//**
 * Macro definitions
 ******************************************************************************/


#define ATTACK_LEVEL        40      //磁攻击最低门限
//#define INIT_STATE_TIME_SEC     3U



/***************************************************************************//**
 * Global variables
 ******************************************************************************/
static volatile bool secTimerFired = false;
static volatile uint8_t eventCounter = 0U;


//DAC和比较电压
static uint32_t V_dac = 2100;  //震荡中心电压
static uint8_t V_ACMP = 31;  //-比较电压
static _PulseManaData PulseManaData;          //脉冲功能
static uint8_t Attackcnt;                           //攻击计时
volatile uint8_t scanresult[3] = {0};

bool Attackflg = false,lcTriggered = false;;             //攻击标志
uint32_t Pulse_cnt=0;		//总脉冲数

static volatile bool CH1Ready;
static volatile bool forward_flag = true;
uint32_t Negative_cnt=0;
/**********************************/
extern LCSENSE_GlobalState_TypeDef appStateGlobal;


/***************************************************************************//**
 * @brief  Write DAC conversion value
 ******************************************************************************/
void writeDataDAC(DAC_TypeDef *dac, unsigned int value, unsigned int ch)
{
  /* Write data output value to the correct register. */
  if (!ch) {
    /* Write data to DAC ch 0 */
    dac->CH0DATA = value;
  } else {
    /* Write data to DAC ch 1 */
    dac->CH1DATA = value;
  }
}

/***************************************************************************//**
 * @brief  Sets up the DAC
 ******************************************************************************/
void setupDAC(void)
{
  /* Configuration structure for the DAC */
  DAC_Init_TypeDef dacInit = DAC_INIT_DEFAULT;
  /* Change the reference for Vdd */
      dacInit.refresh = dacRefresh64;   //刷新时间
  dacInit.reference = dacRefVDD;
  /* Initialize DAC */
  DAC_Init(DAC0, &dacInit);
  //DAC0_OUT0 /OPAMP_OUT0 PB11
  /* Set data for DAC channel 0 */
  writeDataDAC(DAC0, V_dac, 0);   //1900 :1.5V  1700:1.3V    800:0.68V   

  //writeDataDAC(DAC0, 1700, 1);
}



/***************************************************************************//**
 * @brief  Setup the ACMP
 ******************************************************************************/
void setupACMP(void)
{
  /* ACMP configuration constant table. */
  ACMP_Init_TypeDef initACMP =
  {
    .fullBias                 = false,                  /* fullBias */
    .halfBias                 = true,                   /* halfBias */
    .biasProg                 = 0xE,                    /* biasProg */
    .interruptOnFallingEdge   = false,                  /* interrupt on rising edge */
    .interruptOnRisingEdge    = false,                  /* interrupt on falling edge */
    .warmTime                 = acmpWarmTime512,        /* 512 cycle warmup to be safe */
    .hysteresisLevel          = acmpHysteresisLevel1,   /* hysteresis level 0 acmpHysteresisLevel0 */
    .inactiveValue            = false,                  /* inactive value */
    .lowPowerReferenceEnabled = false,                  /* low power reference */
    .vddLevel                 = V_ACMP                    /* VDD level    0x0D  20*/
  };

  /* Initialize ACMP */
  ACMP_Init(ACMP0, &initACMP);

  ACMP_ChannelSet(ACMP0, acmpChannelVDD , acmpChannel0); //ACMP1_CH6 PC14      acmpChannelVDD
  ACMP_ChannelSet(ACMP0, acmpChannelVDD, acmpChannel1); //ACMP1_CH7 PC15

  /* don't disable ACMP so that output can be routed out to a pin */
  //ACMP0->CTRL &= ~ACMP_CTRL_EN;
}

/***************************************************************************//**
 * @brief  Setup the LESENSE
 ******************************************************************************/
void setupLESENSE(void)
{
  /* LESENSE channel configuration constant table. */
  static const LESENSE_ChAll_TypeDef initChs = LESENSE_LCSENSE_SCAN_CONF;
  /* LESENSE central configuration constant table. */
  static const LESENSE_Init_TypeDef initLESENSE =
  {
    .coreCtrl =
    {
      .scanStart      = lesenseScanStartPeriodic,	//lesenseScanStartOneShot lesenseScanStartPeriodic
      .prsSel         = lesensePRSCh0,
      .scanConfSel    = lesenseScanConfDirMap,
      .invACMP0       = false,
      .invACMP1       = false,
      .dualSample     = false,
      .storeScanRes   = false,
      .bufOverWr      = true,
      .bufTrigLevel   = lesenseBufTrigHalf,
      .wakeupOnDMA    = lesenseDMAWakeUpDisable,
      .biasMode       = lesenseBiasModeDutyCycle,
      .debugRun       = false
    },

    .timeCtrl =
    {
      .startDelay     = 0U
    },

    .perCtrl =
    {
      .dacCh0Data     = lesenseDACIfData,
      .dacCh0ConvMode = lesenseDACConvModeSampleHold,    //lesenseDACConvModeContinuous  lesenseDACConvModeSampleHold lesenseDACConvModeSampleOff
      .dacCh0OutMode  = lesenseDACOutModePinADCACMP ,          //DAC0_OUT0 PB11
      .dacCh1Data     = lesenseDACIfData,
      .dacCh1ConvMode = lesenseDACConvModeSampleOff,
      .dacCh1OutMode  = lesenseDACOutModeDisable,       //lesenseDACOutModeDisable  lesenseDACOutModeADCACMP
      .dacPresc       = 31U,
      .dacRef         = lesenseDACRefVdd,
      .acmp0Mode      = lesenseACMPModeMux,                 //ACMP0_CH0,1       lesenseACMPModeMux  lesenseACMPModeMuxThres
      .acmp1Mode      = lesenseACMPModeDisable ,             //ACMP1_CH6 PC14,ACMP1_CH7 PC15
      .warmupMode     = lesenseWarmupModeNormal
    },

    .decCtrl =
    {
      .decInput       = lesenseDecInputSensorSt,
      .initState      = 0U,
      .chkState       = false,
      .intMap         = false,
      .hystPRS0       = false,
      .hystPRS1       = false,
      .hystPRS2       = false,
      .hystIRQ        = false,
      .prsCount       = true,
      .prsChSel0      = lesensePRSCh0,
      .prsChSel1      = lesensePRSCh1,
      .prsChSel2      = lesensePRSCh2,
      .prsChSel3      = lesensePRSCh3
    }
  };

  /* Initialize LESENSE interface with RESET. */
  LESENSE_Init(&initLESENSE, true);

  /* Configure scan channels. */
  LESENSE_ChannelAllConfig(&initChs);

  /* Set scan frequency (in Hz). */
  (void)LESENSE_ScanFreqSet(0U, 10);	//10U

  /* Set clock divisor for LF clock. */
  LESENSE_ClkDivSet(lesenseClkLF, lesenseClkDiv_2);
  /* Set clock divisor for HF clock. */
  LESENSE_ClkDivSet(lesenseClkHF, lesenseClkDiv_1);
  
  LESENSE_IntEnable(_LESENSE_IF_SCANCOMPLETE_MASK);	//扫描完成中断
  
  NVIC_EnableIRQ(LESENSE_IRQn);
    
  /* Start scanning LESENSE channels. */
  LESENSE_ScanStart();
//
//  /* Assuming that there is not metal close by in the first
//     scan we can use the result as the threshold value
//     thus calibrating the LC sensor */
//  /* Waiting for buffer to be full */
//  while (!(LESENSE->STATUS & LESENSE_STATUS_BUFFULL)) ;
//
//  /* Read last result and use as counter threshold */
//  LESENSE_ChannelThresSet(6, 0, LESENSE_ScanResultDataBufferGet(15));
}



//计量初始化
//包括时钟 ACMP DAC LESENSE
void SetupMeter()
{

  setupACMP();

  setupDAC();

  setupLESENSE();
  
}


/***************************************************************************//**
 * @brief  Main function
 ******************************************************************************/

#if 0
int main(void)
{
  CORE_DECLARE_IRQ_STATE;

  /* Disable interrupts */
  CORE_ENTER_ATOMIC();
  
  UartInit();  //串口初始化
  
  SetupMeter();         //计量初始化

  setupRTC();

  /* Enable interrupt in NVIC. */
  NVIC_EnableIRQ(RTC_IRQn);

  /* Initialization done, enable interrupts globally. */
  CORE_EXIT_ATOMIC();
  
  //测试串口
  printf(" Test begin.\r\n");
  

  while (1) 
  {
      switch (appStateGlobal)
      {
        case TIMER_RESET_STATE:     //开启延时
        {
          RTC_Enable(false);
          RTC_Enable(true);

          appStateGlobal = AWAKE_STATE;
        }
        break;

        case AWAKE_STATE:           //延时后采样
        {
          appStateGlobal = AWAKE_STATE;
          if (secTimerFired) 
          {
            appStateGlobal = TIMER_RESET_STATE;
            secTimerFired = false;
            RTC_Enable(false);
            PrintCnt++;
            if(PrintCnt%2 == 0) 
            {
              printf("%d\t%d\t %d\r\n",scanresult[0],scanresult[1],Pulse_cnt);    //计量时串口发数据200ms周期
              _delay_ms(1);
            }
            LESENSE_ScanStart();						//定时调用触发
            EMU_EnterEM2(true);
          } 
          else 
            EMU_EnterEM2(true);
        }
        break;

        default:
          break;
      }
 }
    
 
}
#endif


/********************************************************************************/

//计量功能
void MeterFunc(void)
{
  //磁攻击
  if(scanresult[0] <ATTACK_LEVEL && scanresult[1]<ATTACK_LEVEL && Attackcnt<60)
    Attackcnt++;
  else if(Attackcnt)
    Attackcnt--;
  if(Attackcnt>=30)        //100*30ms
  {
    Attackflg = true;  
    return;
  }
  //通道1处理
  PulseManaData.TimeCnt = scanresult[0];  //
  
  if(PulseManaData.CH1Data[2] == 0)
    PulseManaData.CH1Data[2] = PulseManaData.TimeCnt;                    //第一次记录
  else //if(RF_WORK_Mode&0x0F != BOARD_RF_TXing)          // && RF_WORK_Mode&0x0F != BOARD_RF_RXing)        //发射对脉冲影响  
  {
    if(PulseManaData.TimeCnt >= PulseManaData.CH1Data[2]+P_FILTER || PulseManaData.TimeCnt <= PulseManaData.CH1Data[2]-P_FILTER)  //变动3
    {
      PulseManaData.CH1Data[0] = PulseManaData.CH1Data[1];                            
      PulseManaData.CH1Data[1] = PulseManaData.CH1Data[2];
      PulseManaData.CH1Data[2] = PulseManaData.TimeCnt;
      
      if(PulseManaData.CH1Data[0] != 0)
      {
        if(PulseManaData.CH1Data[1] > PulseManaData.CH1Data[0]  &&  PulseManaData.CH1Data[1]>PulseManaData.CH1Data[2])             //CH1顶点
        {
          PulseManaData.CH1peak = true;
          PulseManaData.CH1valley = false;
          CH1Ready = true;
        }
        
        if(PulseManaData.CH1Data[1] < PulseManaData.CH1Data[0]  &&  PulseManaData.CH1Data[1]<PulseManaData.CH1Data[2])
        {
          PulseManaData.CH1valley = true;                                      //谷点
          CH1Ready = false;
        }
      }
    } //endof变化2us
  }
  
  //通道2处理
  PulseManaData.TimeCnt = scanresult[1];  //
  
  if(PulseManaData.CH2Data[2] == 0)
    PulseManaData.CH2Data[2] = PulseManaData.TimeCnt;                    //第一次记录
  else //if(RF_WORK_Mode&0x0F != BOARD_RF_TXing)            // && RF_WORK_Mode&0x0F != BOARD_RF_RXing)        //发射对脉冲影响
  {
    if(PulseManaData.TimeCnt >= PulseManaData.CH2Data[2]+P_FILTER || PulseManaData.TimeCnt <= PulseManaData.CH2Data[2]-P_FILTER)  //变动
    {
      PulseManaData.CH2Data[0] = PulseManaData.CH2Data[1];                            
      PulseManaData.CH2Data[1] = PulseManaData.CH2Data[2];
      PulseManaData.CH2Data[2] = PulseManaData.TimeCnt;
      
      if(PulseManaData.CH2Data[0] != 0)
      {
        if(PulseManaData.CH2Data[1] > PulseManaData.CH2Data[0]  &&  PulseManaData.CH2Data[1]>PulseManaData.CH2Data[2])             //CH2顶点
        {
          PulseManaData.CH2peak = true;
          PulseManaData.CH2valley = false;   
           //反向
          if(CH1Ready)
            forward_flag = false;
          else
            forward_flag = true;         
        }
        
        if(PulseManaData.CH2Data[1] < PulseManaData.CH2Data[0]  &&  PulseManaData.CH2Data[1]<PulseManaData.CH2Data[2])
        {
          PulseManaData.CH2valley = true;                                      //谷点
        }
        
      }
    } //endof变化2us
  }
    
    //判断一圈有效
    if(PulseManaData.CH1valley && PulseManaData.CH2valley && PulseManaData.CH1peak && PulseManaData.CH2peak)                              //4个点
    {
      if(forward_flag)
      {
        Increase_meter_to_ROM();//记录4000脉冲
        Det_Meter_up_state();//记录4000倍数
      }
      else
      {
        Negative_cnt++;
        Negative_cnt_Save(Negative_cnt);
      }
      
      PulseManaData.CH1peak = false;
      PulseManaData.CH2peak = false;                            
      PulseManaData.CH1valley = false;                                      //谷点失效
      PulseManaData.CH2valley = false;                                      //谷点失效
    
    }
}
/***************************************************************************//**
 * @brief  LESENSE interrupt handler
 ******************************************************************************/

void LESENSE_IRQHandler(void)
{
  uint16_t ScanResultChan;
  uint8_t i;
  
  //扫描完成中断
  if (_LESENSE_IF_SCANCOMPLETE_MASK & LESENSE_IntGetEnabled()) 
  {
     LESENSE_IntClear(_LESENSE_IF_SCANCOMPLETE_MASK);
      /* Increase the event counter... */ 
      eventCounter++;
      ScanResultChan = LESENSE_ScanResultGet();

      //DAC_Enable(DAC0,0,false);                 //DAC 为持续模式时，手动关闭DAC
        for(i = 0; i<3; i++)
        {
        if(ScanResultChan & (1<<i))
          scanresult[i] = LESENSE_ScanResultDataGet();
        }
//        if(scanresult[i-1] >6)
//          GPIO_PinOutToggle(gpioPortC,13);  //灯光
        MeterFunc();         //计量
      

      /* ...and go to INIT_STATE. */
      appStateGlobal = TIMER_RESET_STATE;
      lcTriggered = true;
  }
}


