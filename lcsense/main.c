/***************************************************************************//**
 *
 ******************************************************************************/

//#include <stdint.h>
//#include <stdbool.h>
#include <stdio.h> 

#include "em_device.h"
//#include "em_assert.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_core.h"
#include "em_rtc.h"

#include "lcsense_conf.h"
#include "app_uart.h"
#include "delay.h"
#include "sample.h"
#include "Agreement.h"
/***************************************************************************//**
 * Macro definitions
 ******************************************************************************/



/******************************************************************//**
 * Global variables
 ******************************************************************************/

volatile LCSENSE_GlobalState_TypeDef appStateGlobal = TIMER_RESET_STATE;
static volatile bool secTimerFired = false;
static volatile uint8_t eventCounter = 0U;
static uint8_t PrintCnt;       //打印计时


/***************************************************************************//**
 * @brief  Setup the CMU
 ******************************************************************************/
void setupCMU(void)
{
  /* Ensure core frequency has been updated */
  SystemCoreClockUpdate();

  /* Select clock source for HF clock. */
  CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFRCO);
  /* Select clock source for LFA clock. */
  CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFRCO);
  /* Enable HF peripheral clock. */
  CMU_ClockEnable(cmuClock_HFPER, true);
  /* Enable clock for GPIO. */
  CMU_ClockEnable(cmuClock_GPIO, true);
  /* Enable clock for ACMP0. */
  CMU_ClockEnable(cmuClock_ACMP0, true);
//  /* Enable clock for ACMP1. */
//  CMU_ClockEnable(cmuClock_ACMP1, true);
  /* Enable CORELE clock. */
  CMU_ClockEnable(cmuClock_CORELE, true);
  /* Enable clock on RTC. */
  CMU_ClockEnable(cmuClock_RTC, true);
  /* Enable clock for LESENSE. */
  CMU_ClockEnable(cmuClock_LESENSE, true);
  /* Set clock divider for LESENSE. */
  CMU_ClockDivSet(cmuClock_LESENSE, cmuClkDiv_1);
  /* Enable clock for DAC */
  CMU_ClockEnable(cmuClock_DAC0, true);
}

/***************************************************************************//**
 * @brief  Setup the RTC
 ******************************************************************************/
void setupRTC(void)
{
  /* RTC configuration constant table. */
  static const RTC_Init_TypeDef initRTC = RTC_INIT_DEFAULT;

  /* Initialize RTC. */
  RTC_Init(&initRTC);

  /* Set COMP0 to overflow at the configured value (in seconds). */
  RTC_CompareSet(0, (uint32_t)CMU_ClockFreqGet(cmuClock_RTC)/10);	//	1/10秒

  /* Make sure that all pending interrupt is cleared. */
  RTC_IntClear(0xFFFFFFFFUL);

  /* Enable interrupt for COMP0. */
  RTC_IntEnable(RTC_IEN_COMP0);

  /* Finally enable RTC. */
  RTC_Enable(true);

  /* Wait while sync is busy. */
  while (RTC->SYNCBUSY) ;
}

/***************************************************************************//**
 * @brief  Setup the GPIO
 ******************************************************************************/
void setupGPIO(void)
{
  /* Configure measuring pin as push pull */
  GPIO_PinModeSet(LCSENSE_SENSOR_PORT, LCSENSE_SENSOR_PIN, gpioModePushPull, 0);
  //GPIO_DbgSWOEnable(false);         //PC15
  GPIO_PinModeSet(LCSENSE_SENSOR_PORT, LCSENSE_SENSOR_PIN1, gpioModePushPull, 0);//gpioModeDisabled
  
  GPIO_PinModeSet(gpioPortC, 13, gpioModePushPull, 0);          //LED
           //串口初始化    
    GPIO_PinModeSet(gpioPortD,6,gpioModeInputPull , 0);//gpioModePushPull
    GPIO_PinModeSet(gpioPortD,7,gpioModePushPull,0);            //Tx

}


/**************************************************************************//**
* @brief  IO初始化
  @param  void
*****************************************************************************/
void GPIO_Init(void)
{
  CMU_ClockEnable(cmuClock_GPIO, true);

  GPIO_DbgSWOEnable(false);   //EPW电源
//
//#ifndef DEBUG
  GPIO_DbgSWDIOEnable(false);
  GPIO_DbgSWDClkEnable(false);    //按键

    //GPIO_LED_On();			GPIO_LED_MakeINPUTPULL();			//led 直接驱动 高电平亮
    GPIO_PinModeSet(gpioPortC, 13, gpioModePushPull, 0);
  //GPIO_RF_RST_Off();	    GPIO_RF_RST_MakePUSHPULL();   //
  GPIO_PinModeSet(gpioPortA, 2, gpioModePushPull, 0);
  //GPIO_RF_RX_EN_MakeINPUTPULL();
  GPIO_PinModeSet(gpioPortB, 14, gpioModePushPull, 0);//gpioModeInputPull
  //GPIO_RF_TX_EN_MakeINPUTPULL();
  GPIO_PinModeSet(gpioPortB, 13, gpioModePushPull, 0);
  //GPIO_RF_TX_EN_Off(); GPIO_RF_RX_EN_Off();   //RX_TX_EN下拉
                                              //信号采集
  //GPIO_BT_Off();			GPIO_BT_MakePUSHPULL();       ////电池电压检测不使能
                  GPIO_PinModeSet(gpioPortB, 11, gpioModePushPull, 0);
  //GPIO_MR1_Clr();			GPIO_MR1_MakePUSHPULL();        ////计量1
  GPIO_PinModeSet(gpioPortD, 6, gpioModePushPull, 0);
  //GPIO_MR2_Clr();			GPIO_MR2_MakePUSHPULL();        ////计量2
  GPIO_PinModeSet(gpioPortD, 7, gpioModePushPull, 0);

  //电磁阀控制
  //GPIO_MCTR_P_Clr();	GPIO_MCTR_P_MakePUSHPULL();   //电机+		//电磁阀控制,初始状态为 00 全关断
    GPIO_PinModeSet(gpioPortA, 1, gpioModePushPull, 0);
  //GPIO_MCTR_N_Clr();	GPIO_MCTR_N_MakePUSHPULL();   //电机-
    GPIO_PinModeSet(gpioPortE, 11, gpioModePushPull, 0);
  //GPIO_MCTR_PB_Clr();	GPIO_MCTR_PB_MakePUSHPULL();    //电机+		//电磁阀控制,初始状态为 00 全关断
    GPIO_PinModeSet(gpioPortC, 0, gpioModePushPull, 0);
  //GPIO_MCTR_NB_Clr();	GPIO_MCTR_NB_MakePUSHPULL();    //电机-
    GPIO_PinModeSet(gpioPortC, 1, gpioModePushPull, 0);
  //GPIO_V_MCTR_Set();	GPIO_V_MCTR_MakeINPUT();      //电机过流采样
    GPIO_PinModeSet(gpioPortD, 4, gpioModePushPull, 0);
  //GPIO_VOD_Clr();		GPIO_VOD_MakePUSHPULL();        ////开阀到位检测
          GPIO_PinModeSet(gpioPortC, 14, gpioModePushPull, 0);
  //GPIO_VCD_Clr();		GPIO_VCD_MakePUSHPULL();        ////关阀到位检测
          GPIO_PinModeSet(gpioPortC, 15, gpioModePushPull, 0);

 // GPIO_EPW_Off();	        GPIO_EPW_MakePUSHPULL();    //按键使能，关闭
            GPIO_PinModeSet(gpioPortF, 2, gpioModePushPull, 0); //gpioModePushPull     pv1.5 epw
  //GPIO_LOCL_On();	        GPIO_LOCL_MakePUSHPULL();    //外部电源使能，关闭
    GPIO_PinModeSet(gpioPortF, 1, gpioModePushPull, 0); //gpioModePushPull
                                                      //GPIO_KEY_Clr();			GPIO_KEY_MakePUSHPULL();//GPIO_KEY_MakePUSHPULL();			//KEY
  //GPIO_KEY_Clr();			GPIO_KEY_MakeINPUTPULL(); //GPIO_KEY_MakePUSHPULL();			//KEY
    GPIO_PinModeSet(gpioPortF, 0,gpioModePushPull , 0);//gpioModeInputPull
    
  //晶振
    GPIO_PinModeSet(gpioPortB, 7,gpioModePushPull , 0);//gpioModeInputPull
    GPIO_PinModeSet(gpioPortB, 8,gpioModePushPull , 0);//gpioModeInputPull
   //spi

    GPIO_PinModeSet(gpioPortE, 10,gpioModePushPull , 1);
    GPIO_PinModeSet(gpioPortA, 0,gpioModePushPull , 0);
    GPIO_PinModeSet(gpioPortE, 12,gpioModePushPull , 0);
    GPIO_PinModeSet(gpioPortE, 13,gpioModePushPull , 0);
}



/***************************************************************************//**
 * @brief  Main function
 ******************************************************************************/


int main(void)
{
  CORE_DECLARE_IRQ_STATE;

  /* Disable interrupts */
  CORE_ENTER_ATOMIC();
  
  setupCMU(); 
  
  _delay_ms(50);
  
  //GPIO_Init();
  //EMU_EnterEM2(false);      //测试功耗  

  setupGPIO();
  
  UartInit();  //串口初始化
  
  SetupMeter();         //计量初始化

  //setupRTC();

  /* Enable interrupt in NVIC. */
  //NVIC_EnableIRQ(RTC_IRQn);

  /* Initialization done, enable interrupts globally. */
  CORE_EXIT_ATOMIC();
  
  find_Valid_Pulse();//获得计量初值
  CheckFlash();//检测初始化标志
  //测试串口
  printf(" Test begin.Pulse_cnt = %d\r\n",Pulse_cnt);
  

  while (1) 
  {
    EMU_EnterEM2(false);
    if (lcTriggered) 
    {
      //ProcessUART_Task();         //Le串口轮询
      Do_ProcessUART_Data_Task();	//普通串口
      WakeNB_loop();                    //重发轮询控制
#if 0
      PrintCnt++;
      if(PrintCnt%2 == 0) 
        printf("%d\t%d\t%d\r\n",scanresult[0],scanresult[1],Pulse_cnt);    //计量时串口发数据
        _delay_ms(1);
#endif
      lcTriggered = false;
    }
    
  }
}
/***************************************************************************//**
 * @brief  RTC common interrupt handler
 ******************************************************************************/
void RTC_IRQHandler(void)
{
  uint32_t tmp;

  /* Store enabled interrupts in temp variable. */
  tmp = RTC->IEN;

  /* Check if COMP0 interrupt is enabled and set. */
  if (RTC_IF_COMP0 & (tmp & RTC_IntGet())) {
    /* Timer has fired, clear interrupt flag... */
    RTC_IntClear(RTC_IFC_COMP0);
    /* ...and set the global flag. */
    secTimerFired = true;
  }
}

