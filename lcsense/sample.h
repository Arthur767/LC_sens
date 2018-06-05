#ifndef __SAMPLE_H
#define __SAMPLE_H

/* Type definition for global state. */
typedef enum {
  TIMER_RESET_STATE = 1,
  AWAKE_STATE = 2,
  SENSE_PREPARE_STATE = 3
} LCSENSE_GlobalState_TypeDef;

typedef struct //脉冲控制相关
{   
  // uint8_t PulseManaSta;   //脉冲状态机
   uint8_t TimeCnt;    //定时值
   uint8_t CH1Data[3];
   uint8_t CH2Data[3];  
   bool  CH1peak;      //第一通道顶点
   bool  CH1valley;      //第一通道谷点
   bool  CH2peak;      //第二通道顶点
   bool  CH2valley;      //第二通道谷点
}_PulseManaData;

#define LCSENSE_SENSOR_PORT  gpioPortC      //PC14  15
#define LCSENSE_SENSOR_PIN   0
#define LCSENSE_SENSOR_PIN1  1

#define P_FILTER      2             //变化

extern uint32_t Pulse_cnt;
extern uint32_t Negative_cnt;
extern volatile uint8_t scanresult[];
extern bool Attackflg,lcTriggered;             //攻击标志
extern uint8_t DataErr;                //保存错误

void SetupMeter();         //计量初始化

//extern _PulseManaData PulseManaData;
#endif