#ifndef __SAMPLE_H
#define __SAMPLE_H

/* Type definition for global state. */
typedef enum {
  TIMER_RESET_STATE = 1,
  AWAKE_STATE = 2,
  SENSE_PREPARE_STATE = 3
} LCSENSE_GlobalState_TypeDef;

typedef struct //����������
{   
  // uint8_t PulseManaSta;   //����״̬��
   uint8_t TimeCnt;    //��ʱֵ
   uint8_t CH1Data[3];
   uint8_t CH2Data[3];  
   bool  CH1peak;      //��һͨ������
   bool  CH1valley;      //��һͨ���ȵ�
   bool  CH2peak;      //�ڶ�ͨ������
   bool  CH2valley;      //�ڶ�ͨ���ȵ�
}_PulseManaData;

#define LCSENSE_SENSOR_PORT  gpioPortC      //PC14  15
#define LCSENSE_SENSOR_PIN   0
#define LCSENSE_SENSOR_PIN1  1

#define P_FILTER      2             //�仯

extern uint32_t Pulse_cnt;
extern uint32_t Negative_cnt;
extern volatile uint8_t scanresult[];
extern bool Attackflg,lcTriggered;             //������־
extern uint8_t DataErr;                //�������

void SetupMeter();         //������ʼ��

//extern _PulseManaData PulseManaData;
#endif