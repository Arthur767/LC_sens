/*!
 *******************************************************************************
 @copyright SHENZHEN HAC TELECOM TECHNOLOGY CO.,LTD
 @file      Agreement.h
*/

#ifndef AGREEMENT_H
#define AGREEMENT_H

#define 	Flash_Max_meter_cnt 10000000//*((uint32_t*)(user_info_base_addr+Max_meter_cnt_Addr))      //������ֵ�����ã�
#define     Negative_cnt_max    100000  //��ת���ֵ
//��������
void Det_Meter_up_state();
void Increase_meter_to_ROM();
uint8_t find_Valid_Pulse();
uint8_t Init_meter_cnt(uint32_t Pulse_cnt_tmp);
void meter_cnt_CHK();
uint8_t Negative_cnt_Save(uint32_t Negative_cnt);       //���淴ת����
uint32_t GetPulls();        //�õ�ʵ��������
uint8_t CheckFlash();           //����ʼ����־
#endif