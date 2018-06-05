/*!
 *******************************************************************************
 @copyright SHENZHEN HAC TELECOM TECHNOLOGY CO.,LTD
 @file      Agreement.h
*/

#ifndef AGREEMENT_H
#define AGREEMENT_H

#define 	Flash_Max_meter_cnt 10000000//*((uint32_t*)(user_info_base_addr+Max_meter_cnt_Addr))      //最大计量值可配置？
#define     Negative_cnt_max    100000  //反转最大值
//计量操作
void Det_Meter_up_state();
void Increase_meter_to_ROM();
uint8_t find_Valid_Pulse();
uint8_t Init_meter_cnt(uint32_t Pulse_cnt_tmp);
void meter_cnt_CHK();
uint8_t Negative_cnt_Save(uint32_t Negative_cnt);       //保存反转计量
uint32_t GetPulls();        //得到实际脉冲数
uint8_t CheckFlash();           //检测初始化标志
#endif