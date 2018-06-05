/*!
 *******************************************************************************
 @copyright SHENZHEN HAC TELECOM TECHNOLOGY CO.,LTD
 @file      DV_CC1120.h
 @author    PJ  
 @mcu       Any MCU
 @compiler  IAR EWARM 6.21
 @date      2014.09.20
 @brief     CC1120基础配置：初始化、RX、TX、Idle、参数配置等
*/

#ifndef FLASH_H
#define FLASH_H

#include "em_msc.h"

	
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define RAM_BASE       0x20000000
#define RAM_BB_BASE    0x22000000

/* Private macro -------------------------------------------------------------*/
#define  Var_ResetBit_BB(VarAddr, BitNumber)    \
          (*(__IO uint32_t *) (RAM_BB_BASE | ((VarAddr - RAM_BASE) << 5) | ((BitNumber) << 2)) = 0)

#define Var_SetBit_BB(VarAddr, BitNumber)       \
          (*(__IO uint32_t *) (RAM_BB_BASE | ((VarAddr - RAM_BASE) << 5) | ((BitNumber) << 2)) = 1)

#define Var_GetBit_BB(VarAddr, BitNumber)       \
          (*(__IO uint32_t *) (RAM_BB_BASE | ((VarAddr - RAM_BASE) << 5) | ((BitNumber) << 2)))


#define user_info_base_addr					0x0FE00000		//用户信息首地址
#define user_info_end_addr					0x0FE001FF		//

#define FROZEN_BASE_addr					0x00007400		//  冻结数据	
#define FROZEN_END_addr						0x000077FF		//  

#define ROMA_BASE_addr						0x00007800		//	计量存放区
#define ROMA_END_addr						0x000079FF		//

#define ROMB_BASE_addr						0x00007A00		//	计量备份区
#define ROMB_END_addr						0x00007BFF		//
            

	#define ROMAB_Meter_CNT_Addr			0x00
	#define ROMAB_Meter_CNT_CS_Addr			0x04
	#define ROMAB_Meter_up_state_Addr		0x08	//计量值更新状态
	#define ROMAB_Meter_up_state	*(uint16_t*)(ROMB_BASE_addr+ROMAB_Meter_up_state_Addr)

											// 0xFFF0：正在计量，没有进位
											// 0xFF00：需要进位
											// 0xF000：ROMA OK;计量出错;ROMB OK
											// 0x0000：ROMA,计量 OK; ROMB出错
											// 0xFFF0：正在计量，没有进位

#define PULSE_BASE_addr						0x00007C00		//		4000脉冲存放区
#define PULSE_END_addr						0x00007DFF		//

#define ROMN_BASE_addr						0x00007E00		//	反转脉冲
#define ROMN_END_addr						0x00007FFF		//

//#define user_info_backup_base_addr			0x00007E00		//备份用户信息首地址		TG210F32
//#define user_info_backup_end_addr			0x00007FFF		//备份用户信息最大地址

//#define	Frq_Calibrate_Addr				0x04	//射频校准数据	1 byte
//#define	PID_Addr					0x08	//PID	2 byte
//#define	CH_Addr						0x0C	//工作信道	1 byte
//
//#define	DID_Addr					0x10	//表ID	4 byte
//#define SW_MAX_Overtime_Addr		0x14	//阀门控制超时时间
//#define TPN_Addr					0x18	//TPN
//#define Bubo_EN_Addr				0x1C	//冒泡使能
//
//#define Valve_STATE_Addr			0x20	//阀门状态
//#define Max_meter_cnt_Addr			0x24	//最大脉冲数
//#define PwrOff_ColSV_Addr			0x28	//掉电关阀状态
//#define Atk_ColSV_Addr              0x2C    //磁攻击关阀
//              
//#define AbRead_ColSV_Addr           0x30    //干簧管故障
//#define TopSV_Addr                  0x34    //远程控制阀门状态
//#define CAD_Mode_Addr               0x38    //控制模式，包含CAD检测模式,磁攻击关阀使能，疏通阀使能等模式
//#define Pulls_decre_Addr            0x3C    //倒流计量


              
#define Encrypt_KeyList_Addr        0x50    // 加密序列 KeyList_BUF[16]

              
#define Flash_PageErase        MSC_ErasePage    //擦除页

extern uint32_t Pulls_decre;            //反转计量



//uint32_t	Scan_userinfo_writeEN_Flag();
//uint8_t Flash_WriteData(uint32_t address,uint8_t *dat,uint16_t len);
//uint8_t Flash_Wirte_word(uint32_t addr, uint32_t data);


//uint8_t Read_UserInfo();
//void Save_UserInfo();


//uint8_t Init_FrozenPage();                // 出厂设置擦除一遍，以后再设置出场怎么办，设置时会全部清除
//uint8_t Write_FrozenData(uint8_t y, uint8_t m, uint32_t* dat);
//uint32_t Read_FrozenData(uint8_t year, uint8_t month);
//uint8_t Flash_Write_KeyList(uint32_t addr, uint8_t buf[]);
//void Flash_Read_KeyList(uint32_t addr, uint8_t buf[]);


#endif