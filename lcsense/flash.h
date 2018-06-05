/*!
 *******************************************************************************
 @copyright SHENZHEN HAC TELECOM TECHNOLOGY CO.,LTD
 @file      DV_CC1120.h
 @author    PJ  
 @mcu       Any MCU
 @compiler  IAR EWARM 6.21
 @date      2014.09.20
 @brief     CC1120�������ã���ʼ����RX��TX��Idle���������õ�
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


#define user_info_base_addr					0x0FE00000		//�û���Ϣ�׵�ַ
#define user_info_end_addr					0x0FE001FF		//

#define FROZEN_BASE_addr					0x00007400		//  ��������	
#define FROZEN_END_addr						0x000077FF		//  

#define ROMA_BASE_addr						0x00007800		//	���������
#define ROMA_END_addr						0x000079FF		//

#define ROMB_BASE_addr						0x00007A00		//	����������
#define ROMB_END_addr						0x00007BFF		//
            

	#define ROMAB_Meter_CNT_Addr			0x00
	#define ROMAB_Meter_CNT_CS_Addr			0x04
	#define ROMAB_Meter_up_state_Addr		0x08	//����ֵ����״̬
	#define ROMAB_Meter_up_state	*(uint16_t*)(ROMB_BASE_addr+ROMAB_Meter_up_state_Addr)

											// 0xFFF0�����ڼ�����û�н�λ
											// 0xFF00����Ҫ��λ
											// 0xF000��ROMA OK;��������;ROMB OK
											// 0x0000��ROMA,���� OK; ROMB����
											// 0xFFF0�����ڼ�����û�н�λ

#define PULSE_BASE_addr						0x00007C00		//		4000��������
#define PULSE_END_addr						0x00007DFF		//

#define ROMN_BASE_addr						0x00007E00		//	��ת����
#define ROMN_END_addr						0x00007FFF		//

//#define user_info_backup_base_addr			0x00007E00		//�����û���Ϣ�׵�ַ		TG210F32
//#define user_info_backup_end_addr			0x00007FFF		//�����û���Ϣ����ַ

//#define	Frq_Calibrate_Addr				0x04	//��ƵУ׼����	1 byte
//#define	PID_Addr					0x08	//PID	2 byte
//#define	CH_Addr						0x0C	//�����ŵ�	1 byte
//
//#define	DID_Addr					0x10	//��ID	4 byte
//#define SW_MAX_Overtime_Addr		0x14	//���ſ��Ƴ�ʱʱ��
//#define TPN_Addr					0x18	//TPN
//#define Bubo_EN_Addr				0x1C	//ð��ʹ��
//
//#define Valve_STATE_Addr			0x20	//����״̬
//#define Max_meter_cnt_Addr			0x24	//���������
//#define PwrOff_ColSV_Addr			0x28	//����ط�״̬
//#define Atk_ColSV_Addr              0x2C    //�Ź����ط�
//              
//#define AbRead_ColSV_Addr           0x30    //�ɻɹܹ���
//#define TopSV_Addr                  0x34    //Զ�̿��Ʒ���״̬
//#define CAD_Mode_Addr               0x38    //����ģʽ������CAD���ģʽ,�Ź����ط�ʹ�ܣ���ͨ��ʹ�ܵ�ģʽ
//#define Pulls_decre_Addr            0x3C    //��������


              
#define Encrypt_KeyList_Addr        0x50    // �������� KeyList_BUF[16]

              
#define Flash_PageErase        MSC_ErasePage    //����ҳ

extern uint32_t Pulls_decre;            //��ת����



//uint32_t	Scan_userinfo_writeEN_Flag();
//uint8_t Flash_WriteData(uint32_t address,uint8_t *dat,uint16_t len);
//uint8_t Flash_Wirte_word(uint32_t addr, uint32_t data);


//uint8_t Read_UserInfo();
//void Save_UserInfo();


//uint8_t Init_FrozenPage();                // �������ò���һ�飬�Ժ������ó�����ô�죬����ʱ��ȫ�����
//uint8_t Write_FrozenData(uint8_t y, uint8_t m, uint32_t* dat);
//uint32_t Read_FrozenData(uint8_t year, uint8_t month);
//uint8_t Flash_Write_KeyList(uint32_t addr, uint8_t buf[]);
//void Flash_Read_KeyList(uint32_t addr, uint8_t buf[]);


#endif