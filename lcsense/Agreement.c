
#include "em_msc.h"
#include "flash.h"
#include "Agreement.h"
#include "sample.h"



uint8_t DataErr = 0;                //保存错误


//----------------------------------------------------------------------------//
//函数功能：检测ROMB记录的计量状态
//输入：Meter_up_state
// 0xFFF0：正在计量，没有进位
// 0xFF00：需要进位
// 0xF000：ROMA OK;计量出错;ROMB OK
// 0x0000：ROMA,计量 OK; ROMB出错
// 0xFFF0：正在计量，没有进位
//----------------------------------------------------------------------------//
void Det_Meter_up_state()   // meter measure Data update in flash , 1.9us  , 21us (未初始化计量flash才会调用 update_ROMAB_Meter_data();// 19.2us)
{
	uint32_t temp;
	if (ROMAB_Meter_up_state==0xFFF0) return;
	//关中断
	__disable_irq(); //关闭总中断
        MSC_Init();		//unlock Flash
	if (ROMAB_Meter_up_state==0xFF00)	//有进位，但ROMA没有记录
	{
		if ((uint8_t)(*((uint8_t*)ROMB_BASE_addr)+*((uint8_t*)(ROMB_BASE_addr+1))+\
			*((uint8_t*)(ROMB_BASE_addr+2))+*((uint8_t*)(ROMB_BASE_addr+3)))!=*((uint8_t*)(ROMB_BASE_addr+4)))
		{
			DataErr = 1;
		}
		else
		{
			temp = *((uint32_t*)ROMB_BASE_addr);
			temp = (temp+4000)%Flash_Max_meter_cnt;
			MSC_ErasePage((uint32_t*)ROMA_BASE_addr);		//擦A区
			MSC_WriteWord((uint32_t*)ROMA_BASE_addr,&temp,4);	//写A区
			
			temp = (*((uint8_t*)ROMA_BASE_addr)+*((uint8_t*)(ROMA_BASE_addr+1))+\
				*((uint8_t*)(ROMA_BASE_addr+2))+*((uint8_t*)(ROMA_BASE_addr+3)));
			temp &= 0x000000FF;
			MSC_WriteWord((uint32_t*)(ROMA_BASE_addr+ROMAB_Meter_CNT_CS_Addr),&temp,4);	//写A区CS
			
			temp = 0xF000;
			MSC_WriteWord((uint32_t*)(ROMB_BASE_addr+ROMAB_Meter_up_state_Addr),&temp,4);
		}
		goto no_operation;
	}
	if (ROMAB_Meter_up_state==0xF000)
	{
		MSC_ErasePage((uint32_t*)PULSE_BASE_addr);
		temp = 0x0000;
		MSC_WriteWord((uint32_t*)(ROMB_BASE_addr+ROMAB_Meter_up_state_Addr),&temp,4);
		goto no_operation;
	}
	if ((ROMAB_Meter_up_state==0x0000)||(ROMAB_Meter_up_state==0xFFFF))
	{
		if ((uint8_t)(*((uint8_t*)ROMA_BASE_addr)+*((uint8_t*)(ROMA_BASE_addr+1))+\
			*((uint8_t*)(ROMA_BASE_addr+2))+*((uint8_t*)(ROMA_BASE_addr+3)))!=*((uint8_t*)(ROMA_BASE_addr+4)))
		{
			DataErr = 1;
		}
		else
		{
			temp = *((uint32_t*)ROMA_BASE_addr);
			MSC_ErasePage((uint32_t*)ROMB_BASE_addr);
			MSC_WriteWord((uint32_t*)ROMB_BASE_addr,&temp,4);
			temp = *((uint8_t*)(ROMA_BASE_addr+4)); 		//取A区 CS;不需要+1
			MSC_WriteWord((uint32_t*)(ROMB_BASE_addr+ROMAB_Meter_CNT_CS_Addr),&temp,4);
			temp = 0xFFF0;
			MSC_WriteWord((uint32_t*)(ROMB_BASE_addr+ROMAB_Meter_up_state_Addr),&temp,4);
		}
	}
  no_operation:
    MSC_Deinit();	//lock Flash
    __enable_irq();//开启总中断
}

//extern uint32_t Pulse_cnt;		//总脉冲数  le中定义
//uint32_t Pulls_decre = 0;     //倒流计量
uint32_t Pulse_addr=0;		//不为0的脉冲地址
uint32_t Pulse_addr_cnt=0;	//不为0的脉冲字节上的脉冲数
//返回错误标志，0正常，1脉冲错误
_Pragma("optimize=none")
uint8_t find_Valid_Pulse()
{
	uint32_t temp_addr;
    uint8_t temp_data_der=0;
	Pulse_cnt = 0;
	for (temp_addr=0;temp_addr<140;temp_addr++)
	{
		Pulse_cnt+=32;		//退出时，Pulse_cnt会多加几个'1'
		if (*((uint32_t*)PULSE_BASE_addr+temp_addr)!=0) break;
	}
	if (*((uint32_t*)PULSE_BASE_addr+(temp_addr+1))!=0xFFFFFFFF) 
    {
        temp_data_der = 1;
        DataErr = 1;	//下一个地址上不为0xFFFFFFFF，则为异常
    }
	Pulse_addr = PULSE_BASE_addr+temp_addr*4;
	Pulse_addr_cnt = *((uint32_t*)PULSE_BASE_addr+temp_addr);
	if (Pulse_addr<(PULSE_BASE_addr+0x1F4))		//超过500字节
	{
		for (uint8_t i=0; i<32;i++)
		{
			Pulse_cnt--;	//减掉多加的'1'
			Pulse_addr_cnt<<=1;
			if(!Pulse_addr_cnt) break;
		}
		Pulse_addr_cnt = *(uint32_t*)(PULSE_BASE_addr+temp_addr*4);
	}
	else //0xFFFF
	{
		Pulse_addr = PULSE_BASE_addr + 0x1F2;
		Pulse_cnt = Pulse_cnt-32;
	}
	temp_addr = *(uint32_t*)ROMB_BASE_addr;
	if (((temp_addr+(temp_addr>>8)+(temp_addr>>16)+(temp_addr>>24))&0x000000FF)!=*((uint8_t*)(ROMB_BASE_addr+4)))
	{
		temp_data_der = 1;
        DataErr = 1;
	}
	Pulse_cnt = Pulse_cnt + *(uint32_t*)ROMB_BASE_addr;
	if(Pulse_cnt>=Flash_Max_meter_cnt) Pulse_cnt=0;
    
    return temp_data_der;
}

//写4000脉冲
void Increase_meter_to_ROM()    //100us
{
	uint32_t tempdata;
	__disable_irq(); //关闭总中断
        MSC_Init();		//unlock Flash
        
	find_Valid_Pulse();
	Pulse_addr_cnt<<=1;
	if (MSC_WriteWord((uint32_t*)Pulse_addr,&Pulse_addr_cnt,4)!=mscReturnOk) DataErr = 1;;
	if (Pulse_addr >= (PULSE_BASE_addr+0x1F0))	//最后一个字节
	{
		if (Pulse_addr_cnt==0)
		{
			tempdata = 0xFF00;
			MSC_WriteWord((uint32_t*)(ROMB_BASE_addr+ROMAB_Meter_up_state_Addr),&tempdata,4);
		}
	}
	Pulse_cnt++;
	if(Pulse_cnt>=Flash_Max_meter_cnt) Pulse_cnt=0;

	//保存 和 触发发送 0.5m3发一次
/*	if((Pulse_cnt%Send_space_Per_meter_cnt)==0)
	{
		if(Flash_Bubb_EN==1)
		{
			Ask_for_RF_Send_low0_Flag_Set;
			Ask_for_RF_Send_low1_Flag_Set;
		}
	}*/
	
        MSC_Deinit();	//lock Flash
        __enable_irq();//开启总中断
}

//设置脉冲数
uint8_t Init_meter_cnt(uint32_t Pulse_cnt_tmp)
{
	uint32_t ROMAB_data;		//RAOMAB计数
	uint32_t Pulse_data;		//脉冲区计数
	uint8_t  Pulse_word_cnt;	//为0的长字节数
	uint8_t  Pulse_last_cnt;	//最后一个字节0的个数
	uint32_t temp_data;
    
    uint8_t DataErr = 0;
	
	__disable_irq(); //关闭总中断
        MSC_Init();		//unlock Flash
	
	MSC_ErasePage((uint32_t*)ROMA_BASE_addr);
	MSC_ErasePage((uint32_t*)ROMB_BASE_addr);
	MSC_ErasePage((uint32_t*)PULSE_BASE_addr);
	
	Pulse_data = Pulse_cnt_tmp%4000;
	ROMAB_data = Pulse_cnt_tmp-Pulse_data;
	Pulse_word_cnt = Pulse_data/32;
	Pulse_last_cnt = Pulse_data%32;
	
	//写脉冲区
	temp_data = 0;
	for (uint8_t i=0; i<Pulse_word_cnt; i++)
	{
		MSC_WriteWord((uint32_t*)(PULSE_BASE_addr)+i,&temp_data,4);
		if (*((uint32_t*)(PULSE_BASE_addr)+i)!=0) DataErr = 1;
	}
	temp_data = 0xFFFFFFFF;
	temp_data = (temp_data<<Pulse_last_cnt);
	MSC_WriteWord((uint32_t*)(PULSE_BASE_addr+Pulse_word_cnt*4),&temp_data,4);
	if (*(uint32_t*)(PULSE_BASE_addr+Pulse_word_cnt*4)!=temp_data) DataErr = 1;
	
	//写ROMA
	MSC_WriteWord((uint32_t*)(ROMA_BASE_addr),&ROMAB_data,4);					//计量
	if (*(uint32_t*)(ROMA_BASE_addr)!=ROMAB_data) DataErr = 1;
	
	temp_data = (uint8_t)((*(uint8_t*)ROMA_BASE_addr) + (*(uint8_t*)(ROMA_BASE_addr+1))\
		+ (*(uint8_t*)(ROMA_BASE_addr+2)) + (*(uint8_t*)(ROMA_BASE_addr+3)));
	MSC_WriteWord((uint32_t*)(ROMA_BASE_addr+4),&temp_data,4);
	if (*(uint32_t*)(ROMA_BASE_addr+4)!=temp_data) DataErr = 1;
	
	//写ROMB
	MSC_WriteWord((uint32_t*)(ROMB_BASE_addr),&ROMAB_data,4);					//计量
	if (*(uint32_t*)(ROMB_BASE_addr)!=ROMAB_data) DataErr = 1;
	MSC_WriteWord((uint32_t*)(ROMB_BASE_addr+4),&temp_data,4);
	if (*(uint32_t*)(ROMB_BASE_addr+4)!=temp_data) DataErr = 1;
	temp_data = 0xFFF0;
	MSC_WriteWord((uint32_t*)(ROMB_BASE_addr+8),&temp_data,4);
	if (*(uint32_t*)(ROMB_BASE_addr+8)!=temp_data) DataErr = 1;
	
        MSC_Deinit();	//lock Flash
	__enable_irq();//开启总中断
    
    return DataErr;
}


//检测初始化标志
uint8_t CheckFlash()
{
    uint32_t F_flag = 0x56;    //flash初始化标志
	if (*(uint32_t*)(ROMN_BASE_addr+8) != F_flag)     //flash没有初始化
    {
    Negative_cnt = 0;
    return Negative_cnt_Save(Negative_cnt);
    }
    else
      Negative_cnt = *(uint32_t*)(ROMN_BASE_addr);        //反转数据
    return 0;
}

//反转脉冲保存
uint8_t Negative_cnt_Save(uint32_t Negative_cnt)
{
	uint32_t temp_data, F_flag = 0x56;    //flash初始化标志
    uint8_t DataErr = 0;
	
	__disable_irq(); //关闭总中断
    MSC_Init();		//unlock Flash
	MSC_ErasePage((uint32_t*)ROMN_BASE_addr);
	
    MSC_WriteWord((uint32_t*)(ROMN_BASE_addr+8),&F_flag,4);					//flash初始化标志
	if (*(uint32_t*)(ROMN_BASE_addr+8)!=F_flag) DataErr = 1;
	//写ROMN
	MSC_WriteWord((uint32_t*)(ROMN_BASE_addr),&Negative_cnt,4);					//计量
	if (*(uint32_t*)(ROMN_BASE_addr)!=Negative_cnt) DataErr = 1;
	
	temp_data = (uint8_t)((*(uint8_t*)ROMN_BASE_addr) + (*(uint8_t*)(ROMN_BASE_addr+1))\
		+ (*(uint8_t*)(ROMN_BASE_addr+2)) + (*(uint8_t*)(ROMN_BASE_addr+3)));
	MSC_WriteWord((uint32_t*)(ROMN_BASE_addr+4),&temp_data,4);
	if (*(uint32_t*)(ROMN_BASE_addr+4)!=temp_data) DataErr = 1;
    
    MSC_Deinit();	//lock Flash
	__enable_irq();//开启总中断
    
    return DataErr;
}

void meter_cnt_CHK()
{
    if(find_Valid_Pulse())  //计量区错误
    {
        Pulse_cnt = 0;      //初始化为0
        Init_meter_cnt(0);
    }
}

//得到实际脉冲数
uint32_t GetPulls()
{
  if(Pulse_cnt >= Negative_cnt)
  {
    if(Negative_cnt > Negative_cnt_max)
      Negative_cnt = Negative_cnt_max;
    return Pulse_cnt - Negative_cnt;
  }
  else
    return 0;
}

