#ifndef __HC05_H
#define __HC05_H	 
#include "sys.h" 
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32开发板
//ATK-HC05蓝牙模块驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2014/3/29
//版本：V1.1
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
//********************************************************************************
//V1.1 20140329
//修改LED的连接，原来接PC5，改为PA4，以兼容MiniSTM32开发板V3.0									  
//////////////////////////////////////////////////////////////////////////////////   

#define HC05_KEY  	PCout(4) 	//蓝牙控制KEY信号
#define HC05_LED  	PAin(4)		//蓝牙连接状态信号
  
u8 HC05_Init(void);
void HC05_CFG_CMD(u8 *str);
void HC05_CFG_CMDshow(void);
u8 HC05_Get_Role(void);
u8 HC05_Set_Cmd(u8* atstr);	   
#endif  
















