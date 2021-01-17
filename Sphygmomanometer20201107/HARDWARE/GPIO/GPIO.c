#include "sys.h"   
#include "gpio.h"
#include "gimbal.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK Mini STM32开发板
//LED驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2014/3/05
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	   

//初始化PA8和PD2为输出口.并使能这两个口的时钟		    
//LED IO初始化
void VALVE_Init(void)
{
	 GPIO_InitTypeDef  GPIO_InitStructure;
 	
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	 //使能端口时钟
	
 GPIO_InitStructure.GPIO_Pin = VALVE ;				 //端口配置
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
 GPIO_Init(VALVE_GROUP, &GPIO_InitStructure);					 //根据设定参数初始化
 GPIO_SetBits(VALVE_GROUP,VALVE );						 //PA.6 输出高

}
void MOTOR_Init(void)
{
	 GPIO_InitTypeDef  GPIO_InitStructure;
 	
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	 //使能端口时钟
	
 GPIO_InitStructure.GPIO_Pin = MOTOR;				 //端口配置
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
 GPIO_Init(MOTOR_GROUP , &GPIO_InitStructure);					 //根据设定参数初始化
 GPIO_ResetBits(MOTOR_GROUP ,MOTOR);						 //PA.7 输出低
}






