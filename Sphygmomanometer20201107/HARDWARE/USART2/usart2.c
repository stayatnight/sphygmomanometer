#include "delay.h"
#include "usart2.h"
#include "stdarg.h"	 	 
#include "stdio.h"	 	 
#include "string.h"	
#include "parameter.h"
#include "flash.h"
#include "string.h"
#include"eeprom.h"
#define charnumber(x) (x-'0') 
#include "gimbal.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32开发板
//串口2驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2014/3/29
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	   

//串口发送缓存区 
/*******************************************************************************************************************/
__align(8) u8 *AT_MeasureMode0   = (uint8_t*)"AT+CONFIG=0";			 		//in configure instrument mode0
__align(8) u8 *AT_MeasureMode1   = (uint8_t*)"AT+CONFIG=10";			 	 	//in configure instrument mode10
__align(8) u8 *AT_MeasureMode2   = (uint8_t*)"AT+CONFIG=11";			 		//in configure instrument mode11
__align(8) u8 *AT_MeasureMode3   = (uint8_t*)"AT+CONFIG=12";			 	 	//in configure instrument mode12
__align(8) u8 *AT_MeasureMode4   = (uint8_t*)"AT+CONFIG=13";			 	 	//in configure instrument mode13


__align(8) u8 *cmd_start_measure = (uint8_t*)"start";
__align(8) u8 *cmd_start_measure_single = (uint8_t*)"begin";			  		//setup instrument
		//setup instrument
__align(8) u8 *cmd_tongbu        = (uint8_t*)"AT+TONGB=1";			
__align(8) u8 *cmd_time_calibrate= (uint8_t*)"AT+TIMECALIBRATE\r\n";	//set require time calibrate
__align(8) u8 *cmd_set_time      = (uint8_t*)"T";//"AT+SETTIME";						//set rtc time
__align(8) u8 *cmd_stop = (uint8_t*)"stop";		
__align(8) u8 *cmd_close_motor   = (uint8_t*)"AT+MOTOR=0";									
__align(8) u8 *cmd_close_valve   = (uint8_t*)"AT+VALVE=0";									
__align(8) u8 *cmd_close_motor_valve = (uint8_t*)"AT+MOTORVALVE=0";		
__align(8) u8 *cmd_read_save_all = (uint8_t*)"readall";
__align(8) u8 *cmd_open_motor    = (uint8_t*)"AT+MOTOR=1";									
__align(8) u8 *cmd_open_valve    = (uint8_t*)"open";									
__align(8) u8 *cmd_open_motor_valve = (uint8_t*)"AT+MOTORVALVE=1";

__align(8) u8 *cmd_feed_dog = (uint8_t*)"AT+FEEDDOG";		

__align(8) u8 *single_measure_completed = (uint8_t*)"MEASURE_COMPLETED\r\n";	

__align(8) u8 *test = (uint8_t*)"test";	
__align(8) u8 USART2_TX_BUF[USART2_MAX_SEND_LEN]; 	//发送缓冲,最大USART2_MAX_SEND_LEN字节
u16 tz[8];u16 count_save=0;
u32 page_read=0x08030000;
/***********************************************************
	* @brief 		uncode time from client then set rtc time //从客户端解码时间并且设置rtc时间
	* @input  	none
	* @return	none
*************************************************************/

void uncode_time_set_time(uint8_t *data)
{
	uint8_t 	i=2;//根据时间指令格式不一样，需进行修改

	sCalendar_management.w_year=(uint16_t)((data[i+1]-0x30)*10+(data[i+2]-0x30+2000));
	sCalendar_management.w_month=(uint8_t)((data[i+4]-0x30)*10+(data[i+5]-0x30));
	sCalendar_management.w_date=(uint8_t)((data[i+7]-0x30)*10+(data[i+8]-0x30));
	sCalendar_management.hour=(uint8_t)((data[i+10]-0x30)*10+(data[i+11]-0x30));
	sCalendar_management.min=(uint8_t)((data[i+13]-0x30)*10+(data[i+14]-0x30));	
	sCalendar_management.sec=(uint8_t)((data[i+16]-0x30)*10+(data[i+17]-0x30));
}
#ifdef USART2_RX_EN   								//如果使能了接收   	  
//串口接收缓存区 	
u16 data4; u16 ddd=3;

extern VirtAddVarTab[NumbOfVar];
uint16_t VarValue = 0;

u8 USART2_RX_BUF[USART2_MAX_RECV_LEN]; 				//接收缓冲,最大USART2_MAX_RECV_LEN个字节.
extern u16 counter_1second; 


//通过判断接收连续2个字符之间的时间差不大于10ms来决定是不是一次连续的数据.
//如果2个字符接收间隔超过10ms,则认为不是1次连续数据.也就是超过10ms没有接收到
//任何数据,则表示此次接收完毕.
//接收到的数据状态
//[15]:0,没有接收到数据;1,接收到了一批数据.
//[14:0]:接收到的数据长度
u16 USART2_RX_STA=0;   	 
void USART2_IRQHandler(void)
{
	u8 res;	
	u16 yx=20;
	while (yx--)
	{
		if(USART2->SR&(1<<5))//接收到数据
	{	 
		res=USART2->DR; 	
		if(USART2_RX_STA<USART2_MAX_RECV_LEN)		//还可以接收数据
		{
			TIM4->CNT=0;         					//计数器清空
			if(USART2_RX_STA==0)TIM4_Set(1);	 	//使能定时器4的中断 
			USART2_RX_BUF[USART2_RX_STA++]=res;		//记录接收到的值	 
		}else 
		{
			USART2_RX_STA|=1<<15;					//强制标记接收完成
//			printf("KKKK");
		} 
	}
	}
	if(!(memcmp(USART2_RX_BUF, (const char*)AT_MeasureMode0, strlen((const char*)AT_MeasureMode0))))
		{
			sFunctionCode.FunctionCode = ConfigParameter;
			sFunctionCode.MeasureMode = MeasureMode0;
			counter_1second = 0;
			u2_printf("OK1");
			memset(USART2_RX_BUF,0,30);
			return;
		}
		if(strstr(USART2_RX_BUF, (const char*)cmd_stop))
		{
			 sFunctionCode.FunctionCode = None;
			 sData_single_management.data_number=99999;
			 memset(USART2_RX_BUF,0,30);
			 return;
		}
		if(strstr(USART2_RX_BUF, (const char*)cmd_open_valve))
		{
			 sFunctionCode.FunctionCode = None;
		   Motor_open;
			 sData_single_management.data_number=99999;
			 memset(USART2_RX_BUF,0,30);
			 return;
		}
		if(!(memcmp(USART2_RX_BUF, (const char*)AT_MeasureMode1, strlen((const char*)AT_MeasureMode1))))
		{
			sFunctionCode.FunctionCode = ConfigParameter;
			sFunctionCode.MeasureMode = MeasureMode1;
			counter_1second = 0;
			u2_printf("OK2");
			memset(USART2_RX_BUF,0,30);
			return;
		}
		if(strstr(USART2_RX_BUF, (const char*)cmd_start_measure))
		{
//			printf("OKKKK1\r\n");
			 sFunctionCode.FunctionCode = StartMeasure;
			 counter_1second = 0;
			 Motor_open;
			 MeasureSection.all_times=10*charnumber(USART2_RX_BUF[0])+charnumber(USART2_RX_BUF[1]);
       MeasureSection.Interval_Time=10*charnumber(USART2_RX_BUF[2])+charnumber(USART2_RX_BUF[3]);
			 memset(USART2_RX_BUF,0,30);
			 return;
		}
		if(strstr(USART2_RX_BUF,(const char*)cmd_start_measure_single))
{
			 sFunctionCode.FunctionCode = StartMeasure;
			 sFunctionCode.MeasureMode = MeasureMode0;
			 memset(USART2_RX_BUF,0,30);
			 return;
}
		if(strstr(USART2_RX_BUF, (const char*)cmd_read_save_all))
		{    //u2_printf("hello");
		FLASH_Unlock();
               while (1)
                 { 
					STMFLASH_Read(page_read,tz,sizeof(tz));
					//delay_ms(100);
					u2_printf("\n%d%02d%02d%02d%02d%03d%03d%03d",tz[0],tz[1],tz[2],tz[3],tz[4],tz[5],tz[6],tz[7]);
					page_read=page_read+0x20;
					delay_ms(50);
					count_save++;
					if(*(vu32*)page_read==0xffffffff||count_save==100)
					   break; 
				 }
				 count_save=0;
				 page_read=0x08030000;
			memset(USART2_RX_BUF,0,30);
			return;
		}
		if(!(memcmp(USART2_RX_BUF, (const char*)cmd_set_time, strlen((const char*)cmd_set_time))))
		{
			sFunctionCode.FunctionCode = TimeCalibrate;
			uncode_time_set_time(USART2_RX_BUF); 
			counter_1second = 0;
			u2_printf("OK\r\n");
			memset(USART2_RX_BUF,0,30);
			return;
		}
		if(!(memcmp(USART2_RX_BUF, (const char*)cmd_tongbu, strlen((const char*)cmd_tongbu))))
		{
			sFunctionCode.FunctionCode = Synchronous;
			u2_printf("OK\r\n");
			return;
		}

	/*	if (strstr(USART2_RX_BUF,cmd_start_measure))
		{
			a
		}
		*/
}   
//初始化IO 串口2
//pclk1:PCLK1时钟频率(Mhz)
//bound:波特率	  
void USART2_Init(u32 pclk1,u32 bound)
{  	 		 
	RCC->APB2ENR|=1<<2;   	//使能PORTA口时钟  
	GPIOA->CRL&=0XFFFF00FF;	//IO状态设置
	GPIOA->CRL|=0X00008B00;	//IO状态设置	 //IO状态设置(PA2 TX  PA3 RX)	 
	RCC->APB1ENR|=1<<17;  	//使能串口时钟 	 
	RCC->APB1RSTR|=1<<17;   //复位串口2
	RCC->APB1RSTR&=~(1<<17);//停止复位	   	   
	//波特率设置
	USART2->BRR=(pclk1*1000000)/(bound);// 波特率设置	 
	USART2->CR1|=0X200C;  	//1位停止,无校验位.
	USART2->CR3=1<<7;   	//使能串口2的DMA发送
	UART_DMA_Config(DMA1_Channel7,(u32)&USART2->DR,(u32)USART2_TX_BUF);//DMA1通道7,外设为串口2,存储器为USART2_TX_BUF 
#ifdef USART2_RX_EN		  	//如果使能了接收
	//使能接收中断
	USART2->CR1|=1<<8;    	//PE中断使能
	USART2->CR1|=1<<5;    	//接收缓冲区非空中断使能	    	
	MY_NVIC_Init(2,3,USART2_IRQn,2);//组2，最低优先级 
	TIM4_Init(199,7199);		//10ms中断
	USART2_RX_STA=0;		//清零
	TIM4_Set(0);			//关闭定时器4
#endif										  	
}
//串口2,printf 函数
//确保一次发送数据不超过USART2_MAX_SEND_LEN字节
void u2_printf(char* fmt,...)  
{  
	va_list ap;
	va_start(ap,fmt);
	vsprintf((char*)USART2_TX_BUF,fmt,ap);
	va_end(ap);
	while(DMA1_Channel7->CNDTR!=0);	//等待通道7传输完成   
	UART_DMA_Enable(DMA1_Channel7,strlen((const char*)USART2_TX_BUF)); 	//通过dma发送出去
}
//定时器4中断服务程序		    
void TIM4_IRQHandler(void)
{ 	
	if(TIM4->SR&0X01)//是更新中断
	{	 			   
		USART2_RX_STA|=1<<15;	//标记接收完成
		TIM4->SR&=~(1<<0);		//清除中断标志位		   
		TIM4_Set(0);			//关闭TIM4  
	}	    
}
//设置TIM4的开关
//sta:0，关闭;1,开启;
void TIM4_Set(u8 sta)
{
	if(sta)
	{
    TIM4->CNT=0;         //计数器清空
		TIM4->CR1|=1<<0;     //使能定时器4
	}
	else TIM4->CR1&=~(1<<0);//关闭定时器4	   
}
//通用定时器中断初始化
//这里始终选择为APB1的2倍，而APB1为36M
//arr：自动重装值。
//psc：时钟预分频数		 
void TIM4_Init(u16 arr,u16 psc)
{
	RCC->APB1ENR|=1<<2;	//TIM4时钟使能    
 	TIM4->ARR=arr;  	//设定计数器自动重装值   
	TIM4->PSC=psc;  	//预分频器
 	TIM4->DIER|=1<<0;   //允许更新中断				
 	TIM4->CR1|=0x01;  	//使能定时器4	  	   
   	MY_NVIC_Init(1,3,TIM4_IRQn,2);//抢占2，子优先级3，组2	在2中优先级最低								 
}
#endif		 
///////////////////////////////////////USART2 DMA发送配置部分//////////////////////////////////	   		    
//DMA1的各通道配置
//这里的传输形式是固定的,这点要根据不同的情况来修改
//从存储器->外设模式/8位数据宽度/存储器增量模式
//DMA_CHx:DMA通道CHx
//cpar:外设地址
//cmar:存储器地址    
void UART_DMA_Config(DMA_Channel_TypeDef*DMA_CHx,u32 cpar,u32 cmar)
{
 	RCC->AHBENR|=1<<0;			//开启DMA1时钟
	delay_us(5);
	DMA_CHx->CPAR=cpar; 		//DMA1 外设地址 
	DMA_CHx->CMAR=cmar; 		//DMA1,存储器地址	 
	DMA_CHx->CCR=0X00000000;	//复位
	DMA_CHx->CCR|=1<<4;  		//从存储器读
	DMA_CHx->CCR|=0<<5;  		//普通模式
	DMA_CHx->CCR|=0<<6;  		//外设地址非增量模式
	DMA_CHx->CCR|=1<<7;  		//存储器增量模式
	DMA_CHx->CCR|=0<<8;  		//外设数据宽度为8位
	DMA_CHx->CCR|=0<<10; 		//存储器数据宽度8位
	DMA_CHx->CCR|=1<<12; 		//中等优先级
	DMA_CHx->CCR|=0<<14; 		//非存储器到存储器模式		  	
} 
//开启一次DMA传输
void UART_DMA_Enable(DMA_Channel_TypeDef*DMA_CHx,u16 len)
{
	DMA_CHx->CCR&=~(1<<0);       //关闭DMA传输 
	DMA_CHx->CNDTR=len;          //DMA1,传输数据量 
	DMA_CHx->CCR|=1<<0;          //开启DMA传输
}	   
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 									 





















