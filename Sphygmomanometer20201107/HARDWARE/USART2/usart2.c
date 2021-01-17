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
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32������
//����2��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2014/3/29
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	   

//���ڷ��ͻ����� 
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
__align(8) u8 USART2_TX_BUF[USART2_MAX_SEND_LEN]; 	//���ͻ���,���USART2_MAX_SEND_LEN�ֽ�
u16 tz[8];u16 count_save=0;
u32 page_read=0x08030000;
/***********************************************************
	* @brief 		uncode time from client then set rtc time //�ӿͻ��˽���ʱ�䲢������rtcʱ��
	* @input  	none
	* @return	none
*************************************************************/

void uncode_time_set_time(uint8_t *data)
{
	uint8_t 	i=2;//����ʱ��ָ���ʽ��һ����������޸�

	sCalendar_management.w_year=(uint16_t)((data[i+1]-0x30)*10+(data[i+2]-0x30+2000));
	sCalendar_management.w_month=(uint8_t)((data[i+4]-0x30)*10+(data[i+5]-0x30));
	sCalendar_management.w_date=(uint8_t)((data[i+7]-0x30)*10+(data[i+8]-0x30));
	sCalendar_management.hour=(uint8_t)((data[i+10]-0x30)*10+(data[i+11]-0x30));
	sCalendar_management.min=(uint8_t)((data[i+13]-0x30)*10+(data[i+14]-0x30));	
	sCalendar_management.sec=(uint8_t)((data[i+16]-0x30)*10+(data[i+17]-0x30));
}
#ifdef USART2_RX_EN   								//���ʹ���˽���   	  
//���ڽ��ջ����� 	
u16 data4; u16 ddd=3;

extern VirtAddVarTab[NumbOfVar];
uint16_t VarValue = 0;

u8 USART2_RX_BUF[USART2_MAX_RECV_LEN]; 				//���ջ���,���USART2_MAX_RECV_LEN���ֽ�.
extern u16 counter_1second; 


//ͨ���жϽ�������2���ַ�֮���ʱ������10ms�������ǲ���һ������������.
//���2���ַ����ռ������10ms,����Ϊ����1����������.Ҳ���ǳ���10msû�н��յ�
//�κ�����,���ʾ�˴ν������.
//���յ�������״̬
//[15]:0,û�н��յ�����;1,���յ���һ������.
//[14:0]:���յ������ݳ���
u16 USART2_RX_STA=0;   	 
void USART2_IRQHandler(void)
{
	u8 res;	
	u16 yx=20;
	while (yx--)
	{
		if(USART2->SR&(1<<5))//���յ�����
	{	 
		res=USART2->DR; 	
		if(USART2_RX_STA<USART2_MAX_RECV_LEN)		//�����Խ�������
		{
			TIM4->CNT=0;         					//���������
			if(USART2_RX_STA==0)TIM4_Set(1);	 	//ʹ�ܶ�ʱ��4���ж� 
			USART2_RX_BUF[USART2_RX_STA++]=res;		//��¼���յ���ֵ	 
		}else 
		{
			USART2_RX_STA|=1<<15;					//ǿ�Ʊ�ǽ������
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
//��ʼ��IO ����2
//pclk1:PCLK1ʱ��Ƶ��(Mhz)
//bound:������	  
void USART2_Init(u32 pclk1,u32 bound)
{  	 		 
	RCC->APB2ENR|=1<<2;   	//ʹ��PORTA��ʱ��  
	GPIOA->CRL&=0XFFFF00FF;	//IO״̬����
	GPIOA->CRL|=0X00008B00;	//IO״̬����	 //IO״̬����(PA2 TX  PA3 RX)	 
	RCC->APB1ENR|=1<<17;  	//ʹ�ܴ���ʱ�� 	 
	RCC->APB1RSTR|=1<<17;   //��λ����2
	RCC->APB1RSTR&=~(1<<17);//ֹͣ��λ	   	   
	//����������
	USART2->BRR=(pclk1*1000000)/(bound);// ����������	 
	USART2->CR1|=0X200C;  	//1λֹͣ,��У��λ.
	USART2->CR3=1<<7;   	//ʹ�ܴ���2��DMA����
	UART_DMA_Config(DMA1_Channel7,(u32)&USART2->DR,(u32)USART2_TX_BUF);//DMA1ͨ��7,����Ϊ����2,�洢��ΪUSART2_TX_BUF 
#ifdef USART2_RX_EN		  	//���ʹ���˽���
	//ʹ�ܽ����ж�
	USART2->CR1|=1<<8;    	//PE�ж�ʹ��
	USART2->CR1|=1<<5;    	//���ջ������ǿ��ж�ʹ��	    	
	MY_NVIC_Init(2,3,USART2_IRQn,2);//��2��������ȼ� 
	TIM4_Init(199,7199);		//10ms�ж�
	USART2_RX_STA=0;		//����
	TIM4_Set(0);			//�رն�ʱ��4
#endif										  	
}
//����2,printf ����
//ȷ��һ�η������ݲ�����USART2_MAX_SEND_LEN�ֽ�
void u2_printf(char* fmt,...)  
{  
	va_list ap;
	va_start(ap,fmt);
	vsprintf((char*)USART2_TX_BUF,fmt,ap);
	va_end(ap);
	while(DMA1_Channel7->CNDTR!=0);	//�ȴ�ͨ��7�������   
	UART_DMA_Enable(DMA1_Channel7,strlen((const char*)USART2_TX_BUF)); 	//ͨ��dma���ͳ�ȥ
}
//��ʱ��4�жϷ������		    
void TIM4_IRQHandler(void)
{ 	
	if(TIM4->SR&0X01)//�Ǹ����ж�
	{	 			   
		USART2_RX_STA|=1<<15;	//��ǽ������
		TIM4->SR&=~(1<<0);		//����жϱ�־λ		   
		TIM4_Set(0);			//�ر�TIM4  
	}	    
}
//����TIM4�Ŀ���
//sta:0���ر�;1,����;
void TIM4_Set(u8 sta)
{
	if(sta)
	{
    TIM4->CNT=0;         //���������
		TIM4->CR1|=1<<0;     //ʹ�ܶ�ʱ��4
	}
	else TIM4->CR1&=~(1<<0);//�رն�ʱ��4	   
}
//ͨ�ö�ʱ���жϳ�ʼ��
//����ʼ��ѡ��ΪAPB1��2������APB1Ϊ36M
//arr���Զ���װֵ��
//psc��ʱ��Ԥ��Ƶ��		 
void TIM4_Init(u16 arr,u16 psc)
{
	RCC->APB1ENR|=1<<2;	//TIM4ʱ��ʹ��    
 	TIM4->ARR=arr;  	//�趨�������Զ���װֵ   
	TIM4->PSC=psc;  	//Ԥ��Ƶ��
 	TIM4->DIER|=1<<0;   //��������ж�				
 	TIM4->CR1|=0x01;  	//ʹ�ܶ�ʱ��4	  	   
   	MY_NVIC_Init(1,3,TIM4_IRQn,2);//��ռ2�������ȼ�3����2	��2�����ȼ����								 
}
#endif		 
///////////////////////////////////////USART2 DMA�������ò���//////////////////////////////////	   		    
//DMA1�ĸ�ͨ������
//����Ĵ�����ʽ�ǹ̶���,���Ҫ���ݲ�ͬ��������޸�
//�Ӵ洢��->����ģʽ/8λ���ݿ��/�洢������ģʽ
//DMA_CHx:DMAͨ��CHx
//cpar:�����ַ
//cmar:�洢����ַ    
void UART_DMA_Config(DMA_Channel_TypeDef*DMA_CHx,u32 cpar,u32 cmar)
{
 	RCC->AHBENR|=1<<0;			//����DMA1ʱ��
	delay_us(5);
	DMA_CHx->CPAR=cpar; 		//DMA1 �����ַ 
	DMA_CHx->CMAR=cmar; 		//DMA1,�洢����ַ	 
	DMA_CHx->CCR=0X00000000;	//��λ
	DMA_CHx->CCR|=1<<4;  		//�Ӵ洢����
	DMA_CHx->CCR|=0<<5;  		//��ͨģʽ
	DMA_CHx->CCR|=0<<6;  		//�����ַ������ģʽ
	DMA_CHx->CCR|=1<<7;  		//�洢������ģʽ
	DMA_CHx->CCR|=0<<8;  		//�������ݿ��Ϊ8λ
	DMA_CHx->CCR|=0<<10; 		//�洢�����ݿ��8λ
	DMA_CHx->CCR|=1<<12; 		//�е����ȼ�
	DMA_CHx->CCR|=0<<14; 		//�Ǵ洢�����洢��ģʽ		  	
} 
//����һ��DMA����
void UART_DMA_Enable(DMA_Channel_TypeDef*DMA_CHx,u16 len)
{
	DMA_CHx->CCR&=~(1<<0);       //�ر�DMA���� 
	DMA_CHx->CNDTR=len;          //DMA1,���������� 
	DMA_CHx->CCR|=1<<0;          //����DMA����
}	   
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 									 





















