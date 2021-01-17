/*
 * @Author: your name
 * @Date: 2020-10-12 14:03:02
 * @LastEditTime: 2020-11-05 10:53:18
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \USERc:\Users\Administrator\Desktop\boold _pressure\hh\HARDWARE\TASK\task.c
 */
 #include "task.h"
 #include "parameter.h"
 #include "usart.h"
 #include "string.h"
 #include "gimbal.h"
 #include "rtc.h"
 #include "flash.h"
 #include "wkup.h"
 #include "math.h"
 #include "parameter.h"	
 #include"eeprom.h"
 #include"sys.h"
 #include<jansson.h>
float  HR_Time[255]; //存取脉搏波峰位置的时间间隔
u16 datal;
u16 MABO_Adc_MAX=0;
u16 save_buf[8];
float HR_BPM=0;
//extern VirtAddVarTab[NumbOfVar];
static 	u16 	string_temp[100];
				u16 	string_temp_date[100];
				u16   string_temp_time_date_syn_measure[100];
				u8  start_times;
//u16 YALI_Buff[1000];
//u16 MABO_peak_Buff[1000];
u16 maibo_max_position=0,maibo_peak_position=0,peak_number=0; 
u16 adc_mabo=0,max_mmgh = 0,insert_value[4]={0};
u16 PD_position=0,PS_position=0; //收缩压、舒展压中间值的位置
u16 MABO_Adc=0,YALI_Adc=0;
u16 test1[8]={1,223,348,11,112,213,121,343};
u32 page_test=0x08030000;
int code=1;
float msec=0;
float  MABO_Adc_Convert,YALI_Adc_Convert;
float	 avarage_basevalue=0,maibo_peak_percent[100]={0};
extern  u8 M_mode; 

/***********************************************************
	* @brief 		calculate_avarage_basevalue
	* @input  	none
	* @return	  none
*************************************************************/
void shuzu_init(void)
{
  maibo_max_position=0;
	max_mmgh = 0;
	peak_number = 0;
	avarage_basevalue = 0;
	sData_single_management.base_number=0;
	sData_single_management.adc_number=0;	
	sData_single_management.YALI_PS_insert_mmgh=0;
	sData_single_management.YALI_PD_insert_mmgh=0;
	sData_single_management.ReStart_Times_flag=0;
	sData_single_management.Averg_HR=0;
	memset(HR_Time,0,sizeof(HR_Time));
  memset(sData_single_management.ADC_ConvertedValue_maibo31,0,sizeof(sData_single_management.ADC_ConvertedValue_maibo31));
  memset(sData_single_management.ADC_ConvertedValue_yali31,0,sizeof(sData_single_management.ADC_ConvertedValue_yali31));
  memset(sData_single_management.ADC_ConvertedValue0,0,sizeof(sData_single_management.ADC_ConvertedValue0));
	memset(sData_single_management.ADC_ConvertedValue1,0,sizeof(sData_single_management.ADC_ConvertedValue1));
  memset(sData_single_management.ADC_ConvertedValue2,0,sizeof(sData_single_management.ADC_ConvertedValue2));
	memset(sData_single_management.ADC_peak_position,0,sizeof(sData_single_management.ADC_peak_position));
	memset(sData_single_management.ADC_peak_maibo,0,sizeof(sData_single_management.ADC_peak_maibo));
	memset(maibo_peak_percent,0,sizeof(maibo_peak_percent));
	
}

/***********************************************************
	* @brief 		Error Happen ReStart Measure
	* @input  	none
	* @return	1: 
*************************************************************/
void Error_ReStart_Measure(void)//具体内容未完善
{
	code=0;
	Valve_close;
	Motor_close;
	printf("sFunctionCode.ErrorMode=%d\r\n",sFunctionCode.ErrorMode);
	/*switch (sFunctionCode.ErrorMode)//错误打印
	{
	case 0:
		u2_printf("\nNone error");
		break;
	case 1:
		u2_printf("\nError:pressure is over 300");
		delay_ms(50);
	case 2:
		u2_printf("\nError:The pressure value exceeds the sampling value");
		delay_ms(100);
	case 3:
		u2_printf("\nError:Diastolic pressure exceeds telescopic pressure");
		delay_ms(50);
	default:
		//u2_printf("\nError:Else error");
		break;
	}*/
	delay_ms(50);
//	u2_printf("Error=%d\r\n",sFunctionCode.ErrorMode);
	shuzu_init();
//	delay_ms(1000);
//	delay_ms(1000);
	//读取配置模式数据，START//
	//sFunctionCode.FunctionCode = StartMeasure;
	//***********************//
	sData_single_management.ReStart_flag=1;
	sData_single_management.ReStart_Times_flag++;
	counter_1second=0;
}
/***********************************************************
	* @brief 		calculate rent maibo_peak percent
	* @input  	none
	* @return	  maibo_peak_percent
*************************************************************/
void calculate_percent(void)
{
			uint16_t  i;
	    for(i=0; i<60; i++)
			{
				maibo_peak_percent[i]=(float)sData_single_management.ADC_peak_maibo[i]/(float)MABO_Adc_MAX;	
//        printf("%lf\r\n",maibo_peak_percent[i]);								
			}					
}
/***********************************************************
	* @brief 		insert value and calculate yali value
	* @input  	none  150-100
	* @return	  none
*************************************************************/
void insert_value_calculate150_100()//插入100-150的值
{
	int i=0,a=0,b=0,yl1=0,yl2=0,c=0,d=0,yl3=0,yl4=0;
	float bl100=0,bl150=0;
	for(i=0;i<60;i++)
			{
				//100比例
				if((sData_single_management.ADC_peak_position[i]<maibo_max_position)
					&&((maibo_peak_percent[i]<0.8)
					&&(maibo_peak_percent[i+1]>=0.8)))
					{
//					printf("d:%d,%d\r\n", sData_single_management.ADC_peak_position[i+2],sData_single_management.ADC_peak_position[i+3]);
						insert_value[0]=sData_single_management.ADC_peak_position[i];
						insert_value[1]=sData_single_management.ADC_peak_position[i+1];
						PD_position=insert_value[0]+((0.8-maibo_peak_percent[i])
						*(insert_value[1]-insert_value[0]))
						/(maibo_peak_percent[i+1]-maibo_peak_percent[i]);
						//PD_position=((0.5-maibo_peak_percent[i])*insert_value[1]+ maibo_peak_percent[i+1]*insert_value[0]-0.5*insert_value[0])/(maibo_peak_percent[i+1]-maibo_peak_percent[i]);
//								printf("\r\n%f,%f\r\n",maibo_peak_percent[i],maibo_peak_percent[i+1]);//前后分别占比
						delay_ms(5);
//						printf("PD_POSITION:%d\r\n",PD_position);	
            delay_ms(5);	
							a=insert_value[0];b=insert_value[1];
						yl1=sData_single_management.ADC_ConvertedValue1[a];
						yl2=sData_single_management.ADC_ConvertedValue1[b];
						bl100=(maibo_peak_percent[i+1]-maibo_peak_percent[i])*(100-yl1)/(yl2-yl1)+maibo_peak_percent[i];
						u2_printf("%d,%d\r\n",yl1,yl2);//前后对应压力
							delay_ms(50);
						u2_printf("bl100=%1f\r\n\r\n",bl100);//100对应比例
							delay_ms(50);
					sData_single_management.YALI_PD_insert_mmgh = sData_single_management.ADC_ConvertedValue1[PD_position];
						sCalculation_management.PD=sData_single_management.YALI_PD_insert_mmgh;
	    //   printf("150YALI_PD_insert_mmgh:%d\r\n",sData_single_management.YALI_PD_insert_mmgh); 									
					}       
				else	if((sData_single_management.ADC_peak_position[i]>
					maibo_max_position)&&((maibo_peak_percent[i]>0.7)
				&&(maibo_peak_percent[i+1]<=0.7)))
					{
						//150比例
//					printf("s:%d,%d\r\n", sData_single_management.ADC_peak_position[i],sData_single_management.ADC_peak_position[i+1]);
						insert_value[2]=sData_single_management.ADC_peak_position[i];
						insert_value[3]=sData_single_management.ADC_peak_position[i+1];
						PS_position=insert_value[3]-((0.7-maibo_peak_percent[i+1])
						*(insert_value[3]-insert_value[2]))
						/(maibo_peak_percent[i]-maibo_peak_percent[i+1]);
//					PS_position=((0.55-maibo_peak_percent[i])*insert_value[3]+ maibo_peak_percent[i+1]*insert_value[2]-0.55*insert_value[2])/(maibo_peak_percent[i+1]-maibo_peak_percent[i]);
//					printf("%f,%f\r\n",maibo_peak_percent[i],maibo_peak_percent[i+1]);	//前后分别占比
					  c=insert_value[2];d=insert_value[3];
						yl3=sData_single_management.ADC_ConvertedValue1[c];
						yl4=sData_single_management.ADC_ConvertedValue1[d];
						bl150=maibo_peak_percent[i]-(maibo_peak_percent[i]-maibo_peak_percent[i+1])*(yl3-150)/(yl3-yl4);
						u2_printf("%d,%d\r\n",yl3,yl4);//前后对应压力
						delay_ms(50);
						u2_printf("bl150=%1f\r\n\r\n",bl150);//150对应比例
						delay_ms(50);
//						 printf("PS_POSITION:%d\r\n",PS_position);						
						if(0==sData_single_management.YALI_PS_insert_mmgh)
						{
            sData_single_management.YALI_PS_insert_mmgh = sData_single_management.ADC_ConvertedValue1[PS_position];	
							sCalculation_management.PS=sData_single_management.YALI_PD_insert_mmgh;
//						printf("150YALI_PS_insert_mmgh:%d\r\n",sData_single_management.YALI_PS_insert_mmgh);   
						}						
						printf("150YALI_PS_insert_mmgh:%d\r\n",sData_single_management.YALI_PS_insert_mmgh);     									
					}	
			 }
}


void calculate_avarage_basevalue(void)//计算平均基点值
{
			u16  i=0;
	float	  sum_basevalue=0;
	    for(i=0; i<50; i++)
			{
					sum_basevalue =sum_basevalue +	(float)sData_single_management.ADC_ConvertedValue2[i];
//					printf("%d\r\n",sData_single_management.ADC_ConvertedValue2[i]);	
			}	
      avarage_basevalue	= sum_basevalue/50;	
//			printf("avarage_basevalue:%f\r\n",avarage_basevalue);			
}

/***********************************************************
	* @brief 		calculate P-P Time
	* @input  	none
	* @return	none
*************************************************************/



void get_HR(u16 count_peak_number )
{
	float HRCurrentTime;	
//	float HRLoopTime;
	
	if((count_peak_number)==0)
	{
		 TIM3->CNT=0X000000;
//		HRStartTime =TIM_GetCounter(TIM3); //第一个脉搏峰值对应的位置
		//printf("HRStartTime=%d\r\n",HRStartTime); 
	}
	else
	{
		HRCurrentTime = TIM_GetCounter(TIM3);
		msec=HRCurrentTime*0.0005;
   // HRLoopTime = get_system_deltaT(HRStartTime, HRCurrentTime);
	//printf("HRLoopTime =%d\r\n",HRLoopTime );
		HR_Time[count_peak_number-1] = msec; 
		 TIM3->CNT=0X000000;
//    HRStartTime =	HRCurrentTime;
		
	}
}
void data_FIFO(void)
{
  u16 x,y,hello;
//	MABO_Adc =Get_Adc(ADC_Channel_0);
	MABO_Adc = Get_Adc_Average(ADC_Channel_0,100);
	YALI_Adc = Get_Adc_Average(ADC_Channel_1,100);
//	YALI_Adc_Convert=0.2035*YALI_Adc-148.54;
	//YALI_Adc_Convert=0.2139*YALI_Adc-129; //更新标定1228
	YALI_Adc_Convert=0.165*YALI_Adc-147.49;
		//YALI_Adc_Convert=0.1533*YALI_Adc-129.99;  //更新标定0731
	//YALI_Adc_Convert=0.16*YALI_Adc-77.849;  //更新标定0509
	 if(sData_single_management.base_number < 50)
	 {
				sData_single_management.ADC_ConvertedValue2[sData_single_management.base_number] = MABO_Adc;
//		   hello = sData_single_management.ADC_ConvertedValue2[sData_single_management.base_number];
		 
//		    printf("%d\r\n",MABO_Adc);  
					
				if(sData_single_management.base_number == 49)
				{
						calculate_avarage_basevalue();
				}
				sData_single_management.base_number++;
	 }
	else
		{
					Motor_open;		
	   sData_single_management.ADC_ConvertedValue0[sData_single_management.adc_number]=MABO_Adc-avarage_basevalue;
	   sData_single_management.ADC_ConvertedValue1[sData_single_management.adc_number]=YALI_Adc_Convert;
						
			
						x = sData_single_management.ADC_ConvertedValue1[sData_single_management.adc_number];
						y = sData_single_management.ADC_ConvertedValue0[sData_single_management.adc_number];
					//	printf("%5d%5d\r\n",sData_single_management.adc_number,y);  //脉搏曲线
		//	printf("%5d\r\n",y); 
//				printf("%f\r\n",avarage_basevalue); 
		//	printf("%d\n",x);
			if(sData_single_management.adc_number%10==0)
			{
		//	printf("%d,",x);
			//u2_printf("PR%d%d",digit(x),x);	
		//	u2_printf("\n%d",x); 
			}
	//		printf("%d\r\n",MABO_Adc);
		//	u2_printf("%d\r\n",YALI_Adc);
//			printf("%5d%5d\r\n",MABO_Adc,YALI_Adc);
	   sData_single_management.adc_number++;		
	}
}
u16 Getmax_maibo(void)
{
	//printf("Getmax_maibo\n");
	u16  i,max_maibo = 0;
	for(i=0; i<MAX_ADC_PRINT ; i++)
		{
			
			if(i>50)	
	       {				
						if(sData_single_management.ADC_ConvertedValue0[i] > max_maibo)
							{
								max_maibo = sData_single_management.ADC_ConvertedValue0[i];
								maibo_max_position = i;
//							 printf("%d  %d\n",max_maibo,i); 
				
							} 
					}
			}
		return max_maibo;	
}
 void Getpeak_maibo(void)
{
	 // printf("Getpeak_maibo/n");
		u16 i,j,maibo_peak=0,count_peak_number=0;
		
		for(i=0;i<30;i++)
		{
			sData_single_management.ADC_ConvertedValue_maibo31[i] =
			sData_single_management.ADC_ConvertedValue_maibo31[i+1];
		}
		sData_single_management.ADC_ConvertedValue_maibo31[30]= MABO_Adc-avarage_basevalue;
		
		for(j=0;j<31;j++)
		{
			if(sData_single_management.ADC_ConvertedValue_maibo31[j]>maibo_peak)
			{
					maibo_peak = sData_single_management.ADC_ConvertedValue_maibo31[j];
				  maibo_peak_position = j;
			}	
		}
	   if(maibo_peak_position == 15)
		{
	  
			  if((peak_number>1)&&(peak_number<100))
				{
					sData_single_management.ADC_peak_maibo[peak_number]=maibo_peak;
//					b[peak_number]=sData_single_management.ADC_peak_maibo[peak_number];
					sData_single_management.ADC_peak_position[peak_number]=sData_single_management.adc_number-15; 
				
//					printf("peak1:%5d%5d\r\n",sData_single_management.ADC_peak_position[peak_number],sData_single_management.ADC_peak_maibo[peak_number]);
//					printf("peak1:%5d\r\n",sData_single_management.ADC_peak_maibo[peak_number]);
				}	
					if(peak_number>=2)
				{
					count_peak_number=peak_number-2;
					get_HR(count_peak_number);
				}
			  peak_number++;	
		}
	}

u8 calculate_HR_PD_PS(void)//heart rate  Systolic blood pressure  Diastolic pressure
{
	
	u16 cut;
//	u16	maibo_max_adc=0, yali_max_adc=0,i=0,k=0;
//	u16  MABO_peak_adc=0, YALI_peak_adc=0;
 // static bool flag_start = false;
	
	//u16 max_maibo = 0;
	//float SUM_HR=0,HR_BPM=0,peak_d=0;
	float SUM_HR=0,peak_d=0;
	float a[300]={0};
	u16 b[300];
	u16 m_peak[300],n_position[300];//用于存放插值后波峰
	u16 max1=0;
	u8 i;
	Valve_open;       //开启放气阀，保证开始不漏气
//  Motor_open;       //开启电机，使电平处于高电平
//	nrf_set_roll_a_pwm(1600);    //设置电机启动PWM速度为非全速
	
	data_FIFO();         //数据接受和处理
	
  if(YALI_Adc_Convert>=300)
		{
			sFunctionCode.ErrorMode = YALI_Over_300;
			Error_ReStart_Measure();
			//u2_printf("error");
			return 1;
		}
  MABO_Adc_MAX=Getmax_maibo();   //获取脉搏最大值
	Getpeak_maibo();               //获取所有脉搏波峰值并进行滤波
		max1=maibo_max_position;
//	cut=maibo_max_position-sData_single_management.ADC_peak_position[peak_number];

//	printf("sData_single_management.ADC_peak_position[peak_number-1]=%4d\r\n",sData_single_management.ADC_peak_position[peak_number-1]);
	 if(sData_single_management.adc_number>=MAX_ADC_PRINT )
		 { 
			 sFunctionCode.ErrorMode = Over_MAX_PRINT;			 
			 Error_ReStart_Measure();
			 return 1;
//					Valve_close;
//					Motor_close;
//	        sData_single_management.adc_number=0;
//					return 1;
//				  M_complete_flag=0;
//					printf("MABO_Adc_MAX=%d\r\n",MABO_Adc_MAX);
//					printf("maibo_max_position=%d\r\n",maibo_max_position);
				}
	if(sData_single_management.ADC_peak_position[peak_number-1]>maibo_max_position)//峰值位置大于最大值位置判断
	{
	  if((0<sData_single_management.ADC_peak_maibo[peak_number-1])&&(sData_single_management.ADC_peak_maibo[peak_number-1]
			<=(MABO_Adc_MAX*0.6))&&(YALI_Adc_Convert>80))//停止条件判断
		    {
					printf("error1");
					Valve_close;
					Motor_close;
//					printf("peak_number=%d\r\n",peak_number);//波峰个数 
					for(i=peak_number-6;i<peak_number-3;i++)
						{
							SUM_HR+=HR_Time[i];
							sData_single_management.Averg_HR=SUM_HR/3;
							HR_BPM=60/sData_single_management.Averg_HR;
						}
					for(i=0;i<peak_number;i++)	
						{
//							if(sData_single_management.ADC_peak_maibo[i]<=sData_single_management.ADC_peak_maibo[i-1]*2/3)
//							{sData_single_management.ADC_peak_maibo[i]=sData_single_management.ADC_peak_maibo[i+1];}
							
							m_peak[2*i-1]=sData_single_management.ADC_peak_maibo[i-1]+(sData_single_management.ADC_peak_maibo[i]-sData_single_management.ADC_peak_maibo[i-1])/2;
							n_position[2*i-1]=sData_single_management.ADC_peak_position[i-1]+(sData_single_management.ADC_peak_position[i]-sData_single_management.ADC_peak_position[i-1])/2;
							m_peak[2*i]=sData_single_management.ADC_peak_maibo[i];
							n_position[2*i]=sData_single_management.ADC_peak_position[i];
						}  //波峰中间插值（一个）到m n数组中
						for(i=0;i<2*peak_number-1;i++)
						{
							sData_single_management.ADC_peak_maibo[i]=m_peak[i];
							sData_single_management.ADC_peak_position[i]=n_position[i];
						}//中间插值结束，赋值回脉搏数组
					peak_d=sData_single_management.ADC_peak_position[peak_number-1]-sData_single_management.ADC_peak_position[peak_number-2];
					for(i=2*peak_number-1;i<peak_number*3-1;i++)
					{
						sData_single_management.ADC_peak_position[i]=sData_single_management.ADC_peak_position[i-1]+peak_d;
						sData_single_management.ADC_peak_maibo[i]=sData_single_management.ADC_peak_maibo[i-1]-sData_single_management.ADC_peak_maibo[2*peak_number-2]/peak_number;
					}
					sData_single_management.ADC_peak_position[3*peak_number-1]=sData_single_management.ADC_peak_position[peak_number*3-2]+peak_d;
					sData_single_management.ADC_peak_maibo[peak_number*3-1]=0;  //滤波前结尾补充波峰点
					
					for(i=0;i<peak_number*3;i++)
					{
						b[i] = sData_single_management.ADC_peak_maibo[i];  //原波峰赋值
						a[i] = (float)(sData_single_management.ADC_peak_maibo[i]*1.000);
//						printf("a[i]fz=%f\r\n",a[i]);
//						printf("%5d%5d\r\n",sData_single_management.ADC_peak_position[i],sData_single_management.ADC_peak_maibo[i]);
//					printf("%5d\r\n",sData_single_management.ADC_peak_maibo[i]);
					
					}
					
					for(i=0;i<peak_number*3;i++)
					{
					a[i]=
					1.76*a[i-1]
					-1.18*a[i-2]
					+0.278*a[i-3]
					+(float)(0.0181*b[i])
					+(float)(0.05429*b[i-1])
					+(float)(0.05429*b[i-2])
					+(float)(0.0181*b[i-3]); //100HZ三阶滤波
					if(i>2)
					{
						sData_single_management.ADC_peak_maibo[i]=a[i];
						sData_single_management.ADC_peak_position[i]=sData_single_management.ADC_peak_position[i]-123;
					}
					
//					printf("peaka:%d\r\n%f\r\n",sData_single_management.ADC_peak_position[i],a[i]);
//					printf("%5d%5d\r\n",sData_single_management.ADC_peak_position[i],sData_single_management.ADC_peak_maibo[i]);
					}  //滤波波峰
					
					delay_ms(50);
					MABO_Adc_MAX=0;
					for(i=0;i<60;i++)	
	       {				
						if(sData_single_management.ADC_peak_maibo[i] > MABO_Adc_MAX)
							{
								MABO_Adc_MAX = sData_single_management.ADC_peak_maibo[i];
								maibo_max_position = sData_single_management.ADC_peak_position[i];
//								printf("position/max:%5d%5d\r\n",maibo_max_position,MABO_Adc_MAX); 
							} 
							
//							printf("position/max:%5d%5d\r\n",maibo_max_position,MABO_Adc_MAX); 
					}//重新获取脉搏最大值及其坐标
					
					calculate_percent();
					max_mmgh = sData_single_management.ADC_ConvertedValue1[maibo_max_position];
					insert_value_calculate150_100();
					sData_single_management.adc_number=0;
//          printf("150YALI_PD_insert_mmgh:%d,%f\r\n",sData_single_management.YALI_PD_insert_mmgh,sCalculation_management.PD); 
//					printf("150YALI_PS_insert_mmgh:%d\r\n",sData_single_management.YALI_PS_insert_mmgh);
//					u2_printf("[%d]",sData_single_management.YALI_PD_insert_mmgh);
					delay_ms(50);
//					u2_printf("[%d]",sData_single_management.YALI_PS_insert_mmgh);
//					delay_ms(50);
//					for(i=0;i<peak_number-4;i++)
//						{
//							SUM_HR+=HR_Time[i];
//							sData_single_management.Averg_HR=SUM_HR/(peak_number-4);
//							HR_BPM=60/sData_single_management.Averg_HR;
//						}
//						printf("sData_single_management.Averg_HR =%f\r\n",sData_single_management.Averg_HR );
//							printf("HR_BPM =%f\r\n",HR_BPM );
//						u2_printf("[%f]",HR_BPM);
					//code=2333;
					//if(code==2333)
					//{
	delay_ms(50);
//u2_printf("\nsucess:%d,%d,%d",sData_single_management.YALI_PD_insert_mmgh,sData_single_management.YALI_PS_insert_mmgh,(u16)HR_BPM);
//EE_WriteVariable(VirtAddVarTab[0],4);
//EE_ReadVariable(VirtAddVarTab[0],&datal);
delay_ms(50);
save_buf[0]=calendar.w_year;
save_buf[1]=calendar.w_month;
save_buf[2]=calendar.w_date;
save_buf[3]=calendar.hour;
save_buf[4]=calendar.min;
save_buf[5]=sData_single_management.YALI_PD_insert_mmgh;
save_buf[6]=sData_single_management.YALI_PS_insert_mmgh;
save_buf[7]=(u16)HR_BPM;
//u2_printf("%d",save_buf[6]);
delay_ms(50);
u2_printf("\n%d,%d,%d,%d",code,sData_single_management.YALI_PD_insert_mmgh,sData_single_management.YALI_PS_insert_mmgh,(u16)HR_BPM);
delay_ms(50);
//printf("the reword is %d",datal);	
			//		}
					delay_ms(50);
//						for(i=0;i<50;i++)
//					{
//						printf("lbpeak:%5d%5d\r\n",sData_single_management.ADC_peak_position[i],a[i]);
//					}
//						for(i=0;i<peak_number*2-1;i++)
//				printf("maibo_max_position :%5d\r\n",maibo_max_position );
						for(i=0;i<peak_number*3;i++)
					{
//						printf("%5d\r\n",sData_single_management.ADC_peak_maibo[i]);						
//						printf("%5d%5d\r\n",sData_single_management.ADC_peak_position[i],sData_single_management.ADC_peak_maibo[i]);
//						printf("peakmn:%5d%5d\r\n",n_position[i],m_peak[i]);
					}//纯输出波峰点xy坐标
					
					
					if(sData_single_management.YALI_PD_insert_mmgh >= sData_single_management.YALI_PS_insert_mmgh)
					{
						sFunctionCode.ErrorMode = PD_Over_PS;			 
					  Error_ReStart_Measure();
						return 1;
					}
					shuzu_init();
					return 1;
				}
					
	}

return 0;
	}	
/***********************************************************
	* @brief 		attach time stamp
	* @input  	1 for single mode & 0 for repeat measer mode
	* @return	none
*************************************************************/
void attach_time_stamp(uint8_t mode)//附加时间戳
{
	u8 string_temp1[100];
	u16 string_temp_IIC_time_date[100];
	u16 mode_show;
	
	memset(string_temp, 0, sizeof(string_temp));
	memset(string_temp_date,0,sizeof(string_temp_date));
	memset(string_temp1, 0, sizeof(string_temp1));
		

	STMFLASH_Read(Flash_Addr.Mode_Addr,&mode_show,1);
	sprintf((char*)string_temp1, "%d",mode_show );
  strcat((char*)string_temp, (const char*)string_temp1);
	
		memset(string_temp1, 0, sizeof(string_temp1));
	sprintf((char*)string_temp1, "%02d",sData_single_management.data_number );
  strcat((char*)string_temp, (const char*)string_temp1);
	
	
	memset(string_temp1, 0, sizeof(string_temp1));
	sprintf((char*)string_temp1, "%d", calendar.w_year);
  strcat((char*)string_temp, (const char*)string_temp1);
strcat((char*)string_temp, "-");
	
	memset(string_temp1, 0, sizeof(string_temp1));
	sprintf((char*)string_temp1, "%02d", calendar.w_month);
	strcat((char*)string_temp, (const char*)string_temp1);
strcat((char*)string_temp, "-");
	
	memset(string_temp1, 0, sizeof(string_temp1));
	sprintf((char*)string_temp1, "%02d", calendar.w_date);
	strcat((char*)string_temp, (const char*)string_temp1);
	strcat((char*)string_temp, ";");	
	
	memset(string_temp1, 0, sizeof(string_temp1));
	sprintf((char*)string_temp1, "%02d", calendar.hour);
	strcat((char*)string_temp, (const char*)string_temp1);
	strcat((char*)string_temp, ":");	
	
		memset(string_temp1, 0, sizeof(string_temp1));
	sprintf((char*)string_temp1, "%02d", calendar.min);
	strcat((char*)string_temp, (const char*)string_temp1);
	strcat((char*)string_temp, ":");	
	
		memset(string_temp1, 0, sizeof(string_temp1));
	sprintf((char*)string_temp1, "%02d", calendar.sec);
	strcat((char*)string_temp, (const char*)string_temp1);
	strcat((char*)string_temp, ";");	
		memset(string_temp1, 0, sizeof(string_temp1));
	

	
	
	if(mode == 0)
	{
    sprintf((char*)string_temp1, "%d", mode_show);
		strcat((char*)string_temp_date, (const char*)string_temp1);
		
		sprintf((char*)string_temp1, "%02d", sData_single_management.data_number);
		strcat((char*)string_temp_date, (const char*)string_temp1);
		
		
		memset(string_temp1, 0, sizeof(string_temp1));
		sprintf((char*)string_temp1, "%03d", sCalculation_management.HR);
		strcat((char*)string_temp_date, (const char*)string_temp1);
		strcat((char*)string_temp_date, ";");
		
		memset(string_temp1, 0, sizeof(string_temp1));
		sprintf((char*)string_temp1, "%03d", (uint8_t)(sCalculation_management.PD));
		strcat((char*)string_temp_date, (const char*)string_temp1);
		strcat((char*)string_temp_date, ";");
		
		memset(string_temp1, 0, sizeof(string_temp1));
		sprintf((char*)string_temp1, "%03d", (uint8_t)(sCalculation_management.PS));
		strcat((char*)string_temp_date, (const char*)string_temp1);
	  
	}
	

	  
  memset(string_temp_IIC_time_date, 0, sizeof(string_temp_IIC_time_date));
	STMFLASH_Write(Flash_Addr.IIC_attach_Addr,string_temp,20);
	STMFLASH_Read(Flash_Addr.IIC_attach_Addr,string_temp_IIC_time_date,20);
	Flash_Addr.IIC_attach_Addr=Flash_Addr.IIC_attach_Addr+20;
//	u2_printf("xx\r\n");

//u2_printf("%s\r\n",(char*)string_temp_IIC_time_date);//here
//	u2_printf("string_temp_IIC_time_date\r\n");
	memset(string_temp_IIC_time_date, 0, sizeof(string_temp_IIC_time_date));
	STMFLASH_Write(Flash_Addr.IIC_attach_Addr,string_temp_date,17);
	STMFLASH_Read(Flash_Addr.IIC_attach_Addr,string_temp_IIC_time_date,17);
	Flash_Addr.IIC_attach_Addr=Flash_Addr.IIC_attach_Addr+17;
//	u2_printf("%s\r\n",(char*)string_temp_IIC_time_date);//here
//	u2_printf("string_temp_IIC_time_date\r\n");
	memset(string_temp_IIC_time_date, 0, sizeof(string_temp_IIC_time_date));

}

/***********************************************************
	* @brief 		measure data in different mode
	* @input  	none
	* @return	1:completed 0:uncompleted
*************************************************************/
u8 data_measure(void)
{
	switch(sFunctionCode.MeasureMode)
	{
		/*single mesure mode*/
		case MeasureMode0:
		{
			MeasureSection.All_times=1;
	//		AT24CXX_Write(IIC_AT24C_Addr.IIC_times_Addr,&MeasureSection.All_times,1);
			/*if calculate data completed*/
			if(calculate_HR_PD_PS())
			{	
				 while (1)
                 { 
					if(*(vu16*)page_test==0xffff)
					   break;
					page_test+=0x20;
				 }
			 	FLASH_Unlock();
			 	STMFLASH_Write_NoCheck(page_test,save_buf,sizeof(save_buf));
			 	FLASH_Lock();
				sFunctionCode.FunctionCode = None;
				sData_single_management.data_number = 1;
				attach_time_stamp(0);
				STMFLASH_Write(Flash_Addr.IIC_times_Addr,&MeasureSection.All_times,1);
//				attach_time_stamp(0);
//				AT24CXX_Write(IIC_AT24C_Addr.IIC_times_Addr,&MeasureSection.All_times,1);	
				return 0;
			}
			break;
		}
				
		case MeasureMode1:
		{
			/*some error in this, so clear the value then ..forget  oops~*//* amend*/
			STMFLASH_Write(Flash_Addr.IIC_times_Addr,&MeasureSection.All_times,1);
		//	AT24CXX_Write(IIC_AT24C_Addr.IIC_times_Addr,&MeasureSection.All_times,1);
		if(sData_single_management.data_number<MeasureSection.all_times)			
		{
			if(calculate_HR_PD_PS())
			{	
				
			   u2_printf("\nsucess:%d,%d,%d",sData_single_management.YALI_PD_insert_mmgh,sData_single_management.YALI_PS_insert_mmgh,(u16)HR_BPM);
			   while (1)
                 { 
					if(*(vu16*)page_test==0xffff)
					   break;
					page_test+=0x20;
				 }
				// u2_printf("%d",save_buf[6]);
			 // STMFLASH_Write(page_test,save_buf,sizeof(save_buf));
			 //u2_printf("\n%d%d%d\n" ,save_buf[5],save_buf[6],save_buf[7]);
			 FLASH_Unlock();
			 STMFLASH_Write_NoCheck(page_test,save_buf,sizeof(save_buf));
			 FLASH_Lock();
			 //delay_ms(50);
			 //u2_printf("all_time is %d\n",MeasureSection.all_times);
			 delay_ms(50);
			// u2_printf("data_number is %u\n",sData_single_management.data_number);
			 sData_single_management.data_number++;
			 attach_time_stamp(0);
						//	if((sCalendar_management.hour>=7)&&(sCalendar_management.hour<19))
			  printf("calendar.min is %d",calendar.min);
			  RTC_WaitForLastTask();	//等待最近一次对RTC寄存器的写操作完成
			  RTC_SetAlarm(RTC_GetCounter()+MeasureSection.Interval_Time);//为了形成循环（我调我自己）
              sFunctionCode.FunctionCode = None;
/*if(((calendar.min%30)>=10)&&((calendar.min%30)<20))
			  {
			    MeasureSection.TimeDiv=1;
					RTC_WaitForLastTask();	//等待最近一次对RTC寄存器的写操作完成	
          RTC_SetAlarm(RTC_GetCounter()+MeasureSection.Interval_Time);
		  printf("我在这1");
			//	 MeasureSection.M_number=(19-sCalendar_management.hour)*2-sCalendar_management.min/30;
			//	MeasureSection.M_number=(8-(sCalendar_management.min%10))*2-sCalendar_management.sec/30;
					MeasureSection.M_number=(2-((calendar.min%30)/10))*2-((calendar.min%30)/15);

			  }
			  else
        {
					
			    MeasureSection.TimeDiv=2;
					RTC_WaitForLastTask();	//等待最近一次对RTC寄存器的写操作完成	
          RTC_SetAlarm(RTC_GetCounter()+MeasureSection.Interval_Time);//为了形成循环（我调我自己）
		   //printf("我在这000");
		   sFunctionCode.FunctionCode = None;
			//	if(sCalendar_management.hour<7)
				  if((calendar.min%30)<10)
				//{MeasureSection.M_number=7-sCalendar_management.hour;}
				  {MeasureSection.M_number=1;}
				  else
				//{MeasureSection.M_number=31-sCalendar_management.hour;}
				  {MeasureSection.M_number=2;}
			  }*/
				//printf("MeasureSection.TimeDiv0=%d\r\n",MeasureSection.TimeDiv);
				//printf("MeasureSection.M_number0=%d\r\n",MeasureSection.M_number);
				if(sData_single_management.data_number==MeasureSection.M_number)
					{
					switch(MeasureSection.TimeDiv)
						{
						case TimeZone1:
							MeasureSection.TimeDiv = TimeZone2;
//							MeasureSection.M_number += 12;
							MeasureSection.M_number += 2;
							break;	
				    case TimeZone2:
							MeasureSection.TimeDiv= TimeZone1;
							MeasureSection.M_number += 2;
					    break;	
			      }
		   }
					//printf("switchMeasureSection.TimeDiv0=%d\r\n",MeasureSection.TimeDiv);
				  //printf("switchMeasureSection.M_number0=%d\r\n",MeasureSection.M_number);
//							MeasureSection.M_number += 12;
						//	printf("MeasureSection.TimeDiv0=%d\n",MeasureSection.TimeDiv);
					}
				//printf("MeasureSection.M_number0=%d\n",MeasureSection.M_number);
			}
	//	if(sData_single_management.data_number >= 36)
    	else if(sData_single_management.data_number >= MeasureSection.all_times)
		{
			
			//sData_single_management.data_number=0;//添加后会变成连续测量
			MeasureSection.TimeDiv=0;
			MeasureSection.M_number=0;
		//	memset(string_data_store_temp, NULL, sizeof(string_data_store_temp));
			shuzu_init();
			sFunctionCode.FunctionCode = None;
			return 1;
		}
		//if(sData_single_management.data_number < 36)
	  /* else if(sData_single_management.data_number < 4)
		{		
		//	printf("sData_single_management.data_number is %d",sData_single_management.data_number);
			switch(MeasureSection.TimeDiv)
			{
				//printf("xxx");
				case TimeZone1:
				{
					//counter_1second
					if(counter_1second>300)	
					{
						printf("yyy");
//						printf("300counter_1second=%d\r\n",counter_1second);
//						printf("300MeasureSection.M_number=%d\r\n",MeasureSection.M_number);
//					  printf("300sData_single_management.data_number=%d\r\n",sData_single_management.data_number);
//				  	printf("300sCalendar_management.min1=%d\r\n",calendar.min);
						MeasureSection.TimeUp=1;
						Standby_Time = 299;
					  RTC_WaitForLastTask();	//等待最近一次对RTC寄存器的写操作完成	
            RTC_SetAlarm(RTC_GetCounter()+Standby_Time);
			 printf("我在这3");
						counter_1second=0;
					}
//          if((counter_1second>1800) &&(sData_single_management.data_number<=MeasureSection.M_number))
					if((MeasureSection.TimeUp==1)&&(sData_single_management.data_number<=MeasureSection.M_number))
					{
						if(calculate_HR_PD_PS())
						{
							MeasureSection.TimeUp=0;
							sData_single_management.data_number++;
							printf("sData_single_management.data_number=%d\n\r",sData_single_management.data_number);
							attach_time_stamp(0);
							if(sData_single_management.data_number==MeasureSection.M_number)
						  {
						    MeasureSection.TimeDiv = TimeZone2;
//							MeasureSection.M_number += 12;
							  MeasureSection.M_number += 2;
								printf("MeasureSection.TimeDiv=%d\n\r",MeasureSection.TimeDiv);
						  }
						}
					}
         break;					
				}
				case TimeZone2:
				{
					printf("zzz");
					if(counter_1second>600)
					{printf("counter_1second=%d\n\r",counter_1second);
						printf("600MeasureSection.M_number=%d\n\r",MeasureSection.M_number);
					  printf("600sData_single_management.data_number=%d\n\r",sData_single_management.data_number);
				  	printf("600sCalendar_management.min2=%d\n\r",calendar.min);
						MeasureSection.TimeUp=1;
						Standby_Time = 599;
					  RTC_WaitForLastTask();	//等待最近一次对RTC寄存器的写操作完成	
            RTC_SetAlarm(RTC_GetCounter()+Standby_Time);
			 printf("我在这4");
						counter_1second=0;
					}
//					if((counter_1second>3600) &&(sData_single_management.data_number<=MeasureSection.M_number))
						if((MeasureSection.TimeUp==1) &&(sData_single_management.data_number<=MeasureSection.M_number))
					{
						if(calculate_HR_PD_PS())
						{
							MeasureSection.TimeUp=0;
							sData_single_management.data_number++;
							attach_time_stamp(0);
							if(sData_single_management.data_number==MeasureSection.M_number)
						  {
						    MeasureSection.TimeDiv= TimeZone1;
							  MeasureSection.M_number += 2;
								printf("MeasureSection.TimeDiv=%d\n",MeasureSection.TimeDiv);
						  }
						}
					}
         break;	
				}
			}
		} */
	     
			break;
		}
	    }
	
	return 0;
}

/*************************************************************
	* @brief 		execution instruction 
	* @input  	none
	* @return	none
*************************************************************/
void execution_instruction(void)
{	
	u16 temp;
	u16 temp3;
	if(sFunctionCode.FunctionCode == ConfigParameter)//功能选择
	{
	
		switch(sFunctionCode.MeasureMode)
		{
			case MeasureMode0:
			{
				temp = 1;
				STMFLASH_Write(Flash_Addr.Mode_Addr,&temp,1);
				break;
			}
			case MeasureMode1:
			{
				temp = 2;
				STMFLASH_Write(Flash_Addr.Mode_Addr,&temp,1);
				break;
			}
			case MeasureMode2:
			{
				temp = 3;
				STMFLASH_Write(Flash_Addr.Mode_Addr,&temp,1);
				break;
			}
			case MeasureMode3:
			{
				temp = 4;
				STMFLASH_Write(Flash_Addr.Mode_Addr,&temp,1);
				break;
			}
			case MeasureMode4:
			{
				temp = 5;
				STMFLASH_Write(Flash_Addr.Mode_Addr,&temp,1);
				break;
			}
   	    }
		sFunctionCode.FunctionCode = None;//因为是在while里面为了不必要的循环
	}
	
	/*clear data measure number in case data array overrun*/
	if(sFunctionCode.FunctionCode != StartMeasure)
	{
		//sData_single_management.data_number = 0;
		sData_single_management.adc_number = 0;	
		Motor_close;
		Valve_close;
		if(0!=sData_single_management.ReStart_flag)
		{
			if(counter_1second==120)
			{
				sFunctionCode.FunctionCode = StartMeasure;
				sData_single_management.ReStart_flag = 0;
				counter_1second=0;
			}
		}
	}
	
	if(sFunctionCode.FunctionCode == StartMeasure)
	{ 
		STMFLASH_Read(Flash_Addr.Mode_Addr,&temp3,1);
		switch(temp3)
		{
			case 1:{
				sFunctionCode.MeasureMode = MeasureMode0;			
				break;
			}
			case 2:{
				sFunctionCode.MeasureMode = MeasureMode1;
				break;
				
			}
			case 3:{
				sFunctionCode.MeasureMode = MeasureMode2; 
				break;
			}
			case 4:{
				sFunctionCode.MeasureMode = MeasureMode3; 
				break;
			}
			case 5:{
				sFunctionCode.MeasureMode = MeasureMode4; 
				break;
			}
		}
		if(data_measure()) {
			/* measure finish */
			printf("measure finish\n");
			sFunctionCode.FunctionCode = None;
			//Sys_Enter_Standby();
		}
		return;
	}
	
	if(sFunctionCode.FunctionCode == TimeCalibrate)
	{
    RTC_Set(sCalendar_management.w_year,sCalendar_management.w_month,sCalendar_management.w_date,sCalendar_management.hour,sCalendar_management.min,sCalendar_management.sec);  //设置时间
	  sFunctionCode.FunctionCode = None;
	}
	if(sFunctionCode.FunctionCode == Synchronous)
	{
		int i,j;
	//	hal.pflash->read(address_configure_mode, 1, &temp);
     STMFLASH_Read(Flash_Addr.IIC_times_Addr,&MeasureSection.All_times,1);
		
		printf("MeasureSection.All_times=%d",MeasureSection.All_times);	
		memset(string_temp_time_date_syn_measure, NULL, sizeof(string_temp_time_date_syn_measure));
				for(i=0;i<MeasureSection.All_times;i++)
			{	
				 STMFLASH_Read(Flash_Addr.IIC_syn_Addr,string_temp_time_date_syn_measure,20);
       Flash_Addr.IIC_syn_Addr=Flash_Addr.IIC_syn_Addr+20;
				u2_printf("string_temp_time_date_syn_measure\r\n");
			 for(j=0;j<20;j++) 
				{printf("time=%d\r\n",string_temp_time_date_syn_measure[j]);}
				memset(string_temp_time_date_syn_measure, NULL, sizeof(string_temp_time_date_syn_measure));
				STMFLASH_Read(Flash_Addr.IIC_syn_Addr,string_temp_time_date_syn_measure,20);
				Flash_Addr.IIC_syn_Addr=Flash_Addr.IIC_syn_Addr+17;
				u2_printf("string_temp_time_date_syn_measure\r\n");
				//blue_transfer_msg_packet(string_temp_time_date_syn_measure, strlen((const char*)string_temp_time_date_syn_measure));
				for(j=0;j<17;j++) 
				{printf("date=%d\r\n",string_temp_time_date_syn_measure[j]);	}	 	   
		}
		sFunctionCode.FunctionCode = None;
		return;
	}
	}
int digit(int number)
{ 
	int i;
    if(number==0)
        return 1;
    for(i=0;number>0;i++)
    {
			number/=10;
    }
    return i;
}