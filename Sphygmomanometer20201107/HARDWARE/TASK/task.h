#ifndef __TASKS_H__
#define __TASKS_H__


#ifdef __cplusplus
extern "C"{
#endif
	
#include "sys.h"
#include "gimbal.h"
#include "adc.h"
#include "delay.h"
	
//void server_tick(void);	
//void analysis_instruction(void); 
//void execution_instruction(void);
//void run_20hz_task(void);
	
extern u16 counter_1second;
u8 calculate_HR_PD_PS(void);
void calculate_avarage_basevalue(void);
void 	data_FIFO(void);
u16 Getmax_maibo(void);
void Getpeak_maibo(void);
void shuzu_init(void);
void insert_value_calculate150_100(void);
void calculate_percent(void);
void execution_instruction(void);
u8 data_measure(void);
int digit(int number);
#ifdef __cplusplus
} // extern "C"
#endif

#endif

