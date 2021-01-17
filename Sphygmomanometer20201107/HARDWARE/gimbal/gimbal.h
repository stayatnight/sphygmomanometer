#ifndef __GIMBAL_H__
#define __GIMBAL_H__

//ADC GPIO config
#define 	YALI_PIN                GPIO_Pin_1
#define 	YALI_GROUP              GPIOA

#define 	MABO_PIN                GPIO_Pin_0
#define 	MABO_GROUP              GPIOA

//USART GPIO Config	
#define   USART1_TX               GPIO_Pin_9
#define   USART1_TX_GROUP         GPIOA

#define   USART1_RX               GPIO_Pin_10
#define   USART1_RX_GROUP         GPIOA

#define   VALVE                   GPIO_Pin_6
#define   VALVE_GROUP             GPIOA

#define   MOTOR                   GPIO_Pin_7
#define   MOTOR_GROUP             GPIOA

#define Motor_open                GPIO_SetBits(GPIOA,GPIO_Pin_6); 
#define Motor_close               GPIO_ResetBits(GPIOA,GPIO_Pin_6); 

#define Valve_close               GPIO_ResetBits(GPIOA,GPIO_Pin_7); 
#define Valve_open                GPIO_SetBits(GPIOA,GPIO_Pin_7); 
#define MAX_ADC_PRINT             3000





//#define sei()  __enable_irq()  //enable all interrupt source
//#define cli()  __disable_irq() //disable all interrupt source

////LED GPIO Config
//#define LED_RED_PIN 	              17    
//#define LED_GREEN_PIN 	            18
//#define LED_BLUE_PIN 	              19
//#define LEDS_ALL                    ((1<<LED_RED_PIN ) | (1<<LED_GREEN_PIN) | (1<<LED_BLUE_PIN ))

////CONTROL GPIO Config
//#define MOTOR_PIN                   22
//#define VALVE_PIN                   23  
//#define CONTROL_ALL                 ((1<<MOTOR_PIN ) | (1<<VALVE_PIN))

////KEY GPIO Config	
//#define KEY_PIN                     13
//#define KEY_0_MASK                  (1<<KEY_PIN)

////KEY GPIO Config	
//#define TA_PIN                      6
//#define TB_PIN                      7
//#define TA_MASK                     (1<<TA_PIN )
//#define TB_MASK                     (1<<TB_PIN)
//#define TAB_MASK                    ((1<<TA_PIN ) | (1<<TB_PIN))

////USART GPIO Config	
//#define RX_PIN_NUMBER               8
//#define TX_PIN_NUMBER               6
//#define CTS_PIN_NUMBER              7
//#define RTS_PIN_NUMBER              5

////PWM GPIO Config	
////ROLL MOTOR
//#define ROLL_MOTOR_A_GPIO_PIN       17      
//#define ROLL_MOTOR_B_GPIO_PIN       26       

////PITCH MOTOR
//#define PITCH_MOTOR_A_GPIO_PIN      27      
//#define PITCH_MOTOR_B_GPIO_PIN      28     

////YAW MOTOR 
//#define YAW_MOTOR_A_GPIO_PIN        27      
//#define YAW_MOTOR_B_GPIO_PIN        28   

//// Low frequency clock source to be used by the SoftDevice
//#define NRF_CLOCK_LFCLKSRC      {.source        = NRF_CLOCK_LF_SRC_RC, \
//                                 .rc_ctiv       = 16,                  \
//                                 .rc_temp_ctiv  = 2}

//ADC GPIO config
//#define SAADC_CH_PSELP_PSELP_AnalogInput0 (1UL) /* P0.02/AIN0 *///
//#define SAADC_CH_PSELP_PSELP_AnalogInput1 (2UL) /* P0.03/AIN1 */
//#define SAADC_CH_PSELP_PSELP_AnalogInput2 (3UL) /* P0.04/AIN2 *///
//#define SAADC_CH_PSELP_PSELP_AnalogInput3 (4UL) /* P0.05/AIN3 */
//#define SAADC_CH_PSELP_PSELP_AnalogInput4 (5UL) /* P0.28/AIN4 *///
//#define SAADC_CH_PSELP_PSELP_AnalogInput5 (6UL) /* P0.29/AIN5 *///
//#define SAADC_CH_PSELP_PSELP_AnalogInput6 (7UL) /* P0.30/AIN6 *///
//#define SAADC_CH_PSELP_PSELP_AnalogInput7 (8UL) /* P0.31/AIN7 *///

#endif
