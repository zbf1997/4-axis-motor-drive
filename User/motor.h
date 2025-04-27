#ifndef __MOTOR_H
#define	__MOTOR_H
#include "stm32f10x.h"









#define  OUT_Y0_0		GPIO_ResetBits(GPIOC, GPIO_Pin_5)	
#define  OUT_Y0_1		GPIO_SetBits(GPIOC, GPIO_Pin_5)
#define  OUT_Y1_0		GPIO_ResetBits(GPIOB, GPIO_Pin_0)	
#define  OUT_Y1_1		GPIO_SetBits(GPIOB, GPIO_Pin_0)
#define  OUT_Y2_0		GPIO_ResetBits(GPIOB, GPIO_Pin_1)	
#define  OUT_Y2_1		GPIO_SetBits(GPIOB, GPIO_Pin_1)
#define  OUT_Y3_0		GPIO_ResetBits(GPIOE, GPIO_Pin_7)	
#define  OUT_Y3_1		GPIO_SetBits(GPIOE, GPIO_Pin_7)
#define  OUT_Y4_0		GPIO_ResetBits(GPIOE, GPIO_Pin_8)	
#define  OUT_Y4_1		GPIO_SetBits(GPIOE, GPIO_Pin_8)
#define  OUT_Y5_0		GPIO_ResetBits(GPIOE, GPIO_Pin_9)	
#define  OUT_Y5_1		GPIO_SetBits(GPIOE, GPIO_Pin_9)
#define  OUT_Y6_0		GPIO_ResetBits(GPIOE, GPIO_Pin_10)	
#define  OUT_Y6_1		GPIO_SetBits(GPIOE, GPIO_Pin_10)
#define  OUT_Y7_0		GPIO_ResetBits(GPIOE, GPIO_Pin_11)	
#define  OUT_Y7_1		GPIO_SetBits(GPIOE, GPIO_Pin_11)

#define  M4_LED_0		GPIO_ResetBits(GPIOB, GPIO_Pin_12)		
#define  M4_LED_1		GPIO_SetBits(GPIOB, GPIO_Pin_12)
#define  F4_LED_0		GPIO_ResetBits(GPIOB, GPIO_Pin_13)		
#define  F4_LED_1		GPIO_SetBits(GPIOB, GPIO_Pin_13)
#define  M3_LED_0		GPIO_ResetBits(GPIOB, GPIO_Pin_14)		
#define  M3_LED_1		GPIO_SetBits(GPIOB, GPIO_Pin_14)
#define  F3_LED_0		GPIO_ResetBits(GPIOB, GPIO_Pin_15)		
#define  F3_LED_1		GPIO_SetBits(GPIOB, GPIO_Pin_15)

#define  M2_LED_0		GPIO_ResetBits(GPIOD, GPIO_Pin_12)		
#define  M2_LED_1		GPIO_SetBits(GPIOD, GPIO_Pin_12)
#define  F2_LED_0		GPIO_ResetBits(GPIOD, GPIO_Pin_13)		
#define  F2_LED_1		GPIO_SetBits(GPIOD, GPIO_Pin_13)
#define  M1_LED_0		GPIO_ResetBits(GPIOD, GPIO_Pin_14)		
#define  M1_LED_1		GPIO_SetBits(GPIOD, GPIO_Pin_14)
#define  F1_LED_0		GPIO_ResetBits(GPIOD, GPIO_Pin_15)		
#define  F1_LED_1		GPIO_SetBits(GPIOD, GPIO_Pin_15)

#define  PULSE1_PORT    GPIOC		
#define  PULSE2_PORT    GPIOA		
#define  PULSE3_PORT    GPIOE		
#define  PULSE4_PORT    GPIOB		
#define  AXIS_PULSE_1   GPIO_Pin_4
#define  AXIS_PULSE_2   GPIO_Pin_3
#define  AXIS_PULSE_3   GPIO_Pin_0
#define  AXIS_PULSE_4		GPIO_Pin_7

#define  READ_PULSE1		GPIO_ReadOutputDataBit(PULSE1_PORT, AXIS_PULSE_1)	
#define  SET_PULSE1_0		GPIO_ResetBits(PULSE1_PORT, AXIS_PULSE_1)	
#define  SET_PULSE1_1		GPIO_SetBits(PULSE1_PORT, AXIS_PULSE_1)
		
#define  READ_PULSE2		GPIO_ReadOutputDataBit(PULSE2_PORT, AXIS_PULSE_2)	
#define  SET_PULSE2_0		GPIO_ResetBits(PULSE2_PORT, AXIS_PULSE_2)	
#define  SET_PULSE2_1		GPIO_SetBits(PULSE2_PORT, AXIS_PULSE_2)

#define  READ_PULSE3		GPIO_ReadOutputDataBit(PULSE3_PORT, AXIS_PULSE_3)	
#define  SET_PULSE3_0		GPIO_ResetBits(PULSE3_PORT, AXIS_PULSE_3)
#define  SET_PULSE3_1		GPIO_SetBits(PULSE3_PORT, AXIS_PULSE_3)

#define  READ_PULSE4		GPIO_ReadOutputDataBit(PULSE4_PORT, AXIS_PULSE_4)	
#define  SET_PULSE4_0		GPIO_ResetBits(PULSE4_PORT, AXIS_PULSE_4)
#define  SET_PULSE4_1		GPIO_SetBits(PULSE4_PORT, AXIS_PULSE_4)
		
#define  READ_ENCCLK0		GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_12)		
#define  READ_ENCDAT0		GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_13)		

#define  READ_ENCCLK1		GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_14)		
#define  READ_ENCDAT1		GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_15)		

#define  READ_ENCCLK2		GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_8)		
#define  READ_ENCDAT2		GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_9)		

#define  READ_ENCCLK3		GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_10)		
#define  READ_ENCDAT3		GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_11)		

#define  DIR1_PORT    	GPIOA	
#define  DIR2_PORT    	GPIOA	
#define  DIR3_PORT    	GPIOB	
#define  DIR4_PORT    	GPIOB	
#define  AXIS_DIR_1    	GPIO_Pin_7	
#define  AXIS_DIR_2    	GPIO_Pin_1	
#define  AXIS_DIR_3    	GPIO_Pin_9	
#define  AXIS_DIR_4    	GPIO_Pin_5	











#define  EN1_PORT    	GPIOA	
#define  EN2_PORT    	GPIOA	
#define  EN3_PORT    	GPIOB	
#define  EN4_PORT    	GPIOB	
#define  AXIS_EN_1   	GPIO_Pin_6	
#define  AXIS_EN_2   	GPIO_Pin_2	
#define  AXIS_EN_3   	GPIO_Pin_8	
#define  AXIS_EN_4   	GPIO_Pin_6	











#define  LMTPOS1_PORT    	GPIOD		
#define  LMTPOS2_PORT    	GPIOD	
#define  LMTPOS3_PORT    	GPIOD	
#define  LMTPOS4_PORT    	GPIOC	
#define  AXIS_LMTPOS_1    GPIO_Pin_2	
#define  AXIS_LMTPOS_2    GPIO_Pin_1
#define  AXIS_LMTPOS_3    GPIO_Pin_0
#define  AXIS_LMTPOS_4    GPIO_Pin_12


#define  LMTNEG1_PORT    	GPIOA		
#define  LMTNEG2_PORT    	GPIOD		
#define  LMTNEG3_PORT    	GPIOD		
#define  LMTNEG4_PORT    	GPIOD		
#define  AXIS_LMTNEG_1    GPIO_Pin_15
#define  AXIS_LMTNEG_2    GPIO_Pin_5
#define  AXIS_LMTNEG_3    GPIO_Pin_4
#define  AXIS_LMTNEG_4    GPIO_Pin_3


#define  HOME1_PORT    	GPIOB
#define  HOME2_PORT    	GPIOB
#define  HOME3_PORT    	GPIOD
#define  HOME4_PORT    	GPIOD
#define  AXIS_HOME_1    GPIO_Pin_4
#define  AXIS_HOME_2    GPIO_Pin_3
#define  AXIS_HOME_3    GPIO_Pin_7
#define  AXIS_HOME_4    GPIO_Pin_6

#define CW  0
#define CCW 1


#define T1_FREQ 1000000








#define STOP  0
#define ACCEL 1
#define DECEL 2
#define RUN   3


#define MIC_TYPE	0			

#if MIC_TYPE==1	
	#define MICSTEP		3200	
	#define INTSPEED 	100		
	#define MAX_SPEED 10		
	#define MIN_SPEED 1000	

	#define MAX_STEP1	32000	
	#define MAX_STEP2	19200	
	#define MAX_STEP3	6400	
	#define MAX_STEP4	32000	

	#define ZERO_OFF1	300		
	#define ZERO_OFF2	300		
	#define ZERO_OFF3	300		
	#define ZERO_OFF4	300		

#elif MIC_TYPE==0	
	#define MICSTEP		1600	
	#define INTSPEED 	100		
	#define MAX_SPEED 10		
	#define MIN_SPEED 1000	

	#define MAX_STEP1	16000	
	#define MAX_STEP2	9600	
	#define MAX_STEP3	3200	
	#define MAX_STEP4	16000	

	#define ZERO_OFF1	300
	#define ZERO_OFF2	300
	#define ZERO_OFF3	300
	#define ZERO_OFF4	300
#endif
#pragma pack(1)	

typedef struct {
  u8 run_state ;		
  u8 dir ;					
  s32 step_delay;		
  u32 decel_start;	
  u16 decel_val;		
  u16 int_val;			
  u16 max_speed;		
  u16 min_speed;		
  u32 toal_step;		
} speedRampData;

enum
{	 
	IDEL,
	SEEK,
	FORSTOP,
	REVSTOP,
	ORGSTOP
};
#endif 



