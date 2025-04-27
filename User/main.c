/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "misc.h"
#include "stm32f10x_conf.h"
#include "stdio.h"
#include "stdlib.h"
#include "stm32f10x_dac.h"
#include "usart3.h"	
#include "motor.h"	
#include "stmflash.h"	
/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define SYSCLK_FREQ_HSE
#define TIMx_PRE_EMPTION_PRIORITY 1
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/




	s32 position[4]={0,0,0,0};
	s32 HomePos[4]={0,0,0,0};
	s32 Maxstep[4]={MAX_STEP1, MAX_STEP2, MAX_STEP3, MAX_STEP4};			
	u16 ZeroOffset[4]={ZERO_OFF1, ZERO_OFF2, ZERO_OFF3, ZERO_OFF4}; 	
	
	s32 TestStep[4]={32000, 19200, 4800, 19200};			
	
	
	
	

	u16 AxisPulsePin[4]={AXIS_PULSE_1, AXIS_PULSE_2, AXIS_PULSE_3, AXIS_PULSE_4};
	u16 AxisDirPin[4]={AXIS_DIR_1, AXIS_DIR_2, AXIS_DIR_3, AXIS_DIR_4}; 
	u16 AxisEnPin[4]={AXIS_EN_1, AXIS_EN_2, AXIS_EN_3, AXIS_EN_4}; 

	u16 LmtPosPin[4]={AXIS_LMTPOS_1, AXIS_LMTPOS_2, AXIS_LMTPOS_3, AXIS_LMTPOS_4};
	u16 LmtNegPin[4]={AXIS_LMTNEG_1, AXIS_LMTNEG_2, AXIS_LMTNEG_3, AXIS_LMTNEG_4};
	u16 HomePin[4]={AXIS_HOME_1, AXIS_HOME_2, AXIS_HOME_3, AXIS_HOME_4};

	GPIO_TypeDef *AxisLmtPosPort[4]={LMTPOS1_PORT, LMTPOS2_PORT, LMTPOS3_PORT, LMTPOS4_PORT};	
	GPIO_TypeDef *AxisLmtNegPort[4]={LMTNEG1_PORT, LMTNEG2_PORT, LMTNEG3_PORT, LMTNEG4_PORT}; 
	GPIO_TypeDef *AxisPulsePort[4]={PULSE1_PORT, PULSE2_PORT, PULSE3_PORT, PULSE4_PORT};
	GPIO_TypeDef *AxisDirPort[4]={DIR1_PORT, DIR2_PORT, DIR3_PORT, DIR4_PORT}; 
	GPIO_TypeDef *AxisEnPort[4]={EN1_PORT, EN2_PORT, EN3_PORT, EN4_PORT}; 

	GPIO_TypeDef *HomePort[4]={HOME1_PORT, HOME2_PORT, HOME3_PORT, HOME4_PORT};

	uint32_t AxisEXTILine[4]={EXTI_Line4, EXTI_Line3, EXTI_Line7, EXTI_Line6};
	uint8_t AxisEXTIPinSource[4]={GPIO_PinSource4, GPIO_PinSource3, GPIO_PinSource7, GPIO_PinSource6};

	bool bLmtPos[4]={FALSE, FALSE, FALSE, FALSE};
	bool bLmtNeg[4]={FALSE, FALSE, FALSE, FALSE};
	bool bStopCmd[4]={FALSE, FALSE, FALSE, FALSE};
	bool bEmgStopping[4]={FALSE, FALSE, FALSE, FALSE}; 
	bool bEnableSoftLmt[4]={FALSE, FALSE, FALSE, FALSE};
	bool B_EncDir[4]={FALSE, FALSE, FALSE, FALSE};

	bool bHomeOK[4]={FALSE, FALSE, FALSE, FALSE};
	bool bZeroCapture[4]={FALSE, FALSE, FALSE, FALSE};
	u8 ZeroStep[4]={0, 0, 0, 0};

	u8 TestStat[4]={0, 0, 0, 0};	
  u8 EncoStat[4]={0, 0, 0, 0};		
	u8 OldClk[4]={0, 0, 0, 0};
	u8 OldDat[4]={0, 0, 0, 0};
	
	speedRampData srd[4];
 
	s32	encoder_cnt[4]={0, 0, 0, 0};

	u16 PwmCnt1=0;
 
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void NVIC_Configuration(void);
void Timer_Init(void);
void EXTILine_Config(void);
void GPIO_Configuration(void);
int fputc(int ch, FILE *f);
 
 
u8 MotionStatus[4]={0, 0, 0, 0};

bool bDataOK;
USART_InitTypeDef USART_InitStructure;
EXTI_InitTypeDef EXTI_InitStructure;

uint8_t RxBuffer[12];
uint8_t RxCounter;

void AxisMoveRel(u8 axis, s32 step, u16 accel, u16 decel, u8 speed);
void AxisMoveAbs(u8 axis, s32 step, u16 accel, u16 decel, u8 speed);
void LimitDetect(u8 axis);
void USART_Configuration(void);
void AxisHome(u8 axis);
void TestMove(u8 axis);
void TIM2_Int_Init(u16 arr,u16 psc);
void TIM3_Int_Init(u16 arr,u16 psc);
void TIM4_Int_Init(u16 arr,u16 psc);
void TIM5_Int_Init(u16 arr,u16 psc);


#define EnableHomeCapture1	EXTI->IMR |= GPIO_Pin_4
#define DisableHomeCapture1	EXTI->IMR &=~GPIO_Pin_4
#define EnableHomeCapture2	EXTI->IMR |= GPIO_Pin_3
#define DisableHomeCapture2	EXTI->IMR &=~GPIO_Pin_3  
#define EnableHomeCapture3	EXTI->IMR |= GPIO_Pin_7
#define DisableHomeCapture3	EXTI->IMR &=~GPIO_Pin_7  
#define EnableHomeCapture4	EXTI->IMR |= GPIO_Pin_6
#define DisableHomeCapture4	EXTI->IMR &=~GPIO_Pin_6  

#define EnableHomeCapture12		EXTI->IMR |= GPIO_Pin_12
#define DisableHomeCapture12	EXTI->IMR &=~GPIO_Pin_12  
#define EnableHomeCapture13		EXTI->IMR |= GPIO_Pin_13
#define DisableHomeCapture13	EXTI->IMR &=~GPIO_Pin_13  







void DAC1_Configure(void)
{
	DAC_InitTypeDef	DAC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE); 
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
  DAC_InitStructure.DAC_Trigger = DAC_Trigger_None;	
	DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;	
	DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bits8_0;	
	DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
	DAC_Init(DAC_Channel_1, &DAC_InitStructure);
 	DAC_Cmd(DAC_Channel_1, ENABLE);
 	DAC_SetChannel1Data(DAC_Align_12b_R,0);
}

void DAC2_Configure(void)
{
	DAC_InitTypeDef	DAC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE); 
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
  DAC_InitStructure.DAC_Trigger = DAC_Trigger_None;
	DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
	DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bits8_0;
	DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Enable;	
	DAC_Init(DAC_Channel_2, &DAC_InitStructure);
 	DAC_Cmd(DAC_Channel_2, ENABLE);
 	DAC_SetChannel2Data(DAC_Align_12b_R,0);
}

void TIM8_PWM_Init(u16 arr,u16 psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
 


   	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8|RCC_APB2Periph_GPIOC, ENABLE);
   	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	




 
	










 
	
	TIM_TimeBaseStructure.TIM_Period = arr; 
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure); 
 
 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
	TIM_OCInitStructure.TIM_Pulse = 0; 
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 
	TIM_OC1Init(TIM8, &TIM_OCInitStructure);  
	
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
	TIM_OCInitStructure.TIM_Pulse = 0; 
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 
	TIM_OC2Init(TIM8, &TIM_OCInitStructure);  
	
	












 
  TIM_CtrlPWMOutputs(TIM8,ENABLE);	
 
	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);  
	TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);  


	TIM_SetCompare1(TIM8,0);
	TIM_SetCompare2(TIM8,0);
	
	TIM_ARRPreloadConfig(TIM8, ENABLE); 
	TIM_Cmd(TIM8, ENABLE);  
}

void	IREF_out2(u16 Iset)
{ 
	
	
	
	
	u16	dac_out;
	u32	calc;	
	calc=(u32)(Iset*771/1000);	
	if(calc>3475)
		calc=3475;	
	dac_out=calc;		
	DAC_SetChannel1Data(DAC_Align_12b_R,dac_out);	
}

void	IREF_out1(u16 Iset)
{ 
	
	
	
	
	u16	dac_out;
	u32	calc;	
	calc=(u32)(Iset*771/1000);	
	if(calc>3475)
		calc=3475;	
	dac_out=calc;
	DAC_SetChannel2Data(DAC_Align_12b_R,dac_out);	
}

void	IREF_out3(u16 Iset)
{ 
	
	
	
	
	u16	dac_out;
	u32	calc;	
	calc=(u32)(Iset*6166/1000);	
	if(calc>27747)
		calc=27747;	
	dac_out=calc;			

	
	TIM_SetCompare2(TIM8,dac_out);
}

void	IREF_out4(u16 Iset)
{ 
	
	
	
	
	u16	dac_out;
	u32	calc;	
	calc=(u32)(Iset*6166/1000);	
	if(calc>27747)
		calc=27747;	
	dac_out=calc;			
 	
	
	TIM_SetCompare1(TIM8,dac_out);
}

void	Parm_init(void)	
{
	u8 axis;
	for(axis=1;axis<5;axis++)
	{	
		GPIO_SetBits(AxisPulsePort[axis-1], AxisPulsePin[axis-1]);	
		GPIO_SetBits(AxisDirPort[axis-1], AxisDirPin[axis-1]);			
		GPIO_SetBits(AxisEnPort[axis-1], AxisEnPin[axis-1]);				
	}
	M1_LED_1;
	M2_LED_1;
	M3_LED_1;
	M4_LED_1;
	F1_LED_1;
	F2_LED_1;
	F3_LED_1;
	F4_LED_1;
	
	for(axis=0;axis<4;axis++)
	{
		Sys[axis].MaxStep=Maxstep[axis];
		Sys[axis].RunStep=MICSTEP;
		
		Sys[axis].Acce=200;
		Sys[axis].Dece=200;
		Sys[axis].Speed=100;
		Sys[axis].Cur=1000;		
		Sys[axis].Mode=0;
		
		Sys[axis].RealStep=0;
		Sys[axis].Stat=0;
	}
}

void	MotoCmdWork(u8 axis)	
{
	
	
	
	
	
	
	if(Sys[axis].Cur>4000)
		Sys[axis].Cur=4000;	
	
	switch(axis)
	{
		case 0:
			IREF_out1(Sys[axis].Cur);	
			break;
		case 1:
			IREF_out2(Sys[axis].Cur);	
			break;
		case 2:
			IREF_out3(Sys[axis].Cur);	
			break;
		case 3:
			IREF_out4(Sys[axis].Cur);	
			break;
	}
	
	if(srd[axis].run_state==STOP && Sys[axis].Mode!=0) 
	{
		if(Sys[axis].Mode==1)	
		{
			Sys[axis].NoStop=0;	
			AxisMoveRel(axis+1, Sys[axis].RunStep, Sys[axis].Acce, Sys[axis].Dece, Sys[axis].Speed);	
		}
		else if(Sys[axis].Mode==2)	
		{
			Sys[axis].NoStop=0;	
			AxisMoveAbs(axis+1, Sys[axis].RunStep, Sys[axis].Acce, Sys[axis].Dece, Sys[axis].Speed);	
		}
		else if(Sys[axis].Mode==3)	
		{
			Sys[axis].NoStop=0;	
			ZeroStep[axis]=SEEK;
		}
		else if(Sys[axis].Mode==7)	
		{
			Sys[axis].NoStop=1;	
			if(Sys[axis].RunStep<0)
			{
				Sys[axis].RunStep=-2000000000;	
				Sys[axis].MaxStep=2100000000;
				position[axis]=2000000000;		
			}
			else
			{
				Sys[axis].RunStep=2000000000;	
				Sys[axis].MaxStep=2100000000;
				position[axis]=0;		
			}
			AxisMoveRel(axis+1, Sys[axis].RunStep, Sys[axis].Acce, Sys[axis].Dece, Sys[axis].Speed);	
		}
		else if(Sys[axis].Mode==8)	
		{
			Sys[axis].NoStop=0;	
			TestStat[axis]=1;	
		}			
	}		
	else if(MotionStatus[axis] && Sys[axis].Mode==4)
	{ 
		Sys[axis].NoStop=0;	
		bStopCmd[axis]=TRUE;
		TestStat[axis]=0;	
	}
	
	Sys[axis].Mode=0;	
	SysModeClr(axis);
}

void check_key(void)	
{


	
	if(MotionStatus[2] && GPIO_ReadInputDataBit(AxisLmtNegPort[2], LmtNegPin[2])==0) 
	{
		bStopCmd[2]=TRUE;
		TestStat[2]=0;			
	}
	
	if(MotionStatus[3] && GPIO_ReadInputDataBit(AxisLmtNegPort[3], LmtNegPin[3])==0) 
	{
		bStopCmd[3]=TRUE;
		TestStat[3]=0;	
	}	
	
	if(!MotionStatus[0] && GPIO_ReadInputDataBit(AxisLmtNegPort[0], LmtNegPin[0])==0) 
	{
		AxisMoveRel(1, 320000, 200, 200, 100);
	}
	
	if(!MotionStatus[0] && GPIO_ReadInputDataBit(AxisLmtNegPort[1], LmtNegPin[1])==0) 
	{
		AxisMoveRel(1, -320000, 200, 200, 100);
	}
}

int main(void)
{
	u8 i;
	GPIO_Configuration();		
	Parm_init();	
	ReadSYSFromFlash();
	

	TIM2_Int_Init(1000,64);
	TIM3_Int_Init(1000,64);
	TIM4_Int_Init(1000,64);
	TIM5_Int_Init(1000,64);
	
	EXTILine_Config();		
  NVIC_Configuration();

	DAC1_Configure();
	DAC2_Configure();
	USART3_INIT();	
	TIM8_PWM_Init(32768, 0);	
	
	IREF_out1(1000);	
	IREF_out2(1000);	
	IREF_out3(1000);	
	IREF_out4(1000);	
		
	for(i=0; i<4; i++)
	{
		if(Set.Zero[i]<1600)
			ZeroOffset[i]=Set.Zero[i];	
	}	
	
  while (1)																
  {
		for(i=1; i<5; i++)
		{
			if(Sys[i-1].Mode!=7)	
			{			
				LimitDetect(i);		
				AxisHome(i);			
				TestMove(i);
			}
			Sys[i-1].RealStep=position[i-1];	
			Sys[i-1].Stat=MotionStatus[i-1];	
		}		
		
		if(READ_ENCCLK0)
			M1_LED_1;
		else
			M1_LED_0;
		if(READ_ENCCLK1)
			F1_LED_1;
		else
			F1_LED_0;
		
		if(Flag_NewCmd==SET)
		{	
			for(i=0; i<4; i++)
			{
				MotoCmdWork(i);	
			}
			Flag_NewCmd=RESET;
		}		
		
		if(Flag_Save==SET)	
		{
			WriteSYSToFlash();
			Flag_Save=RESET;
		}
		
		check_key();	
		if(Flag_RxOk==SET)
		{
			RxMdbus_Data3();	
			Flag_RxOk=RESET;
		}
		
  }
}

int fputc(int ch, FILE *f)
{
  USART_SendData(USART1, (uint8_t) ch); /*发送一个字符函数*/ 

  /* Loop until the end of transmission */
  while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)/*等待发送完成*/
  {
  
  }
  return ch;
}

void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;		
	
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE); 








	
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE); 

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);	


	
















	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOB, &GPIO_InitStructure);   	  
	
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_3 | GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);	

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE); 





	
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);


  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD,ENABLE); 
















	
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE,ENABLE); 












	
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	


























}


void TIM2_Int_Init(u16 arr,u16 psc)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); 
	
	TIM_TimeBaseStructure.TIM_Period = arr; 	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); 
 
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE ); 

	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);  
	
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update  );  
  TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE ); 

}

void TIM3_Int_Init(u16 arr,u16 psc)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); 
	
	TIM_TimeBaseStructure.TIM_Period = arr; 	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); 
 
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE ); 
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure); 

  TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );  
  TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE ); 


}

void TIM4_Int_Init(u16 arr,u16 psc)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); 
	
	TIM_TimeBaseStructure.TIM_Period = arr; 	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); 
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE ); 

	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2; 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure); 

  TIM_ClearITPendingBit(TIM4, TIM_IT_Update  );  
  TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE ); 

}

void TIM5_Int_Init(u16 arr,u16 psc)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE); 
	
	TIM_TimeBaseStructure.TIM_Period = arr; 	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure); 
 
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3; 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);  
	
	TIM_ClearITPendingBit(TIM5, TIM_IT_Update  );  
  TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE ); 
	
}

void Timer_Init(void)
{ 
	uint16_t PrescalerValue;
	
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

  TIM_TimeBaseStructure.TIM_Period =65535;
  TIM_TimeBaseStructure.TIM_Prescaler =0;    
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

	PrescalerValue=(uint16_t) ((SystemFrequency_SysClk / 2) / 1000000) - 1;
	TIM_PrescalerConfig(TIM2, PrescalerValue, TIM_PSCReloadMode_Immediate);
	TIM_PrescalerConfig(TIM3, PrescalerValue, TIM_PSCReloadMode_Immediate);
	TIM_PrescalerConfig(TIM4, PrescalerValue, TIM_PSCReloadMode_Immediate);
	TIM_PrescalerConfig(TIM5, PrescalerValue, TIM_PSCReloadMode_Immediate);
  /* Output Compare Active Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Active;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 10;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC1Init(TIM2, &TIM_OCInitStructure);
  TIM_OC1Init(TIM3, &TIM_OCInitStructure);
  TIM_OC1Init(TIM4, &TIM_OCInitStructure);
  TIM_OC1Init(TIM5, &TIM_OCInitStructure);
	
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);
	TIM_ClearFlag(TIM3, TIM_FLAG_Update);
	TIM_ClearFlag(TIM4, TIM_FLAG_Update);
	TIM_ClearFlag(TIM5, TIM_FLAG_Update);

  /* TIM IT enable */
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
  TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
  TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
}

void NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
		
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	   
	#ifdef  VECT_TAB_RAM  
		/* Set the Vector Table base location at 0x20000000 */ 
		NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0); 		
	#else  /* VECT_TAB_FLASH  */
		/* Set the Vector Table base location at 0x08000000 */ 
		NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);   
	#endif

	  /* Enable the TIM2 Interrupt */



     /* Enable the USART1 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;

  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);


  NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;			
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;					
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	

	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_Init(&NVIC_InitStructure);



}

void EXTILine_Config(void)
{		
	GPIO_InitTypeDef GPIO_InitStructure; 
	EXTI_InitTypeDef EXTI_InitStructure; 
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO,ENABLE); 
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable , ENABLE);

  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_3 | GPIO_Pin_4;       
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	 
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO,ENABLE); 
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	 
  GPIO_Init(GPIOD, &GPIO_InitStructure);
	




	
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,  GPIO_PinSource3); 
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,  GPIO_PinSource4); 
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOD,  GPIO_PinSource6); 
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOD,  GPIO_PinSource7); 
 









	


  EXTI_InitStructure.EXTI_Line =  EXTI_Line3 | EXTI_Line4 | EXTI_Line6 | EXTI_Line7; 
	
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; 
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; 
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure); 
	EXTI_ClearITPendingBit(EXTI_Line3);
	EXTI_ClearITPendingBit(EXTI_Line4);
	EXTI_ClearITPendingBit(EXTI_Line6);
	EXTI_ClearITPendingBit(EXTI_Line7);
	








}

void LimitDetect(u8 axis)
{
	if(srd[axis-1].dir==CW && (position[axis-1]>Sys[axis-1].MaxStep || GPIO_ReadInputDataBit(AxisLmtPosPort[axis-1], LmtPosPin[axis-1])==0))
	{
		bLmtPos[axis-1]=TRUE;		
	}
	else 
	{
		bLmtPos[axis-1]=FALSE;
	}	
	
	
	if(position[axis-1]>0)
	{
		bLmtNeg[axis-1]=FALSE;
	}		
	else if(srd[axis-1].dir==CCW)		
	{
		bLmtNeg[axis-1]=TRUE;
	}








		
}

void AxisHome(u8 axis)
{
	axis--;	
	switch(ZeroStep[axis])
	{
		case IDEL:
			break;
		
		case SEEK:	
			if(!MotionStatus[axis])
			{
				if(GPIO_ReadInputDataBit(HomePort[axis], HomePin[axis]))
				{
					bZeroCapture[axis]=FALSE;	
					switch(axis)
					{
						case 0:
							EnableHomeCapture1;
							break;
						case 1:
							EnableHomeCapture2;
							break;
						case 2:
							EnableHomeCapture3;
							break;
						case 3:
							EnableHomeCapture4;
							break;
					}
					
					position[axis]=Sys[axis].MaxStep+MICSTEP;	
					
					AxisMoveRel(axis+1, -(Sys[axis].MaxStep+MICSTEP), 200, 200, Sys[axis].Speed);
					ZeroStep[axis]=REVSTOP;							
				}
				else
				{
					switch(axis)
					{
						case 0:
							DisableHomeCapture1;
							break;
						case 1:
							DisableHomeCapture2;
							break;
						case 2:
							DisableHomeCapture3;
							break;
						case 3:
							DisableHomeCapture4;
							break;
					}
					position[axis]=MICSTEP+200;
					AxisMoveRel(axis+1, MICSTEP, 200, 200, Sys[axis].Speed);
					ZeroStep[axis-1]=FORSTOP;				
				}
			}
			break;
			
		case FORSTOP:
			if(!MotionStatus[axis])
			{
				bZeroCapture[axis]=FALSE;	
				switch(axis)
				{
					case 0:
						EnableHomeCapture1;
						break;
					case 1:
						EnableHomeCapture2;
						break;
					case 2:
						EnableHomeCapture3;
						break;
					case 3:
						EnableHomeCapture4;
						break;
				}
				AxisMoveRel(axis+1, -(MICSTEP+200), 300, 300, 20);	
				ZeroStep[axis]=REVSTOP;
			}			
			break;
			
		case REVSTOP:
			if(!MotionStatus[axis] && bZeroCapture[axis])
			{
					switch(axis)
					{
						case 0:
							DisableHomeCapture1;
							break;
						case 1:
							DisableHomeCapture2;
							break;
						case 2:
							DisableHomeCapture3;
							break;
						case 3:
							DisableHomeCapture4;
							break;
					}
					
				bStopCmd[axis]=FALSE;
				position[axis]=0;	
				AxisMoveRel(axis+1, ZeroOffset[axis], 100, 100, 20);	
				ZeroStep[axis]=ORGSTOP;
			}			
			break;			
			
		case ORGSTOP:
			if(!MotionStatus[axis])
			{
				ZeroStep[axis]=IDEL;
				position[axis]=0;
				encoder_cnt[axis]=0;
			}
			break;
	}
}

void TestMove(u8 axis)
{	
	axis--;	
	switch(TestStat[axis])
	{
		case 0:
			break;
		
		case 1:	
			if(!MotionStatus[axis]) 
			{
				ZeroStep[axis]=SEEK;	
				TestStat[axis]++;
			}
			break;
			
		case 2:	
			if(ZeroStep[axis]==IDEL) 
			{
				TestStat[axis]++;		
			}
			break;
			
		case 3:	
			if(!MotionStatus[axis]) 
			{
			
				AxisMoveRel(axis+1, Sys[axis].RunStep, Sys[axis].Acce, Sys[axis].Dece, Sys[axis].Speed);	
				TestStat[axis]++;		
			}
			break;
			
		case 4:	
			if(!MotionStatus[axis]) 
			{
			
				AxisMoveRel(axis+1, -Sys[axis].RunStep, Sys[axis].Acce, Sys[axis].Dece, Sys[axis].Speed);	
				TestStat[axis]=3;		
			}
			break;
	}
}

void AxisMoveRel(u8 axis, s32 step, u16 accel, u16 decel, u8 speed)
{	
					
	if(step <0)
	{









		if(position[axis-1]<=0)		
		{
			bLmtNeg[axis-1]=TRUE;
			return;
		}
		
		srd[axis-1].dir = CCW;
		GPIO_SetBits(AxisDirPort[axis-1], AxisDirPin[axis-1]);	
		GPIO_ResetBits(AxisEnPort[axis-1], AxisEnPin[axis-1]);	
		step =-step;	
	}
	else
	{
		if(position[axis-1]>Sys[axis-1].MaxStep || GPIO_ReadInputDataBit(AxisLmtPosPort[axis-1], LmtPosPin[axis-1])==0)
		{
			bLmtPos[axis-1]=TRUE;		
			return;
		}
		else
		{
			bLmtPos[axis-1]=FALSE;				
		}









		srd[axis-1].dir = CW;
		GPIO_ResetBits(AxisDirPort[axis-1], AxisDirPin[axis-1]);
		GPIO_ResetBits(AxisEnPort[axis-1], AxisEnPin[axis-1]);	
	}
	
	
	switch(axis)
	{
		case 1:
			OldClk[0]=READ_ENCCLK0;
			OldDat[0]=READ_ENCDAT0;
			break;
		case 2:
			OldClk[1]=READ_ENCCLK1;
			OldDat[1]=READ_ENCDAT1;
			break;
		case 3:
			OldClk[2]=READ_ENCCLK2;
			OldDat[2]=READ_ENCDAT2;
			break;
		case 4:
			OldClk[3]=READ_ENCCLK3;
			OldDat[3]=READ_ENCDAT3;
			break;
	}    
	
	
	bStopCmd[axis-1]=FALSE;
	bLmtNeg[axis-1]=FALSE;
	bLmtPos[axis-1]=FALSE;		
	srd[axis-1].toal_step=step;	
	
	if((accel+decel)>step)
	{
		accel=step/2;
		decel=accel;
	}

  if(step == 1)
  {
    srd[axis-1].run_state = DECEL;	
    srd[axis-1].step_delay = MIN_SPEED;	
		switch(axis)
		{
			case 1:
				 TIM2->CCR1=10;
				 TIM2->ARR=10;
				 break;
			case 2:
				 TIM3->CCR1=10;
				 TIM3->ARR=10;
				 break;
			case 3:
				 TIM4->CCR1=10;
				 TIM4->ARR=10;
				 break;
			case 4:
				 TIM5->CCR1=10;
				 TIM5->ARR=10;
				 break;
		}    
		MotionStatus[axis-1] = 1;	
		switch(axis)
		{
			case 1:
				 TIM_Cmd(TIM2, ENABLE);
			 break;
			case 2:
			 TIM_Cmd(TIM3, ENABLE);
			 break;
			case 3:
				 TIM_Cmd(TIM4, ENABLE);
			 break;
			case 4:
				 TIM_Cmd(TIM5, ENABLE);
			 break;
		}
  }
  
  else if(step != 0)
  {
    srd[axis-1].max_speed =10+(100-speed)*2;
																										
		if(srd[axis-1].max_speed<MAX_SPEED)
			srd[axis-1].max_speed=MAX_SPEED;
		
		if(accel>MIN_SPEED)
			srd[axis-1].int_val=(MIN_SPEED-srd[axis-1].max_speed)/accel;
		else if(srd[axis-1].max_speed>accel)
			srd[axis-1].int_val=1;			
		else if(accel<INTSPEED)
			srd[axis-1].int_val=(INTSPEED-srd[axis-1].max_speed)/accel;
		else
			srd[axis-1].int_val=(accel-srd[axis-1].max_speed)/accel; 
		if(srd[axis-1].int_val<=0)
			srd[axis-1].int_val=1;
		
		if(decel>MIN_SPEED)
			srd[axis-1].decel_val=(MIN_SPEED-srd[axis-1].max_speed)/decel;
		else if(srd[axis-1].max_speed>decel)
			srd[axis-1].decel_val=1;			
		else if(decel<INTSPEED)
			srd[axis-1].decel_val=(INTSPEED-srd[axis-1].max_speed)/decel; 
		else
			srd[axis-1].decel_val=(decel-srd[axis-1].max_speed)/decel; 
		if(srd[axis-1].decel_val<=0)
			srd[axis-1].decel_val=1;
		
		if(accel>MIN_SPEED)	
			srd[axis-1].step_delay = MIN_SPEED;		
		else if(accel<INTSPEED)
			srd[axis-1].step_delay = INTSPEED;	
		else
			srd[axis-1].step_delay = accel;	
		
    srd[axis-1].decel_start = step - decel;	
    srd[axis-1].run_state = ACCEL;	
    MotionStatus[axis-1] = 1;	
    
		switch(axis)
	  {
			case 1:
				TIM2->CCR1=10;
				TIM2->ARR=10;
				break;
			case 2:
				TIM3->CCR1=10;
				TIM3->ARR=10;
				break;
			case 3:
				TIM4->CCR1=10;
				TIM4->ARR=10;
				break;
			case 4:
				TIM5->CCR1=10;
				TIM5->ARR=10;
				break;
	  }
	
		switch(axis)
		{
			case 1:
				TIM_Cmd(TIM2, ENABLE);
				break;
			case 2:
				TIM_Cmd(TIM3, ENABLE);
				break;
			case 3:
				TIM_Cmd(TIM4, ENABLE);
				break;
			case 4:
				TIM_Cmd(TIM5, ENABLE);
				break;
		}
  }
}


void AxisMoveAbs(u8 axis, s32 step, u16 accel, u16 decel, u8 speed)
{  
	step=step-position[axis-1];		
	AxisMoveRel(axis,step,accel,decel,speed);	
}

void TIM2_IRQHandler(void)
{    
  static u32 step_count = 0;		
  static u8 i=0;
	
  if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
  {
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
  }
  TIM2->CCR1=srd[0].step_delay;
	TIM2->ARR=srd[0].step_delay;
	
  if(srd[0].run_state)
  {
	  if(READ_PULSE1==1)
		{
		  SET_PULSE1_0;
		}
	  else
	  {
			SET_PULSE1_1;
	  }
	}

	
	switch(EncoStat[0]) 
	{
		case 0:
			if(OldClk[0]!=READ_ENCCLK0)
			{			
				EncoStat[0]++;		
				OldClk[0]=READ_ENCCLK0;
			}
			break;
		case 1:
			if(OldDat[0]!=READ_ENCDAT0)
			{		
				if(srd[0].dir==CW)
					encoder_cnt[0]++;
				else
					encoder_cnt[0]--;
				EncoStat[0]=0;		
				OldDat[0]=READ_ENCDAT0;
			}
			break;
	}	
	
  i++;
  if(i==2)
	{
		i=0;
		switch(srd[0].run_state) 
		{
			case STOP:	
				step_count = 0;
				TIM_Cmd(TIM2, DISABLE);			
				MotionStatus[0] = 0;		
				bEmgStopping[0]=FALSE;
				break;

			case ACCEL:		
				step_count++;				
				if(srd[0].dir==CW)
				{	
					position[0]++;
				}
				else
				{ 
					position[0]--;
				}
				
				if(srd[0].step_delay > srd[0].int_val)
					srd[0].step_delay -= srd[0].int_val;
				else
					srd[0].step_delay = srd[0].max_speed;
				
				if(bStopCmd[0] || bLmtPos[0] || bLmtNeg[0])
				{
					step_count = srd[0].decel_start;	
					srd[0].run_state = DECEL;			
				}
				else if(step_count >= srd[0].decel_start)
				{	
					srd[0].run_state = DECEL;
				}
				else if(srd[0].step_delay <= srd[0].max_speed)
				{ 
					srd[0].run_state = RUN;	
				}
				break;

			case RUN:	
				step_count++;
				if(Sys[0].NoStop==1)
					break;	
				if(srd[0].dir==CW)
				{	
					position[0]++;
				}
				else
				{ 
					position[0]--;
				}
				
				if(bStopCmd[0] || bLmtPos[0] || bLmtNeg[0])
				{
					step_count = srd[0].decel_start;	
					srd[0].run_state = DECEL;			
				}
				else if(step_count >= srd[0].decel_start)
				{
					srd[0].run_state = DECEL;
				}
				break;

			case DECEL:
				step_count++;
					if(srd[0].dir==CW)
					{	
						position[0]++;
					}
					else
					{ 
						position[0]--;
					}
				
				srd[0].step_delay += srd[0].decel_val;
				if(srd[0].step_delay>MIN_SPEED)
					srd[0].step_delay=MIN_SPEED;
				
				if(step_count>=srd[0].toal_step)
				{
					srd[0].run_state = STOP;
				}
				break;
    }
  }
}

void TIM3_IRQHandler(void)
{ 
  static u32 step_count = 0;
  static u8 i=0;

  if(TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
  {
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
  }
  TIM3->CCR1=srd[1].step_delay;
	TIM3->ARR=srd[1].step_delay;
	
  if(srd[1].run_state)
  {
	  if(READ_PULSE2==1)
		{
		  SET_PULSE2_0;
		}
	  else
	  {
			SET_PULSE2_1;
	  }
	}
	
	switch(EncoStat[1]) 
	{
		case 0:
			if(OldClk[1]!=READ_ENCCLK1)
			{			
				EncoStat[1]++;		
				OldClk[1]=READ_ENCCLK1;
			}
			break;
		case 1:
			if(OldDat[1]!=READ_ENCDAT1)
			{		
				if(srd[1].dir==CW)
					encoder_cnt[1]++;
				else
					encoder_cnt[1]--;
				EncoStat[1]=0;		
				OldDat[1]=READ_ENCDAT1;
			}
			break;
	}	
	
  i++;
  if(i==2)
  {
		i=0;
		switch(srd[1].run_state) 
		{
			case STOP:
				step_count = 0;
				TIM_Cmd(TIM3, DISABLE);
				MotionStatus[1] = 0;
				bEmgStopping[1]=FALSE;
				break;

			case ACCEL:
				step_count++;
				if(srd[1].dir==CW)
				{	  	
					position[1]++;
				}
				else
				{
					position[1]--;
				}
				
				if(srd[1].step_delay > srd[1].int_val)
					srd[1].step_delay -= srd[1].int_val;
				else
					srd[1].step_delay = srd[1].max_speed;
				
				if(bStopCmd[1] || bLmtPos[1] || bLmtNeg[1])
				{
					step_count = srd[1].decel_start;	
					srd[1].run_state = DECEL;			
				}
				else if(step_count >= srd[1].decel_start)
				{	
					srd[1].run_state = DECEL;
				}
				else if(srd[1].step_delay <= srd[1].max_speed)
				{ 
					srd[1].run_state = RUN;	
				}
				break;

			case RUN:
				step_count++;
				if(Sys[1].NoStop==1)
					break;	
				if(srd[1].dir==CW)
				{	  	
					position[1]++;
				}
				else
				{
					position[1]--;
				}
				
				if(bStopCmd[1] || bLmtPos[1] || bLmtNeg[1])
				{
					step_count = srd[1].decel_start;	
					srd[1].run_state = DECEL;			
				}
				else if(step_count >= srd[1].decel_start)
				{
					srd[1].run_state = DECEL;
				}
				break;

			case DECEL:
				step_count++;
				if(srd[1].dir==CW)
				{	  	
					position[1]++;
				}
				else
				{
					position[1]--;
				}
				
				srd[1].step_delay += srd[1].decel_val;
				if(srd[1].step_delay>MIN_SPEED)
					srd[1].step_delay=MIN_SPEED;
				
				if(step_count>=srd[1].toal_step)
				{
					srd[1].run_state = STOP;
				}
				break;
		}
  }
}

void TIM4_IRQHandler(void)
{ 
  static u32 step_count = 0;
  static u8 i=0;

  if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
  {
    TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
  }
  TIM4->CCR1=srd[2].step_delay;
	TIM4->ARR=srd[2].step_delay;
	
  if(srd[2].run_state)
	{
	  if(READ_PULSE3==1)
		{
		  SET_PULSE3_0;
		}
	  else
	  {
			SET_PULSE3_1;
	  }
	}	   
	
	switch(EncoStat[2]) 
	{
		case 0:
			if(OldClk[2]!=READ_ENCCLK2)
			{			
				EncoStat[2]++;		
				OldClk[2]=READ_ENCCLK2;
			}
			break;
		case 1:
			if(OldDat[2]!=READ_ENCDAT2)
			{		
				if(srd[2].dir==CW)
					encoder_cnt[2]++;
				else
					encoder_cnt[2]--;
				EncoStat[2]=0;		
				OldDat[2]=READ_ENCDAT2;
			}
			break;
	}	
	
  i++;
 if(i==2)
 {
	 i=0;
	switch(srd[2].run_state) 
	{
		case STOP:
			step_count = 0;
			TIM_Cmd(TIM4, DISABLE);
			MotionStatus[2] = 0;
			bEmgStopping[2]=FALSE;
			break;


			case ACCEL:		
				step_count++;				
				if(srd[2].dir==CW)
				{	
					position[2]++;
				}
				else
				{ 
					position[2]--;
				}
				
				if(srd[2].step_delay > srd[2].int_val)
					srd[2].step_delay -= srd[2].int_val;
				else
					srd[2].step_delay = srd[2].max_speed;
				
				
				if(bStopCmd[2])
				{
					srd[2].run_state = STOP;
				}
				else if(bLmtPos[2] || bLmtNeg[2])
				{
					step_count = srd[2].decel_start;	
					srd[2].run_state = DECEL;			
				}
				else if(step_count >= srd[2].decel_start)
				{	
					srd[2].run_state = DECEL;
				}
				else if(srd[2].step_delay <= srd[2].max_speed)
				{ 
					srd[2].run_state = RUN;	
				}
				break;

    case RUN:	
      step_count++;
			if(Sys[2].NoStop==1)
				break;	
			if(srd[2].dir==CW)
			{	  	
				position[2]++;
			}
			else
			{
				position[2]--;
			}
			
			if(bStopCmd[2])
			{
				srd[2].run_state = STOP;
			}
			else if(bLmtPos[2] || bLmtNeg[2])
		
			{
				step_count = srd[2].decel_start;	
				srd[2].run_state = DECEL;			
			}
      else if(step_count >= srd[2].decel_start)
			{
        srd[2].run_state = DECEL;
      }
      break;

    case DECEL:
      step_count++;
			if(srd[2].dir==CW)
			{	  	
				position[2]++;
			}
			else
			{
				position[2]--;
			}
			
			srd[2].step_delay += srd[2].decel_val;
			if(srd[2].step_delay>MIN_SPEED)
				srd[2].step_delay=MIN_SPEED;
			
			if(step_count>=srd[2].toal_step || bStopCmd[2])
			{
        srd[2].run_state = STOP;
      }
      break;
    }
  }
}

void TIM5_IRQHandler(void)
{ 
  static u32 step_count = 0;
  static u8 i=0;

  if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET)
  {
    TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
  }
  TIM5->CCR1=srd[3].step_delay;
	TIM5->ARR=srd[3].step_delay;

  if(srd[3].run_state)
  {
	  if(READ_PULSE4==1)
		{
		  SET_PULSE4_0;
		}
	  else
	  {
			SET_PULSE4_1;
	  }
	}	  
	
	switch(EncoStat[3]) 
	{
		case 0:
			if(OldClk[3]!=READ_ENCCLK3)
			{			
				EncoStat[3]++;		
				OldClk[3]=READ_ENCCLK3;
			}
			break;
		case 1:
			if(OldDat[3]!=READ_ENCDAT3)
			{		
				if(srd[3].dir==CW)
					encoder_cnt[3]++;
				else
					encoder_cnt[3]--;
				EncoStat[3]=0;		
				OldDat[3]=READ_ENCCLK3;
			}
			break;
	}	
	
  i++;
  if(i==2)
  {
	 i=0;
		switch(srd[3].run_state) 
		{
			case STOP:
				step_count = 0;
				TIM_Cmd(TIM5, DISABLE);
				MotionStatus[3] = 0;
				bEmgStopping[3]=FALSE;
				break;

			case ACCEL:		
				step_count++;				
				if(srd[3].dir==CW)
				{	
					position[3]++;
				}
				else
				{ 
					position[3]--;
				}
				
				if(srd[3].step_delay > srd[3].int_val)
					srd[3].step_delay -= srd[3].int_val;
				else
					srd[3].step_delay = srd[3].max_speed;
				
				if(bStopCmd[3] || bLmtPos[3] || bLmtNeg[3])
				{
					step_count = srd[3].decel_start;	
					srd[3].run_state = DECEL;			
				}
				else if(step_count >= srd[3].decel_start)
				{	
					srd[3].run_state = DECEL;
				}
				else if(srd[3].step_delay <= srd[3].max_speed)
				{ 
					srd[3].run_state = RUN;	
				}
				break;

    case RUN:	
      step_count++;
			if(Sys[3].NoStop==1)
				break;	
			if(srd[3].dir==CW)
			{	  	
				position[3]++;
			}
			else
			{
				position[3]--;
			}
			
			if(bStopCmd[3] || bLmtPos[3] || bLmtNeg[3])
			{
				step_count = srd[3].decel_start;	
				srd[3].run_state = DECEL;			
			}
      else if(step_count >= srd[3].decel_start)
			{
        srd[3].run_state = DECEL;
      }
      break;

    case DECEL:
      step_count++;
			if(srd[3].dir==CW)
			{	  	
				position[3]++;
			}
			else
			{
				position[3]--;
			}
			
			srd[3].step_delay += srd[3].decel_val;
			if(srd[3].step_delay>MIN_SPEED)
				srd[3].step_delay=MIN_SPEED;
			
			if(step_count>=srd[3].toal_step)
			{
        srd[3].run_state = STOP;
      }
      break;
    }
  }
}

void EXTI4_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line4) != RESET)
  {
    HomePos[0]=position[0];
		bZeroCapture[0]=TRUE;
		if(MotionStatus[0])
		{
			bStopCmd[0]=TRUE;
		}
    EXTI_ClearITPendingBit(EXTI_Line4);
		DisableHomeCapture1;
		
  }
}

void EXTI3_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line3) != RESET)
  {
    HomePos[1]=position[1];
		bZeroCapture[1]=TRUE;
		if(MotionStatus[1])
		{
			bStopCmd[1]=TRUE;
		}
    EXTI_ClearITPendingBit(EXTI_Line3);
		DisableHomeCapture2;
		
  }
}

void EXTI9_5_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line7) != RESET)
  {
    HomePos[2]=position[2];
		bZeroCapture[2]=TRUE;
		if(MotionStatus[2])
		{
			bStopCmd[2]=TRUE;
		}
    EXTI_ClearITPendingBit(EXTI_Line7);
		DisableHomeCapture3;
		
  }
  if(EXTI_GetITStatus(EXTI_Line6) != RESET)
  {
    HomePos[3]=position[3];
		bZeroCapture[3]=TRUE;
		if(MotionStatus[3])
		{
			bStopCmd[3]=TRUE;
		}
    EXTI_ClearITPendingBit(EXTI_Line6);
		DisableHomeCapture4;
		
  }
	
  if(EXTI_GetITStatus(EXTI_Line8) != RESET)
  {
    EXTI_ClearITPendingBit(EXTI_Line8);
  }
  if(EXTI_GetITStatus(EXTI_Line9) != RESET)
  {
    EXTI_ClearITPendingBit(EXTI_Line9);
  }
}

void EXTI15_10_IRQHandler(void)
{
	
  if(EXTI_GetITStatus(EXTI_Line10) != RESET)
  {
    EXTI_ClearITPendingBit(EXTI_Line10);
  }
  if(EXTI_GetITStatus(EXTI_Line11) != RESET)
  {
    EXTI_ClearITPendingBit(EXTI_Line11);
  }
  if(EXTI_GetITStatus(EXTI_Line12) != RESET)
  {
		DisableHomeCapture12;















		
    EXTI_ClearITPendingBit(EXTI_Line12);

  }
  if(EXTI_GetITStatus(EXTI_Line13) != RESET)
  {
		DisableHomeCapture13;
    EXTI_ClearITPendingBit(EXTI_Line13);
	
  }
  if(EXTI_GetITStatus(EXTI_Line14) != RESET)
  {
    EXTI_ClearITPendingBit(EXTI_Line14);
  }
  if(EXTI_GetITStatus(EXTI_Line15) != RESET)
  {
    EXTI_ClearITPendingBit(EXTI_Line15);
  }
}

void USART3_IRQHandler(void)
{	
	USART3_Rx_IRQHandler();	
}

void DMA1_Channel2_IRQHandler(void)
{
	USART3_DMAHANDLER_Eago();
}










#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval : None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */
