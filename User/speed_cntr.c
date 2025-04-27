#include "stm32f10x.h"
#include "speed_cntr.h"
#include "stm32f10x_conf.h"

u8 i;   
speedRampData srd[4];
extern uint8_t RxBuffer[6];
extern uint8_t RxCounter;
extern bool bDataOK;
extern s32 position;
extern bool bLmtPos;
extern bool bLmtNeg; 
extern bool bStopCmd;

void speed_cntr_Move(unsigned int axis, signed int step, unsigned int accel, unsigned int decel, unsigned int speed)
{
  
  u32 max_s_lim;
  
  u32 accel_lim;

  
   if(step <0)
    {









      srd[axis-1].dir = CCW;
	  switch(axis)
	  {
	  case 1:
	  	 GPIO_ResetBits(GPIOA, AXIS_DIR_1);
		 break;
	  case 2:
		 GPIO_ResetBits(GPIOA, AXIS_DIR_2);
		 break;
	  case 3:
	     GPIO_ResetBits(GPIOA, AXIS_DIR_3);
		 break;
	  case 4:
	     GPIO_ResetBits(GPIOA, AXIS_DIR_4);
		 break;
	  }

	  step =-step;
    }
  else
   {









      srd[axis-1].dir = CW;

	  switch(axis)
	  {
	  case 1:
	  	 GPIO_SetBits(GPIOA, AXIS_DIR_1);
		 break;
	  case 2:
		 GPIO_SetBits(GPIOA, AXIS_DIR_2);
		 break;
	  case 3:
	     GPIO_SetBits(GPIOA, AXIS_DIR_3);
		 break;
	  case 4:
	     GPIO_SetBits(GPIOA, AXIS_DIR_4);
		 break;
	  }
	   
   }
  
  if(step == 1)
  {
    
    srd[axis-1].accel_count = -1;
    
    srd[axis-1].run_state = DECEL;
    
    srd[axis-1].step_delay = 1000;
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
    
	status = TRUE;
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
    

    
    
    srd[axis-1].min_delay = T1_FREQ/speed/2;

    
    
    
    srd[axis-1].step_delay = ((long)T1_FREQ*0.676* sqrt(2000000 / accel))/1000/2;
    
    
    max_s_lim = speed*speed/(2*accel);
    
    
    if(max_s_lim == 0){
      max_s_lim = 1;
    }

    
    
    accel_lim = step/(accel+decel)*decel;
    
    if(accel_lim == 0){
      accel_lim = 1;
    }

    
    if(accel_lim <= max_s_lim){
      srd[axis-1].decel_val = accel_lim - step;
    }
    else{
      srd[axis-1].decel_val =-(s32)(max_s_lim*accel/decel);
    }
    
    if(srd[axis-1].decel_val == 0){
      srd[axis-1].decel_val = -1;
    }

    
    srd[axis-1].decel_start = step + srd[axis-1].decel_val;

    
    if(srd[axis-1].step_delay <= srd[axis-1].min_delay)
	{
      srd[axis-1].step_delay = srd[axis-1].min_delay;
      srd[axis-1].run_state = RUN;
    }
    else{
      srd[axis-1].run_state = ACCEL;
    }

    
    srd[axis-1].accel_count = 0;
    status = TRUE;
    
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

void TIM4_IRQHandler(void)
{ 
  
  u16 new_step_delay;
  
  static u16 last_accel_delay;
  
  static u32 step_count = 0;
  
  static s32 rest = 0;
  static u8 i=0;

  if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
   {
    /* Clear TIM2 Capture Compare1 interrupt pending bit*/
    TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
   }
  	TIM4->CCR1=srd[2].step_delay;
	TIM4->ARR=srd[2].step_delay;
 
  if(srd[2].run_state)
    {
	  if(GPIO_ReadOutputDataBit(CONTROL_PORT, AXIS_PULSE_3)==1)
		 {
		   GPIO_ResetBits(CONTROL_PORT, AXIS_PULSE_3);
		 }
	   else
	     {
			GPIO_SetBits(CONTROL_PORT, AXIS_PULSE_3);
	     }


	}
	
  i++;
  if(i==2)
 {i=0;
  switch(srd[2].run_state) 
  {
    case STOP:
      step_count = 0;
      rest = 0;
      
	  TIM_Cmd(TIM4, DISABLE);
      
      status = FALSE;
      break;

    case ACCEL:
      
      step_count++;
	  if(srd[2].dir==CW)
	  {	  	
	  	position++;
		if(bLmtPos)
		{
			srd[2].run_state = STOP;
			break;	
		}
	  }
	  else
	  {
	  	position--;
		if(bLmtNeg)
		{
			srd[2].run_state = STOP;
			break;	
		}
	  }
      srd[2].accel_count++;
      new_step_delay = srd[2].step_delay - (((2 * (long)srd[2].step_delay) + rest)/(4 * srd[2].accel_count + 1));
      rest = ((2 * (long)srd[2].step_delay)+rest)%(4 * srd[2].accel_count + 1);
      
      if(step_count >= srd[2].decel_start  || bStopCmd)
	  {
	  	if(bStopCmd)
		{
			bStopCmd=FALSE;
		}
        srd[2].accel_count = srd[2].decel_val;
        srd[2].run_state = DECEL;
      }
      
      else if(new_step_delay <= srd[2].min_delay)
	  {
        last_accel_delay = new_step_delay;
        new_step_delay = srd[2].min_delay;
        rest = 0;
        srd[2].run_state = RUN;
      }
      break;

    case RUN:
      
      step_count++;
	  if(srd[2].dir==CW)
	  {	  	
	  	position++;
		if(bLmtPos)
		{
			srd[2].run_state = STOP;
			break;	
		}
	  }
	  else
	  {
	  	position--;
		if(bLmtNeg)
		{
			srd[2].run_state = STOP;
			break;	
		}
	  }
      new_step_delay = srd[2].min_delay;
      
      if(step_count >= srd[2].decel_start || bStopCmd)
	   {
	   	if(bStopCmd)
		{
			bStopCmd=FALSE;
		}
        srd[2].accel_count = srd[2].decel_val;
        
        new_step_delay = last_accel_delay;
        srd[2].run_state = DECEL;
      }
      break;

    case DECEL:
      
      step_count++;
	  if(srd[2].dir==CW)
	  {	  	
	  	position++;
		if(bLmtPos)
		{
			srd[2].run_state = STOP;
			break;	
		}
	  }
	  else
	  {
	  	position--;
		if(bLmtNeg)
		{
			srd[2].run_state = STOP;
			break;	
		}
	  }
      srd[2].accel_count++;
      new_step_delay = srd[2].step_delay - (2*srd[2].step_delay+rest)/(4*srd[2].accel_count+1);
      rest = (2 * srd[2].step_delay+rest)%(4 * srd[2].accel_count + 1);
      
      if(srd[2].accel_count >= 0)
	  {
        srd[2].run_state = STOP;
      }
	  if(bStopCmd)
	  {
	  	bStopCmd=FALSE;
	  }
      break;
   }
  srd[2].step_delay = new_step_delay;
  
  }
}



 void TIM3_IRQHandler(void)
{ 
  
  u16 new_step_delay;
  
  static u16 last_accel_delay;
  
  static u32 step_count = 0;
  
  static s32 rest = 0;
  static u8 i=0;

  if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
   {
    /* Clear TIM2 Capture Compare1 interrupt pending bit*/
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
   }
  	TIM3->CCR1=srd[1].step_delay;
	TIM3->ARR=srd[1].step_delay;
 
  if(srd[1].run_state)
    {
	  if(GPIO_ReadOutputDataBit(CONTROL_PORT, AXIS_PULSE_2)==1)
		 {
		   GPIO_ResetBits(CONTROL_PORT, AXIS_PULSE_2);
		 }
	   else
	     {
			GPIO_SetBits(CONTROL_PORT, AXIS_PULSE_2);
	     }


	}
	
  i++;
  if(i==2)
 {i=0;
  switch(srd[1].run_state) 
  {
    case STOP:
      step_count = 0;
      rest = 0;
      
	  TIM_Cmd(TIM3, DISABLE);
      
      status = FALSE;
      break;

    case ACCEL:
      
      step_count++;
	  if(srd[1].dir==CW)
	  {	  	
	  	position++;
		if(bLmtPos)
		{
			srd[1].run_state = STOP;
			break;	
		}
	  }
	  else
	  {
	  	position--;
		if(bLmtNeg)
		{
			srd[1].run_state = STOP;
			break;	
		}
	  }
      srd[1].accel_count++;
      new_step_delay = srd[1].step_delay - (((2 * (long)srd[1].step_delay) + rest)/(4 * srd[1].accel_count + 1));
      rest = ((2 * (long)srd[1].step_delay)+rest)%(4 * srd[1].accel_count + 1);
      
      if(step_count >= srd[1].decel_start  || bStopCmd)
	  {
	  	if(bStopCmd)
		{
			bStopCmd=FALSE;
		}
        srd[1].accel_count = srd[1].decel_val;
        srd[1].run_state = DECEL;
      }
      
      else if(new_step_delay <= srd[1].min_delay)
	  {
        last_accel_delay = new_step_delay;
        new_step_delay = srd[1].min_delay;
        rest = 0;
        srd[1].run_state = RUN;
      }
      break;

    case RUN:
      
      step_count++;
	  if(srd[1].dir==CW)
	  {	  	
	  	position++;
		if(bLmtPos)
		{
			srd[1].run_state = STOP;
			break;	
		}
	  }
	  else
	  {
	  	position--;
		if(bLmtNeg)
		{
			srd[1].run_state = STOP;
			break;	
		}
	  }
      new_step_delay = srd[1].min_delay;
      
      if(step_count >= srd[1].decel_start || bStopCmd)
	   {
	   	if(bStopCmd)
		{
			bStopCmd=FALSE;
		}
        srd[1].accel_count = srd[1].decel_val;
        
        new_step_delay = last_accel_delay;
        srd[1].run_state = DECEL;
      }
      break;

    case DECEL:
      
      step_count++;
	  if(srd[1].dir==CW)
	  {	  	
	  	position++;
		if(bLmtPos)
		{
			srd[1].run_state = STOP;
			break;	
		}
	  }
	  else
	  {
	  	position--;
		if(bLmtNeg)
		{
			srd[1].run_state = STOP;
			break;	
		}
	  }
      srd[1].accel_count++;
      new_step_delay = srd[1].step_delay - (((2 * (long)srd[1].step_delay) + rest)/(4 * srd[1].accel_count + 1));
      rest = ((2 * (long)srd[1].step_delay)+rest)%(4 * srd[1].accel_count + 1);
      
      if(srd[1].accel_count >= 0)
	  {
        srd[1].run_state = STOP;
      }
	  if(bStopCmd)
	  {
	  	bStopCmd=FALSE;
	  }
      break;
   }
  srd[1].step_delay = new_step_delay;
  
  }
}

 void TIM2_IRQHandler(void)
{ 
  
  u16 new_step_delay;
  
  static u16 last_accel_delay;
  
  static u32 step_count = 0;
  
  static s32 rest = 0;
  static u8 i=0;

  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
   {
    /* Clear TIM2 Capture Compare1 interrupt pending bit*/
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
   }
  	TIM2->CCR1=srd[0].step_delay;
	TIM2->ARR=srd[0].step_delay;
 
  if(srd[0].run_state)
    {
	  if(GPIO_ReadOutputDataBit(CONTROL_PORT, AXIS_PULSE_1)==1)
		 {
		   GPIO_ResetBits(CONTROL_PORT, AXIS_PULSE_1);
		 }
	   else
	     {
			GPIO_SetBits(CONTROL_PORT, AXIS_PULSE_1);
	     }


	}
	
  i++;
  if(i==2)
 {i=0;
  switch(srd[0].run_state) 
  {
    case STOP:
      step_count = 0;
      rest = 0;
      
	  TIM_Cmd(TIM2, DISABLE);
      
      status = FALSE;
      break;

    case ACCEL:
      
      step_count++;
	  if(srd[0].dir==CW)
	  {	  	
	  	position++;
		if(bLmtPos)
		{
			srd[0].run_state = STOP;
			break;	
		}
	  }
	  else
	  {
	  	position--;
		if(bLmtNeg)
		{
			srd[0].run_state = STOP;
			break;	
		}
	  }
      srd[0].accel_count++;
      new_step_delay = srd[0].step_delay - (((2 * (long)srd[0].step_delay) + rest)/(4 * srd[0].accel_count + 1));
      rest = ((2 * (long)srd[0].step_delay)+rest)%(4 * srd[0].accel_count + 1);
      
      if(step_count >= srd[0].decel_start  || bStopCmd)
	  {
	  	if(bStopCmd)
		{
			bStopCmd=FALSE;
		}
        srd[0].accel_count = srd[0].decel_val;
        srd[0].run_state = DECEL;
      }
      
      else if(new_step_delay <= srd[0].min_delay)
	  {
        last_accel_delay = new_step_delay;
        new_step_delay = srd[0].min_delay;
        rest = 0;
        srd[0].run_state = RUN;
      }
      break;

    case RUN:
      
      step_count++;
	  if(srd[0].dir==CW)
	  {	  	
	  	position++;
		if(bLmtPos)
		{
			srd[0].run_state = STOP;
			break;	
		}
	  }
	  else
	  {
	  	position--;
		if(bLmtNeg)
		{
			srd[0].run_state = STOP;
			break;	
		}
	  }
      new_step_delay = srd[0].min_delay;
      
      if(step_count >= srd[0].decel_start || bStopCmd)
	   {
	   	if(bStopCmd)
		{
			bStopCmd=FALSE;
		}
        srd[0].accel_count = srd[0].decel_val;
        
        new_step_delay = last_accel_delay;
        srd[0].run_state = DECEL;
      }
      break;

    case DECEL:
      
      step_count++;
	  if(srd[0].dir==CW)
	  {	  	
	  	position++;
		if(bLmtPos)
		{
			srd[0].run_state = STOP;
			break;	
		}
	  }
	  else
	  {
	  	position--;
		if(bLmtNeg)
		{
			srd[0].run_state = STOP;
			break;	
		}
	  }
      srd[0].accel_count++;
      new_step_delay = srd[0].step_delay - (((2 * (long)srd[0].step_delay) + rest)/(4 * srd[0].accel_count + 1));
      rest = ((2 * (long)srd[0].step_delay)+rest)%(4 * srd[0].accel_count + 1);
      
      if(srd[0].accel_count >= 0)
	  {
        srd[0].run_state = STOP;
      }
	  if(bStopCmd)
	  {
	  	bStopCmd=FALSE;
	  }
      break;
   }
  srd[0].step_delay = new_step_delay;
  
  }
}



 void TIM5_IRQHandler(void)
{ 
  
  u16 new_step_delay;
  
  static u16 last_accel_delay;
  
  static u32 step_count = 0;
  
  static s32 rest = 0;
  static u8 i=0;

  if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET)
   {
    /* Clear TIM2 Capture Compare1 interrupt pending bit*/
    TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
   }
  	TIM5->CCR1=srd[3].step_delay;
	TIM5->ARR=srd[3].step_delay;
 
  if(srd[3].run_state)
    {
	  if(GPIO_ReadOutputDataBit(CONTROL_PORT, AXIS_PULSE_4)==1)
		 {
		   GPIO_ResetBits(CONTROL_PORT, AXIS_PULSE_4);
		 }
	   else
	     {
			GPIO_SetBits(CONTROL_PORT, AXIS_PULSE_4);
	     }


	}
	
  i++;
  if(i==2)
 {i=0;
  switch(srd[3].run_state) 
  {
    case STOP:
      step_count = 0;
      rest = 0;
      
	  TIM_Cmd(TIM5, DISABLE);
      
      status = FALSE;
      break;

    case ACCEL:
      
      step_count++;
	  if(srd[3].dir==CW)
	  {	  	
	  	position++;
		if(bLmtPos)
		{
			srd[3].run_state = STOP;
			break;	
		}
	  }
	  else
	  {
	  	position--;
		if(bLmtNeg)
		{
			srd[3].run_state = STOP;
			break;	
		}
	  }
      srd[3].accel_count++;
      new_step_delay = srd[3].step_delay - (((2 * (long)srd[3].step_delay) + rest)/(4 * srd[3].accel_count + 1));
      rest = ((2 * (long)srd[3].step_delay)+rest)%(4 * srd[3].accel_count + 1);
      
      if(step_count >= srd[3].decel_start  || bStopCmd)
	  {
	  	if(bStopCmd)
		{
			bStopCmd=FALSE;
		}
        srd[3].accel_count = srd[3].decel_val;
        srd[3].run_state = DECEL;
      }
      
      else if(new_step_delay <= srd[3].min_delay)
	  {
        last_accel_delay = new_step_delay;
        new_step_delay = srd[3].min_delay;
        rest = 0;
        srd[3].run_state = RUN;
      }
      break;

    case RUN:
      
      step_count++;
	  if(srd[3].dir==CW)
	  {	  	
	  	position++;
		if(bLmtPos)
		{
			srd[3].run_state = STOP;
			break;	
		}
	  }
	  else
	  {
	  	position--;
		if(bLmtNeg)
		{
			srd[3].run_state = STOP;
			break;	
		}
	  }
      new_step_delay = srd[3].min_delay;
      
      if(step_count >= srd[3].decel_start || bStopCmd)
	   {
	   	if(bStopCmd)
		{
			bStopCmd=FALSE;
		}
        srd[3].accel_count = srd[3].decel_val;
        
        new_step_delay = last_accel_delay;
        srd[3].run_state = DECEL;
      }
      break;

    case DECEL:
      
      step_count++;
	  if(srd[3].dir==CW)
	  {	  	
	  	position++;
		if(bLmtPos)
		{
			srd[3].run_state = STOP;
			break;	
		}
	  }
	  else
	  {
	  	position--;
		if(bLmtNeg)
		{
			srd[3].run_state = STOP;
			break;	
		}
	  }
      srd[3].accel_count++;
      new_step_delay = srd[3].step_delay - (((2 * (long)srd[3].step_delay) + rest)/(4 * srd[3].accel_count + 1));
      rest = ((2 * (long)srd[3].step_delay)+rest)%(4 * srd[3].accel_count + 1);
      
      if(srd[3].accel_count >= 0)
	  {
        srd[3].run_state = STOP;
      }
	  if(bStopCmd)
	  {
	  	bStopCmd=FALSE;
	  }
      break;
   }
  srd[3].step_delay = new_step_delay;
  
  }
}

/*! \brief Square root routine.
 *
 * sqrt routine 'grupe', from comp.sys.ibm.pc.programmer
 * Subject: Summary: SQRT(int) algorithm (with profiling)
 *    From: warwick@cs.uq.oz.au (Warwick Allison)
 *    Date: Tue Oct 8 09:16:35 1991
 *
 *  \param x  Value to find square root of.
 *  \return  Square root of x.
 */
static u32 sqrt(u32 x)
{
  register u32 xr;  
  register u32 q2;  
  register u8 f;   

  xr = 0;                     
  q2 = 0x40000000L;           
  do
  {
    if((xr + q2) <= x)
    {
      x -= xr + q2;
      f = 1;                  
    }
    else{
      f = 0;                  
    }
    xr >>= 1;
    if(f){
      xr += q2;               
    }
  } while(q2 >>= 2);          
  if(xr < x){
    return xr +1;             
  }
  else{
    return xr;
	  }
}

/*! \brief Find minimum value.
 *
 *  Returns the smallest value.
 *
 *  \return  Min(x,y).
 */
unsigned int min(unsigned int x, unsigned int y)
{
  if(x < y)
  {
    return x;
  }
  else
  {
    return y;
  }
}

void USART1_IRQHandler(void)
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
  	{
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

		RxBuffer[RxCounter] = USART_ReceiveData(USART1);


		if(RxBuffer[RxCounter]=='#')
		{
			RxBuffer[RxCounter]=0;
			RxCounter=0;
			bDataOK=TRUE;
		}
		else
		{
			RxCounter++;
			if(RxCounter==12)
			{
				RxCounter=0;
			}
			
		}
		
		
		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	}





}

void EXTI0_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line0) != RESET)
	{
		
		printf("%d\r\n", position);
		EXTI_ClearITPendingBit(EXTI_Line0);
	}
}

void EXTI1_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line1) != RESET)
	{
		printf("%d\r\n", position);
		EXTI_ClearITPendingBit(EXTI_Line1);
	}
}
