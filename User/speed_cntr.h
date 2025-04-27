
#ifndef SPEED_CNTR_H
#define SPEED_CNTR_H


#define CW  0
#define CCW 1
/*! \brief Holding data used by timer interrupt for speed ramp calculation.
 *
 *  Contains data used by timer interrupt to calculate speed profile.
 *  Data is written to it by move(), when stepper motor is moving (timer
 *  interrupt running) data is read/updated when calculating a new step_delay
 */
typedef struct {
  
  u8 run_state ;
  
  u8 dir ;
  
  s32 step_delay;
  
  u32 decel_start;
  
  s32 decel_val;
  
  s32 min_delay;
  
  s32 accel_count;
} speedRampData;


/*! \Brief Frequency of timer1 in [Hz].
 *
 * Modify this according to frequency used. Because of the prescaler setting,
 * the timer1 frequency is the clock frequency divided by 8.
 */

#define T1_FREQ 1000000
#define SPR 1600

#define ALPHA (2*3.14159/SPR)                    
#define A_T_x100 ((long)(ALPHA*T1_FREQ*100))     
#define T1_FREQ_148 ((int)((T1_FREQ*0.676)/100)) 
#define A_SQ (long)(ALPHA*2*100000*100000)         
#define A_x20000 (int)(ALPHA*20000)              

#define STOP  0
#define ACCEL 1
#define DECEL 2
#define RUN   3

extern void speed_cntr_Move(unsigned int axis,signed int step, unsigned int accel, unsigned int decel, unsigned int speed);
extern u32 sqrt(u32 x);
extern bool status;








#endif
