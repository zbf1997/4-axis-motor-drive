#ifndef __USART3_H
#define	__USART3_H
#include "stm32f10x.h"


#define USART3_DR_Base  0x40004804



#define BUFF_LEN  1024	


#define Cmd_Read        0x03    
#define Cmd_WritSing    0x06    
#define Cmd_WritMul     0x10    



#define REG_MAXSTEP1	0x00		
#define REG_STEP1			0x02		
#define REG_ACCE1			0x04		
#define REG_DECE1			0x05		
#define REG_SPEED1		0x06		
#define REG_CUR1			0x07		
#define REG_MODE1			0x08		
#define REG_REALSTEP1	0x09		
#define REG_STAT1			0x0B		


#define REG_MAXSTEP2	0x0C		
#define REG_STEP2			0x0E		
#define REG_ACCE2			0x10		
#define REG_DECE2			0x11		
#define REG_SPEED2		0x12		
#define REG_CUR2			0x13		
#define REG_MODE2			0x14		
#define REG_REALSTEP2	0x15		
#define REG_STAT2			0x17		


#define REG_MAXSTEP3	0x18		
#define REG_STEP3			0x1A		
#define REG_ACCE3			0x1C		
#define REG_DECE3			0x1D		
#define REG_SPEED3		0x1E		
#define REG_CUR3			0x1F		
#define REG_MODE3			0x20		
#define REG_REALSTEP3	0x21		
#define REG_STAT3			0x23		


#define REG_MAXSTEP4	0x24		
#define REG_STEP4			0x26		
#define REG_ACCE4			0x28		
#define REG_DECE4			0x29		
#define REG_SPEED4		0x2A		
#define REG_CUR4			0x2B		
#define REG_MODE4			0x2C		
#define REG_REALSTEP4	0x2D		
#define REG_STAT4			0x2F		


#define GRUP_NUM	REG_MAXSTEP2		
#define REG_NUM		REG_STAT4+1		

#define REG_ZERO1 0x4E    
#define REG_ZERO2 0x4F    
#define REG_ZERO3 0x50    
#define REG_ZERO4 0x51    

#define REG_STNO  0x60 //更改通讯地址寄存器   
#define REG_BTR	  0x61 //更改波特率寄存器   

#pragma pack(1)	




typedef struct
{
	u8  Addr;       
	u8  Cmd;        
	u8  RegAddr_H;  
	u8  RegAddr_L;
	u8  Data_H;     
	u8  Data_L;
	u16 Data[64];    
}WoRxSing;

typedef struct
{
	u8  Addr;       
	u8  Cmd;        
	u8  RegAddr_H;  
	u8  RegAddr_L;
	u16 RegNum;   	
	u8  DatCnt;   	
	u16 Data[64];   
}WoRxMul;		


typedef union _WO_MBDATA_EX
{ 
	WoRxSing Sin;	
	WoRxMul Mul;
	u8 Data[256]; 	
}WoMbRxData3;


typedef struct
{
	u8  Addr;       
	u8  Cmd;        
	u8  Len;        
	u16 Data[64];   
}WoMbTxData3;

typedef struct
{
	s32	MaxStep;	
	s32	RunStep;	
	u16	Acce;			
	u16	Dece;			
	u16	Speed;		
	u16	Cur;			
	u16	Mode;			
	s32	RealStep;	
	u16	Stat;			
	u16	NoStop;		
}WoSys;	

typedef struct 
{
	uint16_t StNo;				
	uint16_t Btr;					
	uint16_t Zero[4];
	uint16_t crc16;
}WoSet;		







typedef struct
{
	u16 data[256];
}WoReg;

extern FlagStatus Flag_RxOk;
extern	FlagStatus Flag_NewCmd;
extern	FlagStatus Flag_Save;
extern	FlagStatus Flag_RecOk3;
extern	FlagStatus FlagUrt3_Busy;
extern	u8	fre485_time3;



extern	WoSet Set;
extern	WoReg Reg; 
extern	WoSys Sys[4]; 

extern	void  USART3_Config(u32 usart_btl);
extern	void  USART3_SEND_BUFF(u32 SendBitsLength, u8 *dst);
extern	void	USART3_DMAHANDLER_Eago(void);
extern	void 	handle_Data3(void);
extern	void 	USART3_Rx_IRQHandler(void);
extern	void USART3_INIT(void);
extern	void SysModeClr(u8 i);
extern	void RxMdbus_Data3(void);
#endif 



