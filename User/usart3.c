#include "usart3.h"	
#include <string.h>	
#include "stm32f10x_dma.h"
#include "misc.h"


WoMbTxData3 MbTxData3;
WoMbRxData3 MbRxData3; 
WoSys Sys[4]; 
WoReg Reg; 
WoSet Set;
FlagStatus FlagUrt3_Busy=RESET;
FlagStatus Flag_Save=RESET;
FlagStatus Flag_NewCmd=RESET;
FlagStatus Flag_RxOk=RESET;

vu8 	ABuffer3[BUFF_LEN];
u8	fre485_time3=0;
u16 DATA_LEN=0;
extern	u16 ZeroOffset[4];

u16 MB_CRC16_calculate(u8 * array,u8 index)
{
  u16 tmp = 0xffff;
  u16 n = 0;
  u16 i = 0;
  for(n = 0; n < index; n++)
  {
    tmp = (uint16_t)(array[n] ^ tmp);
    for(i = 0;i < 8;i++)
    {
        if(tmp & 0x01)
        {
            tmp = tmp >> 1;
            tmp = tmp ^ 0xa001;
        }
        else
        {
            tmp = tmp >> 1;
        }
    }
  }
  return (uint16_t)(tmp);
}

u16 htons(u16 n)//高8位低8位互换
{
  return (u16)((n & 0xff) << 8) | ((n & 0xff00) >> 8);
}

void Usart3NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure; 
	/* Configure the NVIC Preemption Priority Bits */  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);	
	/* Enable the USARTy Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;	 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void  Usart3DmaNVIC_Config(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  /* Configure one bit for preemption priority */   
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	/* 配置P[A|B|C|D|E]0为中断源 */
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void DMA_Configuration3(void)		
{
   DMA_InitTypeDef DMA_InitStructure; 
  /* DMA clock enable */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  /* DMA1 Channel5 (串口1 RX) Config */
  DMA_DeInit(DMA1_Channel3); //重置DMA1_Channel3
  DMA_InitStructure.DMA_PeripheralBaseAddr = USART3_DR_Base;// 外设地址=USART3数据寄存器
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ABuffer3;// 内存地址=接收缓冲区
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;// 传输方向：外设→内存
  DMA_InitStructure.DMA_BufferSize = BUFF_LEN;// 缓冲区长度
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;// 外设地址固定
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;// 内存地址递增
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//8位数据
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;// 8位数据
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;// 循环模式（持续接收）	  
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;// 高优先级
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;// 禁用内存到内存模式
  DMA_Init(DMA1_Channel3, &DMA_InitStructure);
	/* 使能传输完成和传输错误中断 */
  DMA_ITConfig(DMA1_Channel3, DMA_IT_TC, ENABLE);
  DMA_ITConfig(DMA1_Channel3, DMA_IT_TE, ENABLE);
  /* 使能USART3 DMA接收请求 */
  USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
  DMA_Cmd(DMA1_Channel3, ENABLE);		 
}

void USART3_Config(u32 usart_btl)
{
	USART_InitTypeDef USART_InitStructure;
	/* USART1 mode config */	
	USART_InitStructure.USART_BaudRate = usart_btl;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &USART_InitStructure);

	USART_ITConfig(USART3, USART_IT_IDLE,ENABLE); // 使能空闲中断（用于不定长数据接收） 
	
	USART_Cmd(USART3, ENABLE);	
	
	Usart3NVIC_Configuration();// 配置串口3中断优先级	
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  
	Usart3DmaNVIC_Config();// 配置串口3 DMA中断优先级
	DMA_Configuration3();//配置串口3 DMA
}

void    USART3_SEND_BUFF(u32 SendBitsLength, u8 *dst)
{
  DMA_InitTypeDef DMA_InitStructure;  
	

	FlagUrt3_Busy=SET;//串口3忙置位	
	
	
  DMA_InitStructure.DMA_PeripheralBaseAddr = USART3_DR_Base;// 外设地址=USART3数据寄存器	   
	
  DMA_InitStructure.DMA_MemoryBaseAddr = (u32)dst;// 内存地址
	
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;// 传输方向：内存→外设	
	
  DMA_InitStructure.DMA_BufferSize = SendBitsLength;// 发送数据长度
	
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; //外设地址不变
	
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;// 内存地址递增	
	
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//外设数据类型8位
	
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//内存数据类型8位	 
	
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal ; // 普通模式（单次传输）	 
	
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium; // 中优先级 
	
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;// 禁用内存到内存模式
    	
  DMA_DeInit(DMA1_Channel2); // 复位通道配置
	
  DMA_Init(DMA1_Channel2, &DMA_InitStructure); // 初始化通道	   
	
	DMA_Cmd (DMA1_Channel2,ENABLE);	 // 启动DMA发送
    	
	DMA_ITConfig(DMA1_Channel2,DMA_IT_TC,ENABLE); // 使能传输完成中断 
	
  USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);// 使能USART3 DMA发送请求	

}

void	USART3_DMAHANDLER_Eago(void)
{
   
  if(DMA_GetFlagStatus(DMA1_FLAG_TC2)==SET)//DMA发送完成中断，内存→外设
  {  	  
  	DMA_ClearFlag(DMA1_FLAG_TC2); //清除发送完成标志位
		
		DMA_Cmd (DMA1_Channel2,DISABLE);//关闭DMA发送
		fre485_time3=2;		
		FlagUrt3_Busy=RESET;//清除传开忙标志	
	}	
}

void Modebus_DataSend(u8 SendLen)
{
	u16 calc_data=0;
	u8 len=0;
	
	
	len=(u8)(SendLen*2+3);
	calc_data=(u16)(MB_CRC16_calculate((u8*)&MbTxData3,(u8)len)); 
	
  MbTxData3.Data[MbRxData3.Sin.Data_L]=calc_data;
	USART3_SEND_BUFF(MbTxData3.Len+5,(u8*)&MbTxData3);	
}


void Modebus_Answer(void)
{
	USART3_SEND_BUFF(0x08,(u8*)&MbRxData3.Sin);// 发送固定长度8字节的响应	
}

void Modebus_MulAnswer(void)
{
	
	
	u16 calc_data=0;
	u8 len=0;
	
	len=6;
	calc_data=(u16)(MB_CRC16_calculate((u8*)&MbRxData3.Data,(u8)len)); 
  MbRxData3.Sin.Data[0]=calc_data;
	memcpy((u8*)&MbTxData3, (u8*)&MbRxData3, len+2); 
	USART3_SEND_BUFF(len+2,(u8*)&MbTxData3);	
}


void SysModeClr(u8 i)
{ 
	u16 kk;
	
	kk=GRUP_NUM*i;
	Reg.data[8+kk]=htons(Sys[i].Mode);
}

void DataToReg(void)
{ 
	u16 datH;
	u16 datL;
	u8 i;
	u16 kk;
	
	for(i=0;i<4;i++)
	{
		kk=GRUP_NUM*i;
		
		datH=Sys[i].MaxStep>>16;	datL=Sys[i].MaxStep&0xFFFF;
		Reg.data[0+kk]=htons(datH);
		Reg.data[1+kk]=htons(datL);
		
		datH=Sys[i].RunStep>>16;	datL=Sys[i].RunStep&0xFFFF;
		Reg.data[2+kk]=htons(datH);
		Reg.data[3+kk]=htons(datL);
		
		Reg.data[4+kk]=htons(Sys[i].Acce);
		Reg.data[5+kk]=htons(Sys[i].Dece);
		Reg.data[6+kk]=htons(Sys[i].Speed);
		Reg.data[7+kk]=htons(Sys[i].Cur);
		Reg.data[8+kk]=htons(Sys[i].Mode);
		
		datH=Sys[i].RealStep>>16;	datL=Sys[i].RealStep&0xFFFF;
		
		Reg.data[9+kk]=htons(datH);
		Reg.data[10+kk]=htons(datL);
		
		Reg.data[11+kk]=htons(Sys[i].Stat);		
	}
}

void RegToData(void)
{ 
	
	u16 datH;
	u16 datL;
	u8 i;
	u16 kk;
	
	for(i=0;i<4;i++)
	{
		kk=GRUP_NUM*i;
		datH=htons(Reg.data[0+kk]);	datL=htons(Reg.data[1+kk]);
		Sys[i].MaxStep=datH<<16| datL;
		
		datH=htons(Reg.data[2+kk]);	datL=htons(Reg.data[3+kk]);
		Sys[i].RunStep=datH<<16| datL;
		
		Sys[i].Acce=htons(Reg.data[4+kk]);
		Sys[i].Dece=htons(Reg.data[5+kk]);
		Sys[i].Speed=htons(Reg.data[6+kk]);
		Sys[i].Cur=htons(Reg.data[7+kk]);		
		Sys[i].Mode=htons(Reg.data[8+kk]);
		



	}
}




void RxMdbus_Data3(void)
{
	u8 i;
	u16 CalcLen;
	u16 crc16;
			
	memcpy((u8*)&MbRxData3.Data[0], (u8*)&ABuffer3[0], DATA_LEN); // 从DMA缓冲区复制数据
	crc16=(u16)(MB_CRC16_calculate((u8*)&MbRxData3,DATA_LEN-2));
	if((crc16==(ABuffer3[DATA_LEN-1]<<8 | ABuffer3[DATA_LEN-2])) && (MbRxData3.Sin.Addr==Set.StNo || MbRxData3.Sin.Addr==200))//接收缓冲区的后两个字节与校验值一样以及接收缓冲区的地址与从机号或者是200广播号相同则执行if语句
	{
		
		if(MbRxData3.Sin.Cmd==Cmd_Read)
		{
			switch(MbRxData3.Sin.RegAddr_L)//读取地址寄存器低8位  
			{			
				case REG_STNO:  	
					if(MbRxData3.Sin.Data_L>1)   
					{	break;}	
					MbTxData3.Data[0]=htons(Set.StNo);
					MbTxData3.Addr=MbRxData3.Sin.Addr;
					MbTxData3.Cmd=MbRxData3.Sin.Cmd;
					MbTxData3.Len=(uint8_t)(MbRxData3.Sin.Data_L*2); 
					Modebus_DataSend(MbRxData3.Sin.Data_L);  
					break;

				case REG_BTR:  	
					if(MbRxData3.Sin.Data_L>1)   
					{	break;}	
					MbTxData3.Data[0]=htons(Set.Btr);
					MbTxData3.Addr=MbRxData3.Sin.Addr;
					MbTxData3.Cmd=MbRxData3.Sin.Cmd;
					MbTxData3.Len=(uint8_t)(MbRxData3.Sin.Data_L*2); 
					Modebus_DataSend(MbRxData3.Sin.Data_L);  
					break;
					
				default: 
					if(MbRxData3.Sin.RegAddr_L>REG_NUM)   
					{	break;}	
									
					CalcLen=REG_NUM-MbRxData3.Sin.RegAddr_L;
					if(MbRxData3.Sin.Data_L>CalcLen)   
					{	break;}	
					
					DataToReg();	
					
					for(i=0;i<CalcLen;i++)
					{
						MbTxData3.Data[i]=Reg.data[i+MbRxData3.Sin.RegAddr_L];
					}
					MbTxData3.Addr=MbRxData3.Sin.Addr;
					MbTxData3.Cmd=MbRxData3.Sin.Cmd;
					MbTxData3.Len=(uint8_t)(MbRxData3.Sin.Data_L*2); 
					Modebus_DataSend(MbRxData3.Sin.Data_L);  
					break;
			}
		}
		else if(MbRxData3.Sin.Cmd==Cmd_WritSing)
		{
			switch(MbRxData3.Sin.RegAddr_L)  
			{					
				case REG_STNO:  
					Set.StNo=MbRxData3.Sin.Data_H<<8 | +MbRxData3.Sin.Data_L;	
					Modebus_Answer();   
					Flag_Save=SET;	
					break;        
				case REG_BTR:  
					Set.Btr=MbRxData3.Sin.Data_H<<8 | +MbRxData3.Sin.Data_L;	
					Modebus_Answer();   
					USART3_INIT();	
					Flag_Save=SET;	
					break;
				 
				case REG_ZERO1:  
					Set.Zero[0]=MbRxData3.Sin.Data_H<<8 | +MbRxData3.Sin.Data_L;
					if(Set.Zero[0]>1600)
						Set.Zero[0]=1600;
					ZeroOffset[0]=Set.Zero[0];				
					Modebus_Answer();   
					Flag_Save=SET;	
					break;
					
				case REG_ZERO2:  
					Set.Zero[1]=MbRxData3.Sin.Data_H<<8 | +MbRxData3.Sin.Data_L;
					if(Set.Zero[1]>1600)
						Set.Zero[1]=1600;
					ZeroOffset[1]=Set.Zero[1];				
					Modebus_Answer();   
					Flag_Save=SET;	
					break;
					
				case REG_ZERO3:  
					Set.Zero[2]=MbRxData3.Sin.Data_H<<8 | +MbRxData3.Sin.Data_L;
					if(Set.Zero[2]>1600)
						Set.Zero[2]=1600;
					ZeroOffset[2]=Set.Zero[2];				
					Modebus_Answer();   
					Flag_Save=SET;	
					break;
					
				case REG_ZERO4:  
					Set.Zero[3]=MbRxData3.Sin.Data_H<<8 | +MbRxData3.Sin.Data_L;
					if(Set.Zero[3]>1600)
						Set.Zero[3]=1600;
					ZeroOffset[3]=Set.Zero[3];				
					Modebus_Answer();   
					Flag_Save=SET;	
					break;
								
				default: 
					break;
			}
		
		}		
		else if(MbRxData3.Mul.Cmd==Cmd_WritMul)
		{ 
			
			if(MbRxData3.Sin.RegAddr_L>REG_STAT4)
				{	return;}	
			memcpy((u8*)&Reg.data[MbRxData3.Sin.RegAddr_L], (u8*)&MbRxData3.Mul.Data[0], MbRxData3.Mul.DatCnt); 
			RegToData();	
			Flag_NewCmd=SET;	
			Modebus_MulAnswer();   
		}					
	}		
} 

void USART3_Rx_IRQHandler(void)
{ 
	u16	i;

	if(USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)
	{
		DMA_Cmd(DMA1_Channel3, DISABLE);
		USART_ReceiveData(USART3);//返回USART3->DR数据寄存器的值			
		
		DATA_LEN=BUFF_LEN-DMA_GetCurrDataCounter(DMA1_Channel3); //DMA接收缓冲区长度
				
		if(DATA_LEN<128)
		{
			Flag_RxOk=SET;	
		}
		
		DMA_ClearFlag(DMA1_FLAG_GL3 | DMA1_FLAG_TC3 | DMA1_FLAG_TE3 | DMA1_FLAG_HT3);//清除传输完成标志
		DMA1_Channel3->CNDTR = BUFF_LEN;
		DMA_Cmd(DMA1_Channel3, ENABLE);
		
		i = USART3->SR;
		i = USART3->DR;

	}
	if(USART_GetITStatus(USART3, USART_IT_PE | USART_IT_FE | USART_IT_NE) != RESET)
	{
		USART_ClearITPendingBit(USART3, USART_IT_PE | USART_IT_FE | USART_IT_NE);
	}
	
	USART_ClearITPendingBit(USART3, USART_IT_IDLE);
}

void USART3_INIT(void)
{
	if(Set.Btr==0)
	{	USART3_Config(9600); }	
	else if(Set.Btr==1)
	{	USART3_Config(19200); }
	else if(Set.Btr==2)
	{	USART3_Config(38400); }
	else if(Set.Btr==3)
	{	USART3_Config(57600); }
	else
	{	USART3_Config(115200); }	
}
