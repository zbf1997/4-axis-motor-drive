#ifndef __STMFLASH_H__
#define __STMFLASH_H__
#include<stm32f10x.h>



#define STM32_FLASH_SIZE 128 
#define STM32_FLASH_WREN 1              
#define STM32_FLASH_BASE 0x08000000 	





extern	void STMFLASH_Read(u32 ReadAddr,u16 *pBuffer,u16 NumToRead);    
extern	void WriteSYSToFlash(void);
extern	void ReadSYSFromFlash(void);
extern	void	default_parm(void);	
#endif
