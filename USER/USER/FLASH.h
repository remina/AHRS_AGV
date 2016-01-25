
#ifndef _FLASH_H_
#define _FLASH_H_

#include "stm32f10x.h"

#define	FLASH_ADR	0x0801F010		//要写入数据的地址 0x8002000
//#define	FLASH_ADR_1	0x0801F010		//要写入数据的地址 0x8002000
//#define	FLASH_ADR_2	0x0801F020
//#define FLASH_PAGE_SIZE    ((u16)0x800) //2048   2K
extern void FlashWriteStr( u32 flash_add, u16 len, u8* data );
extern void FlashReadStr( u32 flash_add, u16 len, u8* data );
extern void FlashWriteWord( u32 flash_add, int data );
extern u8 writeflashdata[16];

extern u32 FLASH_write(vu32 startAddr, u32 *buf, u32 buf_size);
extern u8 FLASH_read(u32 *buf, vu32 startAddr, u32 buf_size);

#endif
