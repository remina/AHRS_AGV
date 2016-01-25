#include "stm32f10x.h"
#include "OSQMem.h"
#include "FLASH.h"
#include "sensors.h"
#include "magnav.h"
#include "string.h"


//
////u16 writedata[10]={0x1111,0x1112,0x1113,0x1114,0x1115,0x1116,0x1117,0x1118,0x1119,0x111A};
//u8 writedata[10]={0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18,0x19,0x1A};
u8 writeflashdata[16]={0};
//u8 readdata[10]={0};
union union_temp16
{
    unsigned int un_temp16;
    unsigned char  un_temp8[2];		// example 16: 0x0102  8:[0]2 [1]1
}my_unTemp16;
//
//
//#define	 FLASH_DATA	 0x5a5a		    /* 写入的数据 */
//
//
void FlashWriteWord( u32 flash_add, int data )
{
	uint32_t FlashData;
	FlashData=*(vu32*)(flash_add);	 /* 读取地址中的16位数据 */
	if(FlashData==0xffffffff)
	{
		FLASH_Unlock();		/* 每次擦除Flash中数据时得先解锁 */
		FLASH_ProgramWord(flash_add,data);   /* 写16位半字 */
		FLASH_Lock();							   /* 上锁 */
//		printf("\r\n要写入的地址为空,写入认证数据 \r\n");
	}
//	else if(FlashData==data)
//	{
////		printf("\r\n地址数据与认证数据符合 \r\n");
//	}
	else
	{
//		printf("\r\n地址上的数据与认证的数据不符合,有可能是写入失败或者是要写入的地址非空\r\n");
		FLASH_Unlock();
		FLASH_ErasePage(flash_add);		  /* 擦除页 */
		FLASH_ProgramWord(flash_add,data);   /* 写16位半字 */
		FLASH_Lock();
//		printf("\r\n已经刷除要写入的地址\r\n");
	}  

}

/******************************************************
flash 字符串写入
每次存入两个字节
*******************************************************/
void FlashWriteStr( u32 flash_add, u16 len, u8* data )
{	
	//char cp[12];
	//u8 s = 0;
	u16 byteN = 0;
	uint32_t FlashData;
	
	
	//FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_PGERR|FLASH_FLAG_WRPRTERR);
	//FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
	//FLASH_ErasePage(flash_add+FLASH_PAGE_SIZE * 1);
	//FLASH_ErasePage(flash_add);
	//sprintf( cp, "len:%d", len);
	//USART1_Puts(cp);
	while( len )
	{
		FlashData=*(vu32*)(flash_add+byteN);	 /* 读取地址中的16位数据 */
		my_unTemp16.un_temp8[0] = *(data+byteN);
		my_unTemp16.un_temp8[1] = *(data+byteN+1);
		//my_unTemp16.un_temp8[2] = *(data+byteN+2);
		//my_unTemp16.un_temp8[3] = *(data+byteN+3);
		//my_unTemp16.un_temp16=(*(data+byteN))|((*(data+byteN+1))<<8)|((*(data+byteN+2))<<16)|((*(data+byteN+3))<<24);
		if(FlashData==0xffffffff)
		{		
			FLASH_Unlock();
			FLASH_ProgramHalfWord(flash_add+byteN , my_unTemp16.un_temp16 );
			FLASH_Lock();
		}
//		else if(FlashData==my_unTemp16.un_temp16)
//		{  FLASH_COMPLETE
//		}
		else
		{
			FLASH_Unlock();
			FLASH_ErasePage(flash_add+byteN);		  /* 擦除页 */
			FLASH_ProgramHalfWord(flash_add+byteN , my_unTemp16.un_temp16 );
			FLASH_Lock();
		}

		//sprintf( cp, "bye:%d\r\n", s);
		//USART1_Puts(cp);
		if( 1==len )
		{
			//如果数据长度是奇数,为1的时候跳出
			break;													   
		}
		else
		{
			byteN += 2;
			len -= 2;
		}	
	}
	//FLASH_Lock();
}

/******************************************************
flash 字符串读出到指定data中  
地址与写入data地址同 读出的保存类型也必须一致
*******************************************************/
void FlashReadStr( u32 flash_add, u16 len, u8* data )
{
	u16 byteN = 0;
	while( len )
	{
		my_unTemp16.un_temp16 = *(vu16*)(flash_add+byteN);
		if( 1==len )
		{
			*(data+byteN) = my_unTemp16.un_temp8[0];
			break;			   
		}
		else
		{		
			*(data+byteN) = my_unTemp16.un_temp8[0];
			*(data+byteN+1) = my_unTemp16.un_temp8[1];
			byteN += 2;
			len -= 2;
		}
	}
}



#define FLASH_PAGE_SIZE    ((u16)0x400)
typedef enum {FAILED = 0, PASSED = !FAILED} FLASH_Test_Status;

u32 FLASH_write(vu32 startAddr, u32 *buf, u32 buf_size)
{
	volatile FLASH_Status FLASHStatus;
	u32 *bufStart;
	u32 EraseCounter = 0x00, Address = 0x00;
	vu32 NbrOfPage = 0x00;
	
	bufStart = buf;
	FLASHStatus = FLASH_COMPLETE;

	FLASH_Unlock();

	NbrOfPage = (buf_size + FLASH_PAGE_SIZE - 1) / FLASH_PAGE_SIZE;
	FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);	
	
	for(EraseCounter = 0; (EraseCounter < NbrOfPage) && (FLASHStatus == FLASH_COMPLETE); EraseCounter++)
	{
		FLASHStatus = FLASH_ErasePage(startAddr + (FLASH_PAGE_SIZE * EraseCounter));
	}
	
	Address = startAddr;
	while((Address < (startAddr + buf_size)) && (FLASHStatus == FLASH_COMPLETE))
	{
		FLASHStatus = FLASH_ProgramWord(Address, *buf++);
		Address += 4;
	}
	
	//检查写入数据是否正确
	Address = startAddr;
	while((Address < (startAddr + buf_size)))
	{
		if((*(vu32*) Address) != *bufStart++)
		{
			return FALSE;
		}
		Address += 4;
	}

	FLASH_Lock();
	
	return TRUE;
}

u8 FLASH_read(u32 *buf, vu32 startAddr, u32 buf_size)
{
	memcpy(buf, (u32 *)startAddr, buf_size);
	return TRUE;
}


void flash_init()
{
	//u8 dis_data[12];

	//FlashReadStr(FLASH_ADR, 12, dis_data);

	FlashReadStr(FLASH_ADR, 16, writeflashdata);
	//FlashReadStr(FLASH_ADR+12, 8, (u8*)g_pd);
	
	g_front_distance = (writeflashdata[1] << 8) | writeflashdata[0];
	g_mid_distance = (writeflashdata[3] << 8) | writeflashdata[2];
    g_back_distance = (writeflashdata[5] << 8) | writeflashdata[4];
	g_mid_distance_1 = (writeflashdata[7] << 8) | writeflashdata[6];

	pd[PID_FRONT].kp = writeflashdata[8]/1000.0;
	pd[PID_FRONT].kd = writeflashdata[9]/1000.0;
	pd[PID_BACK].kp = writeflashdata[10]/1000.0;
	pd[PID_BACK].kd = writeflashdata[11]/1000.0;
	pd[PID_LEFT].kp = -writeflashdata[12]/1000.0;
	pd[PID_LEFT].kd = -writeflashdata[13]/1000.0;
	pd[PID_RIGHT].kp = writeflashdata[14]/1000.0;
	pd[PID_RIGHT].kd = writeflashdata[15]/1000.0;
		
}
