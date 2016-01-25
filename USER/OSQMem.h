/******************** (C) COPYRIGHT 2012 HandyBot Team ********************
* 文件名             : OSQMem.h
* 作者               : Duxiaoshi
* 版本               : V1.0
* 日期               : 9/10/2012
* 描述               : 内存的管理函数
* 功能               ：内存的管理函数				   				   	
*******************************************************************************/
#ifndef _OSQMEM_H
#define _OSQMEM_H

#include "stm32f10x.h"

#define 	OS_MEM_MAX 				8	   			//最多允许的内存块管理区

typedef struct OSMEMTCB{
	void 		*OSMemFreeList;
	u8 			OSMemBlkSize;
	u8 			OSMemNBlks;
	u8 			OSMemFreeNBlks;
}OSMEMTcb;

OSMEMTcb *OSMemCreate(u8 *ptr,u8 blksize,u8 nblks,u8 *err);
u8 *OSMemGet(OSMEMTcb *ptr,u8 *err);
u8 OSMemDelete(OSMEMTcb *ptr,u8 *index);

#endif

