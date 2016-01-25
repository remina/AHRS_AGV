/******************** (C) COPYRIGHT 2012 HandyBot Team ********************
* �ļ���             : OSQMem.h
* ����               : Duxiaoshi
* �汾               : V1.0
* ����               : 9/10/2012
* ����               : �ڴ�Ĺ�����
* ����               ���ڴ�Ĺ�����				   				   	
*******************************************************************************/
#ifndef _OSQMEM_H
#define _OSQMEM_H

#include "stm32f10x.h"

#define 	OS_MEM_MAX 				8	   			//���������ڴ�������

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

