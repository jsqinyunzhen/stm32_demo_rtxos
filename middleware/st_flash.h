
/****************************************************************************************
* ��Ȩ���� (C)2015,���ݺ����������ּ����ɷ����޹�˾��
* �ļ�����  : main.c
* ����ժҪ  : main file
* ��    ��  : xushixiong
* ��������  : 2016/06/08
****************************************************************************************/

/* ����ͷ�ļ� *****************************************************************/


#include <stdarg.h>  
#include <string.h>  
#include <stdlib.h>  
#include <stdio.h>  
#include <ctype.h>




#ifndef __ST__FLASH__
#define __ST__FLASH__

//#define BufferSize     10
/* ���� ----------------------------------------------------------------------*/
//static __IO uint32_t TimingDelay;

uint16_t IAP_ReadFlag(void);
void IAP_WriteFlag(uint16_t flag);




#endif

