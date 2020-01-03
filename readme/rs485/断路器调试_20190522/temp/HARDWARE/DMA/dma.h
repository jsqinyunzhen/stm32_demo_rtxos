#ifndef __DMA_H
#define	__DMA_H	   
#include "sys.h"


void DMA1_Config_ADC1(DMA_Channel_TypeDef*DMA_CHx,u32 cpar,u32 cmar,u16 cndtr);//≈‰÷√DMA1_Channel1

void DMA_ReadADC_Enable(DMA_Channel_TypeDef*DMA_CHx);// πƒ‹DMA1_Channel1
		   
#endif




