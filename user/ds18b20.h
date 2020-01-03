#ifndef __DS18B20_H
#define __DS18B20_H

#include "stm32f10x.h"
//#include "bsp_SysTick.h" //��ȷ��ʱ����ͷ�ļ�----�ο�http://blog.csdn.net/xuxuechen/article/details/40783209�����һ��
#include "stm32f10x_it.h"

#define HIGH 1
#define LOW 0
#if 0
#define DS18B20_CLK    RCC_APB2Periph_GPIOA
#define DS18B20_PIN    GPIO_Pin_8
#define DS18B20_PORT   GPIOA
//�������DS18B20��GPIO��ΪPB9

#else
#define DS18B20_CLK    RCC_APB2Periph_GPIOB
#define DS18B20_PIN    GPIO_Pin_9
#define DS18B20_PORT   GPIOB
//�������DS18B20��GPIO��ΪPB9
#endif

//���κ꣬��������������һ��ʹ��,����ߵ�ƽ��͵�ƽ
#define DS18B20_DATA_OUT(a)if (a) \
		GPIO_SetBits(DS18B20_PORT,DS18B20_PIN);\
		else \
		GPIO_ResetBits(DS18B20_PORT,DS18B20_PIN)
//��ȡ���ŵĵ�ƽ
#define DS18B20_DATA_IN() GPIO_ReadInputDataBit(DS18B20_PORT,DS18B20_PIN)

uint8_t DS18B20_Init(void);
float DS18B20_Get_Temp(uint8_t *a,uint8_t b);
//float DS18B20_Get_Temp(uint8_t *a,uint16_t go_temp,uint8_t b)
extern uint8_t serial_4[8];
void read_serial(uint8_t *serial);
#endif /* __DS18B20_H */

