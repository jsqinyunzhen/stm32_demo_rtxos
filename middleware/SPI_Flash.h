#ifndef _SPI_FLASH_H_
#define _SPI_FLASH_H_

#include <stdio.h>
#define Dummy_Byte 0

/* Select SPI FLASH: ChipSelect pin low  */
#define Select_Flash()     GPIO_ResetBits(GPIOC, GPIO_Pin_4)
/* Deselect SPI FLASH: ChipSelect pin high */
#define NotSelect_Flash()    GPIO_SetBits(GPIOC, GPIO_Pin_4)



void SPI_Flash_Init(void);	        //SPI��ʼ��
uint8_t SPI_Flash_ReadByte(void);		//flash����������������һ���ֽ�
uint8_t SPI_Flash_SendByte(uint8_t byte);		//	FLASH������������������һ���ֽ�


void FlashWaitBusy(void);			//Flashæ���
void FlashReadID(void);		        //��ȡflashID�ĸ��ֽ�


void st_spi_flash_test(void);

#endif
