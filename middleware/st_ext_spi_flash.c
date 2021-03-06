/**
  ******************************************************************************
  * @file    SPI/SPI_FLASH/main.c
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "project_config.h"

#include "stm32_eval.h"
#include "stm32_eval_spi_flash.h"
#include "st_printf.h"
#include "st_ext_spi_flash.h"

//debug
#if 1

/** @addtogroup STM32F10x_StdPeriph_Examples
  * @{
  */

/** @addtogroup SPI_FLASH
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
//typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;

/* Private define ------------------------------------------------------------*/




/* Private macro -------------------------------------------------------------*/
//#define countof(a) (sizeof(a) / sizeof(*(a)))

/* Private variables ---------------------------------------------------------*/
uint8_t Tx_Buffer[] = "STM32F10x SPI Firmware Library Example: communication with an M25P SPI FLASH";
#define  BufferSize (countof(Tx_Buffer)-1)

uint8_t  Rx_Buffer[BufferSize];
__IO uint8_t Index = 0x0;
volatile TestStatus TransferStatus1 = FAILED, TransferStatus2 = PASSED;

TestStatus Buffercmp_b8(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength);


__IO uint32_t FlashID = 0;

/* Private functions ---------------------------------------------------------*/


/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
#if 1
const osMutexAttr_t os_mutex_def_mutex_ext_flash = {
    "mutex_ext_flash",                          // human readable mutex name
    osMutexRecursive | osMutexPrioInherit,    // attr_bits
    NULL,                                     // memory for control block
    0U                                        // size for control block
};
#else

osMutexDef (mutex_uart);
#endif
osMutexId_t mutex_ext_flash_id;

void ext_flash_os_lock_init(void)
{
    //mutex_ext_flash_id = CreateMutex(osMutex(mutex_ext_flash));
    mutex_ext_flash_id = CreateMutex(&os_mutex_def_mutex_ext_flash);
    if(mutex_ext_flash_id)
    {
        printf("CreateMutex  mutex_ext_flash_id=0x%08x\r\n",(unsigned int)mutex_ext_flash_id);
        printf("osMutexGetName name =%s\r\n",osMutexGetName (mutex_ext_flash_id)?osMutexGetName (mutex_ext_flash_id):"name is null");
        printf("osMutexGetOwner Owner ==0x%08x\r\n",(unsigned int)osMutexGetOwner (mutex_ext_flash_id));

    }
    else
    {
        printf("CreateMutex mutex_uart fail\r\n");
        while(1);


    }
}
#if 1
void ext_flash_erase(uint32_t SectorAddr)
{
   // AcquireMutex(mutex_ext_flash_id);
    sFLASH_EraseSector(SectorAddr);
   // ReleaseMutex(mutex_ext_flash_id);
}
void ext_flash_read(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead)
{
   // AcquireMutex(mutex_ext_flash_id);

    sFLASH_ReadBuffer(pBuffer, ReadAddr, NumByteToRead);

   // ReleaseMutex(mutex_ext_flash_id);
}
void ext_flash_write(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead)
{
   // AcquireMutex(mutex_ext_flash_id);
    sFLASH_WriteBuffer(pBuffer, ReadAddr, NumByteToRead);
   // ReleaseMutex(mutex_ext_flash_id);
}
#endif
int main_ext_spi_flash(void)
{

    int i = 0;
    ext_flash_os_lock_init();

    /* Initialize the SPI FLASH driver */
    sFLASH_Init();

    /* Get SPI Flash ID */
    FlashID = sFLASH_ReadID();
    //  u8 fac_id= 0;//BFH: 工程码SST
    //u8 dev_id= 0;//41H: 器件型号SST25VF016B

    //printf("fac_id=0x%x,dev_id=0x%x",fac_id,dev_id);
    printf("FlashID=0x%08x\r\n",FlashID);

    /* Check the SPI Flash ID */
    if (FlashID == sFLASH_WB25Q64JV_ID)
    {
        /* OK: Turn on LD1 */
        // STM_EVAL_LEDOn(LED1);
        printf("FLASHis WB25Q64JV\r\n");
    }
    else if (FlashID == sFLASH_GD25Q64C_ID)
    {
        /* OK: Turn on LD1 */
        // STM_EVAL_LEDOn(LED1);
        printf("FLASHis gd24q64c\r\n");
    }
    else
    {
        printf("FLASH ID=0x%x ,type not support \r\n",FlashID);
    }


#if (FLASH_VALID_JUDGE ==1 )
    if (FlashID != sFLASH_ID)
    {
        printf("FLASH ID=0x%x ,cur board not support \r\n",FlashID);
        while (1);
    }
#endif

    //return 0;
    //#else
#if 1

    /* Check the SPI Flash ID */
    //if (FlashID == sFLASH_ID)
    {
        /* OK: Turn on LD1 */
        // STM_EVAL_LEDOn(LED1);
        //	printf("FLASHis gd24q64c\r\n");
        printf("FLASH_SectorToErase=0x%08x\r\n",FLASH_SectorToErase);
        /* Perform a write in the Flash followed by a read of the written data */
        /* Erase SPI FLASH Sector to write on */
        ext_flash_erase(FLASH_SectorToErase);
#if 1
        /* Read data from SPI FLASH memory */
        printf("sFLASH_ReadBuffer=0x%08x\r\n",FLASH_ReadAddress);
        ext_flash_read(Rx_Buffer, FLASH_ReadAddress, BufferSize);
        for(i=0; i<BufferSize; i++)
            printf("Rx_Buffer[%d]=0x%02x\r\n",i,Rx_Buffer[i]);
#endif
        /* Write Tx_Buffer data to SPI FLASH memory */
        printf("sFLASH_WriteBuffer=0x%08x\r\n",FLASH_WriteAddress);
        ext_flash_write(Tx_Buffer, FLASH_WriteAddress, BufferSize);

        /* Read data from SPI FLASH memory */
        printf("sFLASH_ReadBuffer=0x%08x Rx_Buffer=0x%p Rx_Buffer=0x%x\r\n",FLASH_ReadAddress,Rx_Buffer,&Rx_Buffer);
        ext_flash_read(Rx_Buffer, FLASH_ReadAddress, BufferSize);

        dump_memeory((uint32_t)Tx_Buffer,BufferSize);
        dump_memeory((uint32_t)Rx_Buffer,BufferSize);
        /* Check the correctness of written dada */
        TransferStatus1 = Buffercmp_b8(Tx_Buffer, Rx_Buffer, BufferSize);
        /* TransferStatus1 = PASSED, if the transmitted and received data by SPI1
           are the same */
        /* TransferStatus1 = FAILED, if the transmitted and received data by SPI1
           are different */
        if(TransferStatus1 == PASSED)
        {
            printf("w r test ok\r\n");
        }
        else
            printf("w r test fail\r\n");
        /* Perform an erase in the Flash followed by a read of the written data */
        /* Erase SPI FLASH Sector to write on */
        ext_flash_erase(FLASH_SectorToErase);

        /* Read data from SPI FLASH memory */
        ext_flash_read(Rx_Buffer, FLASH_ReadAddress, BufferSize);
        TransferStatus1 = PASSED;
        /* Check the correctness of erasing operation dada */
        for (Index = 0; Index < BufferSize; Index++)
        {
            if (Rx_Buffer[Index] != 0xFF)
            {
                TransferStatus2 = FAILED;
                printf("Rx_Buffer[%d]l=0x%x",Index,Rx_Buffer[Index]);
                break;
            }
        }
        if(TransferStatus1 == FAILED)
        {
            printf("e test fail");
        }
        else
            printf("e test ok");
        /* TransferStatus2 = PASSED, if the specified sector part is erased */
        /* TransferStatus2 = FAILED, if the specified sector part is not well erased */
    }


#else
    /* Check the SPI Flash ID */
    //if (FlashID == sFLASH_ID)
    {
        /* OK: Turn on LD1 */
        // STM_EVAL_LEDOn(LED1);
        //	printf("FLASHis gd24q64c\r\n");
        printf("FLASH_SectorToErase=0x%08x\r\n",FLASH_SectorToErase);
        /* Perform a write in the Flash followed by a read of the written data */
        /* Erase SPI FLASH Sector to write on */
        sFLASH_EraseSector(FLASH_SectorToErase);
#if 1
        /* Read data from SPI FLASH memory */
        printf("sFLASH_ReadBuffer=0x%08x\r\n",FLASH_ReadAddress);
        sFLASH_ReadBuffer(Rx_Buffer, FLASH_ReadAddress, BufferSize);
        for(i=0; i<BufferSize; i++)
            printf("Rx_Buffer[%d]=0x%02x\r\n",i,Rx_Buffer[i]);
#endif
        /* Write Tx_Buffer data to SPI FLASH memory */
        printf("sFLASH_WriteBuffer=0x%08x\r\n",FLASH_WriteAddress);
        sFLASH_WriteBuffer(Tx_Buffer, FLASH_WriteAddress, BufferSize);

        /* Read data from SPI FLASH memory */
        printf("sFLASH_ReadBuffer=0x%08x Rx_Buffer=0x%p Rx_Buffer=0x%x\r\n",FLASH_ReadAddress,Rx_Buffer,&Rx_Buffer);
        sFLASH_ReadBuffer(Rx_Buffer, FLASH_ReadAddress, BufferSize);

        dump_memeory((uint32_t)Tx_Buffer,BufferSize);
        dump_memeory((uint32_t)Rx_Buffer,BufferSize);
        /* Check the correctness of written dada */
        TransferStatus1 = Buffercmp_b8(Tx_Buffer, Rx_Buffer, BufferSize);
        /* TransferStatus1 = PASSED, if the transmitted and received data by SPI1
           are the same */
        /* TransferStatus1 = FAILED, if the transmitted and received data by SPI1
           are different */
        if(TransferStatus1 == PASSED)
        {
            printf("w r test ok\r\n");
        }
        else
            printf("w r test fail\r\n");
        /* Perform an erase in the Flash followed by a read of the written data */
        /* Erase SPI FLASH Sector to write on */
        sFLASH_EraseSector(FLASH_SectorToErase);

        /* Read data from SPI FLASH memory */
        sFLASH_ReadBuffer(Rx_Buffer, FLASH_ReadAddress, BufferSize);
        TransferStatus1 = PASSED;
        /* Check the correctness of erasing operation dada */
        for (Index = 0; Index < BufferSize; Index++)
        {
            if (Rx_Buffer[Index] != 0xFF)
            {
                TransferStatus2 = FAILED;
                printf("Rx_Buffer[%d]l=0x%x",Index,Rx_Buffer[Index]);
                break;
            }
        }
        if(TransferStatus1 == FAILED)
        {
            printf("e test fail");
        }
        else
            printf("e test ok");
        /* TransferStatus2 = PASSED, if the specified sector part is erased */
        /* TransferStatus2 = FAILED, if the specified sector part is not well erased */
    }
#endif
    return 0;
}

/**
  * @brief  Compares two buffers.
  * @param  pBuffer1, pBuffer2: buffers to be compared.
  * @param  BufferLength: buffer's length
  * @retval PASSED: pBuffer1 identical to pBuffer2
  *         FAILED: pBuffer1 differs from pBuffer2
  */
TestStatus Buffercmp_b8(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength)
{
    int i =0;
    while (BufferLength--)
    {
        if (*pBuffer1 != *pBuffer2)
        {
            printf("i=%d t:0x%x != r:0x%x\r\n",i,*pBuffer1,*pBuffer2);
            return FAILED;
        }
        i++;
        pBuffer1++;
        pBuffer2++;
    }

    return PASSED;
}
#endif
//uint32_t Tx_Buffer[256];
//uint32_t Rx_Buffer[256];
#if(SPI_FLASH_OP_MODE ==SPI_FLASH_DMA)

static volatile int g_rx_flag = 0;
static volatile int g_tx_flag = 0;
static void spi_wait_rx(void)
{
    while(!g_rx_flag);
    g_rx_flag = 0;
}
void spi_post_rx(void)
{
    g_rx_flag = 1;
}
static void spi_wait_tx(void)
{
    while(!g_tx_flag);
    g_tx_flag = 0;
}
void spi_post_tx(void)
{
    g_tx_flag = 1;
}


void spi1_tx_rx_DMA_Config(unsigned char isread,uint8_t* pBuffer,  uint16_t NumByte)
{
    DMA_InitTypeDef DMA_InitStructure;
    uint32_t temp = 0;//0xffffffff;
    /*开启时钟*/
    //RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    //if(spi_tx_rx == 0)
    {
        //rx
        //DMA_Cmd (DMA1_Channel2,DISABLE);
        DMA_DeInit(DMA1_Channel2);


        DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&SPI1->DR;
        // DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)pBuffer;//Rx_Buffer;
        DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
        DMA_InitStructure.DMA_BufferSize = NumByte;//256;
        DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        if (isread == 1)
        {
            /* 读方式下回读数据放到 buffer 中*/
            DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)pBuffer;
            printf("DMA_InitStructure.DMA_MemoryBaseAddr=0x%08x\r\n",DMA_InitStructure.DMA_MemoryBaseAddr);
            DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
        }
        else
        {   /* 写方式下回读数据丢弃 */
            DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&temp;
            DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
        }

        //DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
        DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
        DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
        DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
        DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
        DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
        DMA_Init(DMA1_Channel2, &DMA_InitStructure);

    }
    //else
    {
        //tx
        //	DMA_Cmd (DMA1_Channel3,DISABLE);
        DMA_DeInit(DMA1_Channel3);
        DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&SPI1->DR;
        // DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)pBuffer;//Tx_Buffer;
        DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
        DMA_InitStructure.DMA_BufferSize = NumByte;//256;
        DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        if (isread == 1)
        {   /* 读方式下发送 0xff */
            DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&temp;
            DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
        }
        else
        {
            /* 写方式下发送 buffer 数据*/
            DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)pBuffer;
            DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
        }
        //DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
        DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
        DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
        DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
        DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
        DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
        DMA_Init(DMA1_Channel3, &DMA_InitStructure);
    }

    /* 1.开启 DMA 数据流传输完成中断 */
#if(SPI_FLASH_DMA_RX_TX_INT ==1)
    DMA_ITConfig(DMA1_Channel2, DMA_IT_TC, ENABLE);
    DMA_ITConfig(DMA1_Channel3, DMA_IT_TC, ENABLE);
#endif
    DMA_Cmd (DMA1_Channel2,ENABLE);
    DMA_Cmd (DMA1_Channel3,ENABLE);

}

#if 0
void sFLASH_WritePage(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
    /*!< Enable the write access to the FLASH */
    sFLASH_WriteEnable();

    /*!< Select the FLASH: Chip Select low */
    sFLASH_CS_LOW();
    /*!< Send "Write to Memory " instruction */
    sFLASH_SendByte(sFLASH_CMD_WRITE);
    /*!< Send WriteAddr high nibble address byte to write to */
    sFLASH_SendByte((WriteAddr & 0xFF0000) >> 16);
    /*!< Send WriteAddr medium nibble address byte to write to */
    sFLASH_SendByte((WriteAddr & 0xFF00) >> 8);
    /*!< Send WriteAddr low nibble address byte to write to */
    sFLASH_SendByte(WriteAddr & 0xFF);

    /*!< while there is data to be written on the FLASH */
    while (NumByteToWrite--)
    {
        /*!< Send the current byte */
        sFLASH_SendByte(*pBuffer);
        /*!< Point on the next byte to be written */
        pBuffer++;
    }

    /*!< Deselect the FLASH: Chip Select high */
    sFLASH_CS_HIGH();

    /*!< Wait the end of Flash writing */
    sFLASH_WaitForWriteEnd();
}

#endif


void SPI_DMA_PageWrite(u32 WriteAddr)
{
    /* Enable the write access to the FLASH */
// SPI_FLASH_WriteEnable();
    sFLASH_WriteEnable();

    /* Select the FLASH: Chip Select low */
    sFLASH_CS_LOW();
    /* Send "Write to Memory " instruction */
    //SPI_FLASH_SendByte(W25X_PageProgram);
    sFLASH_SendByte(sFLASH_CMD_WRITE);
    /* Send WriteAddr high nibble address byte to write to */
    sFLASH_SendByte((WriteAddr & 0xFF0000) >> 16);
    /* Send WriteAddr medium nibble address byte to write to */
    sFLASH_SendByte((WriteAddr & 0xFF00) >> 8);
    /* Send WriteAddr low nibble address byte to write to */
    sFLASH_SendByte(WriteAddr & 0xFF);

#if(SPI_FLASH_DMA_RX_TX_INT ==1)

    /* 3.使能 SPI DMA 请求 */
    SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx|SPI_I2S_DMAReq_Tx, ENABLE);
    /* 5.等待 DMA 传输完成 */
    spi_wait_tx();
    spi_wait_rx();

#else
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_Init(SPI1, &SPI_InitStructure);

    SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, ENABLE);

    while(DMA_GetFlagStatus(DMA1_FLAG_TC3) == RESET);
#endif
    /* Deselect the FLASH: Chip Select high */
    sFLASH_CS_HIGH();

}


void SPI_DMA_BufferRead(u32 ReadAddr)
{
    /* Select the FLASH: Chip Select low */
    sFLASH_CS_LOW();


    /* Send "Read from Memory " instruction */
    sFLASH_SendByte(sFLASH_CMD_READ);


    /* Send ReadAddr high nibble address byte to read from */
    sFLASH_SendByte((ReadAddr & 0xFF0000) >> 16);
    /* Send ReadAddr medium nibble address byte to read from */
    sFLASH_SendByte((ReadAddr& 0xFF00) >> 8);
    /* Send ReadAddr low nibble address byte to read from */
    sFLASH_SendByte(ReadAddr & 0xFF);

#if(SPI_FLASH_DMA_RX_TX_INT ==1)

    /* 3.使能 SPI DMA 请求 */
    SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx|SPI_I2S_DMAReq_Tx, ENABLE);
    /* 5.等待 DMA 传输完成 */
    spi_wait_tx();
    spi_wait_rx();
#else
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_RxOnly;
    SPI_Init(SPI1, &SPI_InitStructure);

    //SPI_FLASH_SendByte(0xff);
    /* Get Current Data Counter value after complete transfer */
    printf("DMA1_Channel2=%d\r\n",DMA_GetCurrDataCounter(DMA1_Channel2));

    SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx, ENABLE);
    while(DMA_GetFlagStatus(DMA1_FLAG_TC2) == RESET);
#endif
    /* Deselect the FLASH: Chip Select high */
    sFLASH_CS_HIGH();
}
void sFLASH_WritePage(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
    spi1_tx_rx_DMA_Config(0,pBuffer,NumByteToWrite);
    SPI_DMA_PageWrite(WriteAddr);
}

void sFLASH_ReadBuffer(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead)
{
    spi1_tx_rx_DMA_Config(1,pBuffer,NumByteToRead);
    SPI_DMA_BufferRead(ReadAddr);
}
#endif

