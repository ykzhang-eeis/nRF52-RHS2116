/****************************************File Info************************************************
** File          name:  RHS2116.h
** Last modified Date:          
** Last       Version:		   
** Descriptions      :					
**----------------------------------------------------------------------------------------------------*/
#ifndef RHS_2116_H__
#define RHS_2116_H__

#include "nrf_drv_spi.h"

#include <stdint.h>

/*****************************************************************************/
//SPI���Ŷ���
#define  SPI_SS_PIN     18
#define  SPI_SCK_PIN    30
#define  SPI_MISO_PIN   28
#define  SPI_MOSI_PIN   29


#define    SPI_CS_LOW    nrf_gpio_pin_clear(SPI_SS_PIN)   //Ƭѡ����͵�ƽ��ʹ��оƬ
#define    SPI_CS_HIGH   nrf_gpio_pin_set(SPI_SS_PIN)     //Ƭѡ����ߵ�ƽ��ȡ��Ƭѡ


//W25Q128 ID
#define  W25Q128_ID     0XEF17




//SPI Flash�����
#define    SPIFlash_WriteEnable       0x06  //дʹ������
#define    SPIFlash_WriteDisable      0x04  //д��ֹ����
#define    SPIFlash_PageProgram       0x02  //ҳд������
#define    SPIFlash_ReadStatusReg		  0x05  //��״̬�Ĵ���1
#define    SPIFlash_WriteStatusReg		0x01  //д״̬�Ĵ���1
#define    SPIFlash_ReadData          0x03  //����������
#define    SPIFlash_SecErase          0x20  //��������
#define    SPIFlash_BlockErase        0xD8  //�����
#define    SPIFlash_ChipErase         0xC7  //ȫƬ����
#define    SPIFlash_ReadID            0x90  //��ȡID


#define    SPIFLASH_CMD_LENGTH        0x04
#define    SPIFLASH_WRITE_BUSYBIT     0x01



#define    SPIFlash_PAGE_SIZE        256
#define    SPIFlash_SECTOR_SIZE      (1024*4)
#define    SPI_TXRX_MAX_LEN          255

#define    FLASH_BLOCK_NUMBLE         7
#define    FLASH_PAGE_NUMBLE          8


void SPI_Flash_Init(void);
uint16_t SpiFlash_ReadID(void);


uint8_t SpiFlash_Write_Page(uint8_t *pBuffer, uint32_t WriteAddr, uint32_t size);
uint8_t SpiFlash_Read(uint8_t *pBuffer,uint32_t ReadAddr,uint32_t size);
void SPIFlash_Erase_Sector(uint32_t SecAddr);
void SPIFlash_Erase_Chip(void);
uint8_t SpiFlash_Write_Buf(uint8_t *pBuffer, uint32_t WriteAddr, uint32_t WriteBytesNum);
uint8_t RHS2116_verify(void);

//��ʼ��ADC CLEAR �C Set ADC calibration
uint8_t CMD_CLEAR_ADC(void);
uint8_t RHS2116_CONFIG(void);
uint8_t SPI_ReadID(void);
uint8_t CMD_START_CONV(uint8_t U,uint8_t M,uint8_t D,uint8_t H,uint8_t C,uint16_t* A,uint16_t* W);

#endif

























