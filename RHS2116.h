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
//SPI引脚定义
#define  SPI_SS_PIN     18
#define  SPI_SCK_PIN    30
#define  SPI_MISO_PIN   28
#define  SPI_MOSI_PIN   29


#define    SPI_CS_LOW    nrf_gpio_pin_clear(SPI_SS_PIN)   //片选输出低电平：使能芯片
#define    SPI_CS_HIGH   nrf_gpio_pin_set(SPI_SS_PIN)     //片选输出高电平：取消片选


//W25Q128 ID
#define  W25Q128_ID     0XEF17




//SPI Flash命令定义
#define    SPIFlash_WriteEnable       0x06  //写使能命令
#define    SPIFlash_WriteDisable      0x04  //写禁止命令
#define    SPIFlash_PageProgram       0x02  //页写入命令
#define    SPIFlash_ReadStatusReg		  0x05  //读状态寄存器1
#define    SPIFlash_WriteStatusReg		0x01  //写状态寄存器1
#define    SPIFlash_ReadData          0x03  //读数据命令
#define    SPIFlash_SecErase          0x20  //扇区擦除
#define    SPIFlash_BlockErase        0xD8  //块擦除
#define    SPIFlash_ChipErase         0xC7  //全片擦除
#define    SPIFlash_ReadID            0x90  //读取ID


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

//初始化ADC CLEAR – Set ADC calibration
uint8_t CMD_CLEAR_ADC(void);
uint8_t RHS2116_CONFIG(void);
uint8_t SPI_ReadID(void);
uint8_t CMD_START_CONV(uint8_t U,uint8_t M,uint8_t D,uint8_t H,uint8_t C,uint16_t* A,uint16_t* W);

#endif

























