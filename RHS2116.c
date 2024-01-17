/****************************************File Info************************************************
** File          name:RHS2116.c
** Created by:			
** Created date:		2019-3-30
** Version:			    1.0
** Descriptions:		串口透传长包传输（最大传输长度244个字节）
**---------------------------------------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "nrf_delay.h"

#include "nrf_drv_spi.h"
#include "RHS2116.h"
#include "nrf_delay.h"


#define SPI_INSTANCE  0 
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  
//SPI传输完成标志
static volatile bool spi_xfer_done;  
//SPI发送缓存数组，使用EasyDMA时一定要定义为static类型
static uint8_t    spi_tx_buf[4];  
//SPI接收缓存数组，使用EasyDMA时一定要定义为static类型
static uint8_t    spi_rx_buf[4];  
static const uint8_t m_length = sizeof(spi_tx_buf);        /**< Transfer length. */

uint32_t RHS2116_RW_WORD(uint32_t dat_32)
{
		  
	uint8_t ret1,ret2,ret3,ret4;
	uint32_t ret_32=0;
	
	ret1=(uint8_t)((dat_32>>24)&0xff);
	ret2=(uint8_t)((dat_32>>16)&0xff);
	ret3=(uint8_t)((dat_32>>8)&0xff);
	ret4=(uint8_t)((dat_32)&0xff);
	uint8_t spi_tx_buf[]={ret1,ret2,ret3,ret4};
	spi_xfer_done = false;	
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, spi_tx_buf, m_length, spi_rx_buf, m_length));
	while (!spi_xfer_done);					
	ret_32=(spi_rx_buf[0]<<24)+(spi_rx_buf[1]<<16)+(spi_rx_buf[2]<<8)+(spi_rx_buf[3]);	
	return ret_32;
}


//写入寄存器 WRITE(R,D) – Write data D to register R
uint8_t CMD_WRITE_REG(uint8_t U,uint8_t M,uint8_t R,uint16_t D)
{
	uint32_t cmd_32=0x80000000;
	uint32_t ret_32;
	if(U!=0) cmd_32|=0x20000000;
	if(M!=0) cmd_32|=0x10000000;
	
	cmd_32|=((0x00ff0000)&(R<<16));
	cmd_32|=((0x0000ffff)&(D));
	
	SPI_CS_LOW;
	RHS2116_RW_WORD(cmd_32);
	SPI_CS_HIGH;
	
	SPI_CS_LOW;
	RHS2116_RW_WORD(0xffffffff);
	SPI_CS_HIGH;
	
	SPI_CS_LOW;
	ret_32=RHS2116_RW_WORD(0xffffffff);
	SPI_CS_HIGH;
	
	if((ret_32&0xffff0000)!=0xffff0000) 
	{return 255;}
	
	if((ret_32&0x0000ffff)!=D) 
	{return 255;}
	
	return 0;
}

//读取寄存器 READ(R) – Read contents of register R
uint8_t CMD_READ_REG(uint8_t U,uint8_t M,uint8_t R,uint16_t* D)
{
	uint32_t cmd_32=0xc0000000;
	uint32_t ret_32;
	if(U!=0) cmd_32|=0x20000000;
	if(M!=0) cmd_32|=0x10000000;
	
	cmd_32|=((0x00ff0000)&(R<<16));
	
	SPI_CS_LOW;
	RHS2116_RW_WORD(cmd_32);
	SPI_CS_HIGH;
	
	SPI_CS_LOW;
	RHS2116_RW_WORD(0xffffffff);
	SPI_CS_HIGH;
	
	SPI_CS_LOW;
	ret_32=RHS2116_RW_WORD(0xffffffff);
	SPI_CS_HIGH;
	
	if((ret_32&0xffff0000)!=0x00000000) 
	{return 255;}
	
	*D=(uint16_t)(ret_32&0x0000ffff);
	
	return 0;
}

//检测芯片信息
uint8_t RHS2116_verify(void)
{
	uint8_t ret;
	uint16_t val;
	ret=CMD_READ_REG(0,0,251,&val);
	
	if(ret!=0)
	{return 255;}
	
	if(((val>>8)&0xff)!='I')
	{return 255;}
	
	if(((val)&0xff)!='N')
	{return 255;}
	
	ret=CMD_READ_REG(0,0,252,&val);
	
	if(ret!=0)
	{return 255;}
	
	if(((val>>8)&0xff)!='T')
	{return 255;}
	
	if(((val)&0xff)!='A')
	{return 255;}
	
	ret=CMD_READ_REG(0,0,253,&val);
	
	if(ret!=0)
	{return 255;}
	
	if(((val>>8)&0xff)!='N')
	{return 255;}
	
	if(((val)&0xff)!=0x00)
	{return 255;}
	
	return 0;
}

//初始化ADC CLEAR – Set ADC calibration
uint8_t CMD_CLEAR_ADC(void)
{
	uint32_t cmd_32=0x6a000000;
	uint32_t ret_32;
	
	SPI_CS_LOW;
	RHS2116_RW_WORD(cmd_32);
	SPI_CS_HIGH;
	
	SPI_CS_LOW;
	RHS2116_RW_WORD(0xffffffff);
	SPI_CS_HIGH;
	
	SPI_CS_LOW;
	ret_32=RHS2116_RW_WORD(0xffffffff);
	SPI_CS_HIGH;
	
	if ((ret_32&0x7fffffff)==0) return 0;
	else return 0xff;
}

uint8_t RHS2116_CONFIG(void)
{
	uint8_t ret;
	uint16_t val;
	//初始化
	nrf_delay_ms(100);
	
	ret=CMD_READ_REG(0,0,255,&val);
	
	CMD_WRITE_REG(0,0,32,0x0000);
	CMD_WRITE_REG(0,0,33,0x0000);
	CMD_WRITE_REG(0,0,38,0xffff);
	
	ret=CMD_CLEAR_ADC();
	
	nrf_delay_ms(100);
	
	//ADC buffer bias=32 MUX bias=40
	//CMD_WRITE_REG(0,0,0,0x0828);
	
	CMD_WRITE_REG(0,0,0,0x00c7);
	CMD_WRITE_REG(0,0,1,0x051a);
	CMD_WRITE_REG(0,0,2,0x0040);
	CMD_WRITE_REG(0,0,3,0x0080);
	CMD_WRITE_REG(0,0,4,0x0016);
	CMD_WRITE_REG(0,0,5,0x0017);
	CMD_WRITE_REG(0,0,6,0x00a8);
	CMD_WRITE_REG(0,0,7,0x000a);
	CMD_WRITE_REG(0,0,8,0xffff);
	CMD_WRITE_REG(1,0,10,0x0000);
	CMD_WRITE_REG(1,0,12,0xffff);
	CMD_WRITE_REG(0,0,34,0x00e2);
	CMD_WRITE_REG(0,0,35,0x00aa);
	CMD_WRITE_REG(0,0,36,0x0080);
	CMD_WRITE_REG(0,0,37,0x4f00);
	CMD_WRITE_REG(1,0,42,0x0000);
	CMD_WRITE_REG(1,0,44,0x0000);
	CMD_WRITE_REG(1,0,46,0x0000);
	CMD_WRITE_REG(1,0,48,0x0000);
	
	//设置刺激电流负电流幅度
	CMD_WRITE_REG(1,0,64,0xff80);
	CMD_WRITE_REG(1,0,65,0xff80);
	CMD_WRITE_REG(1,0,66,0xff80);
	CMD_WRITE_REG(1,0,67,0xff80);
	CMD_WRITE_REG(1,0,68,0xff80);
	CMD_WRITE_REG(1,0,69,0xff80);
	CMD_WRITE_REG(1,0,70,0xff80);
	CMD_WRITE_REG(1,0,71,0xff80);
	CMD_WRITE_REG(1,0,72,0xff80);
	CMD_WRITE_REG(1,0,73,0xff80);
	CMD_WRITE_REG(1,0,74,0xff80);
	CMD_WRITE_REG(1,0,75,0xff80);
	CMD_WRITE_REG(1,0,76,0xff80);
	CMD_WRITE_REG(1,0,77,0xff80);
	CMD_WRITE_REG(1,0,78,0xff80);
	CMD_WRITE_REG(1,0,79,0xff80);
	
	//设置刺激电流正电流幅度
	CMD_WRITE_REG(1,0,96,0xff80);
	CMD_WRITE_REG(1,0,97,0xff80);
	CMD_WRITE_REG(1,0,98,0xff80);
	CMD_WRITE_REG(1,0,99,0xff80);
	CMD_WRITE_REG(1,0,100,0xff80);
	CMD_WRITE_REG(1,0,101,0xff80);
	CMD_WRITE_REG(1,0,102,0xff80);
	CMD_WRITE_REG(1,0,103,0xff80);
	CMD_WRITE_REG(1,0,104,0xff80);
	CMD_WRITE_REG(1,0,105,0xff80);
	CMD_WRITE_REG(1,0,106,0xff80);
	CMD_WRITE_REG(1,0,107,0xff80);
	CMD_WRITE_REG(1,0,108,0xff80);
	CMD_WRITE_REG(1,0,109,0xff80);
	CMD_WRITE_REG(1,0,110,0xff80);
	CMD_WRITE_REG(1,0,111,0xff80);
	
		//开启Stimulation
	CMD_WRITE_REG(0,0,32,0xaaaa);
	CMD_WRITE_REG(0,0,33,0x00ff);
	
	ret=CMD_READ_REG(0,1,255,&val);
	
	return 0;
}

uint8_t SPI_ReadID(void)
{
  uint8_t dat0 = 0;
	uint8_t dat1 = 0;
	uint8_t dat2 = 0;
	uint8_t dat3 = 0;
	//准备数据
	spi_tx_buf[0] = 0xC0;
	spi_tx_buf[1] = 0xFB;
	spi_tx_buf[2] = 0x00;
	spi_tx_buf[3] = 0x00;
	//拉低CS，使能
	SPI_CS_LOW;
	//启动数据传输
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, spi_tx_buf, 4, spi_rx_buf, 4));
	while (!spi_xfer_done)
        {
            __WFE();
        }
  //拉高CS
	SPI_CS_HIGH;
	dat0 |= spi_rx_buf[0];
	dat1 |= spi_rx_buf[1];
	dat2 |= spi_rx_buf[2];	
	dat3 |= spi_rx_buf[3];
  return dat3;		
}

//启动转换 返回结果 CONVERT(C) – Run analog-to-digital conversion on channel C
uint8_t CMD_START_CONV(uint8_t U,uint8_t M,uint8_t D,uint8_t H,uint8_t C,uint16_t* A,uint16_t* W)
{
	uint32_t cmd_32=0x00000000;
	uint32_t ret_32;
	if(U!=0) cmd_32|=0x20000000;
	if(M!=0) cmd_32|=0x10000000;
	if(D!=0) cmd_32|=0x08000000;
	if(H!=0) cmd_32|=0x04000000;
	
	cmd_32|=((0x003f0000)&(C<<16));
	
	SPI_CS_LOW;
	RHS2116_RW_WORD(cmd_32);
	SPI_CS_HIGH;
	
	SPI_CS_LOW;
	RHS2116_RW_WORD(0xffffffff);
	SPI_CS_HIGH;
	
	SPI_CS_LOW;
	ret_32=RHS2116_RW_WORD(0xffffffff);
	SPI_CS_HIGH;
	
	*A=(uint16_t)((ret_32&0xffff0000)>>16);
	*W=(uint16_t)(ret_32&0x000003ff);
	
	
	return 0;
}

/********************************************END FILE*******************************************/

