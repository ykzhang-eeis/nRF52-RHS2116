/****************************************File Info************************************************
** File          name:  RHS2116.h
** Last modified Date:          
** Last       Version:		   
** Descriptions      :					
**----------------------------------------------------------------------------------------------------*/
#ifndef RHS_2116_H__
#define RHS_2116_H__

#include "nrf_drv_spi.h"
#include "nrf_gpio.h"

#include <stdint.h>

/*****************************************************************************/
//SPI���Ŷ���
#define  SPI_SS_PIN     18
#define  SPI_SCK_PIN    30
#define  SPI_MISO_PIN   28
#define  SPI_MOSI_PIN   29

#define    SPI_CS_LOW    nrf_gpio_pin_clear(SPI_SS_PIN)   //Ƭѡ����͵�ƽ��ʹ��оƬ
#define    SPI_CS_HIGH   nrf_gpio_pin_set(SPI_SS_PIN)     //Ƭѡ����ߵ�ƽ��ȡ��Ƭѡ

uint8_t RHS2116_verify(void);
//��ʼ��ADC CLEAR �C Set ADC calibration
void CMD_CLEAR_ADC(void);
uint8_t SPI_ReadID(void);
uint8_t CMD_START_CONV(uint8_t U,uint8_t M,uint8_t D,uint8_t H,uint8_t C,uint16_t* A,uint16_t* W);

#endif

























