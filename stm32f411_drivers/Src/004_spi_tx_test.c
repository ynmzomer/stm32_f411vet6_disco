/*
 * File: 004_spi_tx_test.c
 * Description: STM32F411 driver test application.
 * Purpose: This file contains code to test custom driver implementations for GPIO, SPI, I2C, and UART.
 * Author: Betül Atalay
 */

/*
 * 004_spi_tx_test.c
 *
 *  Created on: Jan 31, 2025
 *      Author: omery
 */

/*mosı pb15
 * mıso pb14
 * sck  pb13
 * nss     pb12    AF5  */


#include "stm32f411xx.h"
#include <string.h>

void SPI2_GPIOInits(void) ;
void SPI2_Inits(void) ;

int main(void){
	char user_data[] = "Hello World" ;
	SPI2_GPIOInits() ;
	SPI2_Inits() ;
	SPI_SSIConfig(SPI2, ENABLE);
	SPI_PeripheralControl(SPI2, ENABLE);

	SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data)) ;
	while(SPI_GetFlagStatus(SPI2,SPI_BUSY_FLAG));
	SPI_PeripheralControl(SPI2, DISABLE);
	while(1);

	return 0 ;

}




void SPI2_GPIOInits(void){


	GPIO_Handle_t SPIPins ;

	SPIPins.pGPIOx = GPIOB ;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN ;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5 ;

	SPIPins.GPIO_PinConfig.GPIO_PinOPtype = GPIO_OPTYPE_PP ;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD ;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST ;

	//SCK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = 13 ;
	GPIO_Init(&SPIPins);
	//MISO
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = 14 ;
	//GPIO_Init(&SPIPins);
	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = 15 ;
	GPIO_Init(&SPIPins);
	//NSS
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = 12 ;
	//GPIO_Init(&SPIPins);

}
void SPI2_Inits(void){
	SPI_Handle_t SPI2handle;

	SPI2handle.pSPIx = SPI2 ;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD ;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER ;
	SPI2handle.SPIConfig.SPI_SclkSpeed =  SPI_SCLK_SPEED_DIV2 ;
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW ;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW ;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_EN ; //software manegenmet enabled for nss pin

	SPI_Init(&SPI2handle);


}

