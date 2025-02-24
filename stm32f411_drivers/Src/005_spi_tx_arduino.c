/*
 * File: 005_spi_tx_arduino.c
 * Description: STM32F411 driver test application.
 * Purpose: This file contains code to test custom driver implementations for GPIO, SPI, I2C, and UART.
 * Author: Betül Atalay
 */

/*
 * 005_spi_tx_arduino.c
 *
 *  Created on: Feb 1, 2025
 *      Author: omery
 */


#include "stm32f411xx.h"
#include <string.h>

void SPI2_GPIOInits(void) ;
void SPI2_Inits(void) ;
void button_init(void) ;
void delay(void) ;

int main(void)
{
	char user_data[] = "An Arduino Uno board is best suited for beginners who have just started using microcontrollers, on the other hand, Arduino Mega board is for enthusiasts who require a lot of I/O pins for their projects";

	button_init();

	//this function is used to initialize the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInits();

	//This function is used to initialize the SPI2 peripheral parameters
	SPI2_Inits();

	/*
	* making SSOE 1 does NSS output enable.
	* The NSS pin is automatically managed by the hardware.
	* i.e when SPE=1 , NSS will be pulled to low
	* and NSS pin will be high when SPE=0
	*/
	SPI_SSOEConfig(SPI2,ENABLE);

	while(1)
	{
		//wait till button is pressed
		while( ! GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0) );

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		//enable the SPI2 peripheral
		SPI_PeripheralControl(SPI2,ENABLE);

		//first send length information
		uint8_t dataLen = strlen(user_data);


		SPI_SendData(SPI2,&dataLen,1);

		//to send data
		SPI_SendData(SPI2,(uint8_t*)user_data,strlen(user_data));

		//lets confirm SPI is not busy
		while( SPI_GetFlagStatus(SPI2,SPI_BUSY_FLAG) );

		//Disable the SPI2 peripheral
		SPI_PeripheralControl(SPI2,DISABLE);
	}

	return 0;

}

void delay(void){
	for (volatile uint32_t i = 0; i < 800000; i++);
}

void button_init(void){
	GPIO_Handle_t GpioButton ;
	GpioButton.pGPIOx = GPIOA ;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0 ;
	GpioButton.GPIO_PinConfig.GPIO_PinMode   = GPIO_MODE_INPUT ;
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW ;
	GpioButton.GPIO_PinConfig.GPIO_PinOPtype = GPIO_OPTYPE_OD ;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_Init(&GpioButton);

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
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = 12 ;
	GPIO_Init(&SPIPins);

}
void SPI2_Inits(void){
	SPI_Handle_t SPI2handle;

	SPI2handle.pSPIx = SPI2 ;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD ;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER ;
	SPI2handle.SPIConfig.SPI_SclkSpeed =  SPI_SCLK_SPEED_DIV32 ;
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW ;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW ;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_DI ; //hw slave manegenmet enabled for nss pin

	SPI_Init(&SPI2handle);


}
