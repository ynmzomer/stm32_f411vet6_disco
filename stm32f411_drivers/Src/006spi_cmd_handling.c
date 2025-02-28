/*
 * File: 006spi_cmd_handling.c
 * Description: STM32F411 driver test application.
 * Purpose: This file contains code to test custom driver implementations for GPIO, SPI, I2C, and UART.
 * Author: omery
 */

/*
 * 005_spi_tx_arduino.c
 *
 *  Created on: Feb 1, 2025
 *      Author: omery
 */


#include "stm32f411xx.h"
#include <string.h>
#include <stdio.h>


//command codes
#define COMMAND_LED_CTRL      		0x50
#define COMMAND_SENSOR_READ      	0x51
#define COMMAND_LED_READ      		0x52
#define COMMAND_PRINT      			0x53
#define COMMAND_ID_READ      		0x54

#define LED_ON     1
#define LED_OFF    0

//arduino analog pins
#define ANALOG_PIN0 	0
#define ANALOG_PIN1 	1
#define ANALOG_PIN2 	2
#define ANALOG_PIN3 	3
#define ANALOG_PIN4 	4

//arduino led

#define LED_PIN  9






void SPI2_GPIOInits(void) ;
void SPI2_Inits(void) ;
void button_init(void) ;
void delay(void) ;
uint8_t SPI_VerifyResponse(uint8_t ackbyte)
{

	if(ackbyte == (uint8_t)0xF5)
	{
		//ack
		return 1;
	}

	return 0;
}


int main(void){
	uint8_t dummy_write = 0xff;
	uint8_t dummy_read;
	//char user_data[] = "Hello World" ;

	button_init();
	SPI2_GPIOInits() ;
	SPI2_Inits() ;
	/*
		* making SSOE 1 does NSS output enable.
		* The NSS pin is automatically managed by the hardware.
		* i.e when SPE=1 , NSS will be pulled to low
		* and NSS pin will be high when SPE=0
		*/
	//lenght info
	SPI_SSOEConfig(SPI2,ENABLE);
	//uint8_t datalen = strlen(user_data);
	for(;;){
		//wait till button is pressed
		while(!GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0));

				//to avoid button de-bouncing related issues 200ms of delay
		delay();

				//enable the SPI2 peripheral
		SPI_PeripheralControl(SPI2,ENABLE);

		uint8_t commandcode = COMMAND_LED_CTRL ;
		uint8_t ackbyte;
		uint8_t args[2];

		SPI_SendData(SPI2, &commandcode, 1);
		SPI_ReciveData(SPI2, &dummy_read, 1);
		SPI_SendData(SPI2, &dummy_write, 1);
		SPI_ReciveData(SPI2, &ackbyte, 1) ;


		if(SPI_VerifyResponse(ackbyte)){
			args[0] = LED_PIN ;
			args[1] = LED_ON ;

			SPI_SendData(SPI2, args, 2);
			SPI_ReciveData(SPI2, args, 2); //byrada neden args a veri çekiyoruz
		}
		printf("LED is On \n");
		while(!GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0));

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

					//enable the SPI2 peripheral
		SPI_PeripheralControl(SPI2,ENABLE);

		commandcode = COMMAND_SENSOR_READ ;

		SPI_SendData(SPI2, &commandcode, 1);
		SPI_ReciveData(SPI2, &dummy_read, 1);
		SPI_SendData(SPI2, &dummy_write, 1);
		SPI_ReciveData(SPI2, &ackbyte, 1) ;

		if(SPI_VerifyResponse(ackbyte)){
			args[0] = ANALOG_PIN0 ;

			SPI_SendData(SPI2, args, 1);
			SPI_ReciveData(SPI2, args, 1);
			delay();
			SPI_SendData(SPI2, &dummy_write, 1);
			uint8_t analog_read ;
			SPI_ReciveData(SPI2, &analog_read, 1);
			printf("analog value %d \n",analog_read);
			}


		while( ! GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0) );

				//to avoid button de-bouncing related issues 200ms of delay
		delay();

		commandcode = COMMAND_LED_READ;

				//send command
		SPI_SendData(SPI2,&commandcode,1);

				//do dummy read to clear off the RXNE
		SPI_ReciveData(SPI2,&dummy_read,1);

				//Send some dummy byte to fetch the response from the slave
		SPI_SendData(SPI2,&dummy_write,1);

				//read the ack byte received
		SPI_ReciveData(SPI2,&ackbyte,1);

		if( SPI_VerifyResponse(ackbyte))
				{
					args[0] = LED_PIN;

					//send arguments
					SPI_SendData(SPI2,args,1); //sending one byte of

					//do dummy read to clear off the RXNE
					SPI_ReciveData(SPI2,&dummy_read,1);

					//insert some delay so that slave can ready with the data
					delay();

					//Send some dummy bits (1 byte) fetch the response from the slave
					SPI_SendData(SPI2,&dummy_write,1);

					uint8_t led_status;
					SPI_ReciveData(SPI2,&led_status,1);
					printf("COMMAND_READ_LED %d\n",led_status);

				}

		while(!GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0));

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

				//enable the SPI2 peripheral
		SPI_PeripheralControl(SPI2,ENABLE);

		commandcode = COMMAND_PRINT ;


		SPI_SendData(SPI2, &commandcode, 1);
		SPI_ReciveData(SPI2, &dummy_read, 1);
		SPI_SendData(SPI2, &dummy_write, 1);
		SPI_ReciveData(SPI2, &ackbyte, 1) ;


		uint8_t message[] = "Hello ! How are you ??";


		if(SPI_VerifyResponse(ackbyte)){
			args[0] = strlen((char*)message) ;


			SPI_SendData(SPI2, args, 1);
			SPI_ReciveData(SPI2, &dummy_read, 1);

			delay();
			for(int i = 0 ; i< args[0] ; i++){
				SPI_SendData(SPI2, &message[i], 1);
				SPI_ReciveData(SPI2, &dummy_read, 1);

			}

			}
		while(!GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0));

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

				//enable the SPI2 peripheral
		SPI_PeripheralControl(SPI2,ENABLE);

		commandcode = COMMAND_ID_READ ;


		SPI_SendData(SPI2, &commandcode, 1);
		SPI_ReciveData(SPI2, &dummy_read, 1);
		SPI_SendData(SPI2, &dummy_write, 1);
		SPI_ReciveData(SPI2, &ackbyte, 1) ;

		uint8_t id[11];
		uint32_t i = 0 ;

		if(SPI_VerifyResponse(ackbyte)){

			for(i = 0 ; i<10 ; i++){
				SPI_SendData(SPI2, &dummy_write, 1);
				SPI_ReciveData(SPI2, &id[i], 1);
			}
		}

		id[10] = '\0' ;

		printf("command_id ; %s \n",id);

		while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

		SPI_PeripheralControl(SPI2, DISABLE);

		printf("spi comm closed \n");



			}

	return 0 ;

}

void delay(void){
	for (volatile uint32_t i = 0; i < 500000; i++);
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
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = 14 ;
	GPIO_Init(&SPIPins);
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
