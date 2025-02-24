/*
 * File: 008i2c_master_tx_test.c
 * Description: STM32F411 driver test application.
 * Purpose: This file contains code to test custom driver implementations for GPIO, SPI, I2C, and UART.
 * Author: Betül Atalay
 */

/*
 * 008i2c_master_tx_test.c
 *
 *  Created on: Feb 8, 2025
 *      Author: omery
 */

//clock ayarlarında sorun olabilir.

#include "stm32f411xx.h"
#include <string.h>
#include <stdio.h>

#define SLAVE_ADDR  0x68

#define MY_ADDR 0x61
// pb6 -> SCL
// pb9 -> SDA
void I2C1_GPIOInits(void);
void I2C1_Inits(void);
void button_init(void);
void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

I2C_Handle_t I2C1_Handle ;

//spmde data

uint8_t some_data[] = "betulum atalay \n" ;

int main(void)
{

	button_init();

	//i2c pin inits
	I2C1_GPIOInits();

	//i2c peripheral configuration
	I2C1_Inits();

	//enable the i2c peripheral
	I2C_PeripheralControl(I2C1,ENABLE);

	while(1)
	{
		//wait till button is pressed
		while( ! GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0) );

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		//send some data to the slave
		I2C_MasterSendData(&I2C1_Handle,some_data,strlen((char*)some_data),SLAVE_ADDR,0);
	}

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



void I2C1_GPIOInits(void){
	GPIO_Handle_t I2C1Pins ;
	I2C1Pins.pGPIOx = GPIOB ;

	I2C1Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN ;
	I2C1Pins.GPIO_PinConfig.GPIO_PinAltFunMode = 4 ;
	I2C1Pins.GPIO_PinConfig.GPIO_PinOPtype = GPIO_OPTYPE_OD ;
	I2C1Pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU ;
	I2C1Pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST ;

	//scl
	I2C1Pins.GPIO_PinConfig.GPIO_PinNumber = 6 ;
	GPIO_Init(&I2C1Pins);
	//sda
	I2C1Pins.GPIO_PinConfig.GPIO_PinNumber = 7 ;
	GPIO_Init(&I2C1Pins);


}
void I2C1_Inits(void){

	I2C1_Handle.pI2Cx = I2C1 ;
	I2C1_Handle.I2C_Config.I2C_AckControl = I2C_ACK_ENABLE ;
	I2C1_Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR ;
	I2C1_Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1_Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM ;

	I2C_Init(&I2C1_Handle);





}
