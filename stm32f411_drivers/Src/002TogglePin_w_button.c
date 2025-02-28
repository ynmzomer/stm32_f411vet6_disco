/*
 * File: 002TogglePin_w_button.c
 * Description: STM32F411 driver test application.
 * Purpose: This file contains code to test custom driver implementations for GPIO, SPI, I2C, and UART.
 * Author: omery
 */

/*
 * 001TogglePin.c
 *
 *  Created on: Jan 28, 2025
 *      Author: omery
 */


#include <stdint.h>
#include "stm32f411xx.h"



int main(void)
{
	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_PeriClockControl(GPIOA, ENABLE);

	uint8_t button_pressed = 0 ;

	GPIO_Handle_t GpioLed ;
	GpioLed.pGPIOx = GPIOD ;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12 ;
	GpioLed.GPIO_PinConfig.GPIO_PinMode   = GPIO_MODE_OUTPUT ;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST ;
	GpioLed.GPIO_PinConfig.GPIO_PinOPtype = GPIO_OPTYPE_PP ;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Handle_t GpioButton ;
	GpioButton.pGPIOx = GPIOA ;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0 ;
	GpioButton.GPIO_PinConfig.GPIO_PinMode   = GPIO_MODE_INPUT ;
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW ;
	GpioButton.GPIO_PinConfig.GPIO_PinOPtype = GPIO_OPTYPE_OD ;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_Init(&GpioLed);
	GPIO_Init(&GpioButton);


	for(;;){
		button_pressed = GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) ;
		if (button_pressed){
			GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
			delay();
			}

		}


}
