/*
 * File: 001TogglePin.c
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


void delay(void){
	for (volatile uint32_t i = 0; i < 500000; i++);
}

int main(void)
{
	GPIO_PeriClockControl(GPIOD, ENABLE);

	GPIO_Handle_t GpioLed ;
	GpioLed.pGPIOx = GPIOD ;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12 ;
	GpioLed.GPIO_PinConfig.GPIO_PinMode   = GPIO_MODE_OUTPUT ;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST ;
	GpioLed.GPIO_PinConfig.GPIO_PinOPtype = GPIO_OPTYPE_PP ;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GpioLed);




    /* Loop forever */
	for(;;){
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
		delay();


	}
}
