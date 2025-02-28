/*
 * File: 003button_interrupt.c
 * Description: STM32F411 driver test application.
 * Purpose: This file contains code to test custom driver implementations for GPIO, SPI, I2C, and UART.
 * Author: omery
 */

/*
 * 003button_inteerupt.c
 *
 *  Created on: Jan 29, 2025
 *      Author: omery
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
	GPIO_PeriClockControl(GPIOA, ENABLE);

	//uint8_t button_pressed = 0 ;

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
	GpioButton.GPIO_PinConfig.GPIO_PinMode   = GPIO_MODE_IT_FT;
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW ;
	GpioButton.GPIO_PinConfig.GPIO_PinOPtype = GPIO_OPTYPE_OD ;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_Init(&GpioLed);
	GPIO_Init(&GpioButton);

	//priority is optional on this example 15 için makro tanımla
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI0, 15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI0, ENABLE);

	while(1);
	return 0 ;

}
void EXTI0_IRQHandler(void){
	delay();
	GPIO_IRQHandling(GPIO_PIN_NO_0);
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);


}
