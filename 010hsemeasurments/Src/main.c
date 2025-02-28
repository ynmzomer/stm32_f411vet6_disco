/*
 * Project: 010hsemeasurments
 * Description: Basic STM32 project demonstrating fundamental concepts.
 * Author: omery
 */

/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
//MCO1 is  pa8 alternete funtinol mode look datasheet

#include <stdint.h>

#define RCC_BASE_REG  0x40023800UL

#define RCC_CFGR_OF 0x08UL

#define RCC_CFGR_REG (RCC_BASE_REG + RCC_CFGR_OF)

#define GPIOA_BASE_REG  0x40020000

#define GPIO_MODER_A (GPIOA_BASE_REG)

#define GPIOx_AFRH (GPIOA_BASE_REG + 0x24)

int main(void)
{

	uint32_t *pRCC_CR = (uint32_t*)RCC_BASE_REG ;
	*pRCC_CR |= (1<<16) ; //enable HSE
	uint32_t data = (uint32_t)*pRCC_CR & (1<<17) ;

	uint32_t *pCfgr_REG =(uint32_t*)RCC_CFGR_REG ;

	uint32_t *pGpio_mod_A =(uint32_t*)GPIO_MODER_A ;

	uint32_t *pRCCAhb1Enr =(uint32_t*)0x40023830 ;

	uint32_t *pAFhighReg = (uint32_t*)GPIOx_AFRH ;
	*pRCCAhb1Enr |= (1<<0) ;

	while (!data) ;



	*pCfgr_REG &= ~(0x03 << 21);
	*pCfgr_REG |= (1<<22) ;

	*pGpio_mod_A &= ~(0X03 << 16) ;
	*pGpio_mod_A |= (0X02 << 16) ;

	*pAFhighReg &= ~(0xf << 0);


    /* Loop forever */
	for(;;);
}
