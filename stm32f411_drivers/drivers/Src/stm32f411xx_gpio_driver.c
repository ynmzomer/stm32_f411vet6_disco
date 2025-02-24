/*
 * stm32f411xx_gpio_driver.c
 *
 *  Created on: Jan 27, 2025
 *      Author: omery
 */

#include "stm32f411xx_gpio_driver.h"

/**
 * @brief  Enables or disables the peripheral clock for a given GPIO port.
 * @param  pGPIOx: Pointer to the GPIO peripheral base address (GPIOA, GPIOB, etc.).
 * @param  EnorDi: ENABLE (1) to enable the clock, DISABLE (0) to disable the clock.
 * @retval None
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi) {
    if (EnorDi == ENABLE) {
        if (pGPIOx == GPIOA) {
            GPIOA_PCLK_EN();
        } else if (pGPIOx == GPIOB) {
            GPIOB_PCLK_EN();
        } else if (pGPIOx == GPIOC) {
            GPIOC_PCLK_EN();
        } else if (pGPIOx == GPIOD) {
            GPIOD_PCLK_EN();
        } else if (pGPIOx == GPIOE) {
            GPIOE_PCLK_EN();
        }else if (pGPIOx == GPIOH) {
            GPIOH_PCLK_EN();
        }
    } else {
        if (pGPIOx == GPIOA) {
            GPIOA_PCLK_DI();
        } else if (pGPIOx == GPIOB) {
            GPIOB_PCLK_DI();
        } else if (pGPIOx == GPIOC) {
            GPIOC_PCLK_DI();
        } else if (pGPIOx == GPIOD) {
            GPIOD_PCLK_DI();
        }else if (pGPIOx == GPIOE) {
            GPIOE_PCLK_DI();
        } else if (pGPIOx == GPIOH) {
            GPIOH_PCLK_DI();
        }
    }
}


/**
 * @brief  Initializes the given GPIO pin based on the configuration provided.
 * @param  pGPIOHandle: Pointer to GPIO_Handle_t structure, containing the pin settings.
 * @retval None
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle) {
    uint32_t temp = 0;

    GPIO_PeriClockControl(pGPIOHandle ->pGPIOx, ENABLE);

    /* Configure the mode of the GPIO pin */
    if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {
        // Set the mode for input, output, analog, or alternate function
        temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));

        // Clear the existing mode bits
        pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));

        // Set the new mode bits
        pGPIOHandle->pGPIOx->MODER |= temp;
    }else {
        // Code for interrupt mode will be added
    	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT){

    		EXTI->FTSR |= (1 << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
    		EXTI->RTSR &= ~(1 << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);


    	}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT){
    		EXTI->RTSR |= (1 << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
    		EXTI->FTSR &= ~(1 << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);

    	}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FRT){
    		EXTI->FTSR |= (1 << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
    		EXTI->RTSR |= (1 << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);

    	}

    	SYSCFG_PCLK_EN();
    	uint8_t temp1 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4);
    	uint8_t temp2 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4);
    	uint8_t port_code = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx) ;
    	SYSCGF -> EXTICR[temp1]   |=  (port_code << temp2*4)  ;



    	EXTI->IMR |= (1 << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);

    }

    /* Configure the speed of the GPIO pin */
    temp = 0;
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));

    // Clear the existing speed bits
    pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));

    // Set the new speed bits
    pGPIOHandle->pGPIOx->OSPEEDR |= temp;

    /* Configure the pull-up / pull-down resistor */
    temp = 0;
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));

    // Clear the existing pull-up/pull-down bits
    pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));

    // Set the new pull-up/pull-down bits
    pGPIOHandle->pGPIOx->PUPDR |= temp;

    /* Configure the output type (Push-Pull or Open-Drain) */
    temp = 0;
    pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // Clear the bit
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPtype << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // Set the bit
    pGPIOHandle->pGPIOx->OTYPER |= temp;

    /* Configure the alternate function if mode is set to alternate function mode */
    if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN) {
        uint32_t temp1, temp2;

        // Determine the AFR register index (0 for AFR[0], 1 for AFR[1])
        temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
        temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;

        // Clear existing alternate function bits
        pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));

        // Set the new alternate function bits
        pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
    }
}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx) {
	if (pGPIOx == GPIOA) {
		GPIOA_REG_RST();
	}else if (pGPIOx == GPIOB) {
		GPIOB_REG_RST();
	} else if (pGPIOx == GPIOC) {
		GPIOC_REG_RST();
	} else if (pGPIOx == GPIOD) {
		GPIOD_REG_RST();
	} else if (pGPIOx == GPIOE) {
		GPIOE_REG_RST();
	}else if (pGPIOx == GPIOH) {
		GPIOH_REG_RST();
	}


}

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx , uint8_t PinNumber) {
	uint8_t value ;
	value = (uint8_t)(pGPIOx->IDR >> (PinNumber)&(0x00000001)) ;
	return value ;

}
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){
	uint16_t value ;
	value = (uint16_t)(pGPIOx->IDR) ;
	return value ;
}
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx , uint8_t PinNumber ,uint8_t value){
	if (value == GPIO_PIN_SET) {
		pGPIOx -> ODR |= (1 << PinNumber) ;
	}
	else{
		pGPIOx -> ODR &= ~(1 << PinNumber) ;

	}
}
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx , uint8_t value ){
	pGPIOx -> ODR |= value ;

}
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx , uint8_t PinNumber){
	pGPIOx ->ODR ^= (1 << PinNumber) ;

}
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= ( 1 << IRQNumber );

		}else if(IRQNumber > 31 && IRQNumber < 64 ) //32 to 63
		{
			//program ISER1 register
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96 )
		{
			//program ISER2 register //64 to 95
			*NVIC_ISER2 |= ( 1 << (IRQNumber % 64) );
		}
	}else
	{
		if(IRQNumber <= 31)
		{
			//program ICER0 register
			*NVIC_ICER0 |= ( 1 << IRQNumber );
		}else if(IRQNumber > 31 && IRQNumber < 64 )
		{
			//program ICER1 register
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96 )
		{
			//program ICER2 register
			*NVIC_ICER2 |= ( 1 << (IRQNumber % 64) );
		}
	}

}



void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	//1. first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section  = IRQNumber %4 ;

	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

	*(  NVIC_PR_BASE_ADDR + iprx ) |=  ( IRQPriority << shift_amount );

}

void GPIO_IRQHandling(uint8_t PinNumber)
{
	//clear the exti pr register corresponding to the pin number
	if(EXTI->PR & ( 1 << PinNumber))
	{
		//clear
		EXTI->PR |= ( 1 << PinNumber);
	}

}

