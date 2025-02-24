/*
 * Project: 006read_IO
 * Description: Basic STM32 project demonstrating fundamental concepts.
 * Author: Betül Atalay
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

#include <stdint.h>

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif
//optimization level is O1. default level O0
int main(void)
{

	uint32_t volatile *pClockreg = (uint32_t*)0x40023830 ;
	uint32_t volatile *pPinModerA = (uint32_t*)0x40020000  ;
	uint32_t volatile *pPinModerD = (uint32_t*)0x40020C00 ;
	uint32_t volatile *pDpinreg = (uint32_t*)0x40020C14 ;
	uint32_t volatile *pApinreg = (uint32_t*)0x40020010 ;
	uint32_t data ;

	*pClockreg |=  (0x09)  ;

	*pPinModerD &= ~(3<<24);

	*pPinModerD |= (1<<24);

	*pPinModerA &= ~(0x03) ;

	while(1){
		data = *pApinreg & 0x01; //masking
		if(data){
			*pDpinreg |= (1<<12);

		}
		else {
			*pDpinreg &= ~(1<<12);
		}

	}

    /* Loop forever */

}
