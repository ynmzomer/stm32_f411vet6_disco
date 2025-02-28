/*
 * Project: 003Add
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

#include <stdint.h>
#include <stdio.h>
#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

/*Analayzing embedded c code
 * bu kodu analiz etmeyi öğrendim. memory browser kullanma
 * .elf dosyasının konumu ve analizi
 * disasammbely kullanma (registerlarları izleme)
 * instruction stepper mode kullanarak.
 *
 *
 *
 *
 */

int g_data1 = -4000;
int g_data2 = 200;
int result = 0;
int main(void)
{
	result = g_data1 + g_data2 ;

	printf("Result: %d\n",result);



    /* Loop forever */
	for(;;);
}
