/*
 * main.h
 *
 *  Created on: Jan 18, 2025
 *      Author: omery
 */

#ifndef MAIN_H_
#define MAIN_H_


typedef struct{

	uint32_t gpioa_en	:1 ;
	uint32_t gpiob_en	:1 ;
	uint32_t gpioc_en	:1 ;
	uint32_t gpiod_en	:1 ;
	uint32_t gpioe_en	:1 ;
	uint32_t reserved1	:2 ;
	uint32_t gpioh_en	:1 ;
	uint32_t reserved2	:4 ;
	uint32_t crc_en		:1 ;
	uint32_t reserved3	:3 ;
	uint32_t reserved4	:5 ;
	uint32_t dma1en		:1 ;
	uint32_t dma2en		:1 ;
	uint32_t reserved5	:9 ;

}RCC_AHB1ENR_t;

typedef struct{
	uint32_t moder0		:2;
	uint32_t moder1		:2;
	uint32_t moder2		:2;
	uint32_t moder3		:2;
	uint32_t moder4		:2;
	uint32_t moder5		:2;
	uint32_t moder6		:2;
	uint32_t moder7		:2;
	uint32_t moder8		:2;
	uint32_t moder9		:2;
	uint32_t moder10	:2;
	uint32_t moder11	:2;
	uint32_t moder12	:2;
	uint32_t moder13	:2;
	uint32_t moder14	:2;
	uint32_t moder15	:2;
	uint32_t reserved	:16 ;

}GPIOx_MODER_t;

typedef struct{
	uint32_t ıdr0:1;
	uint32_t ıdr1:1;
	uint32_t ıdr2:1;
	uint32_t ıdr3:1;
	uint32_t ıdr4:1;
	uint32_t ıdr5:1;
	uint32_t ıdr6:1;
	uint32_t ıdr7:1;
	uint32_t ıdr8:1;
	uint32_t ıdr9:1;
	uint32_t ıdr10:1;
	uint32_t ıdr11:1;
	uint32_t ıdr12:1;
	uint32_t ıdr13:1;
	uint32_t ıdr14:1;
	uint32_t ıdr15:1;
	uint32_t reserved	:16 ;
}GPIOx_IDR_t;

typedef struct{
	uint32_t odr0:1;
	uint32_t odr1:1;
	uint32_t odr2:1;
	uint32_t odr3:1;
	uint32_t odr4:1;
	uint32_t odr5:1;
	uint32_t odr6:1;
	uint32_t odr7:1;
	uint32_t odr8:1;
	uint32_t odr9:1;
	uint32_t odr10:1;
	uint32_t odr11:1;
	uint32_t odr12:1;
	uint32_t odr13:1;
	uint32_t odr14:1;
	uint32_t odr15:1;
	uint32_t reserved	:16 ;
}GPIOx_ODR_t;






#endif /* MAIN_H_ */
