/*
 * stm32f411xx.h
 *
 *  Created on: Jan 26, 2025
 *      Author: omery
 */

#ifndef INC_STM32F411XX_H_
#define INC_STM32F411XX_H_

#include <stdint.h>
#include <stddef.h>

#define __vo volatile // Volatile keyword to prevent compiler optimization for hardware registers



/**********************************START:Processor Specific Details **********************************/
/*
 * ARM Cortex Mx Processor NVIC ISERx register Addresses
 */

#define NVIC_ISER0          ( (__vo uint32_t*)0xE000E100 )
#define NVIC_ISER1          ( (__vo uint32_t*)0xE000E104 )
#define NVIC_ISER2          ( (__vo uint32_t*)0xE000E108 )
#define NVIC_ISER3          ( (__vo uint32_t*)0xE000E10c )


/*
 * ARM Cortex Mx Processor NVIC ICERx register Addresses
 */
#define NVIC_ICER0 			((__vo uint32_t*)0XE000E180)
#define NVIC_ICER1			((__vo uint32_t*)0XE000E184)
#define NVIC_ICER2  		((__vo uint32_t*)0XE000E188)
#define NVIC_ICER3			((__vo uint32_t*)0XE000E18C)


/*
 * ARM Cortex Mx Processor Priority Register Address Calculation
 */
#define NVIC_PR_BASE_ADDR 	((__vo uint32_t*)0xE000E400)

/*
 * ARM Cortex Mx Processor number of priority bits implemented in Priority Register
 */
#define NO_PR_BITS_IMPLEMENTED  4



// Base addresses of memory regions
#define FLASH_BASEADDR		  0x08000000U /* Base address for Flash memory */
#define SRAM1_BASEADDR 		  0x20000000U // Base address for SRAM1 (112KB)
#define SRAM2_BASEADDR        0x2001C000U // Base address for SRAM2 (16KB)
#define ROM_BASEADDR		  0x1FFF0000  // Base address for system memory (ROM)
#define SRAM				  SRAM1_BASEADDR // Define SRAM as SRAM1 for ease of use

/*
 * AHBx and APBx Bus Peripheral base addresses
 * These addresses define the starting points of different buses (APB1, APB2, AHB1, AHB2)
 */
#define PERIPH_BASE   		 0x40000000U // Base address for peripheral memory
#define APB1PERIPH_BASE		 PERIPH_BASE // Base address for APB1 peripherals
#define APB2PERIPH_BASE		 0x40010000 // Base address for APB2 peripherals
#define AHB1PERIPH_BASE		 0x40020000 // Base address for AHB1 peripherals
#define AHB2PERIPH_BASE		 0x50000000 // Base address for AHB2 peripherals

/* Peripheral detailed addresses */
#define GPIOA_BASEADDR		 AHB1PERIPH_BASE                      // Base address for GPIOA
#define GPIOB_BASEADDR		 (AHB1PERIPH_BASE + 0X0400)           // Base address for GPIOB
#define GPIOC_BASEADDR		 (AHB1PERIPH_BASE + 0X0800)           // Base address for GPIOC
#define GPIOD_BASEADDR		 (AHB1PERIPH_BASE + 0X0C00)           // Base address for GPIOD
#define GPIOE_BASEADDR		 (AHB1PERIPH_BASE + 0X1000)           // Base address for GPIOE
#define GPIOH_BASEADDR		 (AHB1PERIPH_BASE + 0X1C00)  // Base address for GPIOH

#define RCC_BASEADDR		 (AHB1PERIPH_BASE + 0X3800)          // Base address for RCC (Reset and Clock Control)

#define I2C1_BASEADDR   	 (APB1PERIPH_BASE + 0X5400)           // Base address for I2C1
#define I2C2_BASEADDR   	 (APB1PERIPH_BASE + 0X5800)           // Base address for I2C2
#define I2C3_BASEADDR   	 (APB1PERIPH_BASE + 0X5C00) // Base address for I2C3

#define SPI1_BASEADDR        (APB2PERIPH_BASE + 0X3000)			  // Base address for SPI1
#define SPI2_BASEADDR   	 (APB1PERIPH_BASE + 0X3800)           // Base address for SPI2
#define SPI3_BASEADDR   	 (APB1PERIPH_BASE + 0X3C00)// Base address for SPI3
#define SPI4_BASEADDR        (APB2PERIPH_BASE + 0X3400)


#define EXTI_BASEADDR        (APB2PERIPH_BASE + 0X3C00)           // Base address for EXTI (External Interrupt)
#define SYSCFG_BASEADDR      (APB2PERIPH_BASE + 0X3800)           // Base address for SYSCFG (System Configuration)

#define USART1_BASEADDR      (APB2PERIPH_BASE + 0X1000)           // Base address for USART1
#define USART2_BASEADDR   	 (APB1PERIPH_BASE + 0X4400)           // Base address for USART2
#define USART6_BASEADDR      (APB2PERIPH_BASE + 0X1400)           // Base address for USART6



// Structure to define GPIO registers
typedef struct {
	__vo uint32_t MODER;      // GPIO mode register
	__vo uint32_t OTYPER;     // GPIO output type register
	__vo uint32_t OSPEEDR;    // GPIO output speed register
	__vo uint32_t PUPDR;      // GPIO pull-up/pull-down register
	__vo uint32_t IDR;        // GPIO input data register
	__vo uint32_t ODR;        // GPIO output data register
	__vo uint32_t BSRR;       // GPIO bit set/reset register
	__vo uint32_t LCKR;       // GPIO lock register
	__vo uint32_t AFR[2];     // GPIO alternate function registers (low and high)
} GPIO_RegDef_t;


typedef struct{
	uint32_t CR1 	;
	uint32_t CR2 	;
	uint32_t SR 	;
	uint32_t DR 	;
	uint32_t CRCPR 	;
	uint32_t RXCRCR ;
	uint32_t SPI_TXCRCR ;
	uint32_t I2SCFGR;
	uint32_t I2SPR 	;

} SPI_RegDef_t;

typedef struct{
	__vo uint32_t CR1 ;
	__vo uint32_t CR2 ;
	__vo uint32_t OAR1 ;
	__vo uint32_t OAR2 ;
	__vo uint32_t DR ;
	__vo uint32_t SR1 ;
	__vo uint32_t SR2 ;
	__vo uint32_t CCR ;
	__vo uint32_t TRISE ;
	__vo uint32_t FLTR ;


} I2C_RegDef_t;



typedef struct{
	__vo uint32_t SR ;
	__vo uint32_t DR ;
	__vo uint32_t BRR ;
	__vo uint32_t CR1 ;
	__vo uint32_t CR2 ;
	__vo uint32_t CR3 ;
	__vo uint32_t GTPR ;
} USART_RegDef_t;

// Structure to define RCC registers
typedef struct {
	__vo uint32_t CR;          // Clock control register
	__vo uint32_t PLLCFGR;     // PLL configuration register
	__vo uint32_t CFGR;        // Clock configuration register
	__vo uint32_t CIR;         // Clock interrupt register
	__vo uint32_t AHB1RSTR;    // AHB1 peripheral reset register
	__vo uint32_t AHB2RSTR;    // AHB2 peripheral reset register
	uint32_t RESERVED[2];      // Reserved
	__vo uint32_t APB1RSTR;    // APB1 peripheral reset register
	__vo uint32_t APB2RSTR;    // APB2 peripheral reset register
	uint32_t RESERVED1[2];     // Reserved
	__vo uint32_t AHB1ENR;     // AHB1 peripheral clock enable register
	__vo uint32_t AHB2ENR;     // AHB2 peripheral clock enable register
	uint32_t RESERVED2[2];     // Reserved
	__vo uint32_t APB1ENR;     // APB1 peripheral clock enable register
	__vo uint32_t APB2ENR;     // APB2 peripheral clock enable register
	uint32_t RESERVED3[2];     // Reserved
	__vo uint32_t AHB1LPENR;   // AHB1 low power enable register
	__vo uint32_t AHB2LPENR;   // AHB2 low power enable register
	uint32_t RESERVED4[2];     // Reserved
	__vo uint32_t APB1LPENR;   // APB1 low power enable register
	__vo uint32_t APB2LPENR;   // APB2 low power enable register
	uint32_t RESERVED5[2];     // Reserved
	__vo uint32_t BDCR;        // Backup domain control register
	__vo uint32_t CSR;         // Clock control & status register
	uint32_t RESERVED6[2];     // Reserved
	__vo uint32_t SSCGR;       // Spread spectrum clock generation register
	__vo uint32_t PLLI2SCFGR;  // PLLI2S configuration register
	__vo uint32_t DCKCFGR;     // Dedicated clocks configuration register
} RCC_RegDef_t;

typedef struct{
	__vo uint32_t IMR ;
	__vo uint32_t EMR ;
	__vo uint32_t RTSR ;
	__vo uint32_t FTSR ;
	__vo uint32_t SWIER ;
	__vo uint32_t PR ;

}EXTI_RegDef_t;


typedef struct{
	uint32_t  MEMRMP ;
	uint32_t  PMC ;
	uint32_t  EXTICR[4] ;
	uint32_t  CMPCR ;
}SYSCFG_t;

// Macros to define GPIO peripheral base addresses as pointers
#define GPIOA ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOH ((GPIO_RegDef_t*)GPIOH_BASEADDR)


#define SPI1 ((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2 ((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3 ((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4 ((SPI_RegDef_t*)SPI4_BASEADDR)

#define I2C1  ((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2  ((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3  ((I2C_RegDef_t*)I2C3_BASEADDR)

#define USART1 ((USART_RegDef_t*)USART1_BASEADDR)
#define USART2 ((USART_RegDef_t*)USART2_BASEADDR)
#define USART6 ((USART_RegDef_t*)USART6_BASEADDR)


// Macro to define RCC base address as a pointer
#define RCC   ((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI ((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SYSCGF ((SYSCFG_t*)SYSCFG_BASEADDR)

// Enable peripheral clock macros
//#define PCLK_RESET()       (RCC->AHB1RSTR |= (1 << 0))
#define GPIOA_PCLK_EN()    (RCC->AHB1ENR  |= (1 << 0)) // Enable clock for GPIOA
#define GPIOB_PCLK_EN()    (RCC->AHB1ENR |= (1 << 1)) // Enable clock for GPIOB
#define GPIOC_PCLK_EN()    (RCC->AHB1ENR |= (1 << 2)) // Enable clock for GPIOC
#define GPIOD_PCLK_EN()    (RCC->AHB1ENR |= (1 << 3)) // Enable clock for GPIOD
#define GPIOE_PCLK_EN()    (RCC->AHB1ENR |= (1 << 4))
#define GPIOH_PCLK_EN()    (RCC->AHB1ENR |= (1 << 7))

#define I2C1_PCLK_EN()    (RCC->APB1ENR |= (1 << 21)) // Enable clock for I2C1
#define I2C2_PCLK_EN()    (RCC->APB1ENR |= (1 << 22)) // Enable clock for I2C2
#define I2C3_PCLK_EN()    (RCC->APB1ENR |= (1 << 23)) // Enable clock for I2C3

#define SPI1_PCLK_EN()    (RCC->APB2ENR |= (1 << 12)) // Enable clock for SPI1
#define SPI2_PCLK_EN()    (RCC->APB1ENR |= (1 << 14)) // Enable clock for SPI2
#define SPI3_PCLK_EN()    (RCC->APB1ENR |= (1 << 15)) // Enable clock for SPI3
#define SPI4_PCLK_EN()    (RCC->APB2ENR |= (1 << 13))

#define SYSCFG_PCLK_EN()    (RCC->APB2ENR |= (1 << 14)) // Enable clock for SYSCFG

#define USART1_PCLK_EN()    (RCC->APB2ENR |= (1 << 4))  // Enable clock for USART1
#define USART2_PCLK_EN()    (RCC->APB1ENR |= (1 << 17)) // Enable clock for USART2
#define USART6_PCLK_EN()    (RCC->APB2ENR |= (1 << 5)) // Enable clock for USART2

// Disable peripheral clock macros
#define GPIOA_PCLK_DI()    (RCC->AHB1ENR &= ~(1 << 0)) // Disable clock for GPIOA
#define GPIOB_PCLK_DI()    (RCC->AHB1ENR &= ~(1 << 1)) // Disable clock for GPIOB
#define GPIOC_PCLK_DI()    (RCC->AHB1ENR &= ~(1 << 2)) // Disable clock for GPIOC
#define GPIOD_PCLK_DI()    (RCC->AHB1ENR &= ~(1 << 3)) // Disable clock for GPIOD
#define GPIOE_PCLK_DI()    (RCC->AHB1ENR &= ~(1 << 4))
#define GPIOH_PCLK_DI()    (RCC->AHB1ENR &= ~(1 << 7))

#define I2C1_PCLK_DI()    (RCC->APB1ENR &= ~(1 << 21)) // Disable clock for I2C1
#define I2C2_PCLK_DI()    (RCC->APB1ENR &= ~(1 << 22)) // Disable clock for I2C2
#define I2C3_PCLK_DI()    (RCC->APB1ENR &= ~(1 << 23)) // Disable clock for I2C3

#define SPI1_PCLK_DI()    (RCC->APB2ENR &= ~(1 << 12)) // Disable clock for SPI1
#define SPI2_PCLK_DI()    (RCC->APB1ENR &= ~(1 << 14)) // Disable clock for SPI2
#define SPI3_PCLK_DI()    (RCC->APB1ENR &= ~(1 << 15)) // Disable clock for SPI3
#define SPI4_PCLK_DI()    (RCC->APB2ENR &= ~(1 << 13))

#define SYSCFG_PCLK_DI()    (RCC->APB2ENR &= ~(1 << 14)) // Disable clock for SYSCFG

#define USART1_PCLK_DI()    (RCC->APB2ENR &= ~(1 << 4))  // Disable clock for USART1
#define USART2_PCLK_DI()    (RCC->APB1ENR &= ~(1 << 17)) // Disable clock for USART2
#define USART6_PCLK_DI()    (RCC->APB2ENR &= ~(1 << 5)) // Disable clock for USART2


#define GPIOA_REG_RST()		do{ (RCC->AHB1RSTR  |= (1<<0)) ; (RCC->AHB1RSTR  &=~(1<<0)); }while(0)
#define GPIOB_REG_RST()	    do{ (RCC->AHB1RSTR  |= (1<<1)) ; (RCC->AHB1RSTR  &=~(1<<1));}while(0)
#define GPIOC_REG_RST()     do{ (RCC->AHB1RSTR  |= (1<<2)) ; (RCC->AHB1RSTR  &=~(1<<2)); }while(0)
#define GPIOD_REG_RST()		do{ (RCC->AHB1RSTR  |= (1<<3)) ; (RCC->AHB1RSTR  &=~(1<<3)); }while(0)
#define GPIOE_REG_RST()		do{ (RCC->AHB1RSTR  |= (1<<4)) ; (RCC->AHB1RSTR  &=~(1<<4)); }while(0)
#define GPIOH_REG_RST()		do{ (RCC->AHB1RSTR  |= (1<<7)) ; (RCC->AHB1RSTR  &=~(1<<7)); }while(0)


#define SPI1_REG_RST()		do{ (RCC->APB2RSTR |= (1<<12)) ; (RCC->APB2RSTR  &=~(1<<12)); }while(0)
#define SPI2_REG_RST()	    do{ (RCC->APB1RSTR  |= (1<<1)) ; (RCC->APB1RSTR  &=~(1<<14));}while(0)
#define SPI3_REG_RST()    	do{ (RCC->APB1RSTR  |= (1<<2)) ; (RCC->APB1RSTR  &=~(1<<15)); }while(0)
#define SPI4_REG_RST()		do{ (RCC->APB2RSTR  |= (1<<13)) ; (RCC->APB2RSTR  &=~(1<<13)); }while(0)



#define I2C1_REG_RST()		do{ (RCC->APB1RSTR |= (1<<21)) ; (RCC->APB1RSTR  &=~(1<<21)); }while(0)
#define I2C2_REG_RST()	    do{ (RCC->APB1RSTR  |= (1<<22)) ; (RCC->APB1RSTR  &=~(1<<22));}while(0)
#define I2C3_REG_RST()    	do{ (RCC->APB1RSTR  |= (1<<23)) ; (RCC->APB1RSTR  &=~(1<<23)); }while(0)


#define USART1_REG_RST()    do{ (RCC->APB2RSTR) |= (1 << 4) ; (RCC->APB2RSTR &= ~(1<<4));}while(0)
#define USART6_REG_RST()    do{ (RCC->APB2RSTR) |= (1 << 5) ; (RCC->APB2RSTR &= ~(1<<5));}while(0)
#define USART2_REG_RST()    do{ (RCC->APB1RSTR) |= (1 << 17) ; (RCC->APB2RSTR &= ~(1<<17));}while(0)


//some generic macros
#define ENABLE 			1
#define DISABLE 		0
#define SET    			ENABLE
#define RESET  			DISABLE
#define GPIO_PIN_SET  	SET
#define GPIO_PIN_RESET  RESET
#define FLAG_RESET		RESET
#define FLAG_SET		SET



#define  GPIO_BASEADDR_TO_CODE(x) ((x == GPIOA)?0:\
									(x == GPIOB)?1:\
									(x == GPIOC)?2:\
									(x == GPIOD)?3:\
									(x == GPIOE)?4:\
									(x == GPIOH)?7:0)
#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10	40

#define IRQ_NO_SPI1		35
#define IRQ_NO_SPI2		36
#define IRQ_NO_SPI3		51
#define IRQ_NO_SPI4		84
#define IRQ_NO_I2C1_EV  31
#define IRQ_NO_I2C1_ER  32

/******************************************************************************************
 *Bit position definitions of SPI peripheral
 ******************************************************************************************/
/*
 * Bit position definitions SPI_CR1
 */
#define SPI_CR1_CPHA     				 0
#define SPI_CR1_CPOL      				 1
#define SPI_CR1_MSTR     				 2
#define SPI_CR1_BR   					 3
#define SPI_CR1_SPE     				 6
#define SPI_CR1_LSBFIRST   			 	 7
#define SPI_CR1_SSI     				 8
#define SPI_CR1_SSM      				 9
#define SPI_CR1_RXONLY      		 	10
#define SPI_CR1_DFF     			 	11
#define SPI_CR1_CRCNEXT   			 	12
#define SPI_CR1_CRCEN   			 	13
#define SPI_CR1_BIDIOE     			 	14
#define SPI_CR1_BIDIMODE      			15

/*
 * Bit position definitions SPI_CR2
 */
#define SPI_CR2_RXDMAEN		 			0
#define SPI_CR2_TXDMAEN				 	1
#define SPI_CR2_SSOE				 	2
#define SPI_CR2_FRF						4
#define SPI_CR2_ERRIE					5
#define SPI_CR2_RXNEIE				 	6
#define SPI_CR2_TXEIE					7


/*
 * Bit position definitions SPI_SR
 */
#define SPI_SR_RXNE						0
#define SPI_SR_TXE				 		1
#define SPI_SR_CHSIDE				 	2
#define SPI_SR_UDR					 	3
#define SPI_SR_CRCERR				 	4
#define SPI_SR_MODF					 	5
#define SPI_SR_OVR					 	6
#define SPI_SR_BSY					 	7
#define SPI_SR_FRE					 	8

//I2C CR1 bit fields
#define I2C_CR1_PE				0
#define	I2C_CR1_SMBUS			1
#define I2C_CR1_SMBTYPE			3
#define I2C_CR1_ENARP			4
#define I2C_CR1_ENPEC			5
#define I2C_CR1_ENGC			6
#define I2C_CR1_NOSTRETCH		7
#define I2C_CR1_START			8
#define I2C_CR1_STOP			9
#define I2C_CR1_ACK				10
#define I2C_CR1_POS				11
#define I2C_CR1_PEC				12
#define I2C_CR1_ALERT			13
#define I2C_CR1_SWRST			15

//I2C CR2 bit fields
#define  I2C_CR2_FREQ   	0      // FREQ alanının başlangıç biti
#define	 I2C_CR2_ITERREN    8
#define  I2C_CR2_ITEVTEN    9
#define  I2C_CR2_ITBUFEN    10
#define  I2C_CR2_DMAEN      11
#define  I2C_CR2_LAST		12

//I2C OAR1 BİTFİELDS
#define I2C_ADD0_Pos     0    // ADD0 bit pozisyonu
#define I2C_ADD7_1_Pos   1    // ADD[7:1] başlangıç biti
#define I2C_ADD9_8_Pos   8    // ADD[9:8] başlangıç biti
#define I2C_ADD_MODE_Pos 15   // ADDMODE bit pozisyonu (7-bit veya 10-bit adresleme seçimi)



//I2C SR1 BİT FİELDS
#define I2C_SR1_SB        0   // Start bit
#define I2C_SR1_ADDR      1   // Address sent (master mode) / matched (slave mode)
#define I2C_SR1_ADD10     2   // 10-bit header sent (master mode)
#define I2C_SR1_BTF       3   // Byte transfer finished
#define I2C_SR1_STOPF     4   // Stop detection (slave mode)
#define I2C_SR1_RXNE      6   // Data register not empty (receiver)
#define I2C_SR1_TXE       7   // Data register empty (transmitter)
#define I2C_SR1_BERR      8   // Bus error
#define I2C_SR1_ARLO      9   // Arbitration lost
#define I2C_SR1_AF        10  // Acknowledge failure
#define I2C_SR1_OVR       11  // Overrun/Underrun
#define I2C_SR1_PECERR    12  // PEC error in reception
#define I2C_SR1_TIMEOUT   14  // Timeout or Tlow error
#define I2C_SR1_SMBALERT  15  // SMBus alert

//I2C SR BİT FİELDS

#define I2C_SR2_MSL         0   // Master/Slave mode
#define I2C_SR2_BUSY        1   // Bus busy
#define I2C_SR2_TRA         2   // Transmitter/Receiver
#define I2C_SR2_GENCALL     4   // General call address (slave mode)
#define I2C_SR2_SMBDEFAULT  5   // SMBus device default address detected
#define I2C_SR2_SMBHOST     6   // SMBus host header detected
#define I2C_SR2_DUALF       7   // Dual flag (slave mode, dual addressing enabled)
#define I2C_SR2_PEC_STATUS  8   // Packet Error Checking (PEC) value starts at bit 8

//I2C ccr bit fields
#define  I2C_CCR        10
#define  I2C_DUTY	    11
#define  I2C_FS			12

//12C TRİSE BİTFİELDS


#include "stm32f411xx_gpio_driver.h"

#include "stm32f411xx_spi_driver.h"

#include "stm32f4xx_i2c_driver.h"

#include "stm32f411xx.h"

#include "stm32f411xx_rcc_driver.h"

#include "stm32f411xx_usart_driver.h"



#endif /* INC_STM32F411XX_H_ */
