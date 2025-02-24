/*
 * stm32f411xx_gpio_driver.h
 *
 *  Created on: Jan 27, 2025
 *      Author: omery
 */

#ifndef INC_STM32F411XX_GPIO_DRIVER_H_
#define INC_STM32F411XX_GPIO_DRIVER_H_

#include "stm32f411xx.h"  // Include the main MCU header file

/*
 * @brief Structure to configure GPIO pin settings
 */
typedef struct {
	uint8_t GPIO_PinNumber;       // Specifies the GPIO pin number (0-15)
	uint8_t GPIO_PinMode;         // Specifies the mode of the GPIO pin (input, output, alternate function, or analog)
	uint8_t GPIO_PinSpeed;        // Specifies the speed of the GPIO pin (low, medium, fast, high)
	uint8_t GPIO_PinPuPdControl;  // Specifies pull-up/pull-down configuration (no pull-up, pull-up, pull-down)
	uint8_t GPIO_PinOPtype;       // Specifies the output type (Push-Pull or Open-Drain)
	uint8_t GPIO_PinAltFunMode;   // Specifies the alternate function mode (if alternate function mode is selected)
} GPIO_PinConfig_t;

/*
 * @brief Structure to handle a GPIO pin
 * Contains the GPIO port base address and pin configuration settings.
 */
typedef struct {
	GPIO_RegDef_t *pGPIOx;         // Pointer to GPIO port base address (GPIOA, GPIOB, etc.)
	GPIO_PinConfig_t GPIO_PinConfig; // GPIO pin configuration structure
} GPIO_Handle_t;

/*
 * GPIO Peripheral Clock Control
 * Enables or disables the peripheral clock for the specified GPIO port.
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
 * GPIO Initialization and Deinitialization
 * Configures the specified GPIO pin based on user settings.
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);  // Resets all registers of a GPIO port

/*
 * GPIO Read Functions
 * Reads input pin or port values.
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);

/*
 * GPIO Write Functions
 * Writes values to an output pin or port.
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint8_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);  // Toggles the output state of a pin

/*
 * GPIO Interrupt Configuration and Handling
 * Configures interrupt settings for a GPIO pin and handles IRQ events.
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);


/*
 * GPIO Pin Mode Macros
 * Defines various operating modes for GPIO pins.
 */
#define GPIO_MODE_INPUT     0  // GPIO input mode
#define GPIO_MODE_OUTPUT    1  // GPIO output mode
#define GPIO_MODE_ALTFN     2  // GPIO alternate function mode
#define GPIO_MODE_ANALOG    3  // GPIO analog mode
#define GPIO_MODE_IT_FT     4  // Interrupt mode: Falling edge trigger
#define GPIO_MODE_IT_RT     5  // Interrupt mode: Rising edge trigger
#define GPIO_MODE_IT_FRT    6  // Interrupt mode: Both edges trigger

/*
 * GPIO Output Type Macros
 * Defines whether the output is Push-Pull or Open-Drain.
 */
#define GPIO_OPTYPE_PP      0  // Push-Pull output type
#define GPIO_OPTYPE_OD      1  // Open-Drain output type

/*
 * GPIO Speed Macros
 * Defines the output speed of a GPIO pin.
 */
#define GPIO_SPEED_LOW      0  // Low speed
#define GPIO_SPEED_MEDIUM   1  // Medium speed
#define GPIO_SPEED_FAST     2  // Fast speed
#define GPIO_SPEED_HIGH     3  // Very high speed

/*
 * GPIO Pull-Up / Pull-Down Macros
 * Defines whether pull-up, pull-down, or no pull resistor is used.
 */
#define GPIO_NO_PUPD        0  // No pull-up/pull-down
#define GPIO_PIN_PU         1  // Pull-up enabled
#define GPIO_PIN_PD         2  // Pull-down enabled

/*
 * GPIO Pin Number Macros
 * Defines GPIO pin numbers for ease of use.
 */
#define GPIO_PIN_NO_0       0
#define GPIO_PIN_NO_1       1
#define GPIO_PIN_NO_2       2
#define GPIO_PIN_NO_3       3
#define GPIO_PIN_NO_4       4
#define GPIO_PIN_NO_5       5
#define GPIO_PIN_NO_6       6
#define GPIO_PIN_NO_7       7
#define GPIO_PIN_NO_8       8
#define GPIO_PIN_NO_9       9
#define GPIO_PIN_NO_10      10
#define GPIO_PIN_NO_11      11
#define GPIO_PIN_NO_12      12
#define GPIO_PIN_NO_13      13
#define GPIO_PIN_NO_14      14
#define GPIO_PIN_NO_15      15



#endif /* INC_STM32F411XX_GPIO_DRIVER_H_ */
