/**
 * @file stm32f4xx_gpio.h
 * @brief GPIO driver for STM32F4xx series
 *
 * This file contains the GPIO driver API for STM32F4xx microcontrollers.
 * It provides basic functions to configure and control GPIO pins.
 *
 * @author  Brandon Tsai
 * @date    8/23/2024
 */

#ifndef STM32F4XX_GPIO_H
#define STM32F4XX_GPIO_H

#include "stm32f407xx.h"

/* GPIO Pin definitions */
#define GPIO_PIN_12       (1U << 12) // Green LED is on PD12


/* GPIO Mode definitions */
#define GPIO_MODE_INPUT   0x00U
#define GPIO_MODE_OUTPUT  0x01U
#define GPIO_MODE_AF      0x02U
#define GPIO_MODE_ANALOG  0x03U

/* GPIO Output Type definitions */
#define GPIO_OTYPE_PP     0x00U // Push-pull
#define GPIO_OTYPE_OD     0x01U // Open-drain

/* GPIO Output Speed definitions */
#define GPIO_SPEED_LOW    0x00U
#define GPIO_SPEED_MEDIUM 0x01U
#define GPIO_SPEED_HIGH   0x02U
#define GPIO_SPEED_VHIGH  0x03U

/* GPIO Pull-up/Pull-down definitions */
#define GPIO_PUPD_NONE    0x00U
#define GPIO_PUPD_PU      0x01U
#define GPIO_PUPD_PD      0x02U

/* Function prototypes */
void GPIO_Init(GPIO_RegDef_t *GPIOx, uint32_t pin, uint8_t mode, uint8_t otype, uint8_t speed, uint8_t pupd);
void GPIO_WritePin(GPIO_RegDef_t *GPIOx, uint32_t pin, uint8_t state);
uint8_t GPIO_ReadPin(GPIO_RegDef_t *GPIOx, uint32_t pin);
void GPIO_TogglePin(GPIO_RegDef_t *GPIOx, uint32_t pin);

#endif /* STM32F4XX_GPIO_H */
