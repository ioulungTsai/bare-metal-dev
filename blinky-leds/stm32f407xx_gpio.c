/**
 * @file stm32f4xx_gpio.c
 * @brief GPIO driver implementation for STM32F4xx series
 *
 * This file contains the GPIO driver implementation for STM32F4xx microcontrollers.
 * It provides basic functions to configure and control GPIO pins.
 *
 * @author  Brandon Tsai
 * @date    8/23/2024
 */

#include "stm32f407xx_gpio.h"

/**
 * @brief Initialize a GPIO pin
 * @param GPIOx: GPIO port base address
 * @param pin: GPIO pin number
 * @param mode: GPIO mode (Input, Output, Alternate Function, Analog)
 * @param otype: GPIO output type (Push-pull or Open-drain)
 * @param speed: GPIO output speed
 * @param pupd: GPIO pull-up/pull-down configuration
 */
void gpio_init(GPIO_RegDef_t *GPIOx, uint32_t pin, uint8_t mode, uint8_t otype, uint8_t speed, uint8_t pupd)
{
    // Configure GPIO mode
    GPIOx->MODER &= ~(0x3U << (2 * pin));
    GPIOx->MODER |= (mode << (2 * pin));

    // Configure GPIO output type
    GPIOx->OTYPER &= ~(0x1U << pin);
    GPIOx->OTYPER |= (otype << pin);

    // Configure GPIO output speed
    GPIOx->OSPEEDR &= ~(0x3U << (2 * pin));
    GPIOx->OSPEEDR |= (speed << (2 * pin));

    // Configure GPIO pull-up/pull-down
    GPIOx->PUPDR &= ~(0x3U << (2 * pin));
    GPIOx->PUPDR |= (pupd << (2 * pin));
}

/**
 * @brief Write a value to a GPIO pin
 * @param GPIOx: GPIO port base address
 * @param pin: GPIO pin number
 * @param state: Pin state (0 or 1)
 */
void gpio_write_pin(GPIO_RegDef_t *GPIOx, uint32_t pin, uint8_t state)
{
    if (state) {
        GPIOx->BSRR = pin;
    } else {
        GPIOx->BSRR = (pin << 16);
    }
}

/**
 * @brief Read the state of a GPIO pin
 * @param GPIOx: GPIO port base address
 * @param pin: GPIO pin number
 * @return Pin state (0 or 1)
 */
uint8_t gpio_read_pin(GPIO_RegDef_t *GPIOx, uint32_t pin)
{
    return (GPIOx->IDR & pin) ? 1 : 0;
}

/**
 * @brief Toggle the state of a GPIO pin
 * @param GPIOx: GPIO port base address
 * @param pin: GPIO pin number
 */
void gpio_toggle_pin(GPIO_RegDef_t *GPIOx, uint32_t pin)
{
    GPIOx->ODR ^= pin;
}
