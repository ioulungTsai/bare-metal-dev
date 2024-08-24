/**
 * @file main.c
 * @brief Main application for LED blinking on STM32F407xx
 *
 * This file contains the main application for blinking the green LED
 * on the STM32F407 Discovery board (LED connected to PD12).
 *
 * @author  Brandon Tsai
 * @date    8/24/2024
 */

#include "stm32f407xx.h"
#include "stm32f407xx_gpio.h"

// Simple delay function
static void delay(volatile uint32_t count)
{
    while (count--);
}

int main(void)
{
    // Enable clock for GPIOD
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;

    // Initialize PD12 as output for green LED
    gpio_init(GPIOD, GPIO_PIN_12, GPIO_MODE_OUTPUT, GPIO_OTYPE_PP, GPIO_SPEED_LOW, GPIO_PUPD_NONE);

    while (1)
    {
        // Toggle the LED
        gpio_toggle_pin(GPIOD, GPIO_PIN_12);

        // Delay
        delay(500000);
    }

    return 0;
}
