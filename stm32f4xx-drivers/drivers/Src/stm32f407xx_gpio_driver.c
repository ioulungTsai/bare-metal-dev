#include "stm32f407xx_gpio_driver.h"


/**
 * @fn void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
 * @brief Control the peripheral clock for a GPIO port.
 *
 * This function is responsible for enabling or disabling the peripheral clock for a specified GPIO port.
 * When enabling the clock, it calls the corresponding clock enable function for the specified GPIO port.
 * When disabling the clock, it calls the corresponding clock disable function for the specified GPIO port.
 *
 * @param pGPIOx Pointer to the GPIO peripheral's register structure.
 * @param EnorDi  Specify whether to enable (ENABLE) or disable (DISABLE) the peripheral clock.
 *
 * @note This function assumes that clock enable and disable functions (e.g., GPIOA_PCLK_EN, GPIOA_PCLK_DI)
 *       are defined elsewhere in the codebase for each GPIO port.
 *
 * @warning It is the responsibility of the user to ensure that clock enable and disable functions are defined
 *          and handle the clock control for the specific microcontroller being used.
 *
 * @see GPIOA_PCLK_EN, GPIOA_PCLK_DI, GPIOB_PCLK_EN, GPIOB_PCLK_DI, ... (similar functions for other GPIO ports)
 */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
    {
        if (pGPIOx == GPIOA)
        {
            GPIOA_PCLK_EN();
        }
        else if (pGPIOx == GPIOB)
        {
            GPIOB_PCLK_EN();
        }
        else if (pGPIOx == GPIOC)
        {
            GPIOC_PCLK_EN();
        }
        else if (pGPIOx == GPIOD)
        {
            GPIOD_PCLK_EN();
        }
        else if (pGPIOx == GPIOE)
        {
            GPIOE_PCLK_EN();
        }
        else if (pGPIOx == GPIOF)
        {
            GPIOF_PCLK_EN();
        }
        else if (pGPIOx == GPIOG)
        {
            GPIOG_PCLK_EN();
        }
        else if (pGPIOx == GPIOH)
        {
            GPIOH_PCLK_EN();
        }
        else if (pGPIOx == GPIOI)
        {
            GPIOI_PCLK_EN();
        }
    }
    else
    {
        if (pGPIOx == GPIOA)
        {
            GPIOA_PCLK_DI();
        }
        else if (pGPIOx == GPIOB)
        {
            GPIOB_PCLK_DI();
        }
        else if (pGPIOx == GPIOC)
        {
            GPIOC_PCLK_DI();
        }
        else if (pGPIOx == GPIOD)
        {
            GPIOD_PCLK_DI();
        }
        else if (pGPIOx == GPIOE)
        {
            GPIOE_PCLK_DI();
        }
        else if (pGPIOx == GPIOF)
        {
            GPIOF_PCLK_DI();
        }
        else if (pGPIOx == GPIOG)
        {
            GPIOG_PCLK_DI();
        }
        else if (pGPIOx == GPIOH)
        {
            GPIOH_PCLK_DI();
        }
        else if (pGPIOx == GPIOI)
        {
            GPIOI_PCLK_DI();
        }
    }
}

/*
 * Init and De-init
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{   
}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
}

/*
 * Data read and write
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
    return 0;
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
    return 0;
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
}

/*
 * IRQ Configuration and ISR handling
 */

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi)
{
}

void GPIO_IRQHandling(uint8_t PinNumber)
{
}
