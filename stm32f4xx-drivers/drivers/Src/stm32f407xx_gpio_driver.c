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
    uint32_t temp = 0; //temperary register

    // 1. Configure the mode of GPIO pin
    if(pGPIOHandle->GPIO_Pinconfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {
        temp = (pGPIOHandle->GPIO_Pinconfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber));
        pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber);
        pGPIOHandle->pGPIOx->MODER |= temp;
    } else {
        // This part reserve for interrupt mode
        if(pGPIOHandle->GPIO_Pinconfig.GPIO_PinMode == GPIO_MODE_IT_FT){
            // 1. Configure the FTSR
        }else if(pGPIOHandle->GPIO_Pinconfig.GPIO_PinMode == GPIO_MODE_IT_RT){
            // 1. Configure the RTSR
        }else if(pGPIOHandle->GPIO_Pinconfig.GPIO_PinMode == GPIO_MODE_IT_RFT){
            // 1. Configure both FTSR and RTSR
        }

        // 2. Configure the GPIO port selection in SYSCFG_EXTICR

        // 3. Enable the EXTI Interrupt delivery using IMR
    }
    
    temp = 0;

    // 2. Configure the speed
    temp = (pGPIOHandle->GPIO_Pinconfig.GPIO_PinOPSpeed << (2 * pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber);
    pGPIOHandle->pGPIOx->OSPEEDR |= temp;

    temp = 0;

    // 3. Configure the pull up pull down setting
    temp = (pGPIOHandle->GPIO_Pinconfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber);
    pGPIOHandle->pGPIOx->PUPDR |= temp;
    
    temp = 0;
    
    // 4. Configure the output type
    temp = (pGPIOHandle->GPIO_Pinconfig.GPIO_PinOPType << pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber);
    pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber);
    pGPIOHandle->pGPIOx->OTYPER |= temp;
    
    temp = 0;
    
    // 5. Configure the alternate functionality
    if(pGPIOHandle->GPIO_Pinconfig.GPIO_PinMode == GPIO_MODE_ALTFN) {
        // Configure the alternate function registers 
        uint8_t temp1, temp2;

        temp1 = pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber / 8;
        temp2 = pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber % 8;
        pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));
        pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_Pinconfig.GPIO_PinAltFunMode << (4 * temp2));
    }
}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
    if (pGPIOx == GPIOA)
    {
        GPIOA_REG_RESET();
    }
    else if (pGPIOx == GPIOB)
    {
        GPIOB_REG_RESET();
    }
    else if (pGPIOx == GPIOC)
    {
        GPIOC_REG_RESET();
    }
    else if (pGPIOx == GPIOD)
    {
        GPIOD_REG_RESET();
    }
    else if (pGPIOx == GPIOE)
    {
        GPIOE_REG_RESET();
    }
    else if (pGPIOx == GPIOF)
    {
        GPIOF_REG_RESET();
    }
    else if (pGPIOx == GPIOG)
    {
        GPIOG_REG_RESET();
    }
    else if (pGPIOx == GPIOH)
    {
        GPIOH_REG_RESET();
    }
    else if (pGPIOx == GPIOI)
    {
        GPIOI_REG_RESET();
    }
}

/*
 * Data read and write
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
    uint8_t value;

    value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
    
    return value;
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
    uint16_t value;

    value = (uint16_t)pGPIOx->IDR;
    
    return value;
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
    if(Value == GPIO_PIN_SET) {
        // Write 1 to the output data register at the bit field corresponding to the pin number
        pGPIOx->ODR |= (1 << PinNumber); 
    } else {
        pGPIOx->ODR &= ~(1 << PinNumber);
    }
}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
    pGPIOx->ODR = Value;
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
    pGPIOx->ODR ^= (1 << PinNumber);
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
