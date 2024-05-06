/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: May 5 2024
 *      Author: admin
 */

#include "stm32f407xx_i2c_driver.h"


/*********************************************************************
 * @fn      		  - I2C_PeriClockControl
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
    {
        if (pI2Cx == I2C1)
        {
            I2C1_PCLK_EN();
        }
        else if (pI2Cx == I2C2)
        {
            I2C2_PCLK_EN();
        }
        else if (pI2Cx == I2C3)
        {
            I2C3_PCLK_EN();
        }
    }
    else
    {
        if (pI2Cx == I2C1)
        {
            I2C1_PCLK_DI();
        }
        else if (pI2Cx == I2C2)
        {
            I2C2_PCLK_DI();
        }
        else if (pI2Cx == I2C3)
        {
            I2C3_PCLK_DI();
        }
    }
}


/*
 * Init and De-init
 */

void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
    //todo
}


/*
 * I2C Peripheral Control
 */

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
    if(EnorDi == ENABLE)
    {
        pI2Cx->CR1 |= (1 << I2C_CR1_PE);
    } else {
        pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
    }
}


/*
 * IRQ Configuration
 */

void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
    if(EnorDi == ENABLE) {
        if(IRQNumber <= 31) {
            // Program ISER0 register
            *NVIC_ISER0 |= (1 << IRQNumber);
        } else if(IRQNumber > 31 && IRQNumber < 64) {
            // Program ISER1 register
            *NVIC_ISER1 |= (1 << IRQNumber % 32);
        } else if(IRQNumber >= 64 && IRQNumber < 96) {
            // Program ISER2 register
            *NVIC_ISER2 |= (1 << IRQNumber % 64);
        }
    } else {
         if(IRQNumber <= 31) {
            // Program ICER0 register
            *NVIC_ICER0 |= (1 << IRQNumber);
        } else if(IRQNumber > 31 && IRQNumber < 64) {
            // Program ICER1 register
            *NVIC_ICER1 |= (1 << IRQNumber % 32);
        } else if(IRQNumber >= 64 && IRQNumber < 96) {
            // Program ICER2 register
            *NVIC_ICER2 |= (1 << IRQNumber % 64);
        }
    }
}

void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
    // 1. Find out the IPR Register
    uint8_t iprx = IRQNumber / 4;
    uint8_t iprx_section = IRQNumber % 4;

    uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

    *(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}


