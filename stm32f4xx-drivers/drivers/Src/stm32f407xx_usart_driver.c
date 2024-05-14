/*
 * stm32f407xx_usart_driver.c
 *
 *  Created on: May 13 2024
 *      Author: Brandon
 */

#include "stm32f407xx_usart_driver.h"


/*********************************************************************
 * @fn      		  - USART_PeriClockControl
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

void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
    {
        if (pUSARTx == USART1)
        {
            USART1_PCLK_EN();
        }
        else if (pUSARTx == USART2)
        {
            USART2_PCLK_EN();
        }
        else if (pUSARTx == USART3)
        {
            USART3_PCLK_EN();
        }
        else if (pUSARTx == UART4)
        {
            UART4_PCLK_EN();
        }
        else if (pUSARTx == UART5)
        {
            UART5_PCLK_EN();
        }
        else if (pUSARTx == USART6)
        {
            USART6_PCLK_EN();
        }
    }
    else
    {
         if (pUSARTx == USART1)
        {
            USART1_PCLK_DI();
        }
        else if (pUSARTx == USART2)
        {
            USART2_PCLK_DI();
        }
        else if (pUSARTx == USART3)
        {
            USART3_PCLK_DI();
        }
        else if (pUSARTx == UART4)
        {
            UART4_PCLK_DI();
        }
        else if (pUSARTx == UART5)
        {
            UART5_PCLK_DI();
        }
        else if (pUSARTx == USART6)
        {
            USART6_PCLK_DI();
        }
    }
}


/*
 * Init and De-init
 */

void USART_DeInit(USART_RegDef_t *pUSARTx)
{
    // TODO. Refer to the RCC reset registers
}


void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi)
{
    if(EnOrDi == ENABLE)
    {
        pUSARTx->CR1 |= ( 1 << USART_CR1_UE);
    } else {
        pUSARTx->CR1 &= ~(1 << USART_CR1_UE);
    }
}

uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx , uint32_t FlagName)
{
    if(pUSARTx->SR & FlagName)
    {
        return FLAG_SET;
    }
    return FLAG_RESET;
}

void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName)
{
    pUSARTx->SR &= ~( StatusFlagName );
}


/*
 * IRQ Configuration
 */

void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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

void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
    // 1. Find out the IPR Register
    uint8_t iprx = IRQNumber / 4;
    uint8_t iprx_section = IRQNumber % 4;

    uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

    *(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}


__weak void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t AppEvent)
{
    // This is a weak implementation. The user application may override this function.
}
