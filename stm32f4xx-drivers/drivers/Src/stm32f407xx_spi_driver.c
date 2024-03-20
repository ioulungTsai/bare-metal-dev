/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: 14 Mar 2024
 *      Author: Brandon Tsai
 */

#include "stm32f407xx_spi_driver.h"


/*********************************************************************
 * @fn      		  - SPI_PeriClockControl
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

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
    {
        if (pSPIx == SPI1)
        {
            SPI1_PCLK_EN();
        }
        else if (pSPIx == SPI2)
        {
            SPI2_PCLK_EN();
        }
        else if (pSPIx == SPI3)
        {
            SPI3_PCLK_EN();
        }
    }
    else
    {
        if (pSPIx == SPI1)
        {
            SPI1_PCLK_DI();
        }
        else if (pSPIx == SPI2)
        {
            SPI2_PCLK_DI();
        }
        else if (pSPIx == SPI3)
        {
            SPI3_PCLK_DI();
        }
    }
}


/*
 * Init and De-init
 */

void SPI_Init(SPI_Handle_t *pSPIHandle)
{
    // Configure the SPI_CR1 register
    uint32_t tempReg = 0;

    // 1. Configure the device mode
    tempReg |= pSPIHandle->SPIConfig.SPI_DeviceMode << 2;

    // 2. Configure the Bus configuration
    if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD) {
        // BIDI mode should be cleared
        tempReg &= ~( 1 << 15 );
    } else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD) {
        // BIDI mode should be set
        tempReg |= ( 1 << 15 );
    } else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY) {
        // BIDI mode should be cleared
        tempReg &= ~( 1 << 15 );

        // RXONLY bit must be set
        tempReg |= ( 1 << 10 );
    }

    // 3. Configure the SPI serial clock speed (baud rate)
    tempReg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << 3;

    // 4. Configure the DFF
    tempReg |= pSPIHandle->SPIConfig.SPI_DFF << 11;

    // 5. Configure the CPOL
    tempReg |= pSPIHandle->SPIConfig.SPI_CPOL << 1;

    // 6. Configure the CPHA
    tempReg |= pSPIHandle->SPIConfig.SPI_CPHA << 0;

    pSPIHandle->pSPIx->CR1 = tempReg;
}
