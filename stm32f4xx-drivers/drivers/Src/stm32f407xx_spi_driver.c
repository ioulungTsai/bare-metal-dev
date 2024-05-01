/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: 14 Mar 2024
 *      Author: Brandon Tsai
 */

#include "stm32f407xx_spi_driver.h"

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);


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
    // Peripheral Clock Enable
    SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

    // Configure the SPI_CR1 register
    uint32_t tempReg = 0;

    // 1. Configure the device mode
    tempReg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

    // 2. Configure the Bus configuration
    if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD) {
        // BIDI mode should be cleared
        tempReg &= ~( 1 << SPI_CR1_BIDIMODE );
    } else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD) {
        // BIDI mode should be set
        tempReg |= ( 1 << SPI_CR1_BIDIMODE );
    } else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY) {
        // BIDI mode should be cleared
        tempReg &= ~( 1 << SPI_CR1_BIDIMODE );

        // RXONLY bit must be set
        tempReg |= ( 1 << SPI_CR1_RXONLY );
    }

    // 3. Configure the SPI serial clock speed (baud rate)
    tempReg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

    // 4. Configure the DFF
    tempReg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

    // 5. Configure the CPOL
    tempReg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

    // 6. Configure the CPHA
    tempReg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

    // 7. Configure the SSM
    tempReg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM; 

    pSPIHandle->pSPIx->CR1 = tempReg;
}

void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
 //todo
}


/*
 * SPI Get Flag Status
 */

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
    if(pSPIx->SR & FlagName)
    {
        return FLAG_SET;
    }
    return FLAG_RESET;
}


/*
 * SPI Send Data - This implement is a Blocking Call
 */

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
    while(Len > 0) {
        // 1. Wait until TXE is set
        while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

        // 2. Check the DFF bit in CR1
        if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
        {
            // 16 bit DFF
            // 1. Load the data into the DR
            pSPIx->DR = *((uint16_t*)pTxBuffer);
            Len--;
            Len--;
            (uint16_t*)pTxBuffer++;
        } else {
            // 8 bit DFF
            pSPIx->DR = *pTxBuffer;
            Len--;
            pTxBuffer++;
        }
    }
}


/*
 * SPI Receive Data - This implement is a Blocking Call
 */

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
    while(Len > 0) {
        // 1. Wait until RXE is set
        while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

        // 2. Check the DFF bit in CR1
        if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
        {
            // 16 bit DFF
            // 1. Load the data from DR to Rx Buffer address
            *((uint16_t*)pRxBuffer) = pSPIx->DR;
            Len--;
            Len--;
            (uint16_t*)pRxBuffer++;
        } else {
            // 8 bit DFF
            *pRxBuffer = pSPIx->DR;
            Len--;
            pRxBuffer++;
        }
    }
}


/*
 * SPI Peripheral Control
 */

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
    if(EnorDi == ENABLE)
    {
        pSPIx->CR1 |= (1 << SPI_CR1_SPE);
    } else {
        pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
    }
}


/*
 * SPI SSI Configure
 */

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
    if(EnorDi == ENABLE)
    {
        pSPIx->CR1 |= (1 << SPI_CR1_SSI);
    } else {
        pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
    }
}


/*
 * SPI SSOE Configure
 */

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
    if(EnorDi == ENABLE)
    {
        pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
    } else {
        pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
    }
}


/*
 * IRQ Configuration
 */

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
    // 1. Find out the IPR Register
    uint8_t iprx = IRQNumber / 4;
    uint8_t iprx_section = IRQNumber % 4;

    uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

    *(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}


/*
 * SPI Send Data - This implement is a Interrupt Call
 */

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
    uint8_t state = pSPIHandle->TxState;

    if (state != SPI_BUSY_IN_TX) 
    {
        // 1. Save the Tx buffer address and Len information in some global variables
        pSPIHandle->pTxBuffer = pTxBuffer;
        pSPIHandle->TxLen = Len;

        // 2. Maeke the SPI state as busy in transmission so that no other code can take over
        //    same SPI peripheral until transmission is over
        pSPIHandle->TxState = SPI_BUSY_IN_TX;

        // 3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
        pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);

        // 4. Data Transmission will be handle by the ISR code (will implement later)
    }

    return state;
}


/*
 * SPI Receive Data - This implement is a Interrupt Call
 */

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
    uint8_t state = pSPIHandle->RxState;

        if (state != SPI_BUSY_IN_RX) 
        {
            // 1. Save the Rx buffer address and Len information in some global variables
            pSPIHandle->pRxBuffer = pRxBuffer;
            pSPIHandle->RxLen = Len;

            // 2. Maeke the SPI state as busy in reception so that no other code can take over
            //    same SPI peripheral until reception is over
            pSPIHandle->RxState = SPI_BUSY_IN_RX;

            // 3. Enable the RXNEIE control bit to get interrupt whenever TXE flag is set in SR
            pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);

            // 4. Data reception will be handle by the ISR code (will implement later)
        }

    return state;
}


/*
 * ISR handling
 */ 

void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{
    uint8_t temp1, temp2;

    // 1. Check for TXE
    temp1 = pSPIHandle->pSPIx->SR & ( 1 << SPI_SR_TXE );
    temp2 = pSPIHandle->pSPIx->CR2 & ( 1 << SPI_CR2_TXEIE );

    if (temp1 && temp2) {
        // Handle TXE
        spi_txe_interrupt_handle(pSPIHandle);
    }

    // 2. Check for RXNE
    temp1 = pSPIHandle->pSPIx->SR & ( 1 << SPI_SR_RXNE );
    temp2 = pSPIHandle->pSPIx->CR2 & ( 1 << SPI_CR2_RXNEIE );

    if (temp1 && temp2) {
        // Handle RXNE
        spi_rxne_interrupt_handle(pSPIHandle);
    }

    // 3. Check for OVR flag
    temp1 = pSPIHandle->pSPIx->SR & ( 1 << SPI_SR_OVR );
    temp2 = pSPIHandle->pSPIx->CR2 & ( 1 << SPI_CR2_ERRIE );

    if (temp1 && temp2) {
        // Handle RXNE
        spi_ovr_err_interrupt_handle(pSPIHandle);
    }
}


// Helper functions implementation

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
    // 1. Check the DFF bit in CR1
    if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
    {
        // 16 bit DFF
        // 1. Load the data into the DR
        pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
        pSPIHandle->TxLen--;
        pSPIHandle->TxLen--;
        (uint16_t*)pSPIHandle->pTxBuffer;
    } else {
        // 8 bit DFF
        pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
        pSPIHandle->TxLen--;
        pSPIHandle->pTxBuffer;
    }

    if(!pSPIHandle->TxLen)
    {
        // TxLen is zero, so close the SPI transmission and inform the application that
        // TX is over.

        // This prevents interrupts from setting up of TXE flag
        pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_TXEIE );
        pSPIHandle->pTxBuffer = NULL;
        pSPIHandle->TxLen = 0;
        pSPIHandle->TxState = SPI_READY;
        SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
    }
}

static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
    // 2. Check the DFF bit in CR1
    if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
    {
        // 16 bit DFF
        // 1. Load the data from DR to Rx Buffer address
        *((uint16_t*)pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
        pSPIHandle->RxLen -= 2;
        pSPIHandle->pRxBuffer++;
        pSPIHandle->pRxBuffer++;
    } else {
        // 8 bit DFF
        *(pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
        pSPIHandle->RxLen--;
        pSPIHandle->pRxBuffer++;
    }

    if(!pSPIHandle->TxLen)
    {
        // Reception is complete
        // Turn off the RXNEIE interrupt
        pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_RXNEIE );
        pSPIHandle->pRxBuffer = NULL;
        pSPIHandle->RxLen = 0;
        pSPIHandle->RxState = SPI_READY;
        SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
    }
}

static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{

}
