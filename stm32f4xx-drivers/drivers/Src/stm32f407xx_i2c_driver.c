/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: May 5 2024
 *      Author: admin
 */

#include "stm32f407xx_i2c_driver.h"

static void I2C_GenerateStartCondition(I2C_RegDef_t* pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t* pI2Cx, uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t* pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_Handle_t* pI2CHandle);

static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t* pI2CHandle);
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t* pI2CHandle);

static void
I2C_GenerateStartCondition(I2C_RegDef_t* pI2Cx) {
    pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

static void
I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t* pI2Cx, uint8_t SlaveAddr) {
    SlaveAddr = SlaveAddr << 1;
    SlaveAddr &= ~(1); // SlaveAddr is Slave address + r/w bit = 0
    pI2Cx->DR = SlaveAddr;
}

static void
I2C_ExecuteAddressPhaseRead(I2C_RegDef_t* pI2Cx, uint8_t SlaveAddr) {
    SlaveAddr = SlaveAddr << 1;
    SlaveAddr |= 1; // SlaveAddr is Slave address + r/w bit = 1
    pI2Cx->DR = SlaveAddr;
}

static void
I2C_ClearADDRFlag(I2C_Handle_t* pI2CHandle) {
    uint32_t dummy_read;
    // Check the device mode
    if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)) {
        // The device is in Master mode
        if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {
            if (pI2CHandle->RxSize == 1) {
                // 1. Disable the ACK bit
                I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);

                // 2. Clear the ADDR flag (read SR1, read SR2)
                dummy_read = pI2CHandle->pI2Cx->SR1;
                dummy_read = pI2CHandle->pI2Cx->SR2;
                (void)dummy_read;
            }
        } else {
            // Clear the ADDR flag (read SR1, read SR2)
            dummy_read = pI2CHandle->pI2Cx->SR1;
            dummy_read = pI2CHandle->pI2Cx->SR2;
            (void)dummy_read;
        }
    } else {
        // The device is in Slave mode
        // Clear the ADDR flag (read SR1, read SR2)
        dummy_read = pI2CHandle->pI2Cx->SR1;
        dummy_read = pI2CHandle->pI2Cx->SR2;
        (void)dummy_read;
    }
}

void
I2C_GenerateStopCondition(I2C_RegDef_t* pI2Cx) {
    pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

void
I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t* pI2Cx, uint8_t EnorDi) {
    if (EnorDi == ENABLE) {
        pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);
        pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);
        pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
    } else {
        pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);
        pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
        pI2Cx->CR2 &= ~(1 << I2C_CR2_ITERREN);
    }
}

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

void
I2C_PeriClockControl(I2C_RegDef_t* pI2Cx, uint8_t EnorDi) {
    if (EnorDi == ENABLE) {
        if (pI2Cx == I2C1) {
            I2C1_PCLK_EN();
        } else if (pI2Cx == I2C2) {
            I2C2_PCLK_EN();
        } else if (pI2Cx == I2C3) {
            I2C3_PCLK_EN();
        }
    } else {
        if (pI2Cx == I2C1) {
            I2C1_PCLK_DI();
        } else if (pI2Cx == I2C2) {
            I2C2_PCLK_DI();
        } else if (pI2Cx == I2C3) {
            I2C3_PCLK_DI();
        }
    }
}

/*
 * Init and De-init
 */

void
I2C_Init(I2C_Handle_t* pI2CHandle) {
    uint32_t tempReg = 0;

    // Enable the clock for the I2Cx peripheral
    I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

    // ACK control bit
    tempReg |= pI2CHandle->I2C_Config.I2C_AckControl << 10;
    pI2CHandle->pI2Cx->CR1 = tempReg;

    // Configure the FREQ field of CR2
    tempReg = 0;
    tempReg |= RCC_GetPCLK1Value() / 1000000U;
    pI2CHandle->pI2Cx->CR2 = (tempReg & 0x3F);

    // Program the device own address
    tempReg = 0;
    tempReg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
    tempReg |= (1 << 14);
    pI2CHandle->pI2Cx->OAR1 = tempReg;

    // CCR calculation
    uint16_t ccr_value = 0;
    tempReg = 0;
    if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM) {
        // Mode in standard mode
        ccr_value = (RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
        tempReg |= (ccr_value & 0xFFF);
    } else {
        // Mode in fast mode
        tempReg |= (1 << 15);
        tempReg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);
        if (pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2) {
            ccr_value = (RCC_GetPCLK1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
        } else {
            ccr_value = (RCC_GetPCLK1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
        }
        tempReg |= (ccr_value & 0xFFF);
    }
    pI2CHandle->pI2Cx->CCR = tempReg;

    // TRISE Configuration
    if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM) {
        // Mode in standard mode
        tempReg = (RCC_GetPCLK1Value() / 1000000U) + 1;
    } else {
        // Mode in fast mode
        tempReg = ((RCC_GetPCLK1Value() * 300) / 1000000000U) + 1;
    }
    pI2CHandle->pI2Cx->TRISE = (tempReg & 0x3F);
}

void
I2C_DeInit(I2C_RegDef_t* pI2Cx) {
    // TODO. Refer to the RCC reset registers
}

/*
 * I2C Get Flag Status
 */

uint8_t
I2C_GetFlagStatus(I2C_RegDef_t* pI2Cx, uint32_t FlagName) {
    if (pI2Cx->SR1 & FlagName) {
        return FLAG_SET;
    }
    return FLAG_RESET;
}

/*
 * I2C Master Send Data
 */

void
I2C_MasterSendData(I2C_Handle_t* pI2CHandle, uint8_t* pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr) {
    // 1. Generate the START condition
    I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

    // 2. confirm that start generation is completed by checking the SB flag in the SR1
    //    Note: Until SB is cleared SCL will be stretched (pull to low)
    while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB))
        ;

    // 3. Send the address of the slave with r/nw bit set to w(0) (total 8 bits)
    I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, SlaveAddr);

    // 4. Confirm that address phase is completed by checking the ADDR flag in teh SR1
    while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR))
        ;

    // 5. clear the ADDR flag according to its software sequence
    //   Note: Until ADDR is cleared SCL will be stretched (pulled to LOW)
    I2C_ClearADDRFlag(pI2CHandle);

    // 6. send the data until len becomes 0
    while (Len > 0) {
        while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE))
            ; // Wait till TXE is set
        pI2CHandle->pI2Cx->DR = *pTxBuffer;
        pTxBuffer++;
        Len--;
    }

    // 7. when Len becomes zero wait for TXE=1 and BTF=1 before generating the STOP condition
    //   Note: TXE=1 , BTF=1 , means that both SR and DR are empty and next transmission should begin
    //   when BTF=1 SCL will be stretched (pulled to LOW)
    while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE))
        ;
    while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF))
        ;

    // 8. Generate STOP condition and master need not to wait for the completion of stop condition.
    //   Note: generating STOP, automatically clears the BTF
    if (Sr == I2C_DISABLE_SR) {
        I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
    }
}

/*
 * I2C Master Receive Data
 */

void
I2C_MasterReceiveData(I2C_Handle_t* pI2CHandle, uint8_t* pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr) {
    //1. Generate the START condition
    I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

    //2. Confirm that start generation is completed by checking the SB flag in the SR1
    //   Note: Until SB is cleared SCL will be stretched (pulled to LOW)
    while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB))
        ;

    //3. Send the address of the slave with r/nw bit set to R(1) (total 8 bits)
    I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, SlaveAddr);

    //4. Wait until address phase is completed by checking the ADDR flag in teh SR1
    while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR))
        ;

    // Procedure to read only 1 byte from slave
    if (Len == 1) {
        // Disable Acking
        I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

        // Clear the ADDR flag
        I2C_ClearADDRFlag(pI2CHandle);

        // Wait until  RXNE becomes 1
        while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE))
            ;

        // Generate STOP condition
        if (Sr == I2C_DISABLE_SR) {
            I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
        }
        // Read data in to buffer
        *pRxBuffer = pI2CHandle->pI2Cx->DR;
    }

    // Procedure to read data from slave when Len > 1
    if (Len > 1) {
        // Clear the ADDR flag
        I2C_ClearADDRFlag(pI2CHandle);

        // Read the data until Len becomes zero
        for (uint32_t i = Len; i > 0; i--) {
            // Wait until RXNE becomes 1
            while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE))
                ;

            // if last 2 bytes are remaining
            if (i == 2) {
                //Disable Acking
                I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

                //Generate STOP condition
                if (Sr == I2C_DISABLE_SR) {
                    I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
                }
            }
            //Read the data from data register in to buffer
            *pRxBuffer = pI2CHandle->pI2Cx->DR;

            //Increment the buffer address
            pRxBuffer++;
        }
    }

    //Re-Enable ACKing
    if (pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE) {
        I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
    }
}

void
I2C_ManageAcking(I2C_RegDef_t* pI2Cx, uint8_t EnorDi) {
    if (EnorDi == I2C_ACK_ENABLE) {
        // Enable the ack
        pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
    } else {
        // Disable the ack
        pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
    }
}

/*
 * I2C Peripheral Control
 */

void
I2C_PeripheralControl(I2C_RegDef_t* pI2Cx, uint8_t EnorDi) {
    if (EnorDi == ENABLE) {
        pI2Cx->CR1 |= (1 << I2C_CR1_PE);
    } else {
        pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
    }
}

/*
 * IRQ Configuration
 */

void
I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi) {
    if (EnorDi == ENABLE) {
        if (IRQNumber <= 31) {
            // Program ISER0 register
            *NVIC_ISER0 |= (1 << IRQNumber);
        } else if (IRQNumber > 31 && IRQNumber < 64) {
            // Program ISER1 register
            *NVIC_ISER1 |= (1 << IRQNumber % 32);
        } else if (IRQNumber >= 64 && IRQNumber < 96) {
            // Program ISER2 register
            *NVIC_ISER2 |= (1 << IRQNumber % 64);
        }
    } else {
        if (IRQNumber <= 31) {
            // Program ICER0 register
            *NVIC_ICER0 |= (1 << IRQNumber);
        } else if (IRQNumber > 31 && IRQNumber < 64) {
            // Program ICER1 register
            *NVIC_ICER1 |= (1 << IRQNumber % 32);
        } else if (IRQNumber >= 64 && IRQNumber < 96) {
            // Program ICER2 register
            *NVIC_ICER2 |= (1 << IRQNumber % 64);
        }
    }
}

void
I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {
    // 1. Find out the IPR Register
    uint8_t iprx = IRQNumber / 4;
    uint8_t iprx_section = IRQNumber % 4;

    uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

    *(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}

/*
 * I2C Master Send Data in Interrupt mode
 */

uint8_t
I2C_MasterSendDataIT(I2C_Handle_t* pI2CHandle, uint8_t* pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr) {
    uint8_t busystate = pI2CHandle->TxRxState;

    if ((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX)) {
        pI2CHandle->pTxBuffer = pTxBuffer;
        pI2CHandle->TxLen = Len;
        pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
        pI2CHandle->DevAddr = SlaveAddr;
        pI2CHandle->Sr = Sr;

        //Implement code to Generate START Condition
        I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

        //Implement the code to enable ITBUFEN Control Bit
        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

        //Implement the code to enable ITEVFEN Control Bit
        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

        //Implement the code to enable ITERREN Control Bit
        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
    }

    return busystate;
}

/*
 * I2C Master Receive Data in Interrupt mode
 */

uint8_t
I2C_MasterReceiveDataIT(I2C_Handle_t* pI2CHandle, uint8_t* pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr) {

    uint8_t busystate = pI2CHandle->TxRxState;

    if ((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX)) {
        pI2CHandle->pRxBuffer = pRxBuffer;
        pI2CHandle->RxLen = Len;
        pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
        pI2CHandle->RxSize = Len; //Rxsize is used in the ISR code to manage the data reception
        pI2CHandle->DevAddr = SlaveAddr;
        pI2CHandle->Sr = Sr;

        //Implement code to Generate START Condition
        I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

        //Implement the code to enable ITBUFEN Control Bit
        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

        //Implement the code to enable ITEVFEN Control Bit
        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

        //Implement the code to enable ITERREN Control Bit
        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
    }

    return busystate;
}

static void
I2C_MasterHandleTXEInterrupt(I2C_Handle_t* pI2CHandle) {
    // Have to do the data transmission
    if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX) {
        if (pI2CHandle->TxLen > 0) {
            // 1. Load the data in to DR
            pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);

            // 2. Decrement the TxLen
            pI2CHandle->TxLen--;

            // 3. Increment the buffer address
            pI2CHandle->pTxBuffer++;
        }
    }
}

static void
I2C_MasterHandleRXNEInterrupt(I2C_Handle_t* pI2CHandle) {
    // Have to do the data reception
    if (pI2CHandle->RxSize == 1) {
        *pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
        pI2CHandle->RxLen--;
    }

    if (pI2CHandle->RxSize > 1) {
        if (pI2CHandle->RxLen == 2) {
            // Clear the ack bit
            I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
        }

        // Read DR
        *pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
        pI2CHandle->pRxBuffer++;
        pI2CHandle->RxLen--;
    }

    if (pI2CHandle->RxLen == 0) {
        // Close the I2C data reception and notify the application

        // 1. Generate the STOP condition
        if (pI2CHandle->Sr == I2C_DISABLE_SR) {
            I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
        }

        // 2. Close the I2C RX
        I2C_CloseReceiveData(pI2CHandle);

        // 3. Notify the application
        I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
    }
}

void
I2C_CloseReceiveData(I2C_Handle_t* pI2CHandle) {
    // Implement the code to disable ITBUFEN Control Bit
    pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

    // Implement the code to disable ITEVFEN Control Bit
    pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

    pI2CHandle->TxRxState = I2C_READY;
    pI2CHandle->pRxBuffer = NULL;
    pI2CHandle->RxLen = 0;
    pI2CHandle->RxSize = 0;

    if (pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE) {
        I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);
    }
}

void
I2C_CloseSendData(I2C_Handle_t* pI2CHandle) {
    // Implement the code to disable ITBUFEN Control Bit
    pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

    // Implement the code to disable ITEVFEN Control Bit
    pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

    pI2CHandle->TxRxState = I2C_READY;
    pI2CHandle->pTxBuffer = NULL;
    pI2CHandle->TxLen = 0;
}

void
I2C_SlaveSendData(I2C_RegDef_t* pI2Cx, uint8_t data) {
    pI2Cx->DR = data;
}

uint8_t
I2C_SlaveReceiveData(I2C_RegDef_t* pI2Cx) {
    return (uint8_t)pI2Cx->DR;
}

void
I2C_EV_IRQHandling(I2C_Handle_t* pI2CHandle) {
    //Interrupt handling for both master and slave mode of a device

    uint32_t temp1, temp2, temp3;

    temp1 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN);
    temp2 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN);

    temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB);

    //1. Handle For interrupt generated by SB event
    //	Note : SB flag is only applicable in Master mode
    if (temp1 && temp3) {
        //The interrupt is generated because of SB event
        //This block will not be executed in slave mode because for slave SB is always zero
        //In this block lets executed the address phase
        if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX) {
            I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
        } else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {
            I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
        }
    }

    temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR);

    //2. Handle For interrupt generated by ADDR event
    //Note : When master mode : Address is sent
    //		 When Slave mode   : Address matched with own address
    if (temp1 && temp3) {
        // ADDR flag is set
        I2C_ClearADDRFlag(pI2CHandle);
    }

    temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF);

    //3. Handle For interrupt generated by BTF(Byte Transfer Finished) event
    if (temp1 && temp3) {
        // BTF flag is set
        if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX) {
            // Make sure that TXE is also set
            if (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE)) {
                // BTF & TXE both equal to 1
                if (pI2CHandle->TxLen == 0) {
                    // 1. Generate the STOP condition
                    if (pI2CHandle->Sr == I2C_DISABLE_SR) {
                        I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
                    }

                    // 2. Reset all the members of the handle structure
                    I2C_CloseSendData(pI2CHandle);

                    // 3. Notify the application about transmission complete
                    I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
                }
            }
        } else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {
            ;
        }
    }

    temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_STOPF);

    //4. Handle For interrupt generated by STOPF event
    // Note : Stop detection flag is applicable only slave mode . For master this flag will never be set
    if (temp1 && temp3) {
        // STOPF flag is set
        //Clear the STOPF (1. Read SR1, 2. Write to CR1)

        pI2CHandle->pI2Cx->CR1 |= 0x0000;

        // Notify the application that STOP is detected
        I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
    }

    temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE);

    //5. Handle For interrupt generated by TXE event
    if (temp1 && temp2 && temp3) {
        // Check for device mode
        if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)) {
            // TXE flag is set
            I2C_MasterHandleTXEInterrupt(pI2CHandle);
        } else {
            // Slave
            // Make sure the slave is in the transmitter mode
            if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA)) {
                I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
            }
        }
    }

    temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RXNE);

    //6. Handle For interrupt generated by RXNE event
    if (temp1 && temp2 && temp3) {
        if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)) {
            // RXNE flag is set
            if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {
                I2C_MasterHandleRXNEInterrupt(pI2CHandle);
            }
        } else {
            // Slave
            // Make sure the slave is in the receiver mode
            if (!(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA))) {
                I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
            }
        }
    }
}

void
I2C_ER_IRQHandling(I2C_Handle_t* pI2CHandle) {
    uint32_t temp1, temp2;

    //Know the status of  ITERREN control bit in the CR2
    temp2 = (pI2CHandle->pI2Cx->CR2) & (1 << I2C_CR2_ITERREN);

    /***********************Check for Bus error************************************/
    temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_BERR);
    if (temp1 && temp2) {
        // This is Bus error

        // Implement the code to clear the buss error flag
        pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_BERR);

        // Implement the code to notify the application about the error
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_BERR);
    }

    /***********************Check for arbitration lost error************************************/
    temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_ARLO);
    if (temp1 && temp2) {
        // This is arbitration lost error

        // Implement the code to clear the arbitration lost error flag
        pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_ARLO);

        // Implement the code to notify the application about the error
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_ARLO);
    }

    /***********************Check for ACK failure  error************************************/

    temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_AF);
    if (temp1 && temp2) {
        //This is ACK failure error

        //Implement the code to clear the ACK failure error flag
        pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_AF);

        //Implement the code to notify the application about the error
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_AF);
    }

    /***********************Check for Overrun/underrun error************************************/
    temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_OVR);
    if (temp1 && temp2) {
        //This is Overrun/underrun

        //Implement the code to clear the Overrun/underrun error flag
        pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_OVR);

        //Implement the code to notify the application about the error
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_OVR);
    }

    /***********************Check for Time out error************************************/
    temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_TIMEOUT);
    if (temp1 && temp2) {
        //This is Time out error

        //Implement the code to clear the Time out error flag
        pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_TIMEOUT);

        //Implement the code to notify the application about the error
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_TIMEOUT);
    }
}

__weak void
I2C_ApplicationEventCallback(I2C_Handle_t* pI2CHandle, uint8_t AppEvent) {
    // This is a weak implementation. The user application may override this function.
}
