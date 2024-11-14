
#include <stdio.h>
#include <string.h>
#include "stm32f407xx.h"

extern void initialise_monitor_handles(void);

// Flag Variable
uint8_t rxComplt = RESET;

#define MY_ADDR    0x61
#define SLAVE_ADDR 0x68

void delay(void)
{
        for (uint32_t i = 0; i < 500000 / 2; i++)
                ;
}

I2C_Handle_t I2C1Handle;

// Receive Buffer
uint8_t rcv_buf[32];

/*
 * PB6 or PB8 --> I2C_SCL
 * PB9 or PB7 --> I2C_SDA
 */

void I2C1_GPIOInits(void)
{
        GPIO_Handle_t I2CPins;

        I2CPins.pGPIOx                             = GPIOB;
        I2CPins.GPIO_Pinconfig.GPIO_PinMode        = GPIO_MODE_ALTFN;
        I2CPins.GPIO_Pinconfig.GPIO_PinOPType      = GPIO_OUT_TYPE_OD;
        I2CPins.GPIO_Pinconfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
        I2CPins.GPIO_Pinconfig.GPIO_PinAltFunMode  = 4;
        I2CPins.GPIO_Pinconfig.GPIO_PinOPSpeed     = GPIO_OUT_SPEED_FAST;

        // SCL
        I2CPins.GPIO_Pinconfig.GPIO_PinNumber = GPIO_PIN_NO_8;
        GPIO_Init(&I2CPins);

        // SDA
        I2CPins.GPIO_Pinconfig.GPIO_PinNumber = GPIO_PIN_NO_7;
        GPIO_Init(&I2CPins);
}

void I2C1_Inits(void)
{
        I2C1Handle.pI2Cx                        = I2C1;
        I2C1Handle.I2C_Config.I2C_AckControl    = I2C_ACK_ENABLE;
        I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;
        I2C1Handle.I2C_Config.I2C_FMDutyCycle   = I2C_FM_DUTY_2;
        I2C1Handle.I2C_Config.I2C_SCLSpeed      = I2C_SCL_SPEED_SM;

        I2C_Init(&I2C1Handle);
}

void GPIO_ButtonInit(void)
{
        GPIO_Handle_t GpioBtn;

        GpioBtn.pGPIOx                             = GPIOA;
        GpioBtn.GPIO_Pinconfig.GPIO_PinNumber      = GPIO_PIN_NO_0;
        GpioBtn.GPIO_Pinconfig.GPIO_PinMode        = GPIO_MODE_IN;
        GpioBtn.GPIO_Pinconfig.GPIO_PinOPSpeed     = GPIO_OUT_SPEED_FAST;
        GpioBtn.GPIO_Pinconfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

        GPIO_Init(&GpioBtn);
}

int main(void)
{
        uint8_t commandCode, len;

        initialise_monitor_handles();

        printf("Application is running...\n");

        GPIO_ButtonInit();

        // I2C pin inits
        I2C1_GPIOInits();

        // I2C peripheral configuration
        I2C1_Inits();

        // I2C IRQ configurations
        I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV, ENABLE);
        I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER, ENABLE);

        // Enable the I2C peripheral
        I2C_PeripheralControl(I2C1, ENABLE);

        // ACK bit made 1 after PE=1
        I2C_ManageAcking(I2C1, I2C_ACK_ENABLE);

        while (1) {
                // Wait till button is pressed
                while (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0))
                        ;

                // To avoid button de-bouncing related issues ~400ms of delay
                delay();

                commandCode = 0x51;

                while (I2C_MasterSendDataIT(&I2C1Handle, &commandCode, 1, SLAVE_ADDR, I2C_ENABLE_SR) != I2C_READY)
                        ;

                while (I2C_MasterReceiveDataIT(&I2C1Handle, &len, 1, SLAVE_ADDR, I2C_ENABLE_SR) != I2C_READY)
                        ;

                commandCode = 0x52;

                while (I2C_MasterSendDataIT(&I2C1Handle, &commandCode, 1, SLAVE_ADDR, I2C_ENABLE_SR) != I2C_READY)
                        ;

                while (I2C_MasterReceiveDataIT(&I2C1Handle, rcv_buf, len, SLAVE_ADDR, I2C_DISABLE_SR) != I2C_READY)
                        ;

                rxComplt = RESET;

                // Wait till Rx comletes
                while (rxComplt != SET)
                        ;

                rcv_buf[len + 1] = '\0';

                printf("Data : %s", rcv_buf);

                rxComplt = RESET;
        }
}

void I2C1_EV_IRQHandler(void)
{
        I2C_EV_IRQHandling(&I2C1Handle);
}

void I2C1_ER_IRQHandler(void)
{
        I2C_ER_IRQHandling(&I2C1Handle);
}

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv)
{
        if (AppEv == I2C_EV_TX_CMPLT) {
                printf("Tx is completed\n");
        } else if (AppEv == I2C_EV_RX_CMPLT) {
                printf("Rx is completed\n");
                rxComplt = SET;
        } else if (AppEv == I2C_ERROR_AF) {
                printf("Error : Ack failure\n");
                // In master ack failure happens when slave fails to send ack for the byte
                // sent from the master.
                I2C_CloseSendData(pI2CHandle);

                // Generate the stop condition to release the bus
                I2C_GenerateStopCondition(I2C1);

                // Hang in infinite loop
                while (1)
                        ;
        }
}
