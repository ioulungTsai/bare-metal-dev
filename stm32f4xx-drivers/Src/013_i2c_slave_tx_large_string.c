
#include <stdio.h>
#include <string.h>
#include "stm32f407xx.h"

#define SLAVE_ADDR 0x68
#define MY_ADDR    SLAVE_ADDR

void delay(void)
{
        for (uint32_t i = 0; i < 500000 / 2; i++)
                ;
}

I2C_Handle_t I2C1Handle;
uint32_t data_len = 0;

// Very large message
uint8_t tr_buf[] =
        "The sun set over the horizon, painting the sky in hues of orange and pink. Birds chirped their final songs of the "
        "day as a gentle breeze rustled through the trees. In the distance, the sound of waves crashing against the shore "
        "could be heard, creating a serene backdrop. Families strolled along the beach, children laughing and playing in "
        "the sand. It was a perfect evening, one that would be remembered fon";

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
        data_len = strlen((char *)tr_buf);

        GPIO_ButtonInit();

        // I2C pin inits
        I2C1_GPIOInits();

        // I2C peripheral configuration
        I2C1_Inits();

        // I2C IRQ configurations
        I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV, ENABLE);
        I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER, ENABLE);

        I2C_SlaveEnableDisableCallbackEvents(I2C1, ENABLE);

        // Enable the I2C peripheral
        I2C_PeripheralControl(I2C1, ENABLE);

        // ACK bit made 1 after PE=1
        I2C_ManageAcking(I2C1, I2C_ACK_ENABLE);

        while (1)
                ;
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
        static uint8_t commandCode = 0;
        static uint32_t Cnt        = 0;
        static uint32_t w_ptr      = 0;

        if (AppEv == I2C_EV_DATA_REQ) {
                // Master wants some data. slave has to send it
                if (commandCode == 0x51) {
                        // Send the length information to the master
                        I2C_SlaveSendData(pI2CHandle->pI2Cx, ((data_len >> ((Cnt % 4) * 8)) & 0xFF));
                        Cnt++;
                } else if (commandCode == 0x52) {
                        // Send the contents of Tx_buf
                        I2C_SlaveSendData(pI2CHandle->pI2Cx, tr_buf[w_ptr++]);
                }
        } else if (AppEv == I2C_EV_DATA_RCV) {
                // Data is waiting for the slave to read and slave has to read it
                commandCode = I2C_SlaveReceiveData(pI2CHandle->pI2Cx);

        } else if (AppEv == I2C_ERROR_AF) {
                // This happens only during slave transmitting.
                // Master has sent the NACK. so slave should understand that master doesnt need
                // more data.

                // If the current active code is 0x52 then don't invalidate
                if (!(commandCode == 0x52)) {
                        commandCode = 0xff;
                }

                // Reset the Cnt variable because its end of transmission
                Cnt = 0;

                // Slave concludes it sent all the bytes when w_ptr reaches data_len
                if (w_ptr >= data_len) {
                        w_ptr       = 0;
                        commandCode = 0xff;
                }
        } else if (AppEv == I2C_EV_STOP) {
                // This happens only during slave reception .
                // Slave concludes end of Rx
                Cnt = 0;
        }
}
