
#include <stdio.h>
#include <string.h>
#include "stm32f407xx.h"

#define MY_ADDR    0x61
#define SLAVE_ADDR 0x68

void delay(void)
{
        for (uint32_t i = 0; i < 500000 / 2; i++)
                ;
}

I2C_Handle_t I2C1Handle;

// Some data
uint8_t some_data[] = "Testing I2C master Tx\n";

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
        GPIO_ButtonInit();

        // I2C pin inits
        I2C1_GPIOInits();

        // I2C peripheral configuration
        I2C1_Inits();

        // Enable the I2C peripheral
        I2C_PeripheralControl(I2C1, ENABLE);

        while (1) {
                // Wait till button is pressed
                while (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0))
                        ;

                // To avoid button de-bouncing related issues ~400ms of delay
                delay();

                // Send data to slave
                I2C_MasterSendData(&I2C1Handle, some_data, strlen((char *)some_data), SLAVE_ADDR, I2C_DISABLE_SR);
        }
}
