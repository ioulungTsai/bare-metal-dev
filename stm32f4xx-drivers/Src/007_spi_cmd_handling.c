
#include <stdio.h>
#include <string.h>
#include "stm32f407xx.h"

extern void initialise_monitor_handles(void);

/*
 * PB14 --> SPI2_MISO
 * PB15 --> SPI2_MOSI
 * PB13 --> SPI2_SCLK
 * PB12 --> SPI2_NSS
 * ALT Function : 5
*/

// Command Codes
#define COMMAND_LED_CTRL    0x50
#define COMMAND_SENSOR_READ 0x51
#define COMMAND_LED_READ    0x52
#define COMMAND_PRINT       0x53
#define COMMAND_ID_READ     0x54

#define LED_ON              1
#define LED_OFF             0

// Arduino Analog Pins
#define ANALOG_PIN0         0
#define ANALOG_PIN1         1
#define ANALOG_PIN2         2
#define ANALOG_PIN3         3
#define ANALOG_PIN4         4

// Arduino LED
#define LED_PIN             9

void delay(void)
{
        for (uint32_t i = 0; i < 500000; i++)
                ; // ~10ms
}

void SPI2_GPIOInits(void)
{
        GPIO_Handle_t SPIPins;

        SPIPins.pGPIOx                             = GPIOB;
        SPIPins.GPIO_Pinconfig.GPIO_PinMode        = GPIO_MODE_ALTFN;
        SPIPins.GPIO_Pinconfig.GPIO_PinAltFunMode  = 5;
        SPIPins.GPIO_Pinconfig.GPIO_PinOPType      = GPIO_OUT_TYPE_PP;
        SPIPins.GPIO_Pinconfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
        SPIPins.GPIO_Pinconfig.GPIO_PinOPSpeed     = GPIO_OUT_SPEED_FAST;

        // SCLK
        SPIPins.GPIO_Pinconfig.GPIO_PinNumber = GPIO_PIN_NO_13;
        GPIO_Init(&SPIPins);

        // MOSI
        SPIPins.GPIO_Pinconfig.GPIO_PinNumber = GPIO_PIN_NO_15;
        GPIO_Init(&SPIPins);

        // MISO
        SPIPins.GPIO_Pinconfig.GPIO_PinNumber = GPIO_PIN_NO_14;
        GPIO_Init(&SPIPins);

        // NSS
        SPIPins.GPIO_Pinconfig.GPIO_PinNumber = GPIO_PIN_NO_12;
        GPIO_Init(&SPIPins);
}

void SPI2_Inits(void)
{
        SPI_Handle_t SPI2Handle;

        SPI2Handle.pSPIx                    = SPI2;
        SPI2Handle.SPIConfig.SPI_BusConfig  = SPI_BUS_CONFIG_FD;
        SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
        SPI2Handle.SPIConfig.SPI_SclkSpeed  = SPI_SCLK_SPEED_DIV8; // Generate 2MHz SCLK
        SPI2Handle.SPIConfig.SPI_DFF        = SPI_DFF_8BITS;
        SPI2Handle.SPIConfig.SPI_CPOL       = SPI_CPOL_LOW;
        SPI2Handle.SPIConfig.SPI_CPHA       = SPI_CPHA_LOW;
        SPI2Handle.SPIConfig.SPI_SSM        = SPI_SSM_DI; // Hardware slave management enabled for NSS pins

        SPI_Init(&SPI2Handle);
}

void GPIO_ButtonInit(void)
{
        GPIO_Handle_t GpioBtn, GpioLed;

        GpioBtn.pGPIOx                             = GPIOA;
        GpioBtn.GPIO_Pinconfig.GPIO_PinNumber      = GPIO_PIN_NO_0;
        GpioBtn.GPIO_Pinconfig.GPIO_PinMode        = GPIO_MODE_IN;
        GpioBtn.GPIO_Pinconfig.GPIO_PinOPSpeed     = GPIO_OUT_SPEED_FAST;
        GpioBtn.GPIO_Pinconfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

        GPIO_Init(&GpioBtn);

        GpioLed.pGPIOx                             = GPIOD;
        GpioLed.GPIO_Pinconfig.GPIO_PinNumber      = GPIO_PIN_NO_12;
        GpioLed.GPIO_Pinconfig.GPIO_PinMode        = GPIO_MODE_OUT;
        GpioLed.GPIO_Pinconfig.GPIO_PinOPSpeed     = GPIO_OUT_SPEED_FAST;
        GpioLed.GPIO_Pinconfig.GPIO_PinOPType      = GPIO_OUT_TYPE_OD;
        GpioLed.GPIO_Pinconfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

        GPIO_PeriClockControl(GPIOD, ENABLE);

        GPIO_Init(&GpioLed);
}

uint8_t SPI_VerifyResponse(uint8_t ackByte)
{
        if ((ackByte == 0xF5)) {
                return 1; // ACK
        }
        return 0; // NACK
}

int main(void)
{
        uint8_t dummy_write = 0xff;
        uint8_t dummy_read;

        initialise_monitor_handles();

        printf("Application is running...\n");

        GPIO_ButtonInit();

        SPI2_GPIOInits(); // This function is used to initialize the GPIO pins to behave as SPI2 pins

        SPI2_Inits(); // This function is used to initialize the SPI2 peripheral parameters

        printf("SPI Init. Done!\n");

        /*
	* making SSOE 1 does NSS output enable.
	* The NSS pin is automatically managed by the hardware.
	* i.e when SPE=1 , NSS will be pulled to low
	* and NSS pin will be high when SPE=0
	*/
        SPI_SSOEConfig(SPI2, ENABLE);

        while (1) {
                while (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0))
                        ;

                delay();

                SPI_PeripheralControl(SPI2, ENABLE); // Enable the SPI2 peripheral

                // 1. CMD_LED_CTRL  <Pin No.(1)>    <Value(1)>
                uint8_t commandCode = COMMAND_LED_CTRL;
                uint8_t ackByte;
                uint8_t args[2];

                // Send Command
                SPI_SendData(SPI2, &commandCode, 1);

                // Do dummy read to clear off the RXNE
                SPI_ReceiveData(SPI2, &dummy_read, 1);

                // Send some dummy bits (1byte) to fetch the response from the slave
                SPI_SendData(SPI2, &dummy_write, 1);

                // Read the ack byte received
                SPI_ReceiveData(SPI2, &ackByte, 1);

                if (SPI_VerifyResponse(ackByte)) {
                        args[0] = LED_PIN;
                        args[1] = LED_ON;

                        // Send arguments
                        SPI_SendData(SPI2, args, 2);

                        printf("COMMAND_LED_CTRL: Executed!\n");
                }

                // 2. CMD_SENSOR_READ   <Analog Pin Number(1)>
                while (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0))
                        ;

                delay();

                commandCode = COMMAND_SENSOR_READ;

                // Send Command
                SPI_SendData(SPI2, &commandCode, 1);

                // Do dummy read to clear off the RXNE
                SPI_ReceiveData(SPI2, &dummy_read, 1);

                // Send some dummy bits (1byte) to fetch the response from the slave
                SPI_SendData(SPI2, &dummy_write, 1);

                // Read the ack byte received
                SPI_ReceiveData(SPI2, &ackByte, 1);

                if (SPI_VerifyResponse(ackByte)) {
                        args[0] = ANALOG_PIN0;

                        // Send arguments
                        SPI_SendData(SPI2, args, 1);

                        // Do dummy read to clear off the RXNE
                        SPI_ReceiveData(SPI2, &dummy_read, 1);

                        // Add some delay to allow the slave to be ready with the data
                        delay();

                        // Send some dummy bits (1byte) to fetch the response from the slave
                        SPI_SendData(SPI2, &dummy_write, 1);

                        uint8_t analog_read;
                        SPI_ReceiveData(SPI2, &analog_read, 1);

                        printf("COMMAND_SENSOR_READ: The Value = %d\n", analog_read);
                }

                // 3. CMD_LED_READ  <Pin No.(1)>
                while (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0))
                        ;

                delay();

                commandCode = COMMAND_LED_READ;

                // Send Command
                SPI_SendData(SPI2, &commandCode, 1);

                // Do dummy read to clear off the RXNE
                SPI_ReceiveData(SPI2, &dummy_read, 1);

                // Send some dummy bits (1byte) to fetch the response from the slave
                SPI_SendData(SPI2, &dummy_write, 1);

                // Read the ack byte received
                SPI_ReceiveData(SPI2, &ackByte, 1);

                if (SPI_VerifyResponse(ackByte)) {
                        args[0] = LED_PIN;

                        // Send arguments
                        SPI_SendData(SPI2, args, 1);

                        // Do dummy read to clear off the RXNE
                        SPI_ReceiveData(SPI2, &dummy_read, 1);

                        // Add some delay to allow the slave to be ready with the data
                        delay();

                        // Send some dummy bits (1byte) to fetch the response from the slave
                        SPI_SendData(SPI2, &dummy_write, 1);

                        uint8_t led_status;
                        SPI_ReceiveData(SPI2, &led_status, 1);

                        printf("COMMAND_LED_READ: The Status = %d\n", led_status);
                }

                // 4. CMD_PRINT <len(1)>    <message(len)>
                while (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0))
                        ;

                delay();

                commandCode = COMMAND_PRINT;

                // Send Command
                SPI_SendData(SPI2, &commandCode, 1);

                // Do dummy read to clear off the RXNE
                SPI_ReceiveData(SPI2, &dummy_read, 1);

                // Send some dummy bits (1byte) to fetch the response from the slave
                SPI_SendData(SPI2, &dummy_write, 1);

                // Read the ack byte received
                SPI_ReceiveData(SPI2, &ackByte, 1);

                uint8_t message[] = "Hello ! You can do it ! !";
                if (SPI_VerifyResponse(ackByte)) {
                        args[0] = strlen((char *)message);

                        // Send arguments
                        SPI_SendData(SPI2, args, 1);

                        // Do dummy read to clear off the RXNE
                        SPI_ReceiveData(SPI2, &dummy_read, 1);

                        // Add some delay to allow the slave to be ready with the data
                        delay();

                        // Send the message
                        for (int i = 0; i < args[0]; i++) {
                                SPI_SendData(SPI2, &message[i], 1);
                                SPI_ReceiveData(SPI2, &dummy_read, 1);
                        }

                        printf("COMMAND_PRINT: Executed!\n");
                }

                // 5. CMD_ID_READ
                while (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0))
                        ;

                delay();

                commandCode = COMMAND_ID_READ;

                // Send Command
                SPI_SendData(SPI2, &commandCode, 1);

                // Do dummy read to clear off the RXNE
                SPI_ReceiveData(SPI2, &dummy_read, 1);

                // Send some dummy bits (1byte) to fetch the response from the slave
                SPI_SendData(SPI2, &dummy_write, 1);

                // Read the ack byte received
                SPI_ReceiveData(SPI2, &ackByte, 1);

                uint8_t id[11];
                uint32_t i = 0;

                if (SPI_VerifyResponse(ackByte)) {
                        // Read 10 bytes id from the slave
                        for (i = 0; i < 10; i++) {
                                // Send dummy byte to fetch data from slave
                                SPI_SendData(SPI2, &dummy_write, 1);
                                SPI_ReceiveData(SPI2, &id[i], 1);
                        }

                        id[10] = '\0';

                        printf("COMMAND_ID_READ: The ID = %s\n", id);
                }

                while (SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG))
                        ; // Check if SPI busy

                SPI_PeripheralControl(SPI2, DISABLE); // Disable the SPI2 peripheral

                printf("SPI Communication Closed!");
        }

        return 0;
}
