
#include <string.h>
#include "stm32f407xx.h"

/*
 * PB14 --> SPI2_MISO
 * PB15 --> SPI2_MOSI
 * PB13 --> SPI2_SCLK
 * PB12 --> SPI2_NSS
 * ALT Function : 5
*/

void delay(void)
{
    for(uint32_t i=0; i < 500000; i++);
}

void SPI2_GPIOInits(void)
{
    GPIO_Handle_t SPIPins;

    SPIPins.pGPIOx = GPIOB;
    SPIPins.GPIO_Pinconfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    SPIPins.GPIO_Pinconfig.GPIO_PinAltFunMode = 5;
    SPIPins.GPIO_Pinconfig.GPIO_PinOPType = GPIO_OUT_TYPE_PP;
    SPIPins.GPIO_Pinconfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    SPIPins.GPIO_Pinconfig.GPIO_PinOPSpeed = GPIO_OUT_SPEED_FAST;

    // SCLK
    SPIPins.GPIO_Pinconfig.GPIO_PinNumber = GPIO_PIN_NO_13;
    GPIO_Init(&SPIPins);

    // MOSI
    SPIPins.GPIO_Pinconfig.GPIO_PinNumber = GPIO_PIN_NO_15;
    GPIO_Init(&SPIPins);

    // MISO
    // SPIPins.GPIO_Pinconfig.GPIO_PinNumber = GPIO_PIN_NO_14;
    // GPIO_Init(&SPIPins);

    // NSS
    SPIPins.GPIO_Pinconfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    GPIO_Init(&SPIPins);
}

void SPI2_Inits(void)
{
    SPI_Handle_t SPI2Handle;

    SPI2Handle.pSPIx = SPI2;
    SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
    SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
    SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8; // Generate 2MHz SCLK
    SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
    SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW; 
    SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
    SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_DI; // Hardware slave management enabled for NSS pins

    SPI_Init(&SPI2Handle);
}

 void GPIO_ButtonInit(void)
 {
    GPIO_Handle_t GpioBtn;

    GpioBtn.pGPIOx = GPIOA;
    GpioBtn.GPIO_Pinconfig.GPIO_PinNumber = GPIO_PIN_NO_0;
    GpioBtn.GPIO_Pinconfig.GPIO_PinMode = GPIO_MODE_IN;
    GpioBtn.GPIO_Pinconfig.GPIO_PinOPSpeed = GPIO_OUT_SPEED_FAST;
    GpioBtn.GPIO_Pinconfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_Init(&GpioBtn);
 }

int main(void)
{
    char user_data[] = "Hello World!";

    GPIO_ButtonInit();

    SPI2_GPIOInits(); // This function is used to initialize the GPIO pins to behave as SPI2 pins

    SPI2_Inits(); // This function is used to initialize the SPI2 peripheral parameters

    /*
	* making SSOE 1 does NSS output enable.
	* The NSS pin is automatically managed by the hardware.
	* i.e when SPE=1 , NSS will be pulled to low
	* and NSS pin will be high when SPE=0
	*/
    SPI_SSOEConfig(SPI2, ENABLE);
    
    while(1)
    {
        while( ! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) );

        delay();

        SPI_PeripheralControl(SPI2, ENABLE); // Enable the SPI2 peripheral

        uint8_t dataLen = strlen(user_data); // data length

        SPI_SendData(SPI2, &dataLen, 1); // Send data length information

        SPI_SendData(SPI2, (uint8_t *)user_data, strlen(user_data)); // Send data

        while( SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG) ); // Check if SPI busy

        SPI_PeripheralControl(SPI2, DISABLE); // Disable the SPI2 peripheral 
    }

    return 0;
}
