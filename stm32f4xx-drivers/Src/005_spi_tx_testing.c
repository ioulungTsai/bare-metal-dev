
#include "stm32f407xx.h"

/*
 * PB14 --> SPI2_MISO
 * PB15 --> SPI2_MOSI
 * PB13 --> SPI2_SCLK
 * PB12 --> SPI2_NSS
 * ALT Function : 5
*/

void SPI_GPIOInit()
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
    SPIPins.GPIO_Pinconfig.GPIO_PinNumber = GPIO_PIN_NO_14;
    GPIO_Init(&SPIPins);

    // NSS
    SPIPins.GPIO_Pinconfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    GPIO_Init(&SPIPins);
}

int main(void)
{
    // This function is used to initialize the GPIO pins to behave as SPI2 pins
    SPI_GPIOInit();

    return 0;
}