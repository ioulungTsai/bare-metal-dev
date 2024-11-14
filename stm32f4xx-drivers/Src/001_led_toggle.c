
#include "stm32f407xx.h"

void delay(void)
{
        for (uint32_t i = 0; i < 500000; i++)
                ;
}

int main(void)
{
        GPIO_Handle_t GpioLed;

        GpioLed.pGPIOx                         = GPIOD;
        GpioLed.GPIO_Pinconfig.GPIO_PinNumber  = GPIO_PIN_NO_12;
        GpioLed.GPIO_Pinconfig.GPIO_PinMode    = GPIO_MODE_OUT;
        GpioLed.GPIO_Pinconfig.GPIO_PinOPSpeed = GPIO_OUT_SPEED_FAST;
        // GpioLed.GPIO_Pinconfig.GPIO_PinOPType = GPIO_OUT_TYPE_OD;
        GpioLed.GPIO_Pinconfig.GPIO_PinOPType      = GPIO_OUT_TYPE_PP;
        GpioLed.GPIO_Pinconfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

        GPIO_PeriClockControl(GPIOD, ENABLE);

        GPIO_Init(&GpioLed);

        while (1) {
                GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
                delay();
        }

        return 0;
}
