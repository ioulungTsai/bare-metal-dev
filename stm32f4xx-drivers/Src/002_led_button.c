
#include "stm32f407xx.h"

#define BTN_PRESSED HIGH

void delay(void)
{
        for (uint32_t i = 0; i < 500000; i++)
                ;
}

int main(void)
{
        GPIO_Handle_t GpioLed, GpioBtn;

        GpioLed.pGPIOx                             = GPIOD;
        GpioLed.GPIO_Pinconfig.GPIO_PinNumber      = GPIO_PIN_NO_12;
        GpioLed.GPIO_Pinconfig.GPIO_PinMode        = GPIO_MODE_OUT;
        GpioLed.GPIO_Pinconfig.GPIO_PinOPSpeed     = GPIO_OUT_SPEED_FAST;
        GpioLed.GPIO_Pinconfig.GPIO_PinOPType      = GPIO_OUT_TYPE_PP;
        GpioLed.GPIO_Pinconfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

        GPIO_PeriClockControl(GPIOD, ENABLE);

        GPIO_Init(&GpioLed);

        GpioBtn.pGPIOx                             = GPIOA;
        GpioBtn.GPIO_Pinconfig.GPIO_PinNumber      = GPIO_PIN_NO_0;
        GpioBtn.GPIO_Pinconfig.GPIO_PinMode        = GPIO_MODE_IN;
        GpioBtn.GPIO_Pinconfig.GPIO_PinOPSpeed     = GPIO_OUT_SPEED_FAST;
        GpioBtn.GPIO_Pinconfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

        GPIO_PeriClockControl(GPIOA, ENABLE);

        GPIO_Init(&GpioBtn);

        while (1) {
                if (GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) == BTN_PRESSED) {
                        delay();
                        GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
                }
        }

        return 0;
}
