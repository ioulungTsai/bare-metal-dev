
// #include <string.h>
#include "stm32f407xx.h"

#define BTN_PRESSED LOW

void delay(void)
{
    for(uint32_t i=0; i < 500000; i++);
}

int main(void)
{
    GPIO_Handle_t GpioLed, GpioBtn;

    memset(&GpioLed,0,sizeof(GpioLed));
	memset(&GpioBtn,0,sizeof(GpioLed));

    GpioLed.pGPIOx = GPIOD;
    GpioLed.GPIO_Pinconfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    GpioLed.GPIO_Pinconfig.GPIO_PinMode = GPIO_MODE_OUT;
    GpioLed.GPIO_Pinconfig.GPIO_PinOPSpeed = GPIO_OUT_SPEED_LOW;
    GpioLed.GPIO_Pinconfig.GPIO_PinOPType = GPIO_OUT_TYPE_PP;
    GpioLed.GPIO_Pinconfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_PeriClockControl(GPIOD, ENABLE);

    GPIO_Init(&GpioLed);

    GpioBtn.pGPIOx = GPIOD;
    GpioBtn.GPIO_Pinconfig.GPIO_PinNumber = GPIO_PIN_NO_5;
    GpioBtn.GPIO_Pinconfig.GPIO_PinMode = GPIO_MODE_IT_FT;
    GpioBtn.GPIO_Pinconfig.GPIO_PinOPSpeed = GPIO_OUT_SPEED_FAST;
    GpioBtn.GPIO_Pinconfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

    GPIO_PeriClockControl(GPIOD, ENABLE);

    GPIO_Init(&GpioBtn);

    //IRQ configuration
    GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRI15);
    GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);

    while(1);
    
    return 0;
}

void EXTI9_5_IRQHandler(void)
{
    GPIO_IRQHandling(GPIO_PIN_NO_5);
    GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
}
