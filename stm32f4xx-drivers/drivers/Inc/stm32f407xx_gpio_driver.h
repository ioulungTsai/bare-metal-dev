#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_


#include "stm32f407xx.h"

/*
 * This is a Configuration structure for a GPIO pin   
 */

typedef struct
{
    uint8_t GPIO_PinNumber;
    uint8_t GPIO_PinMode;
    uint8_t GPIO_PinSpeed;
    uint8_t GPIO_PinPuPdControl;
    uint8_t GPIO_PinOPType;
    uint8_t GPIO_PinAltFunMode;

}GPIO_Pinconfig_t;



/*
 * This is a Handle structure for a GPIO pin 
 */

typedef struct
{
    // pointer to hold the base address of the GPIO peripheral
    GPIO_RegDef_t *pGPIOx;                  /*!< This holds the base address of the GPIO port to which the pin belongs >*/
    GPIO_Pinconfig_t GPIO_Pinconfig;        /*!< This holds GPIO pin configuration settings >*/

}GPIO_Handle_t;


/*****************************************************************************
 *                      APIs supported by this driver
 *      For more information about the APIs check the function definitions
******************************************************************************/

/*
 * Peripheral Clock setup
 */

void GPIO_PeriClockControl(void);

/*
 * Init and De-init
 */

void GPIO_Init(void);
void GPIO_DeInit(void);

/*
 * Data read and write
 */

void GPIO_ReadFromInputPin(void);
void GPIO_ReadFromInputPort(void);
void GPIO_WriteToOutputPin(void);
void GPIO_WriteToOutputPort(void);
void GPIO_ToggleOutputPin(void);

/*
 * IRQ Configuration and ISR handling
 */

void GPIO_IRQConfig(void);
void GPIO_IRQHandling(void);




#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */