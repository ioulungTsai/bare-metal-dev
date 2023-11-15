
#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>

#define __vo volatile

/* 
 * base address of Flash and SRAM memories
 */

#define FLASH_BASEADDR                  0x08000000U         /* Base address of Flash        */
#define SRAM1_BASEADDR                  0x20000000U         /* Base address of SRAM1        */
#define SRAM2_BASEADDR                  0x20001C00U         /* Base address of SRAM2        */
#define ROM_BASEADDR                    0x1FFF0000U         /* Base address of ROM          */
#define SRAM_BASEADDR                   SRAM1_BASEADDR      /* Base address of Main SRAM    */


/* 
 * base address of AHBx and APBx buses
 */

#define PERIPH_BASEADDR                 0x40000000U
#define APB1PERIPH_BASEADDR             PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR             0x40010000U
#define AHB1PERIPH_BASEADDR             0x40020000U
#define AHB2PERIPH_BASEADDR             0x50000000U


/* 
 * base address of peripherals which hanging on AHB1 bus
 * TODO : Complete for all other peripherals
 */

#define GPIOA_BASEADDR                  ((AHB1PERIPH_BASEADDR) + 0x0000)
#define GPIOB_BASEADDR                  ((AHB1PERIPH_BASEADDR) + 0x0400)
#define GPIOC_BASEADDR                  ((AHB1PERIPH_BASEADDR) + 0x0800)
#define GPIOD_BASEADDR                  ((AHB1PERIPH_BASEADDR) + 0x0C00)
#define GPIOE_BASEADDR                  ((AHB1PERIPH_BASEADDR) + 0x1000)
#define GPIOF_BASEADDR                  ((AHB1PERIPH_BASEADDR) + 0x1400)
#define GPIOG_BASEADDR                  ((AHB1PERIPH_BASEADDR) + 0x1800)
#define GPIOH_BASEADDR                  ((AHB1PERIPH_BASEADDR) + 0x1C00)
#define GPIOI_BASEADDR                  ((AHB1PERIPH_BASEADDR) + 0x2000)


/* 
 * base address of peripherals which hanging on APB1 bus
 * TODO : Complete for all other peripherals
 */

#define SPI2_BASEADDR                   ((APB1PERIPH_BASEADDR) + 0x3800)
#define SPI3_BASEADDR                   ((APB1PERIPH_BASEADDR) + 0x3C00)
#define USART2_BASEADDR                 ((APB1PERIPH_BASEADDR) + 0x4400)
#define USART3_BASEADDR                 ((APB1PERIPH_BASEADDR) + 0x4800)
#define UART4_BASEADDR                  ((APB1PERIPH_BASEADDR) + 0x4C00)
#define UART5_BASEADDR                  ((APB1PERIPH_BASEADDR) + 0x5000)
#define I2C1_BASEADDR                   ((APB1PERIPH_BASEADDR) + 0x5400)
#define I2C2_BASEADDR                   ((APB1PERIPH_BASEADDR) + 0x5800)
#define I2C3_BASEADDR                   ((APB1PERIPH_BASEADDR) + 0x5C00)


/* 
 * base address of peripherals which hanging on APB2 bus
 * TODO : Complete for all other peripherals
 */

#define USART1_BASEADDR                 ((APB2PERIPH_BASEADDR) + 0x1000)
#define USART6_BASEADDR                 ((APB2PERIPH_BASEADDR) + 0x1400)
#define SPI1_BASEADDR                   ((APB2PERIPH_BASEADDR) + 0x3000)
#define SYSCFG_BASEADDR                 ((APB2PERIPH_BASEADDR) + 0x3800)
#define EXTI_BASEADDR                   ((APB2PERIPH_BASEADDR) + 0x3C00)


/************************** Peripheral Register Definition Structure **************************/

/* 
 * Note : Register of a Peripheral are specific to MCU
 * e.g : Number of Register of SPI peripheral of STM32F4x family of MCUs may be different (more or less)
 * Compared to number of registers of SPI peripheral of STM32Lx or STM32F0x family of MCUs
 * Please check the Device Reference Manual
 */

typedef struct
{
    __vo uint32_t MODER;                     /* GPIO port mode register                          Address offset: 0x00 */
    __vo uint32_t OTYPER;                    /* GPIO port output type register                   Address offset: 0x04 */
    __vo uint32_t OSPEEDR;                   /* GPIO port output speed register                  Address offset: 0x08 */
    __vo uint32_t PUPDR;                     /* GPIO port pull-up/pull-down register             Address offset: 0x0C */
    __vo uint32_t IDR;                       /* GPIO port input data register                    Address offset: 0x10 */
    __vo uint32_t ODR;                       /* GPIO port output data register                   Address offset: 0x14 */
    __vo uint32_t BSRR;                      /* GPIO port bit set/reset register                 Address offset: 0x18 */
    __vo uint32_t LCKR;                      /* GPIO port configuration lock register            Address offset: 0x1C */
    __vo uint32_t AFR[2];                    /* AF[0] GPIO alternate function low register       Address offset: 0x20 */
                                             /* AF[1] GPIO alternate function high register      Address offset: 0x24 */
}GPIO_RegDef_t;


/*
 * Peripheral definitions (Peripheral base addresses typecasted to xxx_RegDef_t) 
 */

#define GPIOA           ((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB           ((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC           ((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD           ((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE           ((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOF           ((GPIO_RegDef_t*) GPIOF_BASEADDR)
#define GPIOG           ((GPIO_RegDef_t*) GPIOG_BASEADDR)
#define GPIOH           ((GPIO_RegDef_t*) GPIOH_BASEADDR)
#define GPIOI           ((GPIO_RegDef_t*) GPIOI_BASEADDR)


#endif /* INC_STM32F407XX_H_ */
