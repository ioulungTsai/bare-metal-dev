
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
#define RCC_BASEADDR                    ((AHB1PERIPH_BASEADDR) + 0x3800)


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

/*
 * Peripheral register definition structure for GPIO
 */

typedef struct
{
    __vo uint32_t MODER;                    /* GPIO port mode register                          Address offset: 0x00 */
    __vo uint32_t OTYPER;                   /* GPIO port output type register                   Address offset: 0x04 */
    __vo uint32_t OSPEEDR;                  /* GPIO port output speed register                  Address offset: 0x08 */
    __vo uint32_t PUPDR;                    /* GPIO port pull-up/pull-down register             Address offset: 0x0C */
    __vo uint32_t IDR;                      /* GPIO port input data register                    Address offset: 0x10 */
    __vo uint32_t ODR;                      /* GPIO port output data register                   Address offset: 0x14 */
    __vo uint32_t BSRR;                     /* GPIO port bit set/reset register                 Address offset: 0x18 */
    __vo uint32_t LCKR;                     /* GPIO port configuration lock register            Address offset: 0x1C */
    __vo uint32_t AFR[2];                   /* AF[0] GPIO alternate function low register       Address offset: 0x20 */
                                            /* AF[1] GPIO alternate function high register      Address offset: 0x24 */
}GPIO_RegDef_t;


/*
 * Peripheral register definition structure for RCC
 */

typedef struct
{
    __vo uint32_t CR;                   /* RCC clock control register                                           Address offset: 0x00        */
    __vo uint32_t PLLCFGR;              /* RCC PLL configuration register                                       Address offset: 0x04        */
    __vo uint32_t CFGR;                 /* RCC clock configuration register                                     Address offset: 0x08        */
    __vo uint32_t CIR;                  /* RCC clock interrupt register                                         Address offset: 0x0C        */
    __vo uint32_t AHB1RSTR;             /* RCC AHB1 peripheral reset register                                   Address offset: 0x10        */
    __vo uint32_t AHB2RSTR;             /* RCC AHB2 peripheral reset register                                   Address offset: 0x14        */
    __vo uint32_t AHB3RSTR;             /* RCC AHB3 peripheral reset register                                   Address offset: 0x18        */
         uint32_t Reserved0;            /* Reserved                                                             Address offset: 0x1C        */
    __vo uint32_t APB1RSTR;             /* RCC APB1 peripheral reset register                                   Address offset: 0x20        */
    __vo uint32_t APB2RSTR;             /* RCC APB2 peripheral reset register                                   Address offset: 0x24        */
         uint32_t Reserved1[2];         /* Reserved                                                             Address offset: 0x28-0x2C   */
    __vo uint32_t AHB1ENR;              /* RCC AHB1 peripheral clock register                                   Address offset: 0x30        */
    __vo uint32_t AHB2ENR;              /* RCC AHB2 peripheral clock register                                   Address offset: 0x34        */
    __vo uint32_t AHB3ENR;              /* RCC AHB3 peripheral clock register                                   Address offset: 0x38        */
         uint32_t Reserved2;            /* Reserved                                                             Address offset: 0x3C        */
    __vo uint32_t APB1ENR;              /* RCC APB1 peripheral clock enable register                            Address offset: 0x40        */
    __vo uint32_t APB2ENR;              /* RCC APB2 peripheral clock enable register                            Address offset: 0x44        */
         uint32_t Reserved3[2];         /* Reserved                                                             Address offset: 0x48-0x4C   */
    __vo uint32_t AHB1LPENR;            /* RCC AHB1 peripheral clock enable in low power mode register          Address offset: 0x50        */
    __vo uint32_t AHB2LPENR;            /* RCC AHB2 peripheral clock enable in low power mode register          Address offset: 0x54        */
    __vo uint32_t AHB3LPENR;            /* RCC AHB3 peripheral clock enable in low power mode register          Address offset: 0x58        */
         uint32_t Reserved4;            /* Reserved                                                             Address offset: 0x5C        */
    __vo uint32_t APB1LPENR;            /* RCC APB1 peripheral clock enable in low power mode register          Address offset: 0x60        */
    __vo uint32_t APB2LPENR;            /* RCC APB2 peripheral clock enable in low power mode register          Address offset: 0x64        */
         uint32_t Reserved5[2];         /* Reserved                                                             Address offset: 0x68-0x6C   */
    __vo uint32_t BDCR;                 /* RCC Backup domain control register                                   Address offset: 0x70        */
    __vo uint32_t CSR;                  /* RCC clock control & status register                                  Address offset: 0x74        */
         uint32_t Reserved6[2];         /* Reserved                                                             Address offset: 0x78-0x7C   */
    __vo uint32_t SSCGR;                /* RCC spread spectrum clock generation register                        Address offset: 0x80        */
    __vo uint32_t PLLI2SCFGR;           /* RCC PLLI2S configuration register                                    Address offset: 0x84        */
    __vo uint32_t PLLSAICFGR;           /* RCC PLL configuration register                                       Address offset: 0x88        */
    __vo uint32_t DCKCFGR;              /* RCC Dedicated Clock Configuration Register                           Address offset: 0x8C        */
    
}RCC_RegDef_t;


/*
 * Peripheral register definition structure for EXTI
 */

typedef struct
{
    __vo uint32_t IMR;                  /* Interrupt mask register                        Address offset: 0x00 */
    __vo uint32_t EMR;                  /* Event mask register                            Address offset: 0x04 */
    __vo uint32_t RTSR;                 /* Rising trigger selection register              Address offset: 0x08 */
    __vo uint32_t FTSR;                 /* Falling trigger selection register             Address offset: 0x0C */
    __vo uint32_t SWIER;                /* Software interrupt event register              Address offset: 0x10 */
    __vo uint32_t PR;                   /* Pending register                               Address offset: 0x14 */

}EXTI_RegDef_t;


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

#define RCC             ((RCC_RegDef_t*) RCC_BASEADDR)

#define EXTI            ((EXTI_RegDef_t*) EXTI_BASEADDR)


/*
 * Clock Eanble Macros for GPIO peripherals
 */

#define GPIOA_PCLK_EN()     ( RCC->AHB1ENR |= ( 1 << 0 ) )
#define GPIOB_PCLK_EN()     ( RCC->AHB1ENR |= ( 1 << 1 ) )
#define GPIOC_PCLK_EN()     ( RCC->AHB1ENR |= ( 1 << 2 ) )
#define GPIOD_PCLK_EN()     ( RCC->AHB1ENR |= ( 1 << 3 ) )
#define GPIOE_PCLK_EN()     ( RCC->AHB1ENR |= ( 1 << 4 ) )
#define GPIOF_PCLK_EN()     ( RCC->AHB1ENR |= ( 1 << 5 ) )
#define GPIOG_PCLK_EN()     ( RCC->AHB1ENR |= ( 1 << 6 ) )
#define GPIOH_PCLK_EN()     ( RCC->AHB1ENR |= ( 1 << 7 ) )
#define GPIOI_PCLK_EN()     ( RCC->AHB1ENR |= ( 1 << 8 ) )


/*
 * Clock Eanble Macros for I2Cx peripherals
 */

#define I2C1_PCLK_EN()      ( RCC->APB1ENR |= ( 1 << 21 ) )
#define I2C2_PCLK_EN()      ( RCC->APB1ENR |= ( 1 << 22 ) )
#define I2C3_PCLK_EN()      ( RCC->APB1ENR |= ( 1 << 23 ) )


/*
 * Clock Eanble Macros for SPIx peripherals
 */

#define SPI1_PCLK_EN()      ( RCC->APB2ENR |= ( 1 << 12 ) )
#define SPI2_PCLK_EN()      ( RCC->APB1ENR |= ( 1 << 14 ) )
#define SPI3_PCLK_EN()      ( RCC->APB1ENR |= ( 1 << 15 ) )


/*
 * Clock Eanble Macros for USARTx peripherals
 */

#define USART1_PCLK_EN()      ( RCC->APB2ENR |= ( 1 << 4  ) )
#define USART2_PCLK_EN()      ( RCC->APB1ENR |= ( 1 << 17 ) )
#define USART3_PCLK_EN()      ( RCC->APB1ENR |= ( 1 << 18 ) )
#define UART4_PCLK_EN()       ( RCC->APB1ENR |= ( 1 << 19 ) )
#define UART5_PCLK_EN()       ( RCC->APB1ENR |= ( 1 << 20 ) )
#define USART6_PCLK_EN()      ( RCC->APB2ENR |= ( 1 << 5  ) )


/*
 * Clock Eanble Macros for SYSCFG peripherals
 */

#define SYSCFG_PCLK_EN()      ( RCC->APB2ENR |= ( 1 << 14  ) )



/*
 * Clock Disable Macros for GPIO peripherals
 */

#define GPIOA_PCLK_DI()     ( RCC->AHB1ENR &= ~( 1 << 0 ) )
#define GPIOB_PCLK_DI()     ( RCC->AHB1ENR &= ~( 1 << 1 ) )
#define GPIOC_PCLK_DI()     ( RCC->AHB1ENR &= ~( 1 << 2 ) )
#define GPIOD_PCLK_DI()     ( RCC->AHB1ENR &= ~( 1 << 3 ) )
#define GPIOE_PCLK_DI()     ( RCC->AHB1ENR &= ~( 1 << 4 ) )
#define GPIOF_PCLK_DI()     ( RCC->AHB1ENR &= ~( 1 << 5 ) )
#define GPIOG_PCLK_DI()     ( RCC->AHB1ENR &= ~( 1 << 6 ) )
#define GPIOH_PCLK_DI()     ( RCC->AHB1ENR &= ~( 1 << 7 ) )
#define GPIOI_PCLK_DI()     ( RCC->AHB1ENR &= ~( 1 << 8 ) )


/*
 * Clock Disable Macros for I2Cx peripherals
 */


/*
 * Clock Disable Macros for SPIx peripherals
 */


/*
 * Clock Disable Macros for USARTx peripherals
 */


/*
 * Clock Disable Macros for SYSCFG peripherals
 */


/*
 * Clock Disable Macros for SYSCFG peripherals
 */

#define GPIOA_REG_RESET()       do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1ENR &= ~(1 << 0)); } while(0)
#define GPIOB_REG_RESET()       do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1ENR &= ~(1 << 1)); } while(0)
#define GPIOC_REG_RESET()       do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1ENR &= ~(1 << 2)); } while(0)
#define GPIOD_REG_RESET()       do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1ENR &= ~(1 << 3)); } while(0)
#define GPIOE_REG_RESET()       do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1ENR &= ~(1 << 4)); } while(0)
#define GPIOF_REG_RESET()       do{ (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1ENR &= ~(1 << 5)); } while(0)
#define GPIOG_REG_RESET()       do{ (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1ENR &= ~(1 << 6)); } while(0)
#define GPIOH_REG_RESET()       do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1ENR &= ~(1 << 7)); } while(0)
#define GPIOI_REG_RESET()       do{ (RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1ENR &= ~(1 << 8)); } while(0)

// General Macros

#define HIGH                  1
#define LOW                   0
#define ENABLE                HIGH
#define DISABLE               LOW
#define SET                   ENABLE
#define RESET                 DISABLE
#define GPIO_PIN_SET          SET
#define GPIO_PIN_RESET        RESET 


#include "stm32f407xx_gpio_driver.h"


#endif /* INC_STM32F407XX_H_ */
