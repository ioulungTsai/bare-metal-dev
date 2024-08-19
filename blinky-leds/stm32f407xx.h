/**
 * @file stm32f407xx.h
 * @brief STM32F407xx device-specific header file
 *
 * This file contains:
 * - Base addresses for memory-mapped regions and peripherals
 * - Peripheral register structures
 * - Peripheral declaration
 * - Peripheral register bit definitions
 *
 * @note This file is for educational purposes and may not be complete.
 *       For production, refer to official ST Microelectronics headers.
 *
 * @author  Brandon Tsai
 * @date    8/15/2024
 */

#ifndef STM32F407XX_H
#define STM32F407XX_H

#include <stdint.h>

#define _IO     volatile

/* Base address for memories */
#define FLASH_BASE              (0x08000000U)
#define SRAM1_BASE              (0x20000000U)
#define SRAM2_BASE              (0x2001C000U)

/* Base address for peripheral buses */
#define PERIPH_BASE             (0x40000000U)
#define PERIPH_BASE_APB1        (PERIPH_BASE + 0x00000)
#define PERIPH_BASE_APB2        (PERIPH_BASE + 0x10000)
#define PERIPH_BASE_AHB1        (PERIPH_BASE + 0x20000)
#define PERIPH_BASE_AHB2        (0x50000000U)
#define PERIPH_BASE_AHB3        (0x60000000U)

/* Base boundary address for all peripherals */

/* AHB1 */
#define GPIO_D_BASE             (PERIPH_BASE_AHB1 + 0x0C00)
#define RCC_BASE                (PERIPH_BASE_AHB1 + 0x3800)

/* GPIO register definition structure */
typedef struct {
        _IO uint32_t MODER;             /* GPIO port mode register */
        _IO uint32_t OTYPER;            /* GPIO port output type register */
        _IO uint32_t OSPEEDR;           /* GPIO port output speed register */
        _IO uint32_t PUPDR;             /* GPIO port pull-up/pull-down register */
        _IO uint32_t IDR;               /* GPIO port input data register */
        _IO uint32_t ODR;               /* GPIO port output data register */
        _IO uint32_t BSRR;              /* GPIO port bit set/reset register */
        _IO uint32_t LCKR;              /* GPIO port configuration lock register */
        _IO uint32_t AFR[2];            /* GPIO alternate function low/high register */
} GPIO_RegDef_t;

/* RCC register definition structure */
typedef struct {
        _IO uint32_t CR;                /* Clock control register */
        _IO uint32_t PLLCFGR;           /* PLL configuration register */
        _IO uint32_t CFGR;              /* Clock configuration register */
        _IO uint32_t CIR;               /* Clock interrupt register */
        _IO uint32_t AHB1RSTR;          /* AHB1 peripheral reset register */
        _IO uint32_t AHB2RSTR;          /* AHB2 peripheral reset register */
        _IO uint32_t AHB3RSTR;          /* AHB3 peripheral reset register */
        uint32_t RESERVED_0;            /* Reserved, 0x1C */
        _IO uint32_t APB1RSTR;          /* APB1 peripheral reset register */
        _IO uint32_t APB2RSTR;          /* APB2 peripheral reset register */
        uint32_t RESERVED_1[2];         /* Reserved, 0x28-0x2C */
        _IO uint32_t AHB1ENR;           /* AHB1 peripheral clock enable register */
        _IO uint32_t AHB2ENR;           /* AHB2 peripheral clock enable register */
        _IO uint32_t AHB3ENR;           /* AHB3 peripheral clock enable register */
        uint32_t RESERVED_2;            /* Reserved, 0x3C */
        _IO uint32_t APB1ENR;           /* APB1 peripheral clock enable register */
        _IO uint32_t APB2ENR;           /* APB2 peripheral clock enable register */
        uint32_t RESERVED_3[2];         /* Reserved, 0x48-0x4C */
        _IO uint32_t AHB1LPENR;         /* AHB1 peripheral clock enable in low power mode register */
        _IO uint32_t AHB2LPENR;         /* AHB2 peripheral clock enable in low power mode register */
        _IO uint32_t AHB3LPENR;         /* AHB3 peripheral clock enable in low power mode register */
        uint32_t RESERVED_4;            /* Reserved, 0x5C */
        _IO uint32_t APB1LPENR;         /* APB1 peripheral clock enable in low power mode register */
        _IO uint32_t APB2LPENR;         /* APB2 peripheral clock enable in low power mode register */
        uint32_t RESERVED_5[2];         /* Reserved, 0x68-0x6C */
        _IO uint32_t BDCR;              /* Backup domain control register */
        _IO uint32_t CSR;               /* Clock control & status register */
        uint32_t RESERVED_6[2];         /* Reserved, 0x78-0x7C */
        _IO uint32_t SSCGR;             /* Spread spectrum clock generation register */
        _IO uint32_t PLLI2SCFGR;        /* PLLI2S configuration register */
} RCC_RegDef_t;

#endif /* STM32F407XX_H */
