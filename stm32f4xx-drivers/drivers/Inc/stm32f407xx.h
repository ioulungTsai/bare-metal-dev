#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

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
#define UART2_BASEADDR                  ((APB1PERIPH_BASEADDR) + 0x4400)
#define UART3_BASEADDR                  ((APB1PERIPH_BASEADDR) + 0x4800)
#define UART4_BASEADDR                  ((APB1PERIPH_BASEADDR) + 0x4C00)
#define UART5_BASEADDR                  ((APB1PERIPH_BASEADDR) + 0x5000)
#define I2C1_BASEADDR                   ((APB1PERIPH_BASEADDR) + 0x5400)
#define I2C2_BASEADDR                   ((APB1PERIPH_BASEADDR) + 0x5800)
#define I2C3_BASEADDR                   ((APB1PERIPH_BASEADDR) + 0x5C00)


/* 
 * base address of peripherals which hanging on APB2 bus
 * TODO : Complete for all other peripherals
 */

#define UART1_BASEADDR                  ((APB2PERIPH_BASEADDR) + 0x1000)
#define UART6_BASEADDR                  ((APB2PERIPH_BASEADDR) + 0x1400)
#define SPI1_BASEADDR                   ((APB2PERIPH_BASEADDR) + 0x3000)
#define SYSCFG_BASEADDR                 ((APB2PERIPH_BASEADDR) + 0x3800)
#define EXTI_BASEADDR                   ((APB2PERIPH_BASEADDR) + 0x3C00)


#endif /* INC_STM32F407XX_H_ */
