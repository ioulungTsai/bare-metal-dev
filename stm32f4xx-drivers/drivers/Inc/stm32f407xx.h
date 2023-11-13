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

#define GPIOA_BASEADDR                  ((AHB1PERIPH_BASEADDR) + 0x0000U)
#define GPIOB_BASEADDR                  ((AHB1PERIPH_BASEADDR) + 0x0400U)
#define GPIOC_BASEADDR                  ((AHB1PERIPH_BASEADDR) + 0x0800U)
#define GPIOD_BASEADDR                  ((AHB1PERIPH_BASEADDR) + 0x0C00U)
#define GPIOE_BASEADDR                  ((AHB1PERIPH_BASEADDR) + 0x1000U)
#define GPIOF_BASEADDR                  ((AHB1PERIPH_BASEADDR) + 0x1400U)
#define GPIOG_BASEADDR                  ((AHB1PERIPH_BASEADDR) + 0x1800U)
#define GPIOH_BASEADDR                  ((AHB1PERIPH_BASEADDR) + 0x1C00U)
#define GPIOI_BASEADDR                  ((AHB1PERIPH_BASEADDR) + 0x2000U)


#endif /* INC_STM32F407XX_H_ */
