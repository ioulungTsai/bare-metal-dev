#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

/* 
 * base address of Flash and SRAM memories
 */

#define FLASH_BASEADDR      0x08000000U         /* Base address of Flash        */
#define SRAM1_BASEADDR      0x20000000U         /* Base address of SRAM1        */
#define SRAM2_BASEADDR      0x20001C00U         /* Base address of SRAM2        */
#define ROM_BASEADDR        0x1FFF0000U         /* Base address of ROM          */
#define SRAM_BASEADDR       SRAM1_BASEADDR      /* Base address of Main SRAM    */




#endif /* INC_STM32F407XX_H_ */
