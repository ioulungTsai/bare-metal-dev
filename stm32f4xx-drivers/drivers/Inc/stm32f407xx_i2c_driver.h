/*
 * stm32f407xx_i2c_driver.h
 *
 *  Created on: May 5 2024
 *      Author: Brandon Tsai
 */

#ifndef INC_STM32F407XX_I2C_DRIVER_H_
#define INC_STM32F407XX_I2C_DRIVER_H_

#include "stm32f407xx.h"

/*
 * Configuration structure for I2Cx peripheral
 */

typedef struct
{
	uint32_t I2C_SCLSpeed;
	uint8_t  I2C_DeviceAddress;
	uint8_t  I2C_AckControl;
	uint8_t  I2C_FMDutyCycle;

}I2C_Config_t;

/*
 * Handle structure for I2Cx peripheral
 */

typedef struct
{
	I2C_RegDef_t 	*pI2Cx;
	I2C_Config_t 	I2C_Config;

}I2C_Handle_t;


/*
 * @I2C_SCLSpeed
 */

#define I2C_SCL_SPEED_SM 	100000
#define I2C_SCL_SPEED_FM4K 	400000
#define I2C_SCL_SPEED_FM2K  200000


/*
 * @I2C_AckControl
 */

#define I2C_ACK_ENABLE        1
#define I2C_ACK_DISABLE       0


/*
 * @I2C_FMDutyCycle
 */

#define I2C_FM_DUTY_2        0
#define I2C_FM_DUTY_16_9     1


#endif
