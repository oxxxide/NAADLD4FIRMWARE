/*
 * I2CFlash.h
 *
 *  Created on: 2018/09/23
 *      Author: devox
 */

#ifndef I2CFLASH_H_
#define I2CFLASH_H_

#include "stm32f4xx_hal.h"

typedef struct {
	I2C_HandleTypeDef *pI2C;
	uint16_t devAddress;
	uint16_t memAddSize;
	uint32_t timeout;
} I2C_EEPROM;

#define ROM_ADDRESS_SYSCONFIG 0

#define ROM_ADDRESS_MIDICONFIG 64

#define ROM_ADDRESS_SEQUENCER 128

#define ROM_ADDRESS_TONE_FACTORY 256

#define ROM_ADDRESS_TONE_USER 896

void I2CFlash_Init(I2C_EEPROM* instance, I2C_HandleTypeDef *hi2c);

HAL_StatusTypeDef I2CFlash_Write(I2C_EEPROM* instance, uint16_t memAddress,
		uint8_t *pData, uint16_t size);

HAL_StatusTypeDef I2CFlash_Read(I2C_EEPROM* instance, uint16_t memAddress1,
		uint8_t *pData, uint16_t size);

#endif /* I2CFLASH_H_ */
