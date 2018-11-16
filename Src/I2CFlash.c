/*
 * I2CFlash.c
 *
 *  Created on: 2018/09/23
 *      Author: NishiAsakusa Audio Developments / oxxxide / Akikazu Iwasa
 */

#include "I2CFlash.h"

#define TIMEOUT_MSEC_EEPROM 2000

void I2CFlash_Init(I2C_EEPROM* instance, I2C_HandleTypeDef *hi2c) {
	instance->pI2C = hi2c;
	instance->devAddress = 80 << 1;
	instance->memAddSize = I2C_MEMADD_SIZE_16BIT;
}

HAL_StatusTypeDef I2CFlash_Write(I2C_EEPROM* instance, uint16_t memAddress,
		uint8_t *pData, uint16_t size) {
	return HAL_I2C_Mem_Write(instance->pI2C, instance->devAddress, memAddress,
			instance->memAddSize, pData, size, TIMEOUT_MSEC_EEPROM);
}

HAL_StatusTypeDef I2CFlash_Read(I2C_EEPROM* instance, uint16_t memAddress,
		uint8_t *pData, uint16_t size) {

	return HAL_I2C_Mem_Read(instance->pI2C, instance->devAddress, memAddress,
			instance->memAddSize, pData, size, TIMEOUT_MSEC_EEPROM);

}

HAL_StatusTypeDef I2CFlash_SaveMidiConfig(I2C_EEPROM* instance, MidiConfig* midiconfig) {
	uint16_t size = sizeof(MidiConfig);
	HAL_StatusTypeDef ret = I2CFlash_Write(instance, ROM_ADDRESS_MIDICONFIG,
			(uint8_t*) midiconfig, size);
	return ret;
}

HAL_StatusTypeDef I2CFlash_FactoryReset(I2C_EEPROM* instance) {

	uint8_t zero[64] = {0};
	//ZeroClear

	/*
	for (int i = 0; i < 512; i++) {  // 64*512 = 32768bytes
		HAL_StatusTypeDef ret = I2CFlash_Write(instance, i * 64, &zero[0], 64);
		if (ret != HAL_OK) {
			return ret;
		}
	}
	*/

	//Write Initial Setting

	MidiConfig mc;
	InitMidiConfig(&mc);
	I2CFlash_Write(instance,ROM_ADDRESS_MIDICONFIG,(uint8_t*)&mc,sizeof(MidiConfig));
	return HAL_OK;
}
