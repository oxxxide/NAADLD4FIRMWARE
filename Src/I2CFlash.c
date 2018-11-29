/*
 * I2CFlash.c
 *
 *  Created on: 2018/09/23
 *      Author: NishiAsakusa Audio Developments / oxxxide / Akikazu Iwasa
 */

#include "I2CFlash.h"
#include "Tone.h"


#define TIMEOUT_MSEC_EEPROM 2000

void I2CFlash_Init(I2C_EEPROM* instance, I2C_HandleTypeDef *hi2c) {
	instance->pI2C = hi2c;
	instance->devAddress = 80 << 1;
	instance->memAddSize = I2C_MEMADD_SIZE_16BIT;
}

HAL_StatusTypeDef I2CFlash_Write(I2C_EEPROM* instance, uint16_t memAddress,
		uint8_t *pData, uint16_t size) {
	HAL_StatusTypeDef ret = waitUntilReady(instance);
	if (ret != HAL_OK) {
		return ret;
	}
	return HAL_I2C_Mem_Write(instance->pI2C, instance->devAddress, memAddress,
			instance->memAddSize, pData, size, TIMEOUT_MSEC_EEPROM);
}

HAL_StatusTypeDef I2CFlash_Read(I2C_EEPROM* instance, uint16_t memAddress,
		uint8_t *pData, uint16_t size) {
	HAL_StatusTypeDef ret = waitUntilReady(instance);
	if (ret != HAL_OK) {
		return ret;
	}
	return HAL_I2C_Mem_Read(instance->pI2C, instance->devAddress, memAddress,
			instance->memAddSize, pData, size, TIMEOUT_MSEC_EEPROM);

}

HAL_StatusTypeDef I2CFlash_SaveMidiConfig(I2C_EEPROM* instance,
		MidiConfig* midiconfig) {

	uint16_t size = sizeof(MidiConfig);
	HAL_StatusTypeDef ret = I2CFlash_Write(instance, ROM_ADDRESS_MIDICONFIG,
			(uint8_t*) midiconfig, size);
	return ret;
}

HAL_StatusTypeDef I2CFlash_SaveSequenceData(I2C_EEPROM* instance,
		Sequencer* seq) {
	uint16_t size = sizeof(Notes);
	Notes *p = &(seq->sequenceData[0]);
	HAL_StatusTypeDef ret;
	for(int i=0;i<16;i++){
		ret = I2CFlash_Write(instance, ROM_ADDRESS_SEQUENCE_DATA + (i*size), (uint8_t*)&p[i] ,size);
		if(ret != HAL_OK){
			return ret;
		}
		ret = waitUntilReady(instance);
	}
	return ret;
}


HAL_StatusTypeDef I2CFlash_LoadSequenceData(I2C_EEPROM* instance,
		Sequencer* seq) {
	uint16_t size = sizeof(Notes);
	Notes *p = &(seq->sequenceData[0]);
	HAL_StatusTypeDef ret;
	for(int i=0;i<16;i++){
		ret = I2CFlash_Read(instance, ROM_ADDRESS_SEQUENCE_DATA + (i*size), (uint8_t*)&p[i] ,size);
		if(ret != HAL_OK){
			return ret;
		}
		ret = waitUntilReady(instance);
	}
	return ret;
}

HAL_StatusTypeDef I2CFlash_FactoryReset(I2C_EEPROM* instance) {
	MidiConfig mc;
	InitMidiConfig(&mc);
	HAL_StatusTypeDef ret = I2CFlash_Write(instance, ROM_ADDRESS_MIDICONFIG,
			(uint8_t*) &mc, sizeof(MidiConfig));
	if (ret != HAL_OK) {
		return ret;
	}
	ret = waitUntilReady(instance);

	Tone t;
	InitTone(&t);
	for (int i = 0; i < 128; i++) {
		ret = I2CFlash_Write(instance, ROM_ADDRESS_TONE_USER + i * 64,
				(uint8_t*) &t, sizeof(Tone));
		if (ret != HAL_OK) {
			return ret;
		}
		ret = waitUntilReady(instance);
		if (ret != HAL_OK) {
			return ret;
		}
	}

	//clear temporary
	for (int i = 0; i < 4; i++) {
		ret = I2CFlash_Write(instance, ROM_ADDRESS_TONE_TMP + i * 64,
				(uint8_t*) &t, sizeof(Tone));
		if (ret != HAL_OK) {
			return ret;
		}
		ret = waitUntilReady(instance);
		if (ret != HAL_OK) {
			return ret;
		}
	}

	return ret;
}

HAL_StatusTypeDef waitUntilReady(I2C_EEPROM* instance) {
	HAL_I2C_StateTypeDef state = HAL_I2C_STATE_RESET;
	int cnt = 0;
	do {
		if (cnt > 500) {
			return HAL_TIMEOUT;
		}
		state = HAL_I2C_GetState(instance->pI2C);
		cnt++;
		HAL_Delay(10);
	} while (state != HAL_I2C_STATE_READY);
	return HAL_OK;
}
