/*
 * I2CFlash.h
 *
 *  Created on: 2018/09/23
 *      Author: NishiAsakusa Audio Developments / oxxxide / Akikazu Iwasa
 */

#ifndef I2CFLASH_H_
#define I2CFLASH_H_

#include "stm32f4xx_hal.h"
#include "MidiConfig.h"
#include "cvconfig.h"
#include "Sequencer.h"

typedef struct {
	I2C_HandleTypeDef *pI2C;
	uint16_t devAddress;
	uint16_t memAddSize;
	uint32_t timeout;
} I2C_EEPROM;

#define ROM_ADDRESS_SYSCONFIG 		0
#define ROM_ADDRESS_SYS_V_INFO 		2 //16byte
#define ROM_ADDRESS_MIDICONFIG 		64
#define ROM_ADDRESS_SEQUENCER 		128
#define ROM_ADDRESS_TONE_TMP 		256
#define ROM_ADDRESS_TONE_USER 		896
#define ROM_ADDRESS_SEQUENCE_DATA 	9088  //4*16byte
#define ROM_ADDRESS_SEQUENCE_BPM (ROM_ADDRESS_SEQUENCE_DATA+(16*sizeof(Notes))) //2byte
#define ROM_ADDRESS_SEQUENCE_STEPLENGTH (ROM_ADDRESS_SEQUENCE_BPM+2)  //4byte
#define ROM_ADDRESS_SEQUENCE_PLAYFX_ENABLED (ROM_ADDRESS_SEQUENCE_STEPLENGTH+4)  //1byte
#define ROM_ADDRESS_SEQUENCE_BEATREPEAT (ROM_ADDRESS_SEQUENCE_PLAYFX_ENABLED+1)  //?
#define ROM_ADDRESS_CV_MAPPING (ROM_ADDRESS_SEQUENCE_BEATREPEAT+20+1)

void I2CFlash_Init(I2C_EEPROM* instance, I2C_HandleTypeDef *hi2c);

HAL_StatusTypeDef I2CFlash_Write(I2C_EEPROM* instance, uint16_t memAddress,
		uint8_t *pData, uint16_t size);

HAL_StatusTypeDef I2CFlash_Read(I2C_EEPROM* instance, uint16_t memAddress1,
		uint8_t *pData, uint16_t size);

HAL_StatusTypeDef I2CFlash_SaveMidiConfig(I2C_EEPROM* instance,
		MidiConfig* midiconfig);

HAL_StatusTypeDef I2CFlash_SaveCvMappingConfig(I2C_EEPROM* instance,
		CV_ASSIGN* data);

HAL_StatusTypeDef I2CFlash_ReadCvMappingConfig(I2C_EEPROM* instance,
		CV_ASSIGN* data);

HAL_StatusTypeDef I2CFlash_SaveSequenceData(I2C_EEPROM* instance,
		Sequencer88* seq);

HAL_StatusTypeDef I2CFlash_LoadSequenceData(I2C_EEPROM* instance,
		Sequencer88* seq);

HAL_StatusTypeDef I2CFlash_SaveVersion(I2C_EEPROM* instance,
		uint16_t* version);

HAL_StatusTypeDef I2CFlash_LoadVersion(I2C_EEPROM* instance,
		uint16_t* version);

HAL_StatusTypeDef I2CFlash_FactoryReset(I2C_EEPROM* instance);

HAL_StatusTypeDef waitUntilReady(I2C_EEPROM* instance);

#endif /* I2CFLASH_H_ */
