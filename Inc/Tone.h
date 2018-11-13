/*
 * Tone.h
 *
 *  Created on: 2018/07/19
 *      Author: devox
 */

#ifndef TONE_H_
#define TONE_H_

#include "Gen.h"
#include "I2CFlash.h"

typedef struct {

	//OSC
	uint8_t osc_wave;
	uint8_t osc_coarse;
	int8_t osc_fine;
	uint8_t osc_mod_depth;

	//AMP
	uint8_t amp_level;
	uint8_t amp_env_a;
	uint8_t amp_env_h;
	uint8_t amp_env_s;
	uint8_t amp_env_r;

	//MOD
	int8_t mod_frequency;
	uint8_t mod_env_a;
	uint8_t mod_env_h;
	uint8_t mod_env_s;
	uint8_t mod_env_r;

	//BEND
	uint8_t bend_amount;
	uint8_t bend_env_a;
	uint8_t bend_env_h;
	uint8_t bend_env_s;
	uint8_t bend_env_r;

	//NOISE
	uint8_t noise_level;
	uint8_t noise_env_a;
	uint8_t noise_env_h;
	uint8_t noise_env_s;
	uint8_t noise_env_r;

	//FILTER
	uint8_t filter_type;
	uint8_t filter_cutoff;
	uint8_t filter_resonance;
	uint8_t filter_accent;
	uint8_t filter_decay;

	//LFO
	uint8_t lfo_destination;
	uint8_t lfo_wave;
	uint8_t lfo_speed;
	uint8_t lfo_depth;

	//routing
	uint8_t mod_type;
	int8_t panpot;

} Tone;

extern Tone tones[16];

HAL_StatusTypeDef SaveTone(I2C_EEPROM* eeprom, int programNumber, Tone* data);
HAL_StatusTypeDef ReadTone(I2C_EEPROM* eeprom, int programNumber, Tone* data);

void ToneCopyFromGen(Tone* dest, Gen* src);
void ToneCopyToGen(Gen* dest, Tone* src);

void InitTone(Tone* tone);

void InitFactorySetTones(void);

#endif /* TONE_H_ */
