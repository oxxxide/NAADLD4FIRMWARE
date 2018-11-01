/*
 * Tone.c
 *
 *  Created on: 2018/07/19
 *      Author: devox
 */

#include <string.h>
#include "Gen.h"
#include "Tone.h"
#include "processing.h"
#include "I2CFlash.h"

HAL_StatusTypeDef SaveTone(I2C_EEPROM* eeprom, int programNumber, Tone* data) {
	if (programNumber < 0 || programNumber > 127) {
		return HAL_ERROR;
	}
	int size = sizeof(Tone);
	HAL_StatusTypeDef ret = I2CFlash_Write(eeprom, ROM_ADDRESS_TONE_USER + (programNumber * 64),
			(uint8_t*) data, size);
	return ret;
}

HAL_StatusTypeDef ReadTone(I2C_EEPROM* eeprom, int programNumber, Tone* data) {
	if (programNumber < 0 || programNumber > 127) {
		return HAL_ERROR;
	}
	int size = sizeof(Tone);
	HAL_StatusTypeDef ret = I2CFlash_Read(eeprom, ROM_ADDRESS_TONE_USER + (programNumber * 64),
			(uint8_t*) data, size);
	return ret;
}

void InitTone(Tone* tone){
	memset(tone, 0,sizeof(Tone));
	tone->amp_level = 63;
}

void ToneCopyFromGen(Tone* dest, Gen* src) {

	//OSC
	dest->osc_wave = Gen_get_carr_wave(src);
	dest->osc_coarse = Gen_get_carr_coarse(src);
	dest->osc_fine = Gen_get_carr_fine(src);
	dest->osc_mod_depth = Gen_get_carr_moddepth(src);

	//AMP
	dest->amp_level = Gen_get_carr_level(src);
	dest->amp_env_a = Gen_get_carr_attack(src);
	dest->amp_env_h = Gen_get_carr_hold(src);
	dest->amp_env_s = Gen_get_carr_slope(src);
	dest->amp_env_r = Gen_get_carr_release(src);

	//MOD
	dest->mod_frequency = Gen_get_fm_harmonics(src);
	dest->mod_env_a = Gen_get_fm_attack(src);
	dest->mod_env_h = Gen_get_fm_hold(src);
	dest->mod_env_s = Gen_get_fm_slope(src);
	dest->mod_env_r = Gen_get_fm_release(src);

	//BEND
	dest->bend_amount = Gen_get_bend_amount(src);
	dest->bend_env_a = Gen_get_bend_attack(src);
	dest->bend_env_h = Gen_get_bend_hold(src);
	dest->bend_env_s = Gen_get_bend_slope(src);
	dest->bend_env_r = Gen_get_bend_release(src);

	//NOISE
	dest->noise_level = Gen_get_noise_level(src);
	dest->noise_env_a = Gen_get_noise_attack(src);
	dest->noise_env_h = Gen_get_noise_hold(src);
	dest->noise_env_s = Gen_get_noise_slope(src);
	dest->noise_env_r = Gen_get_noise_release(src);

	//FILTER
	dest->filter_type = Gen_get_filter_type(src);
	dest->filter_cutoff  = Gen_get_filter_cutoff(src);
	dest->filter_resonance  = Gen_get_filter_resonance(src);
	dest->filter_accent = Gen_get_filter_amount(src);
	dest->filter_decay  = Gen_get_filter_decay(src);

	//LFO
	dest->lfo_depth = src->lfo.i_depth;
	dest->lfo_destination = src->lfo.destination;
	dest->lfo_speed = src->lfo.speed;
	dest->lfo_wave = (uint8_t)src->lfo.waveform;

	//routinng
	dest->mod_type = (uint8_t)Gen_get_modtype(src);

}

void ToneCopyToGen(Gen* dest, Tone* src) {

	//OSC
	Gen_set_carr_wave(dest, src->osc_wave);
	Gen_set_carr_coarse(dest, src->osc_coarse);
	Gen_set_carr_fine(dest, src->osc_fine);
	Gen_set_carr_moddepth(dest, src->osc_mod_depth);

	//AMP
	Gen_set_carr_level(dest, src->amp_level);
	Gen_set_carr_attack(dest, src->amp_env_a);
	Gen_set_carr_hold(dest, src->amp_env_h);
	Gen_set_carr_slope(dest, src->amp_env_s);
	Gen_set_carr_release(dest, src->amp_env_r);

	//MOD
	Gen_set_fm_harmonics(dest, src->mod_frequency);
	Gen_set_fm_attack(dest, src->mod_env_a);
	Gen_set_fm_hold(dest, src->mod_env_h);
	Gen_set_fm_slope(dest, src->mod_env_s);
	Gen_set_fm_release(dest, src->mod_env_r);

	//BEND
	Gen_set_bend_amount(dest, src->bend_amount);
	Gen_set_bend_attack(dest, src->bend_env_a);
	Gen_set_bend_hold(dest, src->bend_env_h);
	Gen_set_bend_slope(dest, src->bend_env_s);
	Gen_set_bend_release(dest, src->bend_env_r);

	//NOISE
	Gen_set_noise_level(dest,src->noise_level);
	Gen_set_noise_attack(dest,src->noise_env_a);
	Gen_set_noise_hold(dest,src->noise_env_h);
	Gen_set_noise_slope(dest,src->noise_env_s);
	Gen_set_noise_release(dest,src->noise_env_r);

	//FILTER
	Gen_set_filter_type(dest,src->filter_type);
	Gen_set_filter_cutoff(dest,src->filter_cutoff);
	Gen_set_filter_resonance(dest,src->filter_resonance);
	Gen_set_filter_amount(dest,src->filter_accent);
	Gen_set_filter_decay(dest,src->filter_decay);


	//LFO
	Gen_set_lfo_depth(dest,src->lfo_depth);
	Gen_set_lfo_dest(dest,src->lfo_destination);
	Gen_set_lfo_speed(dest,src->lfo_speed);
	dest->lfo.waveform = (LFO_WAVEFORM)src->lfo_wave;

	//routing
	Gen_set_modtype(dest,src->mod_type);
}

Tone tones[16] = {0};

void SetDefaultKick1(Tone* t){
	t->amp_level= 127;
	t->amp_env_a = 0;
	t->amp_env_h = 2;
	t->amp_env_r = 30;
	t->amp_env_s = 0;

	t->bend_amount = 90;
	t->bend_env_a = 0;
	t->bend_env_h = 0;
	t->bend_env_r = 25;
	t->bend_env_s = 0;

	t->mod_env_a = 0;
	t->mod_env_h = 2;
	t->mod_env_r = 13;
	t->mod_env_s = 20;

	t->filter_type = BYPASS;

	t->osc_coarse = 24;
	t->osc_fine = 0;
	t->osc_wave = Waveform_SINE;
	t->osc_mod_depth = 0;

	t->lfo_depth = 0;
	t->lfo_speed = 0;
	t->lfo_destination = 0;
	t->lfo_wave = LFO_SIN;
}

void SetDefaultKick2(Tone* t){
	t->amp_level= 127;
	t->amp_env_a = 0;
	t->amp_env_h = 2;
	t->amp_env_r = 30;
	t->amp_env_s = 0;

	t->bend_amount = 90;
	t->bend_env_a = 0;
	t->bend_env_h = 0;
	t->bend_env_r = 25;
	t->bend_env_s = 0;

	t->mod_env_a = 0;
	t->mod_env_h = 2;
	t->mod_env_r = 13;
	t->mod_env_s = 20;

	t->filter_type = BYPASS;
	t->filter_cutoff = 70;
	t->filter_resonance = 0;

	t->osc_coarse = 24;
	t->osc_fine = 0;
	t->osc_wave = Waveform_PULSE;
	t->osc_mod_depth = 0;

	t->lfo_depth = 0;
	t->lfo_speed = 0;
	t->lfo_destination = 0;
	t->lfo_wave = LFO_SIN;
}

void SetDefaultKick3(Tone* t){
	t->amp_level= 127;
	t->amp_env_a = 0;
	t->amp_env_h = 2;
	t->amp_env_r = 30;
	t->amp_env_s = 0;

	t->bend_amount = 110;
	t->bend_env_a = 0;
	t->bend_env_h = 0;
	t->bend_env_r = 25;
	t->bend_env_s = 0;

	t->mod_env_a = 0;
	t->mod_env_h = 2;
	t->mod_env_r = 13;
	t->mod_env_s = 20;

	t->filter_type = BYPASS;
	t->filter_cutoff = 70;
	t->filter_resonance = 0;


	t->osc_coarse = 24;
	t->osc_fine = 0;
	t->osc_wave = Waveform_SINE;
	t->osc_mod_depth = 0;

	t->lfo_depth = 0;
	t->lfo_speed = 0;
	t->lfo_destination = 0;
	t->lfo_wave = LFO_SIN;
}

void SetDefaultKick4(Tone* t){
	t->amp_level= 127;
	t->amp_env_a = 0;
	t->amp_env_h = 5;
	t->amp_env_r = 40;
	t->amp_env_s = 10;

	t->bend_amount = 30;
	t->bend_env_a = 0;
	t->bend_env_h = 0;
	t->bend_env_r = 10;
	t->bend_env_s = 0;

	t->mod_env_a = 0;
	t->mod_env_h = 0;
	t->mod_env_r = 13;
	t->mod_env_s = 20;

	t->filter_type = BYPASS;
	t->filter_cutoff = 70;
	t->filter_resonance = 0;

	t->osc_coarse = 30;
	t->osc_fine = 0;
	t->osc_wave = Waveform_PULSE;
	t->osc_mod_depth = 0;

	t->lfo_depth = 0;
	t->lfo_speed = 0;
	t->lfo_destination = 0;
	t->lfo_wave = LFO_SIN;
}

//FM TamTam
void SetDefaultSynth14(Tone* t){

	t->osc_coarse = 50;
	t->osc_fine = 0;
	t->osc_wave = Waveform_SAW;
	t->osc_mod_depth = 5;

	t->amp_level= 127;
	t->amp_env_a = 0;
	t->amp_env_h = 20;
	t->amp_env_r = 40;
	t->amp_env_s = 0;

	t->bend_amount = 0;
	t->bend_env_a = 0;
	t->bend_env_h = 0;
	t->bend_env_r = 0;
	t->bend_env_s = 30;

	t->mod_env_a = 0;
	t->mod_env_h = 11;
	t->mod_env_r = 20;
	t->mod_env_s = 0;
	t->mod_frequency = 63;

	t->filter_type = HIGHPASS;
	t->filter_cutoff = 70;
	t->filter_resonance = 2;

	t->filter_decay = 10;

	t->lfo_depth = 45;
	t->lfo_speed = 45;
	t->lfo_destination = 0;
	t->lfo_wave = LFO_SIN;
}

//HH CP
void SetDefaultHH(Tone* t){

	t->osc_coarse = 60;
	t->osc_fine = 0;
	t->osc_wave = Waveform_SINE;
	t->osc_mod_depth = 90;

	t->amp_level= 0;
	t->amp_env_a = 0;
	t->amp_env_h = 20;
	t->amp_env_r = 40;
	t->amp_env_s = 0;

	t->bend_amount = 0;
	t->bend_env_a = 0;
	t->bend_env_h = 0;
	t->bend_env_r = 0;
	t->bend_env_s = 0;

	t->mod_env_a = 0;
	t->mod_env_h = 0;
	t->mod_env_r = 0;
	t->mod_env_s = 60;
	t->mod_frequency = 48;

	t->noise_env_a = 0;
	t->noise_env_h = 0;
	t->noise_env_r = 0;
	t->noise_env_s = 0;
	t->noise_level = 47;

	t->filter_type = HIGHPASS;
	t->filter_cutoff = 116;
	t->filter_resonance = 6;

	t->filter_decay = 0;

	t->lfo_depth = 0;
	t->lfo_speed = 0;
	t->lfo_destination = 0;
	t->lfo_wave = LFO_SIN;
}

void InitFactorySetTones(void){
	SetDefaultKick1((Tone*)&tones[0]);

	SetDefaultKick2((Tone*)&tones[1]);

	SetDefaultKick3((Tone*)&tones[2]);

	SetDefaultKick4((Tone*)&tones[3]);

	SetDefaultSynth14((Tone*)&tones[4]);

	SetDefaultSynth14((Tone*)&tones[5]);

	SetDefaultSynth14((Tone*)&tones[6]);
}

