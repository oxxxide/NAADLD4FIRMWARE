/*
 * Gen.h
 *
 *  Created on: 2017/11/23
 *      Author: NishiAsakusa Audio Developments / oxxxide / Akikazu Iwasa
 */

#ifndef GEN_H_
#define GEN_H_

#include "FastMath_.h"
#include "AHR.h"
#include "Oscill.h"
#include "Noise.h"
#include "LFO.h"
#include "Decay.h"
#include "MultiFilter.h"
#include "MidiConfig.h"
#include "main.h"

#define NUM_OF_VOICES 4

typedef enum {
	MODTYPE_FM = 0, MODTYPE_AM = 1,  MODTYPE_FM_NOISE_ROUTED = 2
} MOD_TYPE;


#define NUMOF_MODTYPE 3

typedef struct{
	float pitchShift; //default:1 min:0 max:nolimit
	float cutoff;
	float modDepth;
	float bendAmt; //default:1 min:0 max:1
	float bendRel;
} CVInputParams;
extern CVInputParams CVInputParamsArray[NUM_OF_VOICES];

typedef struct {
	AHR_EG eg_amp;
	AHR_EG eg_mod;
	AHR_EG eg_bend;
	AHR_EG eg_noise;
	Decay decay_filter;

	Oscill carr;
	Oscill modu;
	MOD_TYPE modtype;
	MultiFilter filter;
	LFO lfo;

	float carr_level;
	float noise_level; // range: 0 to 1

	//float filter_amount;

	float mod_depth;
	float cf_ringmod_dev; // 1/(1+mod_depth)

	float bendVelAmount;

	float cv; //control voltage

	int modPitch;

	int i_bend_vel_sense;

	int i_cutoff;

	int i_fm_harmonics;

	//Level
	int i_level;
	int i_noise_level;

	//Pan
	int i_pan;

	float cof_pan_l;
	float cof_pan_r;

} Gen;

extern float adcValue1;
extern float adcValue2;
extern float adcValue3;
extern float adcValue4;

void Gen_trig(Gen *gen, float velocity);

void Gen_trig_note(Gen *gen, float velocity, uint8_t noteNo);

void Gen_init(Gen *gen);

float Gen_process_fm(Gen *gen, CVInputParams* cv);

float Gen_process_fm_plus_noise(Gen *gen, CVInputParams* cv);

float Gen_process_ringmod(Gen *gen, CVInputParams* cv);

//wave
void Gen_set_carr_wave(Gen *gen, Waveform wf);

Waveform Gen_get_carr_wave(Gen *gen);

void Gen_set_modtype(Gen *gen, MOD_TYPE type);

MOD_TYPE Gen_get_modtype(Gen *gen);

void Gen_set_carr_coarse(Gen *gen, int note);

int Gen_get_carr_coarse(Gen *gen);

void Gen_set_carr_fine(Gen *gen, int fine);

int Gen_get_carr_fine(Gen *gen);

void Gen_set_carr_moddepth(Gen *gen, int fm);

int Gen_get_carr_moddepth(Gen *gen);

// amplifier
void Gen_set_carr_level(Gen *gen, int level);

int Gen_get_carr_level(Gen *gen);

void Gen_set_carr_attack(Gen *gen, int v);

int Gen_get_carr_attack(Gen *gen);

void Gen_set_carr_hold(Gen *gen, int v);

int Gen_get_carr_hold(Gen *gen);

void Gen_set_carr_slope(Gen *gen, int v);

int Gen_get_carr_slope(Gen *gen);

void Gen_set_carr_release(Gen *gen, int v);

int Gen_get_carr_release(Gen *gen);

///modulation
void Gen_set_fm_amount(Gen *gen, int v);

int Gen_get_fm_amount(Gen *gen);

void Gen_set_fm_harmonics(Gen *gen, int v);

int Gen_get_fm_harmonics(Gen *gen);

void Gen_set_fm_attack(Gen *gen, int v);

int Gen_get_fm_attack(Gen *gen);

void Gen_set_fm_hold(Gen *gen, int v);

int Gen_get_fm_hold(Gen *gen);

void Gen_set_fm_slope(Gen *gen, int v);

int Gen_get_fm_slope(Gen *gen);

void Gen_set_fm_release(Gen *gen, int v);

int Gen_get_fm_release(Gen *gen);

///bend
void Gen_set_bend_amount(Gen *gen, int v);

int Gen_get_bend_amount(Gen *gen);

void Gen_set_bend_velocity_sense(Gen *gen, int v);

int Gen_get_bend_velocity_sense(Gen *gen);

void Gen_set_bend_attack(Gen *gen, int v);

int Gen_get_bend_attack(Gen *gen);

void Gen_set_bend_hold(Gen *gen, int v);

int Gen_get_bend_hold(Gen *gen);

void Gen_set_bend_slope(Gen *gen, int v);

int Gen_get_bend_slope(Gen *gen);

void Gen_set_bend_release(Gen *gen, int v);

int Gen_get_bend_release(Gen *gen);

//Noise
void Gen_set_noise_level(Gen* gen, int level);

//Noise
int Gen_get_noise_level(Gen* gen);

void Gen_set_noise_attack(Gen* gen, int v);

int Gen_get_noise_attack(Gen* gen);

void Gen_set_noise_hold(Gen* gen, int v);

void Gen_set_noise_slope(Gen* gen, int v);

int Gen_get_noise_hold(Gen* gen);

int Gen_get_noise_slope(Gen* gen);

void Gen_set_noise_release(Gen* gen, int v);

int Gen_get_noise_release(Gen* gen);

//Filter
//
void Gen_set_filter_cutoff(Gen* gen, int v);

uint8_t Gen_get_filter_cutoff(Gen* gen);
//

uint8_t Gen_get_filter_amount(Gen* gen);
//

//
void Gen_set_filter_resonance(Gen* gen, int v);

uint32_t Gen_get_filter_resonance(Gen* gen);
//

void Gen_set_filter_type(Gen* gen, int v);

int Gen_get_filter_type(Gen* gen);

//Filter Envelope
// -64 <= v >= +64
void Gen_set_filter_amount(Gen* gen, int v);

void Gen_set_filter_decay(Gen* gen, int v);

int Gen_get_filter_decay(Gen* gen);

void Gen_set_lfo_speed(Gen* gen, uint8_t v);

void Gen_set_lfo_depth(Gen* gen, uint8_t v);

void Gen_set_lfo_dest(Gen* gen, int8_t v);

void Gen_set_pan(Gen* gen, int v);

void Gen_set_pitchShift(Gen* gen, float rate);

void preset_kikck(Gen* gen);

void preset_hihat(Gen* gen);

void preset_click(Gen* gen);

void preset_hh_rnd(Gen* gen);

void preset_click_rnd(Gen* gen);



#endif /* GEN_H_ */
