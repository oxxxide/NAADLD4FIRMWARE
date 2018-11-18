/*
 * Gen.c
 *
 *  Created on: 2018/05/18
 *      Author: NishiAsakusa Audio Developments / oxxxide / Akikazu Iwasa
 */

#include "Gen.h"
#include "math.h"

float adcValue1 = 0;
float adcValue2 = 0;
float adcValue3 = 0;
float adcValue4 = 0;

void Gen_trig(Gen *gen, float velocity){
	AHR_trig(&gen->eg_noise, velocity);
	Decay_trig(&gen->decay_filter, 1.0f);
	AHR_trig(&gen->eg_mod, 1.0f);
	float reduce = (1.0f-velocity)*gen->bendVelAmount;
	AHR_trig(&gen->eg_bend, 1.0f - reduce);
	AHR_trig(&gen->eg_amp, velocity);
}

void Gen_init(Gen *gen) {

	Filter_Init(&gen->filter);
	Gen_set_carr_level(gen, 64);
	Gen_set_pan(gen, 0);
	Gen_set_bend_velocity_sense(gen, 127);

	gen->noise_level = 0;
	gen->mod_depth = 0;
	gen->cf_ringmod_dev = 1.0f;
	gen->bendVelAmount = 1.0f;
	gen->cv = adcValue1;
	gen->modtype = MODTYPE_FM;

	AHR_Init(&gen->eg_amp);
	AHR_Init(&gen->eg_mod);
	AHR_Init(&gen->eg_bend);
	AHR_Init(&gen->eg_noise);

	Decay_Init(&gen->decay_filter);

	Osc_Init(&gen->carr);
	Osc_Init(&gen->modu);

	LFO_Init(&gen->lfo);

}

float FORCE_INLINE Gen_process_fm(Gen *gen, float cv) {
	float v_eg_amp = AHR_proc(&gen->eg_amp);
	float bend = AHR_proc(&gen->eg_bend);
	float fmv=0;
	int cutoff_mod = 0;
	const LFO_DESTINATION lfo_dest = LFO_getDest(&gen->lfo);
	switch (lfo_dest) {
		case Dest_ModPitch:
			fmv = Osc_proc_lfo(&gen->modu,&gen->lfo);
			break;
		case Dest_Cutoff:
			cutoff_mod = LFO_proc(&gen->lfo);
			fmv = Osc_proc(&gen->modu);
			break;
		default :
			fmv = Osc_proc(&gen->modu);
			break;
	}
	fmv = fmv * AHR_proc(&gen->eg_mod) * gen->mod_depth;
	float v_osc_carr = Osc_proc_bend_fm_lfo(&gen->carr, cv, bend, fmv, &gen->lfo);
	float v_noise = Noise_Generate() * AHR_proc(&gen->eg_noise)
			* gen->noise_level;
	float ret = (v_osc_carr * v_eg_amp * gen->carr_level) + (v_noise);

	switch (gen->filter.filter_type) {
		case BYPASS:
			return ret;
		default:
			return Filter_process_no_envelope_w_lfo(&gen->filter, ret,
					(int32_t) (Decay_proc(&gen->decay_filter) * gen->decay_filter.i_amount + cutoff_mod));
	}
}

float FORCE_INLINE Gen_process_ringmod(Gen *gen, float cv) {
	float v_eg_amp = AHR_proc(&gen->eg_amp);
	float bend = AHR_proc(&gen->eg_bend);
	float amv = 0;
	float v_osc_carr = 0;
	int cutoff_mod = 0;
	const LFO_DESTINATION lfo_dest = LFO_getDest(&gen->lfo);
	switch(lfo_dest){
		case Dest_ModPitch:
			amv = Osc_proc_lfo(&gen->modu,&gen->lfo);
			v_osc_carr = Osc_proc_bend(&gen->carr, cv, bend, 1.0f);
			break;
		case Dest_Cutoff:
			cutoff_mod = LFO_proc(&gen->lfo);
			amv = Osc_proc(&gen->modu);
			v_osc_carr = Osc_proc_bend(&gen->carr, cv, bend, 1.0f);
			break;
		case Dest_ModDepth:
			amv = Osc_proc(&gen->modu);
			amv = ( (LFO_proc(&gen->lfo)+63.9f)/128.0f)*amv;
			v_osc_carr = Osc_proc_bend(&gen->carr, cv, bend, 1.0f);
			break;
		case Dest_OSCPitch :
		default:
			amv = Osc_proc(&gen->modu);
			v_osc_carr = Osc_proc_bend(&gen->carr, cv, bend, LFO_proc_exp(&gen->lfo));
			break;
	}

	amv = amv * AHR_proc(&gen->eg_mod) * gen->mod_depth;
	v_osc_carr = (v_osc_carr * (1+amv)) * gen->cf_ringmod_dev;
	float v_noise = Noise_Generate() * AHR_proc(&gen->eg_noise) * gen->noise_level;
	float ret = (v_osc_carr * v_eg_amp * gen->carr_level) + (v_noise);
	float dk = Decay_proc(&gen->decay_filter);

	switch (gen->filter.filter_type) {
	case BYPASS:
		break;
	default:
		ret = Filter_process_no_envelope_w_lfo(&gen->filter, ret, (int32_t) (dk*gen->decay_filter.i_amount + cutoff_mod));
		break;
	}
	return ret;
}

void Gen_set_carr_wave(Gen *gen, Waveform wf) {
	Osc_set_waveform(&gen->carr,wf);
}

Waveform Gen_get_carr_wave(Gen *gen) {
	return gen->carr.waveform;
}

void Gen_set_modtype(Gen *gen, MOD_TYPE type){
	gen->modtype = type;
}

MOD_TYPE Gen_get_modtype(Gen *gen){
	return gen->modtype;
}

void Gen_set_carr_coarse(Gen *gen, int note) {

	note = LIMIT(note,127,0);

	Osc_set_pitch(&gen->carr, note);

	Osc_set_pitch(&gen->modu, note + gen->i_fm_harmonics);
}

int Gen_get_carr_coarse(Gen *gen) {
	return gen->carr.pitch;
}

void Gen_set_carr_fine(Gen *gen, int fine) {

	fine = LIMIT(fine,63,-63);

	Osc_set_fine(&gen->carr, fine);
	Osc_set_fine(&gen->modu, fine);
}

int Gen_get_carr_fine(Gen *gen) {
	return gen->carr.fine;
}

void Gen_set_carr_moddepth(Gen *gen, int depth) {
	depth = LIMIT(depth,127,0);
	gen->mod_depth = (depth / 127.0f);
	gen->cf_ringmod_dev = 1.0f / (1.0f + gen->mod_depth);
}

int Gen_get_carr_moddepth(Gen *gen) {
	return (int) (gen->mod_depth * 127);
}

// amplifier
void Gen_set_carr_level(Gen *gen, int level) {
	level = LIMIT(level,127,0);
	float v = level / 127.0f;
	gen->i_level = level;
	gen->carr_level = v*v;
}

int Gen_get_carr_level(Gen *gen) {
	return gen->i_level;
}

void Gen_set_carr_attack(Gen *gen, int v) {
	v = LIMIT(v,127,0);
	AHR_set_attack(&gen->eg_amp, v);
}

int Gen_get_carr_attack(Gen *gen) {
	return gen->eg_amp.i_attack;
}

void Gen_set_carr_hold(Gen *gen, int v) {

	v = LIMIT(v,127,0);

	AHR_set_hold(&gen->eg_amp, v);
}

int Gen_get_carr_hold(Gen *gen) {
	return gen->eg_amp.i_hold;
}

void Gen_set_carr_slope(Gen *gen, int v) {

	v = LIMIT(v,127,0);

	AHR_set_slope(&gen->eg_amp, v);
}

int Gen_get_carr_slope(Gen *gen) {
	return gen->eg_amp.i_slope;
}

void Gen_set_carr_release(Gen *gen, int v) {

	v = LIMIT(v,127,0);

	AHR_set_release(&gen->eg_amp, v);
}

int Gen_get_carr_release(Gen *gen) {
	return gen->eg_amp.i_release;
}

///fm
void Gen_set_fm_amount(Gen *gen, int v) {

	v = LIMIT(v,127,0);

	gen->mod_depth = v/127.0f;
}

int Gen_get_fm_amount(Gen *gen) {
	return (int)(gen->mod_depth * 127.0f);
}

void Gen_set_fm_harmonics(Gen *gen, int v){

	v = LIMIT(v,63,-63);

	gen->i_fm_harmonics = v;
	Osc_set_pitch(&gen->modu,gen->carr.pitch + v);
}

int Gen_get_fm_harmonics(Gen *gen){
	return gen->i_fm_harmonics;
}

void Gen_set_fm_attack(Gen *gen, int v) {

	v = LIMIT(v,127,0);

	AHR_set_attack(&gen->eg_mod, v);
}

int Gen_get_fm_attack(Gen *gen) {
	return gen->eg_mod.i_attack;
}

void Gen_set_fm_hold(Gen *gen, int v) {

	v = LIMIT(v,127,0);

	AHR_set_hold(&gen->eg_mod, v);
}

int Gen_get_fm_hold(Gen *gen) {
	return gen->eg_mod.i_hold;
}

void Gen_set_fm_slope(Gen *gen, int v) {

	v = LIMIT(v,127,0);

	AHR_set_slope(&gen->eg_mod, v);
}

int Gen_get_fm_slope(Gen *gen) {
	return gen->eg_mod.i_slope;
}

void Gen_set_fm_release(Gen *gen, int v) {

	v = LIMIT(v,127,0);

	AHR_set_release(&gen->eg_mod, v);
}

int Gen_get_fm_release(Gen *gen) {
	return gen->eg_mod.i_release;
}

///bend

void Gen_set_bend_amount(Gen *gen, int v) {
	v = LIMIT(v,127,0);
	Osc_set_modgain(&gen->carr, v);
}

int Gen_get_bend_amount(Gen *gen) {
	return gen->carr.egAmount;
}

void Gen_set_bend_velocity_sense(Gen *gen, int v){
	v = LIMIT(v,127,0);
	gen->i_bend_vel_sense = v;
	gen->bendVelAmount = v/127.0f;
}

int Gen_get_bend_velocity_sense(Gen *gen){
	return (int)(gen->i_bend_vel_sense);
}

void Gen_set_bend_attack(Gen *gen, int v) {
	v = LIMIT(v,127,0);
	AHR_set_attack(&gen->eg_bend, v);
}

int Gen_get_bend_attack(Gen *gen) {
	return gen->eg_bend.i_attack;
}

void Gen_set_bend_hold(Gen *gen, int v) {
	v = LIMIT(v,127,0);
	AHR_set_hold(&gen->eg_bend, v);
}

int Gen_get_bend_hold(Gen *gen) {
	return gen->eg_bend.i_hold;
}

void Gen_set_bend_slope(Gen *gen, int v) {
	v = LIMIT(v,127,0);
	AHR_set_slope(&gen->eg_bend, v);
}

int Gen_get_bend_slope(Gen *gen) {
	return gen->eg_bend.i_slope;
}

void Gen_set_bend_release(Gen *gen, int v) {
	v = LIMIT(v,127,0);
	AHR_set_release(&gen->eg_bend, v);
}

int Gen_get_bend_release(Gen *gen) {
	return gen->eg_bend.i_release;
}

//Noise
void Gen_set_noise_level(Gen* gen, int level) {
	gen->i_noise_level = level;
	float v = LIMIT(level,127,0) / 127.0f;
	gen->noise_level = v * v;
}

//Noise
int Gen_get_noise_level(Gen* gen) {
	return gen->i_noise_level;
}

void Gen_set_noise_attack(Gen* gen, int v) {
	AHR_set_attack(&gen->eg_noise, v);
}

int Gen_get_noise_attack(Gen* gen) {
	return gen->eg_noise.i_attack;
}

void Gen_set_noise_hold(Gen* gen, int v) {
	AHR_set_hold(&gen->eg_noise, v);
}

void Gen_set_noise_slope(Gen* gen, int v) {
	AHR_set_slope(&gen->eg_noise, v);
}

int Gen_get_noise_hold(Gen* gen) {
	return gen->eg_noise.i_hold;
}

int Gen_get_noise_slope(Gen* gen) {
	return gen->eg_noise.i_slope;
}

void Gen_set_noise_release(Gen* gen, int v) {
	AHR_set_release(&gen->eg_noise, v);
}

int Gen_get_noise_release(Gen* gen) {
	return gen->eg_noise.i_release;
}

//Filter
void Gen_set_filter_cutoff(Gen* gen, int v) {
	gen->i_cutoff = LIMIT(v,127,0);
	Filter_setCutoff(&gen->filter, v, note_to_freq(v));
}

void Gen_set_filter_cutoff_w_lfo(Gen* gen, int v,float lfov1) {
	gen->i_cutoff = LIMIT(v,127,0);
	Filter_setCutoff(&gen->filter, v, note_to_freq(v));
}

uint8_t Gen_get_filter_cutoff(Gen* gen) {
	return gen->i_cutoff;
}

uint8_t Gen_get_filter_amount(Gen* gen) {
	return (uint8_t)gen->decay_filter.i_amount;
}

void Gen_set_filter_resonance(Gen* gen, int v) {
	v = LIMIT(v,6,0);
	gen->filter.resonance = v;
}

uint32_t Gen_get_filter_resonance(Gen* gen) {
	return gen->filter.resonance;
}

void Gen_set_filter_type(Gen* gen, int v) {

	v = LIMIT(v,NUM_OF_FILTER_TYPES,0);

	Filter_setFilterType(&gen->filter, v, 1);
}

int Gen_get_filter_type(Gen* gen) {
	return gen->filter.filter_type;
}

//Filter Envelope
// -64 <= v >= +64
void Gen_set_filter_amount(Gen* gen, int v) {
	if (v < 0) {
		gen->filter_amount = (1.0f / ((-v) / 12.7f)) + 1.0f;
	} else {
		gen->filter_amount = (v / 12.7f) + 1.0f;
	}
}

void Gen_set_filter_decay(Gen* gen, int v){

	v = LIMIT(v,127,0);

	Decay_set_Decay(&gen->decay_filter,v);
}

int Gen_get_filter_decay(Gen* gen){
	return gen->decay_filter.i_decay;
}


void Gen_set_lfo_speed(Gen* gen, uint8_t v) {

	v = LIMIT(v,127,0);

	LFO_setSpeed(&gen->lfo, v);
}

void Gen_set_lfo_depth(Gen* gen, uint8_t v) {

	v = LIMIT(v,127,0);

	LFO_setDepth(&gen->lfo, v);
}

void Gen_set_lfo_dest(Gen* gen, int8_t dest) {
	LFO_setDest(&(gen->lfo), dest);
}

void Gen_set_pan(Gen* gen, int v) {
	v = LIMIT(v,63,-63);
	gen->i_pan = v;

	if (v < 0) {
		//R
		float cof = fabsf((float) v) / 63.0f;
		gen->cof_pan_l = 1.0f;
		gen->cof_pan_r = 1.0f - cof;
	} else if (v > 0) {
		//L
		float cof = fabsf((float) v) / 63.0f;
		gen->cof_pan_l = 1.0f - cof;
		gen->cof_pan_r = 1.0f;
	} else {
		//center
		gen->cof_pan_l = 1.0f;
		gen->cof_pan_r = 1.0f;
	}
}

void preset_kikck(Gen* gen){
	Gen_set_carr_coarse(gen,29);
	Gen_set_carr_moddepth(gen,0);
	Gen_set_carr_hold(gen,10);
	Gen_set_carr_release(gen,40);
	Gen_set_bend_amount(gen,70);
	Gen_set_bend_hold(gen,0);
	Gen_set_bend_release(gen,25);
}

void preset_hihat(Gen* gen) {
	Gen_set_carr_level(gen,63);
	Gen_set_carr_coarse(gen, 120);
	Gen_set_carr_moddepth(gen, 127);
	Gen_set_carr_hold(gen, 7);
	Gen_set_carr_release(gen, 25);

	Gen_set_fm_harmonics(gen, 123);
	Gen_set_fm_hold(gen, 1);
	Gen_set_fm_release(gen, 20);

	Gen_set_bend_amount(gen, 0);
	Gen_set_bend_hold(gen, 0);
	Gen_set_bend_release(gen, 0);

	Gen_set_noise_level(gen, 65);
	Gen_set_noise_hold(gen, 5);
	Gen_set_noise_release(gen, 15);

	Gen_set_filter_type(gen, HIGHPASS);
	Gen_set_filter_cutoff(gen, 110);
	Gen_set_filter_resonance(gen, 70);
}


void preset_click(Gen* gen) {
	Gen_set_carr_level(gen,70);
	Gen_set_carr_coarse(gen, 120);
	Gen_set_carr_moddepth(gen, 100);
	Gen_set_carr_hold(gen, 10);
	Gen_set_carr_release(gen, 15);

	Gen_set_fm_attack(gen, 32);
	Gen_set_fm_harmonics(gen, 90);
	Gen_set_fm_hold(gen, 1);
	Gen_set_fm_release(gen, 10);

	Gen_set_bend_amount(gen, 0);
	Gen_set_bend_hold(gen, 0);
	Gen_set_bend_release(gen, 0);

	Gen_set_noise_level(gen, 30);
	Gen_set_noise_hold(gen, 10);
	Gen_set_noise_release(gen, 5);

	Gen_set_filter_type(gen, HIGHPASS);
	Gen_set_filter_cutoff(gen, 60);
	Gen_set_filter_resonance(gen, 60);
}


void preset_hh_rnd(Gen* gen){
	Gen_set_carr_release(gen, (int)(20 + Noise_Generate() * 20));
	Gen_set_fm_harmonics(gen, (int)(90 + Noise_Generate()*40));
}

void preset_click_rnd(Gen* gen) {

	Gen_set_carr_level(gen, 40 );

	Gen_set_carr_moddepth(gen, (int) 100 + Noise_Generate() * 27);
	Gen_set_carr_hold(gen, 10);
	Gen_set_carr_release(gen, 30);

	Gen_set_fm_attack(gen, (int) (32 + Noise_Generate() * 32));
	Gen_set_fm_harmonics(gen, (int) (90 + Noise_Generate() * 60));
	Gen_set_fm_hold(gen, (int) (20 + Noise_Generate() * 20));
	Gen_set_fm_release(gen, 5);

	Gen_set_bend_amount(gen, 0);
	Gen_set_bend_hold(gen, 0);
	Gen_set_bend_release(gen, 0);

	Gen_set_noise_level(gen, 80);
	Gen_set_noise_attack(gen, (int)(30 + Noise_Generate() * 30));
	Gen_set_noise_hold(gen, (int)(15 + Noise_Generate() * 15));
	Gen_set_noise_release(gen, 10);

	Gen_set_filter_type(gen, HIGHPASS);
	Gen_set_filter_cutoff(gen, (int) (70 + Noise_Generate() * 40));
	Gen_set_filter_resonance(gen, (int) (70 + Noise_Generate() * 40));
}
