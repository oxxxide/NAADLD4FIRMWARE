/*
 * Osc.c
 *
 *  Created on: 2018/05/24
 *      Author: NishiAsakusa Audio Developments / oxxxide / Akikazu Iwasa
 */


#include "Oscill.h"


void Osc_Init(Oscill *osc) {
	osc->phase = 0;
	osc->modDelta = 0;
	Osc_set_pitch(osc, 60);
	Osc_set_fine(osc, 0);
	Osc_set_modgain(osc, 20);
	Osc_set_waveform(osc, Waveform_SINE);
}

const char* NameOf(Waveform wf) {
	switch (wf) {
	case Waveform_SINE:
		return "Sine";
	case Waveform_SAW:
		return "Saw";
	case Waveform_PULSE:
		return "Pulse";
	default:
		return "undefined";
	}
}

void Osc_set_waveform(Oscill *osc, Waveform wf) {
	switch (wf) {
	case Waveform_SINE:
		osc->waveArray = (float*)Table_Sin;
		osc->waveform = Waveform_SINE;
		break;
	case Waveform_SAW:
		osc->waveArray = (float*)Table_Sawtooth;
		osc->waveform = Waveform_SAW;
		break;
	case Waveform_PULSE:
		osc->waveArray = (float*)Table_square;
		osc->waveform = Waveform_PULSE;
		break;
	}
}

void Osc_set_pitch(Oscill *osc, int note) {
	if (note == osc->pitch) {
		return;
	}
	osc->pitch = note;
	float _fine = osc->fine / 128.0f;
	osc->delta =
			note_to_freq(osc->pitch + _fine) / SAMPLING_RATE * MAX_VALUE_FIXEDPOINT_4_16;
}

void Osc_set_fine(Oscill *osc, int finetune) {
	if (finetune == osc->fine) {
		return;
	}
	osc->fine = finetune;
	float _fine = osc->fine / 128.0f;

	osc->delta =
			note_to_freq(osc->pitch + _fine) / SAMPLING_RATE * MAX_VALUE_FIXEDPOINT_4_16;
}

void Osc_set_modgain(Oscill *osc, int note) {
	if (note < 0) {
		note = 0;
	}
	osc->egAmount = note;
	osc->modDelta = note_to_freq(osc->egAmount) / SAMPLING_RATE * MAX_VALUE_FIXEDPOINT_4_16;
}



FORCE_INLINE float Osc_proc(Oscill *osc) {
	osc->phase += osc->delta;
	osc->phase &= MASK_VALUE_FIXEDPOINT_4_16;
	return osc->waveArray[osc->phase>>16];
}

FORCE_INLINE float  Osc_proc_bend_fm_lfo(Oscill *osc, float offset ,float modAmount, float pMod,
		float lfo ) {

	uint32_t cvDelta = osc->delta  * offset;

	float exp = Table_Exponential_ex2 [(int)(pMod*1024+1024)];
	uint32_t d = (uint32_t) (((float) cvDelta + (modAmount * osc->modDelta)) * exp);
	osc->phase = (osc->phase + d) & MASK_VALUE_FIXEDPOINT_4_16;
	return osc->waveArray[osc->phase>>16];
}

