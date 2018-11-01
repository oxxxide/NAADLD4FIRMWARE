/*
 * Osc.h
 *
 *  Created on: 2017/11/24
 *      Author: devox
 */

#ifndef OSCIL_H_
#define OSCIL_H_

#include "main.h"
#include "dspconfig.h"
#include "math.h"

#include "FastMath_.h"
#include "Waveforms.h"

typedef enum {
	Waveform_SINE = 0, Waveform_SAW = 1, Waveform_PULSE = 2
} Waveform;

typedef struct _OSC {
	Waveform waveform;
	int pitch;
	int fine;
	int egAmount;
	uint32_t phase; /* 03FF FFF */
	uint32_t delta;
	float modDelta;
	float* waveArray;
} Oscill;

#define MAX_VALUE_FIXEDPOINT_4_16 67108864
#define MASK_VALUE_FIXEDPOINT_4_16 67108863


extern void Osc_Init(Oscill *osc) ;

const char* NameOf(Waveform wf);

void Osc_set_waveform(Oscill *osc, Waveform wf);

extern void Osc_set_pitch(Oscill *osc, int note);

extern void Osc_set_fine(Oscill *osc, int finetune);

extern void Osc_set_modgain(Oscill *osc, int note);

extern float Osc_proc(Oscill *osc);

extern float Osc_proc_bend_fm_lfo(Oscill *osc, float offset,float modAmount, float pMod, float lfoDepth);

#endif /* OSCIL_H_ */
