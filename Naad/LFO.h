/*
 * LFO.h
 *
 *  Created on: 2018/05/22
 *      Author: NishiAsakusa Audio Developments / oxxxide / Akikazu Iwasa
 */

#ifndef LFO_H_
#define LFO_H_

#include <math.h>
#include "dspconfig.h"
#include "Waveforms.h"
#include "FastMath_.h"
//#include "Oscill.h"

#define LFO_DEST_Osc_Amp 0
#define LFO_DEST_Osc_pitch 1
#define LFO_DEST_Mod_Amp 2
#define LFO_DEST_Mod_Pitch 3

#define NUM_ELEMENTS_FIXEDPOINT16 67108864
//#define NUM_ELEMENTS_LFO_WAVE 1024

typedef enum {
	Dest_Cutoff = 0, Dest_OSCPitch = 1, Dest_ModPitch = 2, Dest_ModDepth = 3
} LFO_DESTINATION;

typedef enum {
	LFO_SIN = 0, LFO_TRI = 1
} LFO_WAVEFORM;

const char* LFO_DEST_NAMES[4];

typedef struct {
	LFO_DESTINATION destination;
	uint32_t phase;
	float depth; //linear
	LFO_WAVEFORM waveform;
	uint8_t speed;
	uint8_t i_depth;
	uint32_t _delta;
	float *__wave_array;
} LFO;

void LFO_setSpeed(LFO *obj, uint8_t speed);

void LFO_setDepth(LFO *obj, uint8_t value);

void LFO_setWave(LFO *obj, LFO_WAVEFORM wave);

void LFO_setDest(LFO *obj, LFO_DESTINATION dest);

uint8_t LFO_getSpeed(LFO *obj);

uint8_t LFO_getDepth(LFO *obj) ;

LFO_DESTINATION LFO_getDest(LFO *obj) ;

const char* LFO_getDest_Name(LFO *obj) ;

const char* LFO_getWave_Name(LFO_WAVEFORM wf);

float LFO_proc(LFO* obj) ;

float LFO_proc_exp(LFO* obj) ;

void LFO_Init(LFO *obj);

#endif /* LFO_H_ */
