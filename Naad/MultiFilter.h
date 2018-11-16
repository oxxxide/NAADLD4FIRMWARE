/*
 * MultiFilter.h
 *
 *  Created on: 2017/12/19
 *      Author: NishiAsakusa Audio Developments / oxxxide / Akikazu Iwasa
 */

#ifndef MULTIFILTER_H_
#define MULTIFILTER_H_

#define BYPASS 0
#define LOWPASS 1
#define HIGHPASS 2
#define BANDPASS 3

#define NUM_OF_FILTER_TYPES 4

extern const char* FILTER_TYPE[];

#include "math.h"
#include "dspconfig.h"
#include "FastMath_.h"

typedef struct {
	float a0;
	float a1;
	float a2;
	float b0;
	float b1;
	float b2;
} FilterCoefficients;

extern const FilterCoefficients LowPassCoefficients[7][128];
extern const FilterCoefficients HighPassCoefficients[7][128];
extern const FilterCoefficients BandPassCoefficients[7][128];

typedef struct {

	float baseFreq;
	float offsetFreqRatio;
	uint32_t filter_type;
	int cutoff;
	volatile int resonance;
	float modulate;
	float velocityAmount;

	float env_amount;

	FilterCoefficients* confs;
	float x1;
	float x2;
	float y1;
	float y2;

} MultiFilter;

void Filter_setVelocityAmount(MultiFilter* flt, float v);

float Filter_process_no_envelope(MultiFilter* flt, float x);

float Filter_process_no_envelope_w_lfo(MultiFilter* flt, float x,
		int32_t lfo);

float Filter_process(MultiFilter* flt, float x, float egv);

void Filter_setCutoff(MultiFilter* flt, int cutoff, float freq);

void Filter_setResonance(MultiFilter* flt, int resonance);

void Filter_setEnvAmount(MultiFilter* flt, float v);

void Filter_setModulate(MultiFilter* flt, float v);

void Filter_setFilterType(MultiFilter* flt, int type, int reset);

void Filter_Init(MultiFilter* flt);

#endif /* MULTIFILTER_H_ */
