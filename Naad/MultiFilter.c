/*
 * MultiFilter.c
 *
 *  Created on: 2018/06/24
 *      Author: devox
 */

#include "MultiFilter.h"

const char* FILTER_TYPE[] = { "Bypass", "Low-pass", "High-pass", "Band-pass" };

static void FORCE_INLINE updateCoeffcients(MultiFilter* flt) {
	switch (flt->filter_type) {
	case LOWPASS:
		flt->confs =
				(FilterCoefficients*) &LowPassCoefficients[flt->resonance][flt->cutoff];
		return;
	case HIGHPASS:
		flt->confs =
				(FilterCoefficients*) &HighPassCoefficients[flt->resonance][flt->cutoff];
		return;
	case BANDPASS:
		flt->confs =
				(FilterCoefficients*) &BandPassCoefficients[flt->resonance][flt->cutoff];
		return;
	}
}

static void FORCE_INLINE updateCoeffcients_temporary(MultiFilter* flt,uint32_t cutoff) {
	switch (flt->filter_type) {
	case LOWPASS:
		flt->confs =
				(FilterCoefficients*) &LowPassCoefficients[flt->resonance][cutoff];
		return;
	case HIGHPASS:
		flt->confs =
				(FilterCoefficients*) &HighPassCoefficients[flt->resonance][cutoff];
		return;
	case BANDPASS:
		flt->confs =
				(FilterCoefficients*) &BandPassCoefficients[flt->resonance][cutoff];
		return;
	}
}

void Filter_setVelocityAmount(MultiFilter* flt, float v) {
	flt->velocityAmount = v;
}

float FORCE_INLINE Filter_process_no_envelope(MultiFilter* flt, float x) {
	FilterCoefficients* cf = flt->confs;
	float y = (cf->b0 / cf->a0) * x + (cf->b1 / cf->a0) * flt->x1
			+ (cf->b2 / cf->a0) * flt->x2 - (cf->a1 / cf->a0) * flt->y1
			- (cf->a2 / cf->a0) * flt->y2;
	flt->x2 = flt->x1;
	flt->x1 = x;
	flt->y2 = flt->y1;
	flt->y1 = y;
	return y;
}

float FORCE_INLINE Filter_process_no_envelope_w_lfo(MultiFilter* flt, float x,
		int32_t lfo) {

	uint32_t index = LIMIT(flt->cutoff + lfo,127,0);
	updateCoeffcients_temporary(flt,index);

	FilterCoefficients* cf = flt->confs;
	float y = (cf->b0 / cf->a0) * x + (cf->b1 / cf->a0) * flt->x1
			+ (cf->b2 / cf->a0) * flt->x2 - (cf->a1 / cf->a0) * flt->y1
			- (cf->a2 / cf->a0) * flt->y2;
	flt->x2 = flt->x1;
	flt->x1 = x;
	flt->y2 = flt->y1;
	flt->y1 = y;
	return y;
}

float FORCE_INLINE Filter_process(MultiFilter* flt, float x, float egv) {

	if (flt->filter_type == BYPASS) {
		return x;
	}

	FilterCoefficients* cf = flt->confs;

	float y = (cf->b0 / cf->a0) * x + (cf->b1 / cf->a0) * flt->x1
			+ (cf->b2 / cf->a0) * flt->x2 - (cf->a1 / cf->a0) * flt->y1
			- (cf->a2 / cf->a0) * flt->y2;
	flt->x2 = flt->x1;
	flt->x1 = x;
	flt->y2 = flt->y1;
	flt->y1 = y;
	return y;
}

void Filter_setCutoff(MultiFilter* flt, int cutoff, float freq) {
	flt->cutoff = cutoff;
	updateCoeffcients(flt);
}

void Filter_setResonance(MultiFilter* flt, int resonance) {
	flt->resonance = resonance;
	updateCoeffcients(flt);
}

void Filter_setEnvAmount(MultiFilter* flt, float v) {
	flt->env_amount = v;
}

void Filter_setModulate(MultiFilter* flt, float v) {
	flt->modulate = FastMath_toPitchCoefficient(v * 2);
}

void Filter_setFilterType(MultiFilter* flt, int type, int reset) {
	flt->filter_type = type;
	updateCoeffcients(flt);
}

void Filter_Init(MultiFilter* flt) {

	flt->cutoff = 127;
	flt->offsetFreqRatio = (1.0f);
	flt->baseFreq = (0.0f);
	flt->velocityAmount = (1.0f);
	flt->filter_type = (BYPASS);
	flt->resonance = 0;
	flt->env_amount = (0.5f);
	flt->confs = (FilterCoefficients*) &LowPassCoefficients[0][127];
	flt->x1 = (0.0f);
	flt->x2 = (0.0f);
	flt->y1 = (0.0f);
	flt->y2 = (0.0f);
	flt->modulate = 1.0f;

	Filter_setCutoff(flt, 127, 0);
	Filter_setResonance(flt, 0);
	Filter_setFilterType(flt, BYPASS, 1);

}
