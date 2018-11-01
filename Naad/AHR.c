/*
 * AHR.c
 *
 *  Created on: 2018/06/24
 *      Author: devox
 */

#include "AHR.h"


void FORCE_INLINE AHR_trig(AHR_EG *ahr) {
    ahr->phase = PHASE_A;
    ahr->hold_count = ahr->hold_length;
}

void FORCE_INLINE AHR_reset(AHR_EG *ahr) {
	ahr->phase = PHASE_A;
	ahr->out = 0;
	ahr->hold_count = ahr->hold_length;
}

void FORCE_INLINE AHR_set_attack(AHR_EG *ahr, int _value) {
	_value = _limit_(_value);
	ahr->i_attack = _value;
	float millisec = powf(10, (_value / 127.0f * 60.0f) / 20.0f);
	float samples = (millisec / 1000.0f) * SAMPLING_RATE;
	ahr->attack_delta = 1.0f / samples;
}

void FORCE_INLINE AHR_set_slope(AHR_EG *ahr, int _value) {
	_value = _limit_(_value);
	ahr->i_slope = _value;
	float s_value = _value / 127.0f;
	ahr->h_value_slope_endpoint = 1.0f - s_value;
	ahr->h_slope_decay_delta = s_value / ahr->hold_length;
}

void FORCE_INLINE AHR_set_hold(AHR_EG *ahr, int _value) {
	_value = _limit_(_value);
	ahr->i_hold = _value;
	ahr->hold_length = (uint32_t) (SAMPLING_RATE * _value / 127.0f);
	if (ahr->hold_count > ahr->hold_length) {
		ahr->hold_count = ahr->hold_length;
	}
	AHR_set_slope(ahr,ahr->i_slope);
}

void FORCE_INLINE AHR_set_release(AHR_EG *ahr, int _value){
    _value = _limit_(_value);
    ahr->i_release = _value;
    float millisec = powf(10, ahr->i_release/20.0f);
    ahr->decay_coefficient = powf(0.1f, 1.0f / (millisec * (SAMPLING_RATE/1000.0f)));
}

float FORCE_INLINE AHR_proc(AHR_EG *ahr) {
	switch (ahr->phase) {
	case PHASE_A:
		ahr->out += ahr->attack_delta;
		if (ahr->out >= 1.0f) {
			ahr->out = 1.0f;
			ahr->phase = PHASE_H;
		}
		break;

	case PHASE_H:
		if (ahr->hold_count > 0) {
			ahr->hold_count--;
			ahr->out -= ahr->h_slope_decay_delta;
		} else {
			ahr->hold_count = 0;
			ahr->phase = PHASE_R;
		}
		;
		break;

	case PHASE_R:
		if (ahr->out > MIN_THRESHOLD) {
			ahr->out *= ahr->decay_coefficient;
		} else {
			ahr->out = 0;
			ahr->phase = PHASE_N;
		}
		break;

	default:
		ahr->out = 0;
		break;
	}
	return ahr->out;
}

void AHR_Init(AHR_EG *ahr) {
	ahr->out = 0;
	ahr->phase = PHASE_N;
	ahr->i_slope = 0;
	ahr->h_value_slope_endpoint = 1.0f;


	AHR_set_attack(ahr,0);
	AHR_set_hold(ahr,10);
	AHR_set_release(ahr,60);

}
