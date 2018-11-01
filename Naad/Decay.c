/*
 * Decay.c
 *
 *  Created on: 2018/06/24
 *      Author: devox
 */

#include "FastMath_.h"
#include "Decay.h"

void FORCE_INLINE Decay_trig(Decay* d) {
	d->out = 1.0f;
}

void FORCE_INLINE Decay_set_Decay(Decay* d, int _value) {
	d->i_decay = _value;
	float millisec = powf(10, d->i_decay / 20.0f);
	d->decay_coefficient = powf(0.1f,
			1.0f / (millisec * (SAMPLING_RATE / 1000.0f)));
}

float FORCE_INLINE Decay_proc(Decay* d) {
	if (d->out > MIN_THRESHOLD) {
		d->out *= d->decay_coefficient;
	} else {
		d->out = 0;
	}
	return d->out;
}

void FORCE_INLINE Decay_Init(Decay* d) {
	d->i_amount = 0;
	Decay_set_Decay(d,30);
}
