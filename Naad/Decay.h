#ifndef DECAY_H_
#define DECAY_H_

#include "dspconfig.h"

typedef struct {
	int i_decay;
	int i_amount;
	float decay_coefficient;
	float out;
} Decay;

void Decay_trig(Decay *d, float max);

void Decay_Init(Decay *d);

void Decay_set_Decay(Decay *d, int _value);

float Decay_proc(Decay *d);

#endif /* DECAY_H_ */
