#ifndef AHR_H_
#define AHR_H_

#include "dspconfig.h"

#define PHASE_N 0
#define PHASE_A 1
#define PHASE_H 2
#define PHASE_R 3

static inline int _limit_(int v) {
	return v < 0 ? 0 : v;
}

typedef struct {

	int i_attack;
	int i_hold;
	int i_release;
	int i_slope;

	float decay_coefficient;

	float attack_delta;

	float h_value_slope_endpoint;
	float h_slope_decay_delta;

	uint32_t hold_length;
	uint32_t hold_count;

	float out;
	int phase;

} AHR_EG;

void AHR_trig(AHR_EG *ahr);

void AHR_reset(AHR_EG *ahr);

void AHR_set_attack(AHR_EG *ahr, int _value);

void AHR_set_slope(AHR_EG *ahr, int _value);

void AHR_set_hold(AHR_EG *ahr, int _value);

void AHR_set_release(AHR_EG *ahr, int _value);

float AHR_proc(AHR_EG *ahr);

void AHR_Init(AHR_EG *ahr);

#endif /* AHR_H_ */
