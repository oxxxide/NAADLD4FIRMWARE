/*
 * r_encoder.c
 *
 *  Created on: 2018/06/22
 *      Author: devox
 */

#include "r_encoder.h"

volatile uint64_t timestamp = 0;
Renc encoders[6];

void InitRotaryEncoders() {

	//A  c2,c3
	Renc *a = &encoders[ROTARY_ENCODER_A];
	a->id = ROTARY_ENCODER_A;
	a->count = 0;
	a->min = 0;
	a->max = 127;
	a->port1 = RoEncA1_GPIO_Port;
	a->port2 = RoEncA2_GPIO_Port;
	a->pin1 = RoEncA1_Pin;
	a->pin2 = RoEncA2_Pin;
	a->onChange = &onChangeRE_A;

	//B
	Renc *b = &encoders[ROTARY_ENCODER_B];
	b->id = ROTARY_ENCODER_B;
	b->count = 0;
	b->min = 0;
	b->max = 127;
	b->port1 = RoEncB1_GPIO_Port;
	b->port2 = RoEncB2_GPIO_Port;
	b->pin1 = RoEncB1_Pin;
	b->pin2 = RoEncB2_Pin;
	b->onChange = &onChangeRE_B;

	//C
	Renc *c = &encoders[ROTARY_ENCODER_C];
	c->id = ROTARY_ENCODER_C;
	c->count = 0;
	c->min = 0;
	c->max = 127;
	c->port1 = RoEncC1_GPIO_Port;
	c->port2 = RoEncC2_GPIO_Port;
	c->pin1 = RoEncC1_Pin;
	c->pin2 = RoEncC2_Pin;
	c->onChange = &onChangeRE_C;

	//D
	Renc *d = &encoders[ROTARY_ENCODER_D];
	d->id = ROTARY_ENCODER_D;
	d->count = 0;
	d->min = 0;
	d->max = 127;
	d->port1 = RoEncD1_GPIO_Port;
	d->port2 = RoEncD2_GPIO_Port;
	d->pin1 = RoEncD1_Pin;
	d->pin2 = RoEncD2_Pin;
	d->onChange = &onChangeRE_D;

	//E  a2,a3
	Renc *e = &encoders[ROTARY_ENCODER_E];
	e->id = ROTARY_ENCODER_E;
	e->count = 0;
	e->min = 0;
	e->max = 2;
	e->port1 = RoEncE1_GPIO_Port;
	e->port2 = RoEncE2_GPIO_Port;
	e->pin1 = RoEncE1_Pin;
	e->pin2 = RoEncE2_Pin;
	e->onChange = &onChangeRE_E;

	//S c0,c1
	encoders[0].id = ROTARY_ENCODER_S;
	encoders[0].count = 0;
	encoders[0].min = 0;
	encoders[0].max = 6;
	encoders[0].port1 = RoEncS1_GPIO_Port;
	encoders[0].port2 = RoEncS2_GPIO_Port;
	encoders[0].pin1 = RoEncS1_Pin;
	encoders[0].pin2 = RoEncS2_Pin;
	encoders[0].onChange = &onChangeRE_S;

}

int Renc_read(Renc *renc) {
	uint32_t a = (uint32_t) HAL_GPIO_ReadPin(renc->port1, renc->pin1); //A‘Š
	uint32_t b = (uint32_t) HAL_GPIO_ReadPin(renc->port2, renc->pin2); //B‘Š
	renc->data = (renc->data << 2) | (a << 1) | b;
	renc->data &= 0xF;
	int retval = 0;
	switch (renc->data) {
	case 0b1000: {
		uint64_t w = timestamp - renc->lastmodified;
		if (w < 7) {
			retval = -24;
		} else if (w < 15) {
			retval = -16;
		} else if (w < 22) {
			retval = -8;
		} else if (w < 30) {
			retval = -4;
		} else {
			retval = -1;
		}
		renc->lastmodified = timestamp;
	}
		break;
	case 0b1101: {
		uint64_t w = timestamp - renc->lastmodified;
		if (w < 7) {
			retval = 24;
		} else if (w < 15) {
			retval = 16;
		} else if (w < 22) {
			retval = 8;
		} else if (w < 30) {
			retval = 4;
		} else {
			retval = 1;
		}
		renc->lastmodified = timestamp;
	}
		break;
	default:
		break;
	}
	return retval;
}

static void checkReStateAndCallListener(Renc* re){
	int val = Renc_read(re);
	if (val) {
		re->onChange(re->id, val);
	}
}

void checkRotaryEncoders(){
	checkReStateAndCallListener(&encoders[ROTARY_ENCODER_A]);
	checkReStateAndCallListener(&encoders[ROTARY_ENCODER_B]);
	checkReStateAndCallListener(&encoders[ROTARY_ENCODER_C]);
	checkReStateAndCallListener(&encoders[ROTARY_ENCODER_D]);
	checkReStateAndCallListener(&encoders[ROTARY_ENCODER_E]);
	checkReStateAndCallListener(&encoders[ROTARY_ENCODER_S]);
}
