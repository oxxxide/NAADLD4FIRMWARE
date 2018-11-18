/*
 * Renc.h
 *
 *  Created on: 2017/11/26
 *      Author: NishiAsakusa Audio Developments / oxxxide / Akikazu Iwasa
 */

#ifndef R_ENCODER_H_
#define R_ENCODER_H_

#include "stm32f4xx.h"

#define ROTARY_ENCODER_S 0
#define ROTARY_ENCODER_A 1
#define ROTARY_ENCODER_B 2
#define ROTARY_ENCODER_C 3
#define ROTARY_ENCODER_D 4
#define ROTARY_ENCODER_E 5

#define COL_LEFT_EDGE 1
#define COL_RIGHT_EDGE 4

typedef struct {
	uint8_t id;
	int8_t max;
	int8_t min;
	GPIO_TypeDef *port1;
	uint32_t pin1;
	GPIO_TypeDef *port2;
	uint32_t pin2;
	uint32_t data;
	int count;
	uint64_t lastmodified;
	void (*onChange)(int id, int add);
} Renc;

extern Renc encoders[6];

extern volatile uint64_t timestamp;

void onChangeRE_S(int id, int add);

void onChangeRE_A(int id, int add);

void onChangeRE_B(int id, int add);

void onChangeRE_C(int id, int add);

void onChangeRE_D(int id, int add);

void onChangeRE_E(int id, int add);

void InitRotaryEncoders(void);

int Renc_read(Renc *renc);

void checkRotaryEncoders(void);

void CheckGPIO_Pins(void);

#endif /* R_ENCODER_H_ */
