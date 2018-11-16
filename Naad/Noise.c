/*
 * Noise.c
 *
 *  Created on: 2018/06/26
 *      Author: NishiAsakusa Audio Developments / oxxxide / Akikazu Iwasa
 */

#include "noise.h"
#include "FastMath_.h"

volatile static uint32_t x, y, z, w;

void Noise_Init() {
	x = 123456789;
	y = 362436069;
	z = 521288629;
	w = 88675123;
}

float FORCE_INLINE Noise_Generate() {
	uint32_t t = x ^ (x << 11);
	x = y;
	y = z;
	z = w;
	w = (w ^ (w >> 19)) ^ (t ^ (t >> 8));
	return (((float)w)/UINT_MAX)*2.0f-1.0f;
}

uint32_t FORCE_INLINE Noise_Generate_uint() {
	uint32_t t = x ^ (x << 11);
	x = y;
	y = z;
	z = w;
	w = (w ^ (w >> 19)) ^ (t ^ (t >> 8));
	return w;
}
