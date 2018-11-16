/*
 * Noise.h
 *
 *  Created on: 2017/11/23
 *      Author: NishiAsakusa Audio Developments / oxxxide / Akikazu Iwasa
 */

#ifndef NOISE_H_
#define NOISE_H_

#include <limits.h>
#include "stm32f4xx_hal.h"

void Noise_Init();

float Noise_Generate();

uint32_t Noise_Generate_uint();

#endif /* NOISE_H_ */
