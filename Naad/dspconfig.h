/*
 * dspconfig.h
 *
 *  Created on: 2017/12/03
 *      Author: NishiAsakusa Audio Developments / oxxxide / Akikazu Iwasa
 */

#ifndef DSPCONFIG_H_
#define DSPCONFIG_H_

#include "stm32f4xx_hal.h"
#include "math.h"
#include "FastMath_.h"

//static const float M_PI = (3.14159265358979323846);/* pi */
#define SAMPLING_RATE (48000)
#define MIN_THRESHOLD (0.00152587890625f)
#define WAVE_LENGTH 1024
#define WAVE_MASK 1023

static const float SAMPLING_RATE_PER_MS = 48000.0f;

/*
unsigned int HIWORD(unsigned int v)
{
    return (v >> 16);
}
*/

float to_decibel_f(float gain);

float to_gain_f(float db);

float seconds_to_frame(float seconds);

float ms_to_frame(float ms);

float note_to_freq(float note);

float gainToCoefficient(float gain);

#endif /* DSPCONFIG_H_ */
