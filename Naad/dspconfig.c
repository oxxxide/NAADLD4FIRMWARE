/*
 * dspconfig.c
 *
 *  Created on: 2018/05/24
 *      Author: devox
 */

#include "dspconfig.h"


float FORCE_INLINE to_decibel_f(float gain)
{
    return 10*log10f(gain);
}

float FORCE_INLINE to_gain_f(float db)
{
    return powf(10,db/20.0f);
}

float FORCE_INLINE seconds_to_frame(float seconds)
{
    return seconds*SAMPLING_RATE_PER_MS;
}

float FORCE_INLINE ms_to_frame(float ms)
{
    return ms*SAMPLING_RATE_PER_MS;
}

float FORCE_INLINE note_to_freq(float note)
{
    return 440.0f * powf(2.0f, (note - 69.0f) / 12.0f);
}

float FORCE_INLINE gainToCoefficient(float gain)
{
    return powf(0.05f, 1.0f / ms_to_frame(to_gain_f(gain)));
}


