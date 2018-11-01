/*
 * dspconfig.c
 *
 *  Created on: 2018/05/24
 *      Author: devox
 */

#include "dspconfig.h"


float to_decibel_f(float gain)
{
    return 10*log10f(gain);
}

float to_gain_f(float db)
{
    return powf(10,db/20.0f);
}

float seconds_to_frame(float seconds)
{
    return seconds*SAMPLING_RATE_PER_MS;
}

float ms_to_frame(float ms)
{
    return ms*SAMPLING_RATE_PER_MS;
}

float note_to_freq(float note)
{
    return 440.0f * powf(2.0f, (note - 69.0f) / 12.0f);
}

float gainToCoefficient(float gain)
{
    return powf(0.05f, 1.0f / ms_to_frame(to_gain_f(gain)));
}


