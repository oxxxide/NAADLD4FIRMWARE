/*
 * MidiConfig.h
 *
 *  Created on: 2018/01/02
 *      Author: devox
 */

#ifndef MIDICONFIG_H_
#define MIDICONFIG_H_

#include "stm32f4xx.h"

#define CCNo_VOLUME 100
#define CCNo_EXPRESSION 99

#define CCNo_OSC_WAVE 1
#define CCNo_OSC_PITCH 2
#define CCNo_OSC_FINE 3
#define CCNo_OSC_MOD_DEPTH 4
#define CCNo_OSC_MOD_TYPE 5

#define CCNo_AMP_LEVEL 6
#define CCNo_AMP_PANPOT 7
#define CCNo_AMP_ENV_ATK 8
#define CCNo_AMP_ENV_HLD 9
#define CCNo_AMP_ENV_SLP 10
#define CCNo_AMP_ENV_REL 11

#define CCNo_BEND_AMT 12
#define CCNo_BEND_VELSENSE 13
#define CCNo_BEND_ENV_ATK 14
#define CCNo_BEND_ENV_HLD 15
#define CCNo_BEND_ENV_SLP 16
#define CCNo_BEND_ENV_REL 17

#define CCNo_NOISE_LEVEL 18
#define CCNo_NOISE_ENV_ATK 19
#define CCNo_NOISE_ENV_HLD 20
#define CCNo_NOISE_ENV_SLP 21
#define CCNo_NOISE_ENV_REL 22

#define CCNo_FLT_TYPE 23
#define CCNo_FLT_CUTOFF 24
#define CCNo_FLT_RESO  25
#define CCNo_FLT_AMOUNT 26
#define CCNo_FLT_DECAY 27

#define CCNo_LFO_DEST 28
#define CCNo_LFO_WAVE  30
#define CCNo_LFO_SPEED 31

#define CCNo_LFO_DEPTH 32

/*
static uint32_t CREATE_ID(uint8_t row, uint8_t col, uint8_t shift){
	return (uint32_t)(shift <<8 | row<<4 | col);
}
*/

typedef struct{

} PARAM_ID;

#include "stm32f4xx.h"

const float VelocityCurve_Linear[128];
const float VelocityCurve_Exponential[128];

typedef enum {
	InternalClock,
	ExternalClock
} SyncMode;

typedef enum {
	Exponential,
	Linear,
	Fixed
} VelocityCurve;

typedef struct {

	uint8_t channel_A;
	uint8_t noteNo_A;

	uint8_t channel_B;
	uint8_t noteNo_B;

	uint8_t channel_C;
	uint8_t noteNo_C;

	uint8_t channel_D;
	uint8_t noteNo_D;

	SyncMode syncMode;

	uint8_t echoBack;

	uint8_t outputSystemMessage;

	VelocityCurve velocityCurve;

} MidiConfig;

void InitMidiConfig(MidiConfig *instance);

#endif /* MIDICONFIG_H_ */
