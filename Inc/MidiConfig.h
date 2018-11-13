/*
 * MidiConfig.h
 *
 *  Created on: 2018/01/02
 *      Author: devox
 */

#ifndef MIDICONFIG_H_
#define MIDICONFIG_H_

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
