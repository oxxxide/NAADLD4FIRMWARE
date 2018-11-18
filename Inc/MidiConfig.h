/*
 * MidiConfig.h
 *
 *  Created on: 2018/01/02
 *      Author: NishiAsakusa Audio Developments / oxxxide / Akikazu Iwasa
 */

#ifndef MIDICONFIG_H_
#define MIDICONFIG_H_

#include "stm32f4xx.h"

#define CCNo_VOLUME 7
#define CCNo_EXPRESSION 11

#define CCNo_OSC_WAVE      9 //
#define CCNo_OSC_PITCH     2 //
#define CCNo_OSC_FINE      3 //
#define CCNo_OSC_MOD_DEPTH 12 //
#define CCNo_OSC_MOD_TYPE  70 //

#define CCNo_AMP_LEVEL   8  //
#define CCNo_AMP_PANPOT  10 //
#define CCNo_AMP_ENV_ATK 73 //
#define CCNo_AMP_ENV_HLD 75 //
#define CCNo_AMP_ENV_SLP 63 //
#define CCNo_AMP_ENV_REL 72 //

#define CCNo_MOD_FREQ 14 //
#define CCNo_MOD_ENV_ATK 15
#define CCNo_MOD_ENV_HLD 16
#define CCNo_MOD_ENV_SLP 17
#define CCNo_MOD_ENV_REL 18

#define CCNo_BEND_AMT      19
#define CCNo_BEND_VELSENSE 20
#define CCNo_BEND_ENV_ATK  21
#define CCNo_BEND_ENV_HLD  22
#define CCNo_BEND_ENV_SLP  23
#define CCNo_BEND_ENV_REL  24

#define CCNo_NOISE_LEVEL 25
#define CCNo_NOISE_ENV_ATK 26
#define CCNo_NOISE_ENV_HLD 27
#define CCNo_NOISE_ENV_SLP 28
#define CCNo_NOISE_ENV_REL 29

#define CCNo_FLT_TYPE   30
#define CCNo_FLT_CUTOFF 74 //
#define CCNo_FLT_RESO   71 //
#define CCNo_FLT_AMOUNT 31
#define CCNo_FLT_DECAY  41

#define CCNo_LFO_DEST   80
#define CCNo_LFO_WAVE   81
#define CCNo_LFO_SPEED  76
#define CCNo_LFO_DEPTH  77


#define ID_Osc_Wave    0x000
#define ID_Osc_Pitch   0x010
#define ID_Osc_Fine    0x011
#define ID_Osc_ModDpth 0x020
#define ID_Osc_ModType 0x030

#define ID_Amp_Level   0x100
#define ID_Amp_Pan     0x101
#define ID_Amp_Atk     0x110
#define ID_Amp_Hld     0x120
#define ID_Amp_Slp     0x121
#define ID_Amp_Rel     0x130

#define ID_Mod_Freq    0x200
#define ID_Mod_Atk     0x210
#define ID_Mod_Hld     0x220
#define ID_Mod_Slp     0x221
#define ID_Mod_Rel     0x230

#define ID_Bend_Amt    0x300
#define ID_Bend_velSns 0x301
#define ID_Bend_Atk    0x310
#define ID_Bend_Hld    0x320
#define ID_Bend_Slp    0x321
#define ID_Bend_Rel    0x330

#define ID_Noise_Lv    0x400
#define ID_Noise_Atk   0x410
#define ID_Noise_Hld   0x420
#define ID_Noise_Slp   0x421
#define ID_Noise_Rel   0x430

#define ID_Flt_Type    0x500
#define ID_Flt_Coff    0x510
#define ID_Flt_Reso    0x520
#define ID_Flt_Amt     0x530
#define ID_Flt_Dcy     0x531

#define ID_Lfo_dest    0x600
#define ID_Lfo_Wave    0x610
#define ID_Lfo_Spd     0x620
#define ID_Lfo_Dpt     0x630

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
	Exponential = 0, Linear = 1, Fixed = 2
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
