/*
 * LFO.c
 *
 *  Created on: 2018/06/24
 *      Author: NishiAsakusa Audio Developments / oxxxide / Akikazu Iwasa
 */


#include "LFO.h"

const char* LFO_DEST_NAMES[4] =
		{ "Cutoff","OSC-Amp", "OSC-Pitch","Mod-Pitch" };

void LFO_setSpeed(LFO *obj, uint8_t speed) {
	obj->speed = speed;
	obj->_delta = (uint32_t)(
			(440.0f / 64.0f) * powf(2.0f, (speed - 69.0f) / 12.0f)
					/ SAMPLING_RATE * NUM_ELEMENTS_FIXEDPOINT16);
}

void LFO_setDepth(LFO *obj, uint8_t value) {
	obj->depth = value / 127.0f;
	obj->i_depth = value;
}

void LFO_setWave(LFO *obj, LFO_WAVEFORM wave) {
	obj->waveform = wave;
	switch (wave) {
	case LFO_SIN:
		obj->__wave_array = (float*) TABLE_INT_SIN;
		break;
	case LFO_TRI:
		obj->__wave_array = (float*) TABLE_INT_TRI;
		break;
	default:
		break;
	}
}

void LFO_setDest(LFO *obj, uint8_t dest) {
	obj->destination = dest;
}

uint8_t LFO_getSpeed(LFO *obj) {
	return obj->speed;
}

uint8_t LFO_getDepth(LFO *obj) {
	return obj->i_depth;
}

uint8_t LFO_getDest(LFO *obj) {
	return obj->destination;
}

const char* LFO_getDest_Name(LFO *obj) {
	return LFO_DEST_NAMES[obj->destination];
}

const char* LFO_getWave_Name(LFO_WAVEFORM wf) {
	switch (wf) {
	case LFO_TRI:
		return "Triangle";
	case LFO_SIN:
		return "Sin";
	default:
		return "Undefined";
	}
}

float LFO_proc(LFO* obj) {
	float* array = obj->__wave_array;
	float ret = array[(obj->phase >> 16)];
	obj->phase = (obj->phase + obj->_delta) & 0x3FFFFFF; //(1024<<16 + 0xFFFF)
	return (ret/8.0f) * obj->depth;
}

float LFO_proc_exp(LFO* obj) {
	float* array = obj->__wave_array;
	float x = array[obj->phase >> 16];
	obj->phase = (obj->phase + obj->_delta) & 0x3FFFFFF; //(1024<<16 + 0xFFFF)
	return Table_Exponential[(uint32_t)((x * obj->depth) + 512.0f)];
}

void LFO_Init(LFO *obj) {
	obj->destination = LFO_DEST_Osc_Amp;
	obj->phase = 0;
	obj->depth = 1.0f;
	obj->speed = 0;
	obj->i_depth = 0;
	obj->_delta = 0;
	LFO_setSpeed(obj, 60);
	LFO_setDepth(obj, 127);
	LFO_setWave(obj, LFO_TRI);
}
