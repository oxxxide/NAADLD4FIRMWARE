/*
 * MIDIParser.c
 *
 *  Created on: 2018/09/26
 *      Author: NishiAsakusa Audio Developments / oxxxide / Akikazu Iwasa
 */

#include "MIDIParser.h"

static void parseSignal(uint8_t b);

void MidiParser_PushByte(uint8_t byte) {
	if (byte == 0xFE) {
		//Ignore Active Sensing
		return;
	}
	if (byte == 0xF8) {
		//MIDI Timing Clock
		return;
	}
	parseSignal(byte);
}

void parseSignal(uint8_t b) {

	uint8_t status = b >> 4;
	static uint8_t firstByte = 0;
	static uint8_t secondByte = 0;
	static unsigned int counter = 0;

	if (status & 0b1000) {
		//Status Byte
		if (counter >= 1) {
			counter = 0;
			//over again
			parseSignal(b);
			return;
		}
	} else {
		//Data Byte
		if (counter == 0) {
			if ((firstByte >> 4) == 0xF) {
				//ignore system message
				return;
			} else {
				//running status
				counter++;
			}
		}
	}


	switch (counter) {
	case 0: {
		counter++;
		firstByte = b;
		break;
	}
	return;
	case 1: {
		switch (firstByte >> 4) {
		case 0xC:
			ON_RECEIVE_PROGRAM_CHANGE(firstByte & 0xF, b);
			counter = 0;
			break;
		case 0xD:
			counter = 0;
			break;
		default:
			counter++;
			secondByte = b;
			break;
		}
	}
	return;
	case 2:{
		const uint8_t channel = firstByte & 0xF;
		counter = 0;
		switch (firstByte >> 4) {
		case 0xF:
			break;
		case 0x8:
			//NOTE OFF
			ON_RECEIVE_NOTE_OFF(channel, secondByte, b);
			break;
		case 0x9:
			//NOTE ON
			if (b) {
				ON_RECEIVE_NOTE_ON(channel, secondByte, b);
			} else {
				// 0 Velocity
				ON_RECEIVE_NOTE_OFF(channel, secondByte, 0);
			}
			break;
		case 0xA:
			//PolyphonicKeyPressure
			break;
		case 0xB:
			//ControlChange
			ON_RECEIVE_CONTROL_CHANGE(channel, secondByte, b);
			break;
		case 0xC:
			//ProgramChange
			break;
		case 0xD:
			//ChannelAfterTouch
			break;
		case 0xE:
			//PitchWheelChange
			break;
		}
		return;
	}
	}
}

__attribute__((weak)) void ON_RECEIVE_NOTE_ON(uint8_t ch, uint8_t note,
		uint8_t velocity) {
}
__attribute__((weak)) void ON_RECEIVE_NOTE_OFF(uint8_t ch, uint8_t note,
		uint8_t velocity) {
}
__attribute__((weak)) void ON_RECEIVE_CONTROL_CHANGE(uint8_t ch, uint8_t no,
		uint8_t value) {
}
__attribute__((weak)) void ON_RECEIVE_PROGRAM_CHANGE(uint8_t ch,
		uint8_t program) {
}
__attribute__((weak)) void ON_RECEIVE_CLOCK() {
}
__attribute__((weak)) void ON_RECEIVE_START() {
}
__attribute__((weak)) void ON_RECEIVE_CONTINUE() {
}
__attribute__((weak)) void ON_RECEIVE_STOP() {
}
