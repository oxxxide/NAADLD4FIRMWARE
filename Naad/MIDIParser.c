/*
 * MIDIParser.c
 *
 *  Created on: 2018/09/26
 *      Author: devox
 */

#include "MIDIParser.h"

static const int BUFF_MASK = MIDI_MESSAGE_BUFF_SIZE - 1;

void MIDI_Message_Buffer_Init(MIDI_Message_Buffer* instance) {
	instance->write_cursor = 0;
	instance->buff_count = 0;
}

void MIDI_Message_Push(MIDI_Message_Buffer* instance, uint8_t byte) {
	instance->buff[instance->write_cursor] = byte;
	instance->write_cursor = (instance->write_cursor + 1) & BUFF_MASK;
	instance->buff_count++;
}

uint8_t MIDI_Message_Read(MIDI_Message_Buffer* instance, int history) {
	return instance->buff[(instance->write_cursor - history) & BUFF_MASK];
}

void MIDI_Message_Clear_Buff(MIDI_Message_Buffer* instance) {
	instance->buff_count = 0;
}

void MIDI_Message_Parse(MIDI_Message_Buffer* buff) {

	uint8_t byte_1 = MIDI_Message_Read(buff, 0);

	if (byte_1 >> 4 == 0xF) {
		//System Message
		switch (byte_1) {
		case 0xFE:
			//Active Sensing
			break;
		case 0xFF:
			break;
			//Reset
		case 0xF8:
			ON_RECEIVE_CLOCK();
			break;
		case 0xFA:
			ON_RECEIVE_START();
			break;
		case 0xFB:
			ON_RECEIVE_CONTINUE();
			break;
		case 0xFC:
			ON_RECEIVE_STOP();
			break;
		}
		MIDI_Message_Clear_Buff(buff);
		return;
	}

	switch (buff->buff_count) {
	case 1:
		//ignore
		MIDI_Message_Clear_Buff(buff);
		break;
	case 2: {

		//2bytes Message

		uint8_t byte_2 = MIDI_Message_Read(buff, 1);
		uint8_t type = byte_2 >> 4;
		uint8_t ch = byte_2 & 0xF;
		switch (type) {

		case 0xC:
			//ProgramChange
			ON_RECEIVE_PROGRAM_CHANGE(ch,byte_1);
			MIDI_Message_Clear_Buff(buff);
			break;

		}
	}
		break;
	case 3:
	{
		//3bytes Message

		uint8_t byte_3 = MIDI_Message_Read(buff, 2);
		uint8_t type = byte_3 >> 4;
		uint8_t ch = byte_3 & 0xF;

		uint8_t byte_2 = MIDI_Message_Read(buff, 1);

		switch (type) {
				case 0x8:
					//NOTE_OFF
					ON_RECEIVE_NOTE_OFF(ch, byte_2, byte_1);
					MIDI_Message_Clear_Buff(buff);
					break;
				case 0x9:
					//NOTE_ON
					if(byte_1){
						ON_RECEIVE_NOTE_ON(ch, byte_2, byte_1);
					}else{
						ON_RECEIVE_NOTE_OFF(ch, byte_2, byte_1);
					}
					MIDI_Message_Clear_Buff(buff);
					break;
				case 0xA:
					//Pressure
					// do nothing
					MIDI_Message_Clear_Buff(buff);
					break;
				case 0xB:
					//ControlChange
					ON_RECEIVE_CONTROL_CHANGE(ch, byte_2, byte_1);
					MIDI_Message_Clear_Buff(buff);
					break;
				case 0xE:
					//PitchBend
					// do nothing
					MIDI_Message_Clear_Buff(buff);
					break;
				}

		break;
	}

	}

}

__attribute__((weak)) void ON_RECEIVE_NOTE_ON(uint8_t ch,uint8_t note,uint8_t velocity){}
__attribute__((weak)) void ON_RECEIVE_NOTE_OFF(uint8_t ch,uint8_t note,uint8_t velocity){}
__attribute__((weak)) void ON_RECEIVE_CONTROL_CHANGE(uint8_t ch,uint8_t no,uint8_t value){}
__attribute__((weak)) void ON_RECEIVE_PROGRAM_CHANGE(uint8_t ch,uint8_t program){}
__attribute__((weak)) void ON_RECEIVE_CLOCK(){}
__attribute__((weak)) void ON_RECEIVE_START(){}
__attribute__((weak)) void ON_RECEIVE_CONTINUE(){}
__attribute__((weak)) void ON_RECEIVE_STOP(){}
