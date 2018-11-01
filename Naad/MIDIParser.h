/*
 * MIDIParser.h
 *
 *  Created on: 2018/09/26
 *      Author: devox
 */

#ifndef MIDIPARSER_H_
#define MIDIPARSER_H_

#include "stm32f4xx_hal.h"

#define MIDI_MESSAGE_BUFF_SIZE 32

typedef struct {
	uint8_t buff[MIDI_MESSAGE_BUFF_SIZE];
	volatile int write_cursor;
	volatile int buff_count;
} MIDI_Message_Buffer;

void MIDI_Message_Buffer_Init(MIDI_Message_Buffer* instance);

void MIDI_Message_Push(MIDI_Message_Buffer* instance, uint8_t byte);

uint8_t MIDI_Message_Read(MIDI_Message_Buffer* instance, int history);

void MIDI_Message_Clear_Buff(MIDI_Message_Buffer* instance);


void ON_RECEIVE_NOTE_ON(uint8_t ch,uint8_t note,uint8_t velocity);
void ON_RECEIVE_NOTE_OFF(uint8_t ch,uint8_t note,uint8_t velocity);
void ON_RECEIVE_CONTROL_CHANGE(uint8_t ch,uint8_t no,uint8_t value);
void ON_RECEIVE_PROGRAM_CHANGE(uint8_t ch,uint8_t program);
void ON_RECEIVE_CLOCK();
void ON_RECEIVE_START();
void ON_RECEIVE_CONTINUE();
void ON_RECEIVE_STOP();

#endif /* MIDIPARSER_H_ */
