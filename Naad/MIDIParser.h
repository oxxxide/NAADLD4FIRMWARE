/*
 * MIDIParser.h
 *
 *  Created on: 2018/09/26
 *      Author: NishiAsakusa Audio Developments / oxxxide / Akikazu Iwasa
 */

#ifndef MIDIPARSER_H_
#define MIDIPARSER_H_

#include "stm32f4xx_hal.h"

void MidiParser_PushByte(uint8_t byte);

void ON_RECEIVE_NOTE_ON(uint8_t ch,uint8_t note,uint8_t velocity);
void ON_RECEIVE_NOTE_OFF(uint8_t ch,uint8_t note,uint8_t velocity);
void ON_RECEIVE_CONTROL_CHANGE(uint8_t ch,uint8_t no,uint8_t value);
void ON_RECEIVE_PROGRAM_CHANGE(uint8_t ch,uint8_t program);
void ON_RECEIVE_CLOCK();
void ON_RECEIVE_START();
void ON_RECEIVE_CONTINUE();
void ON_RECEIVE_STOP();

#endif /* MIDIPARSER_H_ */
