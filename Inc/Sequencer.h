/*
 * Sequencer.h
 *
 *  Created on: 2018/11/27
 *      Author: NishiAsakusa Audio Developments / oxxxide / Akikazu Iwasa
 */

#ifndef SEQUENCER_H_
#define SEQUENCER_H_

#include "stm32f4xx.h"
#include "String.h"

typedef enum {
	SEQ_IDLING = 0, SEQ_RUNNING = 1
} SequencerStatus;

typedef struct {
	uint8_t a;
	uint8_t b;
	uint8_t c;
	uint8_t d;
} Notes;

typedef struct {
	int16_t bpm;
	uint8_t step;
	uint8_t clock_cnt;
	uint8_t num_of_steps;
	Notes sequenceData[16];
	SequencerStatus status;
	int8_t cursor_index;
	int cnt_tick;
} Sequencer;

void tickSequencerClock(Sequencer* seq);

void StartSequencer(Sequencer* seq);

void StopSequencer(Sequencer* seq);

void ClockSequencer(Sequencer* seq);

void InitSequencer(Sequencer* seq);

void ChangeBPM(Sequencer* seq, int add);

void SetBPM(Sequencer* seq, int16_t bpm);

void ON_PROGRESS_SEQUENCER_CLOCK();

void ON_START_SEQUENCER();

void ON_STOP_SEQUENCER();

void SEQUENCER_BEAT_CALLBACK(int step);

#endif /* SEQUENCER_H_ */
