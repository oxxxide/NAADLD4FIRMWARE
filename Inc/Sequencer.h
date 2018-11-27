/*
 * Sequencer.h
 *
 *  Created on: 2018/11/27
 *      Author: NishiAsakusa Audio Developments / oxxxide / Akikazu Iwasa
 */

#ifndef SEQUENCER_H_
#define SEQUENCER_H_

#include "stm32f4xx.h"

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
	uint8_t bpm;
	uint8_t step;
	Notes SequenceData[16];
	SequencerStatus status;
} Sequencer;

void StartSequencer(Sequencer* seq);

void StopSequencer(Sequencer* seq);

void TickSequencer(Sequencer* seq);

void OnStep(int step);

#endif /* SEQUENCER_H_ */
