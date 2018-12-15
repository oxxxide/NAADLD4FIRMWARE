/*
 * Sequencer.c
 *
 *  Created on: 2018/11/27
 *      Author: NishiAsakusa Audio Developments / oxxxide / Akikazu Iwasa
 */

#include "Sequencer.h"

#define TICK_PER_STEP 6

void StartSequencer(Sequencer* seq){
	seq->clock_cnt = 0;
	seq->step[0] = 0;
	seq->step[1] = 0;
	seq->step[2] = 0;
	seq->step[3] = 0;
	seq->status = SEQ_RUNNING;
	ON_START_SEQUENCER();
}

void StopSequencer(Sequencer* seq){
	seq->status = SEQ_IDLING;
	seq->clock_cnt = 0;
	seq->step[0] = 0;
	seq->step[1] = 0;
	seq->step[2] = 0;
	seq->step[3] = 0;
	ON_STOP_SEQUENCER();
}

void ChangeBPM(Sequencer* seq, int add) {
	int tmp = seq->bpm + add;
	seq->bpm = (int16_t)LIMIT(tmp, 300, 20);
	TIM13->ARR = (uint32_t)(6000000.0f / (float)seq->bpm / 24.0f + 0.5f ) ;
}

void SetBPM(Sequencer* seq, int16_t bpm) {
	seq->bpm = (int16_t)LIMIT(bpm, 300, 20);
	TIM13->ARR = (uint32_t)(6000000.0f / (float)seq->bpm / 24.0f + 0.5f ) ;
}

void ClockSequencer(Sequencer* seq) {
	ON_PROGRESS_SEQUENCER_CLOCK();
	if (seq->status == SEQ_RUNNING && seq->clock_cnt <= 0) {
		SEQUENCER_BEAT_CALLBACK(seq->step);
		for(int i=0;i<4;i++){
			seq->step[i]++;
			seq->clock_cnt = TICK_PER_STEP;
			if (seq->step[i] >= seq->step_length_array[i]) {
				seq->step[i] = 0;
			}
		}

	}
	seq->clock_cnt--;
}

void InitSequencer(Sequencer* seq){
	seq->clock_cnt = 0;
	seq->step[0] = 0;
	seq->step[1] = 0;
	seq->step[2] = 0;
	seq->step[3] = 0;
	seq->step_length_array[0] = 16;
	seq->step_length_array[1] = 16;
	seq->step_length_array[2] = 16;
	seq->step_length_array[3] = 16;
	seq->bpm = 120;
	seq->status = SEQ_IDLING;
	seq->cursor_index = 0;
	memset( seq->sequenceData, 0,  (16 * sizeof(Notes)));
	SetBPM(seq, 120);
}

__attribute__((weak)) void ON_PROGRESS_SEQUENCER_CLOCK() {
}

__attribute__((weak)) void ON_START_SEQUENCER() {
}

__attribute__((weak)) void ON_STOP_SEQUENCER() {
}

__attribute__((weak)) void SEQUENCER_BEAT_CALLBACK(uint8_t *step){
}

