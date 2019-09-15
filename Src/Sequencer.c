/*
 * Sequencer.c
 *
 *  Created on: 2018/11/27
 *      Author: NishiAsakusa Audio Developments / oxxxide / Akikazu Iwasa
 */

#include "Sequencer.h"
#include "Noise.h"
#include "processing.h"
#include "Gen.h"

#define TICK_PER_STEP 6

extern Gen synth[NUM_OF_VOICES];

void StartSequencer(Sequencer88* seq){
	seq->clock_cnt = 0;
	seq->step[0] = 0;
	seq->step[1] = 0;
	seq->step[2] = 0;
	seq->step[3] = 0;
	seq->status = SEQ_RUNNING;
	ON_START_SEQUENCER();
}

void StopSequencer(Sequencer88* seq){
	seq->status = SEQ_IDLING;
	seq->clock_cnt = 0;
	seq->step[0] = 0;
	seq->step[1] = 0;
	seq->step[2] = 0;
	seq->step[3] = 0;
	ON_STOP_SEQUENCER();
}

void ChangeBPM(Sequencer88* seq, int add) {
	int tmp = seq->bpm + add;
	seq->bpm = (int16_t)LIMIT(tmp, 300, 20);
	TIM13->ARR = (uint32_t)(6000000.0f / (float)seq->bpm / 24.0f + 0.5f ) ;
}

void SetBPM(Sequencer88* seq, int16_t bpm) {
	seq->bpm = (int16_t)LIMIT(bpm, 300, 20);
	TIM13->ARR = (uint32_t)(6000000.0f / (float)seq->bpm / 24.0f + 0.5f ) ;
}

void ClockSequencer(Sequencer88* seq) {
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

	tickPlayFx(seq,0);
	tickPlayFx(seq,1);
	tickPlayFx(seq,2);
	tickPlayFx(seq,3);

	seq->clock_cnt--;
}

void InitSequencer(Sequencer88* seq){
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
	seq->playFxEnabled = 0;
	memset( seq->sequenceData, 0,  (16 * sizeof(Notes)));
	SetBPM(seq, 120);

	for(int i=0;i<4;i++){
		seq->playfx[i].chance = 0;
		seq->playfx[i].mute = 0;
		seq->playfx[i].decay = 0;
		seq->playfx[i].grain = 0;
		seq->playfx[i].hold_time = 0;

		seq->pfx_status[i].decay_cof = 0;
		seq->pfx_status[i].gate = 0;
		seq->pfx_status[i].rpt_clk = 3;
		seq->pfx_status[i].rpt_cnt = 3;
		seq->pfx_status[i].total_cnt = 12;
		seq->pfx_status[i].total_clk = 12;
	}
}

void OnBeatRdmzer(Sequencer88* seq, int index){

	if(!seq->playFxEnabled){
		return;
	}

	PlayFxStatus* status = &seq->pfx_status[index];

	if(status->gate == 0 ){


		PlayFx* param = &seq->playfx[index];

		uint32_t cnc = Noise_Generate_uint() & 7;
		if(cnc < param->chance){

			status->gate = 1;

			uint32_t rdm_totalLength = Noise_Generate_uint() & 7;
			if (rdm_totalLength >= 7) {
				status->total_cnt = 12;
				status->total_clk = 12;
			} else if (rdm_totalLength >= 5) {
				status->total_cnt = 12;
				status->total_clk = 12;
			} else if (rdm_totalLength >= 4) {
				status->total_cnt = 9;
				status->total_clk = 9;
			} else {
				status->total_cnt = 6;
				status->total_clk = 6;
			}

			uint32_t rdm_grainLength = Noise_Generate_uint() & 7;
			if(rdm_grainLength >= 7){
				status->rpt_clk = 4;
				status->rpt_cnt = 4;
			}else if(rdm_grainLength >= 5){
				status->rpt_clk = 3;
				status->rpt_cnt = 3;
			}else if(rdm_grainLength >= 1){
				status->rpt_clk = 2;
				status->rpt_cnt = 2;
			}else{
				status->rpt_clk = 1;
				status->rpt_cnt = 1;
			}

		}

	}
}

void tickPlayFx(Sequencer88* seq, int index) {
	PlayFxStatus* status = &seq->pfx_status[index];
	if (status->gate > 0) {
		status->total_cnt--;
		status->rpt_cnt--;

		if(status->rpt_cnt == 0){
			Gen_trig(&synth[index],1.0f);
			status->rpt_cnt = status->rpt_clk;
		}

		if (status->total_cnt == 0) {
			status->gate = 0;
		}
	}
}

__attribute__((weak)) void ON_PROGRESS_SEQUENCER_CLOCK() {
}

__attribute__((weak)) void ON_START_SEQUENCER() {
}

__attribute__((weak)) void ON_STOP_SEQUENCER() {
}

__attribute__((weak)) void SEQUENCER_BEAT_CALLBACK(uint8_t *step){
}

