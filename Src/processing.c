/*
 * processing.c
 *
 *  Created on: 2018/06/16
 *      Author: devox
 */

#include "stm32f4xx_hal.h"
#include "processing.h"
#include "FastMath_.h"
#include "arm_math.h"
#include "math.h"
#include "limits.h"
#include "Oscill.h"
#include "Gen.h"
#include "cvconfig.h"

uint16_t circularbuffer[AUDIO_BLOCK_SIZE * 2 * 2] = { 0 };
uint16_t adc_buff[ADC_SAMPLE_LENGTH] = { 0 };

uint16_t* buf1 = &circularbuffer[0];
uint16_t* buf2 = &circularbuffer[AUDIO_BLOCK_SIZE * 2];

uint16_t adcResult1;
uint16_t adcResult2;
uint16_t adcResult3;
uint16_t adcResult4;

Oscill osc1;

Gen synth[NUM_OF_VOICES];

static int16_t workingcache[AUDIO_BLOCK_SIZE * 2] = { 0 };
static FORCE_INLINE void audio_process();

void InitSynthesizer() {
	Noise_Init();
	Osc_Init(&osc1);
	Osc_set_pitch(&osc1, 60);
	for (int i = 0; i < NUM_OF_VOICES; i++) {
		Gen_init(&synth[i]);
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* adcHandle) {
	adcResult1 = adc_buff[0];
	adcResult2 = adc_buff[1];
	adcResult3 = adc_buff[2];
	adcResult4 = adc_buff[3];
}

void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
	if (hi2s->hdmatx == &hdma_spi3_tx) {
		audio_process(buf1);
	}
}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s) {
	if (hi2s->hdmatx == &hdma_spi3_tx) {
		audio_process(buf2);
	}
}

static FORCE_INLINE void audio_process(void* dest) {

	float cv1 = cvToExponential(adcResult1 / 3102.0f);
	float cv2 = cvToExponential(adcResult2 / 3102.0f);
	float cv3 = cvToExponential(adcResult3 / 3102.0f);
	float cv4 = cvToExponential(adcResult4 / 3102.0f);


	float fvalue = 0;
	float sumL = 0;
	float sumR = 0;

	for (int i = 0; i < AUDIO_BLOCK_SIZE * 2;) {
		sumL = 0;
		sumR = 0;

		if(synth[0].modtype==MODTYPE_FM){
			fvalue = (Gen_process_fm(&synth[0], USE_CV_IN ? cv1 : 1.0f));
		}else{
			fvalue = (Gen_process_ringmod(&synth[0], USE_CV_IN ? cv1 : 1.0f));
		}
		sumL += fvalue*synth[0].cof_pan_l;
		sumR += fvalue*synth[0].cof_pan_r;

		if (synth[1].modtype == MODTYPE_FM) {
			fvalue = (Gen_process_fm(&synth[1], USE_CV_IN ? cv2 : 1.0f));
		} else {
			fvalue = (Gen_process_ringmod(&synth[1], USE_CV_IN ? cv2 : 1.0f));
		}
		sumL += fvalue*synth[1].cof_pan_l;
		sumR += fvalue*synth[1].cof_pan_r;

		if (synth[2].modtype == MODTYPE_FM) {
			fvalue = (Gen_process_fm(&synth[2], USE_CV_IN ? cv3 : 1.0f));
		} else {
			fvalue = (Gen_process_ringmod(&synth[2], USE_CV_IN ? cv3 : 1.0f));
		}
		sumL += fvalue*synth[2].cof_pan_l;
		sumR += fvalue*synth[2].cof_pan_r;

		if (synth[3].modtype == MODTYPE_FM) {
			fvalue = (Gen_process_fm(&synth[3], USE_CV_IN ? cv4 : 1.0f));
		} else {
			fvalue = (Gen_process_ringmod(&synth[3], USE_CV_IN ? cv4 : 1.0f));
		}
		sumL += fvalue*synth[3].cof_pan_l;
		sumR += fvalue*synth[3].cof_pan_r;

		sumL = (sumL > 1.0f) ? 1.0f : (sumL < -1.0f) ? -1.0f : sumL;
		sumR = (sumR > 1.0f) ? 1.0f : (sumR < -1.0f) ? -1.0f : sumR;

		workingcache[i++] = (int16_t) (sumL * (SHRT_MAX - 1)); //L
		workingcache[i++] = (int16_t) (sumR * (SHRT_MAX - 1)); //R
	}

	memcpy(dest, workingcache, (sizeof(int16_t) * AUDIO_BLOCK_SIZE * 2));

}
