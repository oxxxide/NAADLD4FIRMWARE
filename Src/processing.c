/*
 * processing.c
 *
 *  Created on: 2018/06/16
 *      Author: NishiAsakusa Audio Developments / oxxxide / Akikazu Iwasa
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

//Oscill osc1;

Gen synth[NUM_OF_VOICES];

static int16_t workingcache[AUDIO_BLOCK_SIZE * 2] = { 0 };
static FORCE_INLINE void audio_process();

void InitSynthesizer() {
	Noise_Init();
	//Osc_Init(&osc1);
	//Osc_set_pitch(&osc1, 60);
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

static FORCE_INLINE void calcCvValue() {
	if (HAL_GPIO_ReadPin(GPIO_INPUT_ADCSW1_GPIO_Port, GPIO_INPUT_ADCSW1_Pin)
			== GPIO_PIN_RESET) {
		cv1 = 1.0f;
	} else {
		cv1 = cvToExponential(adcResult1 / 2500.0f);
	}
	if (HAL_GPIO_ReadPin(GPIO_INPUT_ADCSW2_GPIO_Port, GPIO_INPUT_ADCSW2_Pin)
			== GPIO_PIN_RESET) {
		cv2 = 1.0f;
	} else {
		cv2 = cvToExponential(adcResult2 / 2500.0f);
	}
	if (HAL_GPIO_ReadPin(GPIO_INPUT_ADCSW3_GPIO_Port, GPIO_INPUT_ADCSW3_Pin)
			== GPIO_PIN_RESET) {
		cv3 = 1.0f;
	} else {
		cv3 = cvToExponential(adcResult3 / 2500.0f);
	}
	if (HAL_GPIO_ReadPin(GPIO_INPUT_ADCSW4_GPIO_Port, GPIO_INPUT_ADCSW4_Pin)
			== GPIO_PIN_RESET) {
		cv4 = 1.0f;
	} else {
		cv4 = cvToExponential(adcResult4 / 2500.0f);
	}
}

static FORCE_INLINE float process_main(Gen *gen, float cv) {
	switch (gen->modtype) {
	case MODTYPE_FM:
		return (Gen_process_fm(gen, USE_CV_IN ? cv : 1.0f));
	case MODTYPE_AM:
		return (Gen_process_ringmod(gen, USE_CV_IN ? cv : 1.0f));
	case MODTYPE_FM_NOISE_ROUTED:
		return (Gen_process_fm_plus_noise(gen, USE_CV_IN ? cv : 1.0f));
	default:
		return 0;
	}
}

static FORCE_INLINE void audio_process(void* dest) {

	float fvalue = 0;
	float sumL = 0;
	float sumR = 0;

	calcCvValue();

	for (int i = 0; i < AUDIO_BLOCK_SIZE * 2;) {
		sumL = 0;
		sumR = 0;

		fvalue = process_main(&synth[0], cv1);
		sumL += fvalue * synth[0].cof_pan_l;
		sumR += fvalue * synth[0].cof_pan_r;

		fvalue = process_main(&synth[1], cv2);
		sumL += fvalue * synth[1].cof_pan_l;
		sumR += fvalue * synth[1].cof_pan_r;

		fvalue = process_main(&synth[2], cv3);
		sumL += fvalue * synth[2].cof_pan_l;
		sumR += fvalue * synth[2].cof_pan_r;

		fvalue = process_main(&synth[3], cv4);
		sumL += fvalue * synth[3].cof_pan_l;
		sumR += fvalue * synth[3].cof_pan_r;

		sumL = (sumL > 1.0f) ? 1.0f : (sumL < -1.0f) ? -1.0f : sumL;
		sumR = (sumR > 1.0f) ? 1.0f : (sumR < -1.0f) ? -1.0f : sumR;

		workingcache[i++] = (int16_t) (sumL * (SHRT_MAX - 1)); //L
		workingcache[i++] = (int16_t) (sumR * (SHRT_MAX - 1)); //R
	}

	memcpy(dest, workingcache, (sizeof(int16_t) * AUDIO_BLOCK_SIZE * 2));

}
