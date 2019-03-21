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

uint16_t adcResultA;
uint16_t adcResultB;
uint16_t adcResultC;
uint16_t adcResultD;

extern CV_ASSIGN cv_assignements[4];
//volatile static float cv1 = 1.0f, cv2 = 1.0f, cv3 = 1.0f, cv4 = 1.0f;
CVInputParams CVInputParamsArray[NUM_OF_VOICES];

Gen synth[NUM_OF_VOICES];

static int16_t workingcache[AUDIO_BLOCK_SIZE * 2] = { 0 };
static FORCE_INLINE void audio_process();

static FORCE_INLINE void initCvInputParams(CVInputParams* p) {
	p->pitchShift = 1.0f;
	p->modPitchShift = 1.0f;
	p->modDepth = 1.0f;
	p->cutoff = 0;
	p->bendAmt = 1.0f;
}

void InitSynthesizer() {
	Noise_Init();
	for (int i = 0; i < NUM_OF_VOICES; i++) {
		Gen_init(&synth[i]);
		initCvInputParams(&CVInputParamsArray[i]);
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* adcHandle) {
	adcResultA = adc_buff[0];
	adcResultB = adc_buff[1];
	adcResultC = adc_buff[2];
	adcResultD = adc_buff[3];
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

static FORCE_INLINE void distributeCvInputToParams() {

	CV_ASSIGN* cvass;
	CVInputParams* param;

	initCvInputParams(&CVInputParamsArray[0]);
	initCvInputParams(&CVInputParamsArray[1]);
	initCvInputParams(&CVInputParamsArray[2]);
	initCvInputParams(&CVInputParamsArray[3]);

	uint16_t rawValue;
	for (int i = 0; i < 4; i++) {
		cvass = &cv_assignements[i];
		param = &CVInputParamsArray[cvass->target_channel];

		switch (i) {
		case 0:
			rawValue = adcResultA;
			break;
		case 1:
			rawValue = adcResultB;
			break;
		case 2:
			rawValue = adcResultC;
			break;
		case 3:
			rawValue = adcResultD;
			break;
		default:
			rawValue = 0;
		}

		switch (cvass->assign) {
		case CV_PITCH:
			param->pitchShift = cvToExponential_V_OCT(rawValue / 2500.0f);
			break;
		case CV_CUTOFF:
			param->cutoff = (int)((rawValue / 2500.0f)*90);
			break;
		case CV_MOD_DEPTH:
			param->modDepth = (rawValue / 2500.0f);
			break;
		case CV_MOD_FREQ:
			param->modPitchShift = cvToExponential_V_OCT(rawValue / 2500.0f);
			break;
		case CV_BEND_AMT:
			param->bendAmt = (rawValue / 2500.0f);
			break;
		case CV_AMP_REL:
			AHR_set_cvin(&(synth[cvass->target_channel].eg_amp),
					(rawValue / 2500.0f));
			AHR_set_cvin(&(synth[cvass->target_channel].eg_noise),
					(rawValue / 2500.0f));
			break;
		case CV_OSC_AEG_REL:
			AHR_set_cvin(&(synth[cvass->target_channel].eg_amp),
					(rawValue / 2500.0f));
			break;
		case CV_NOISE_AEG_REL:
			AHR_set_cvin(&(synth[cvass->target_channel].eg_noise),
					(rawValue / 2500.0f));
			break;
		default:
			break;
		}
	}

	//if (HAL_GPIO_ReadPin(GPIO_INPUT_ADCSW1_GPIO_Port, GPIO_INPUT_ADCSW1_Pin)
	//		== GPIO_PIN_RESET) {
	//	cv1 = 1.0f;
	//} else {
	//	CVInputParamsArray[0].pitchShift = cvToExponential_V_OCT(adcResult1 / 2500.0f);
	//}
	//if (HAL_GPIO_ReadPin(GPIO_INPUT_ADCSW2_GPIO_Port, GPIO_INPUT_ADCSW2_Pin)
	//		== GPIO_PIN_RESET) {
	//	cv2 = 1.0f;
	//} else {
	//	CVInputParamsArray[1].pitchShift =  cvToExponential_V_OCT(adcResult2 / 2500.0f);
	//}
	//if (HAL_GPIO_ReadPin(GPIO_INPUT_ADCSW3_GPIO_Port, GPIO_INPUT_ADCSW3_Pin)
	//		== GPIO_PIN_RESET) {
	//	cv3 = 1.0f;
	//} else {
	//	CVInputParamsArray[2].pitchShift = cvToExponential_V_OCT(adcResult3 / 2500.0f);
	//}
	//if (HAL_GPIO_ReadPin(GPIO_INPUT_ADCSW4_GPIO_Port, GPIO_INPUT_ADCSW4_Pin)
	//		== GPIO_PIN_RESET) {
	//	cv4 = 1.0f;
	//} else {
	//	CVInputParamsArray[3].pitchShift = cvToExponential_V_OCT(adcResult4 / 2500.0f);
	//}
}

static FORCE_INLINE float process_main(Gen *gen, CVInputParams *cvps) {
	switch (gen->modtype) {
	case MODTYPE_FM:
		return (Gen_process_fm(gen, cvps));
	case MODTYPE_AM:
		return (Gen_process_ringmod(gen, cvps));
	case MODTYPE_FM_NOISE_ROUTED:
		return (Gen_process_fm_plus_noise(gen, cvps));
	default:
		return 0;
	}
}

static FORCE_INLINE void audio_process(void* dest) {

	float fvalue = 0;
	float sumL = 0;
	float sumR = 0;

	distributeCvInputToParams();

	for (int i = 0; i < AUDIO_BLOCK_SIZE * 2;) {
		sumL = 0;
		sumR = 0;

		fvalue = process_main(&synth[0], &CVInputParamsArray[0]);
		sumL += fvalue * synth[0].cof_pan_l;
		sumR += fvalue * synth[0].cof_pan_r;

		fvalue = process_main(&synth[1], &CVInputParamsArray[1]);
		sumL += fvalue * synth[1].cof_pan_l;
		sumR += fvalue * synth[1].cof_pan_r;

		fvalue = process_main(&synth[2], &CVInputParamsArray[2]);
		sumL += fvalue * synth[2].cof_pan_l;
		sumR += fvalue * synth[2].cof_pan_r;

		fvalue = process_main(&synth[3], &CVInputParamsArray[3]);
		sumL += fvalue * synth[3].cof_pan_l;
		sumR += fvalue * synth[3].cof_pan_r;

		sumL = (sumL > 1.0f) ? 1.0f : (sumL < -1.0f) ? -1.0f : sumL;
		sumR = (sumR > 1.0f) ? 1.0f : (sumR < -1.0f) ? -1.0f : sumR;

		workingcache[i++] = (int16_t) (sumL * (SHRT_MAX - 1)); //L
		workingcache[i++] = (int16_t) (sumR * (SHRT_MAX - 1)); //R
	}

	memcpy(dest, workingcache, (sizeof(int16_t) * AUDIO_BLOCK_SIZE * 2));

}
