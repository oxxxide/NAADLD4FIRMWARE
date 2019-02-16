/*
 * cvconfig.h
 *
 *  Created on: 2018/07/14
 *      Author: NishiAsakusa Audio Developments / oxxxide / Akikazu Iwasa
 */

#ifndef CVCONFIG_H_
#define CVCONFIG_H_
#include "stm32f4xx_hal.h"
#define CV_RESOLUTION 961

extern float cvTable[CV_RESOLUTION];

float cvToExponential(float cv);


typedef enum {
	CV_PITCH = 1,
	CV_MOD_DEPTH = 2,
	CV_MOD_FREQ = 3,
	CV_CUTOFF = 4,
	CV_AEG_REL = 5
} CV_ASSIGNABLE;

typedef struct {
	CV_ASSIGNABLE assign;
	uint8_t channel;
} CV_ASSIGN;

CV_ASSIGN cv_assignes[4];

#endif /* CVCONFIG_H_ */
