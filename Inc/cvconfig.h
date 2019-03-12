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

extern float cvTable_V_OCT[CV_RESOLUTION];

/**
 * @param cv 0 to 960
 */
float cvToExponential_V_OCT(float cv);

/*
typedef struct{
	float pitchShift; //default:1 min:0 max:nolimit
	float cutoff;
	float modDepth;
	float bendAmt; //default:1 min:0 max:1
	float bendRel;
} CVInputParams;
*/

typedef enum {
	CV_PITCH = 0,
	CV_CUTOFF = 1,
	CV_MOD_DEPTH = 2,
	CV_BEND_AMT = 3,
	CV_BEND_REL = 4
} CV_ASSIGNABLE_PARAM_DEFINITION;

typedef struct {
	CV_ASSIGNABLE_PARAM_DEFINITION assign;
	uint8_t target_channel;
} CV_ASSIGN;

extern CV_ASSIGN cv_assignements[4];

void InitCvAssignements(CV_ASSIGN* pobj,int size);

#endif /* CVCONFIG_H_ */
