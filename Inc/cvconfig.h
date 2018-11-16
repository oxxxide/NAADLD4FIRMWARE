/*
 * cvconfig.h
 *
 *  Created on: 2018/07/14
 *      Author: NishiAsakusa Audio Developments / oxxxide / Akikazu Iwasa
 */

#ifndef CVCONFIG_H_
#define CVCONFIG_H_

#define CV_RESOLUTION 961

extern float cvTable[CV_RESOLUTION];

float cvToExponential(float cv);

#endif /* CVCONFIG_H_ */
