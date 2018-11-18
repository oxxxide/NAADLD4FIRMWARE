/*
 * interruptsconfig.h
 *
 *  Created on: 2018/05/09
 *      Author: NishiAsakusa Audio Developments / oxxxide / Akikazu Iwasa
 */

#ifndef BUTTONCONFIG_H_
#define BUTTONCONFIG_H_

#include "stm32f4xx.h"

#define BTN_INDEX_SHIFT 4

typedef struct __gpio__ {
	volatile uint8_t status;
	GPIO_TypeDef* port;
	uint16_t pin;
	void (*onpush)(void);
	void (*onrelease)(void);
} GPIO_PIN_DEF;

void ButtonConfigInit(void);
GPIO_PinState readPin(int btn);

void ON_PUSH_MENU(void);
void ON_PUSH_EXIT(void);
void ON_PUSH_ENTER(void);
void ON_PUSH_PROGRAM(void);
void ON_PUSH_SHIFT(void);
void ON_RELEASE_SHIFT(void);
void ON_PUSH_A(void);
void ON_PUSH_B(void);
void ON_PUSH_C(void);
void ON_PUSH_D(void);

#endif /* BUTTONCONFIG_H_ */
