/*
 * interruptsconfig.h
 *
 *  Created on: 2018/05/09
 *      Author: devox
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

extern void ON_PUSH_MENU(void);
extern void ON_PUSH_EXIT(void);
extern void ON_PUSH_ENTER(void);
extern void ON_PUSH_PROGRAM(void);
extern void ON_PUSH_SHIFT(void);
extern void ON_RELEASE_SHIFT(void);
extern void ON_PUSH_A(void);
extern void ON_PUSH_B(void);
extern void ON_PUSH_C(void);
extern void ON_PUSH_D(void);

#endif /* BUTTONCONFIG_H_ */
