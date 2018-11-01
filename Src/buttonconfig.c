/*
 * buttonconfig.c
 *
 *  Created on: 2018/06/26
 *      Author: devox
 */

#include "buttonconfig.h"

static GPIO_PIN_DEF buttons[9];
static GPIO_PIN_DEF pads[4];
static const uint8_t RISE = 0x3;
static const uint8_t DOWN = 0xC;

static void initBtn(GPIO_PIN_DEF* def, GPIO_TypeDef *port, uint16_t pin,
		void (*fnc_a)(void), void (*fnc_b)(void)) {
	def->status = 0;
	def->port = port;
	def->pin = pin;
	def->onpush = fnc_a;
	def->onrelease = fnc_b;
}

GPIO_PinState readPin(int btn) {
	GPIO_PIN_DEF* pin = &buttons[btn];
	return HAL_GPIO_ReadPin(pin->port, pin->pin);
}

void ButtonConfigInit() {

	initBtn(&buttons[0], BTN1_GPIO_Port, BTN1_Pin, &ON_PUSH_C, NULL);
	initBtn(&buttons[1], BTN2_GPIO_Port, BTN2_Pin, &ON_PUSH_D, NULL);
	initBtn(&buttons[2], BTN3_GPIO_Port, BTN3_Pin, &ON_PUSH_A, NULL);
	initBtn(&buttons[3], BTN4_GPIO_Port, BTN4_Pin, &ON_PUSH_B, NULL);
	initBtn(&buttons[4], BTN5_GPIO_Port, BTN5_Pin, &ON_PUSH_PROGRAM, NULL);
	initBtn(&buttons[5], BTN6_GPIO_Port, BTN6_Pin, &ON_PUSH_SHIFT, &ON_RELEASE_SHIFT);
	initBtn(&buttons[6], BTN7_GPIO_Port, BTN7_Pin, &ON_PUSH_EXIT, NULL);
	initBtn(&buttons[7], BTN8_GPIO_Port, BTN8_Pin, &ON_PUSH_ENTER, NULL);
	initBtn(&buttons[8], BTN9_GPIO_Port, BTN9_Pin, &ON_PUSH_MENU, NULL);

	initBtn(&pads[0], GPIO_IN_PAD1_GPIO_Port, GPIO_IN_PAD1_Pin, &ON_PUSH_A, NULL);
	initBtn(&pads[1], GPIO_IN_PAD2_GPIO_Port, GPIO_IN_PAD2_Pin, &ON_PUSH_B, NULL);
	initBtn(&pads[2], GPIO_IN_PAD3_GPIO_Port, GPIO_IN_PAD3_Pin, &ON_PUSH_C, NULL);
	initBtn(&pads[3], GPIO_IN_PAD4_GPIO_Port, GPIO_IN_PAD4_Pin, &ON_PUSH_D, NULL);

}

static void UpdateState(GPIO_PIN_DEF* pin) {
	GPIO_PinState state = HAL_GPIO_ReadPin(pin->port, pin->pin);
	pin->status <<= 1;
	if (state) {
		pin->status |= 0x1;
	}
	if ((pin->status & 0xF) == RISE) {
		pin->onpush();
	} else if ((pin->status & 0xF) == DOWN && pin->onrelease != NULL) {
		pin->onrelease();
	}
}

static void checkPadState(GPIO_PIN_DEF* pin){
	GPIO_PinState state = HAL_GPIO_ReadPin(pin->port, pin->pin);
	pin->status <<= 1;
	if (state) {
			pin->status |= 0x1;
	}
	if ((pin->status & 0xFF) == DOWN) {
		pin->onpush();
	}

}

void CheckGPIO_Pins() {
	UpdateState(&buttons[0]);
	UpdateState(&buttons[1]);
	UpdateState(&buttons[2]);
	UpdateState(&buttons[3]);
	UpdateState(&buttons[4]);
	UpdateState(&buttons[5]);
	UpdateState(&buttons[6]);
	UpdateState(&buttons[7]);
	UpdateState(&buttons[8]);

	//checkPadState(&pads[0]);
	//checkPadState(&pads[1]);
	//checkPadState(&pads[2]);
	//checkPadState(&pads[3]);
}
