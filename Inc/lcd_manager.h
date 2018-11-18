/*
 * lcd_manager.h
 *
 *  Created on: 2018/06/23
 *      Author: NishiAsakusa Audio Developments / oxxxide / Akikazu Iwasa
 */

#ifndef LCD_MANAGER_H_
#define LCD_MANAGER_H_

#include "stm32f4xx_hal.h"
#include "cui.h"

#define PARAM_GROUP_OSC 0
#define PARAM_GROUP_AMP 1
#define PARAM_GROUP_MODU 2
#define PARAM_GROUP_BEND 3
#define PARAM_GROUP_NOISE 4
#define PARAM_GROUP_FILTER 5
#define PARAM_GROUP_LFO 6

#define NUM_OF_GROUP 7
#define NUM_OF_COLUMN 4

#define CLOCK_SOURCE_INTERNAL 1
#define CLOCK_SOURCE_EXTERNAL 2
#define CLOCK_OUT_ENABlE 1
#define CLOCK_OUT_DISABLE 0

//LCD Status
extern uint8_t LcdMenuSelectedItemIndex;
extern volatile uint8_t LcdMenuState;
extern uint8_t SelectedPartNo;
extern uint8_t triggerThreshold;

typedef struct {
	char* text;
	uint32_t length;
	uint32_t dirt;
} LCDBUFF;

extern volatile LCDBUFF lcd_text_buf1;
extern volatile LCDBUFF lcd_text_buf2;

extern const char* LCDM_EDIT_GROUP_TEXT[NUM_OF_GROUP];
extern const char* LCDM_PARAMETER_TEXT[NUM_OF_GROUP][NUM_OF_COLUMN];
extern const char* LCDM_PARAMETER_TEXT_SHIFT[NUM_OF_GROUP][NUM_OF_COLUMN];

void InitLcdManager(void);
void lcdWriteText(int row, const char* str,int length);

#endif /* LCD_MANAGER_H_ */
