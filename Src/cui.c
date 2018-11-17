/*
 * cui.c
 *
 *  Created on: 2018/06/28
 *      Author: NishiAsakusa Audio Developments / oxxxide / Akikazu Iwasa
 */

#include "cui.h"
#include "MidiConfig.h"
#include "lcd_manager.h"
#include "main.h"
#include "tone.h"

static char displayChannel = 'A';

const char* PresetTones[16] = {
		"Kick1",
		"Kick2",
		"Kick3",
		"Kick4",
		"Synth1",
		"Synth2",
		"Synth3",
		"Synth4",
		"Hat1",
		"Hat2",
		"Hat3",
		"Hat4",
		"Tam1",
		"Tam2",
		"Tam3",
		"Tam4"
};

uint8_t LcdMenuSelectedItemIndex = 0;
int ProgramMenuSelectedItemIndex = 0;

volatile uint8_t LcdMenuState = LCD_STATE_DEFAULT;
uint8_t is_pressed_key_SHIFT = 0;



MidiSyncConfig midiSyncConfig = {CLOCK_SOURCE_INTERNAL,CLOCK_OUT_DISABLE,120};

uint8_t triggerThreshold = 0;

void SelectMenu(int add) {
	int tmp = LcdMenuSelectedItemIndex;
	if (add > 0) {
		tmp++;
	} else if (add < 0) {
		tmp--;
	}

	if (tmp < 0) {
		tmp = 5;
	} else if (tmp > 5) {
		tmp = 0;
	}

	LcdMenuSelectedItemIndex = tmp;

	switch (LcdMenuSelectedItemIndex) {
	case 0:
		lcdWriteText(0,"~SEQ    SYNC    ",16);
		lcdWriteText(1," MIDI   VELC    ",16);
		break;
	case 1:
		lcdWriteText(0," SEQ   ~SYNC    ",16);
		lcdWriteText(1," MIDI   VELC    ",16);
		break;
	case 2:
		lcdWriteText(0," SEQ    SYNC    ",16);
		lcdWriteText(1,"~MIDI   VELC    ",16);
		break;
	case 3:
		lcdWriteText(0," SEQ    SYNC    ",16);
		lcdWriteText(1," MIDI  ~VELC    ",16);
		break;
	case 4:
		lcdWriteText(0,"~CV Monitor     ",16);
		lcdWriteText(1," Factory Reset  ",16);
		break;
	case 5:
		lcdWriteText(0," CV Monitor     ",16);
		lcdWriteText(1,"~Factory Reset  ",16);
		break;
	}
}

void MIDIConfig_Show(MidiConfig* midiConfig) {
	LcdMenuState = LCD_STATE_MIDI_RECEIVE_CONFIG;
	char buff1[17];
	char buff2[17];
	uint8_t ch = 0;
	uint8_t nn = 0;
	switch(displayChannel){
	case 'A':
		ch = midiConfig->channel_A;
		nn = midiConfig->noteNo_A;
		break;
	case 'B':
		ch = midiConfig->channel_B;
		nn = midiConfig->noteNo_B;
		break;
	case 'C':
		ch = midiConfig->channel_C;
		nn = midiConfig->noteNo_C;
		break;
	case 'D':
		ch = midiConfig->channel_D;
		nn = midiConfig->noteNo_D;
		break;
	}
	sprintf(buff1, "Ch.%c MidiCh Note", displayChannel);
	sprintf(buff2, "         %2d  %3d", ch + 1, nn);
	lcdWriteText(0, buff1, 16);
	lcdWriteText(1, buff2, 16);

}

void MIDIConfig_ChangeNt(MidiConfig* midiConfig, int add) {
	switch (displayChannel) {
	case 'A':
		midiConfig->noteNo_A = LIMIT(midiConfig->noteNo_A + add, 127, 0);
		break;
	case 'B':
		midiConfig->noteNo_B = LIMIT(midiConfig->noteNo_B + add, 127, 0);
		break;
	case 'C':
		midiConfig->noteNo_C = LIMIT(midiConfig->noteNo_C + add, 127, 0);
		break;
	case 'D':
		midiConfig->noteNo_D = LIMIT(midiConfig->noteNo_D + add, 127, 0);
		break;
	}
	MIDIConfig_Show(midiConfig);
}

void MIDIConfig_ChangeCh(MidiConfig* midiConfig, int add) {
	switch (displayChannel) {
	case 'A':
		midiConfig->channel_A = LIMIT(midiConfig->channel_A + add, 0xF, 0);
		break;
	case 'B':
		midiConfig->channel_B = LIMIT(midiConfig->channel_B + add, 0xF, 0);
		break;
	case 'C':
		midiConfig->channel_C = LIMIT(midiConfig->channel_C + add, 0xF, 0);
		break;
	case 'D':
		midiConfig->channel_D = LIMIT(midiConfig->channel_D + add, 0xF, 0);
		break;
	}
	MIDIConfig_Show(midiConfig);
}

void MIDIConfig_DisplayChannel(MidiConfig* midiConfig, int add) {
	char ch = displayChannel + add;
	if (ch < 'A') {
		ch = 'D';
	} else if (ch > 'D') {
		ch = 'A';
	}
	displayChannel = ch;
	MIDIConfig_Show(midiConfig);
}

void MIDIConfig_velocity_curve(MidiConfig* midiConfig, int add) {
	LcdMenuState = LCD_STATE_VELC;
	int v = midiConfig->velocityCurve + add;
	midiConfig->velocityCurve = (VelocityCurve) LIMIT(v, Fixed, Exponential);
	lcdWriteText(0, "Velocity Curve  ", 16);
	switch (midiConfig->velocityCurve) {
	case Exponential:
		lcdWriteText(1, "~Exponential    ", 16);
		break;
	case Linear:
		lcdWriteText(1, "~Linear         ", 16);
		break;
	case Fixed:
		lcdWriteText(1, "~Fixed          ", 16);
		break;
	}
}

void ShowProgramMenu(int add) {

	LcdMenuState = LCD_STATE_PROGRAM_MENU;

	ProgramMenuSelectedItemIndex += add;
	if (ProgramMenuSelectedItemIndex > 2) {
		ProgramMenuSelectedItemIndex = 0;
	} else if (ProgramMenuSelectedItemIndex < 0) {
		ProgramMenuSelectedItemIndex = 2;
	}

	switch (ProgramMenuSelectedItemIndex) {
	case 0:
		lcdWriteText(0, "~Temporary save ", 16);
		lcdWriteText(1, " Store Program  ", 16);
		break;
	case 1:
		lcdWriteText(0, " Temporary save ", 16);
		lcdWriteText(1, "~Store Program  ", 16);
		break;
	case 2:
		lcdWriteText(0, "~Load  Program  ", 16);
		lcdWriteText(1, "                ", 16);
		break;
	}

}

void SyncConfig_Show(){

	char* value = NULL;
	switch(midiSyncConfig.clockSource){
	case CLOCK_SOURCE_INTERNAL:
		value = MENU_SYNC_TEXT_Internal;
		break;
	case CLOCK_SOURCE_EXTERNAL:
		value = MENU_SYNC_TEXT_External;
		break;
	}

	lcdWriteText(0,MENU_SYNC_TEXT_1,16);
	lcdWriteText(1,value,16);
}

void SyncConfig_ChangeSync(){
	if(midiSyncConfig.clockSource == CLOCK_SOURCE_INTERNAL){
		midiSyncConfig.clockSource = CLOCK_SOURCE_EXTERNAL;
	}else{
		midiSyncConfig.clockSource = CLOCK_SOURCE_INTERNAL;
	}
	SyncConfig_Show();
}

void TriggerConfig_Show() {
	char p[16];
	sprintf(p, MENU_TRIG_TEXT_2, triggerThreshold);
	lcdWriteText(0, MENU_TRIG_TEXT_1, 16);
	lcdWriteText(1, p, 16);
}

void TriggerConfig_Change(int add) {
	int i = triggerThreshold + (add > 0 ? 1 : -1);
	triggerThreshold = (uint8_t) (i % 127);
	TriggerConfig_Show();
}


void apply(Gen* s, uint8_t pSetNo) {
	ToneCopyToGen(s, &tones[pSetNo]);
}

void ConfirmFactoryReset() {
	lcdWriteText(0, "Do FactoryReset?", 16);
	lcdWriteText(1, "N:EXIT  Y:ENTER ", 16);
}

void CV_Monitor_Show(){

	static const char* fmtstr1 = "A:%3.2f B:%3.2f   ";
	static const char* fmtstr2 = "C:%3.2f D:%3.2f   ";

	static char str1[17] = {'\0'};
	static char str2[17] = {'\0'};



	if (HAL_GPIO_ReadPin(GPIO_INPUT_ADCSW1_GPIO_Port, GPIO_INPUT_ADCSW1_Pin)
			== GPIO_PIN_RESET) {
		cv1 = 0;
	}
	if (HAL_GPIO_ReadPin(GPIO_INPUT_ADCSW2_GPIO_Port, GPIO_INPUT_ADCSW2_Pin)
			== GPIO_PIN_RESET) {
		cv2 = 0;
	}
	if (HAL_GPIO_ReadPin(GPIO_INPUT_ADCSW3_GPIO_Port, GPIO_INPUT_ADCSW3_Pin)
			== GPIO_PIN_RESET) {
		cv3 = 0;
	}
	if (HAL_GPIO_ReadPin(GPIO_INPUT_ADCSW4_GPIO_Port, GPIO_INPUT_ADCSW4_Pin)
			== GPIO_PIN_RESET) {
		cv4 = 0;
	}

	sprintf(str1,fmtstr1,cv1,cv2);
	sprintf(str2,fmtstr2,cv3,cv4);

	lcdWriteText(0, &str1[0], 16);
	lcdWriteText(1, &str2[0], 16);
}
