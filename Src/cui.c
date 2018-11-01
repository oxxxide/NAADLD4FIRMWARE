/*
 * cui.c
 *
 *  Created on: 2018/06/28
 *      Author: devox
 */

#include "cui.h"
#include "MidiConfig.h"
#include "lcd_manager.h"
#include "main.h"
#include "tone.h"

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
uint8_t LcdMenuState = LCD_STATE_DEFAULT;
uint8_t is_pressed_key_SHIFT = 0;

ChannelConfig ChannelConfigs[4] = { { 0, 60 }, { 0, 61 }, { 0, 62 }, { 0, 63 } };

uint8_t SelectedPartNo = 0;

MidiSyncConfig midiSyncConfig = {CLOCK_SOURCE_INTERNAL,CLOCK_OUT_DISABLE,120};

uint8_t triggerThreshold = 0;

char ChPtName[4] = { 'A', 'B', 'C', 'D' };

void SelectMenu(int add) {
	int tmp = LcdMenuSelectedItemIndex;
	if (add > 0) {
		tmp++;
	} else if (add < 0) {
		tmp--;
	}
	LcdMenuSelectedItemIndex = LIMIT(tmp,3,0);

	//ƒƒjƒ…[•`‰æ
	switch (LcdMenuSelectedItemIndex) {
	case 0:
		lcdWriteText(0,"~SEQ    SYNC    ",16);
		lcdWriteText(1," MIDI   TRIG    ",16);
		break;
	case 1:
		lcdWriteText(0," SEQ   ~SYNC    ",16);
		lcdWriteText(1," MIDI   TRIG    ",16);
		break;
	case 2:
		lcdWriteText(0," SEQ    SYNC    ",16);
		lcdWriteText(1,"~MIDI   TRIG    ",16);
		break;
	case 3:
		lcdWriteText(0," SEQ    SYNC    ",16);
		lcdWriteText(1," MIDI  ~TRIG    ",16);
		break;
	}
}

static void MIDIConfig_refresh() {
	char buff[17];
	//MIDI‰æ–Ê‚Ö
	sprintf(buff,
			MENU_MIDI_TEXT_2,
			ChPtName[SelectedPartNo],
			ChannelConfigs[SelectedPartNo].channel + 1,
			ChannelConfigs[SelectedPartNo].noteNo);

	lcdWriteText(0,MENU_MIDI_TEXT_1,16);
	lcdWriteText(1,buff,16);
	LcdMenuState = LCD_STATE_MIDI;
}

void MIDIConfig_Show() {
	MIDIConfig_refresh();
}

void MIDIConfig_ChangeNt(int add) {
	ChannelConfig *p = &ChannelConfigs[SelectedPartNo];
	int result;
	result = p->noteNo + add;
	if (result > 127) {
		p->noteNo = 127;
	} else if (result < 0) {
		p->noteNo = 0;
	} else {
		p->noteNo = result;
	}
	MIDIConfig_refresh();
}

void MIDIConfig_ChangeCh(int add) {

	ChannelConfig *p = &ChannelConfigs[SelectedPartNo];
	int result;
	result = p->channel + (add > 0 ? 1 : -1);
	if (result > 15) {
		p->channel = 15;
	} else if (result < 0) {
		p->channel = 0;
	} else {
		p->channel = result;
	}
	MIDIConfig_refresh();
}

void MIDIConfig_ChangePt(int add) {

	int result;
	if (add > 0) {
		result = (SelectedPartNo + 1) % 4;
	} else {
		result = SelectedPartNo - 1;
		if (result < 0) {
			result = 3;
		}
	}
	SelectedPartNo = result;
	MIDIConfig_refresh();
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
