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
uint8_t seq_menu_item_index = 0;

int ProgramMenuSelectedItemIndex = 0;

volatile LCD_STATE LcdMenuState = LCD_STATE_DEFAULT;
uint8_t is_pressed_key_SHIFT = 0;



//MidiSyncConfig midiSyncConfig = {CLOCK_SOURCE_INTERNAL,CLOCK_OUT_DISABLE,120};

uint8_t triggerThreshold = 0;

void SelectMenu(int add) {
	int tmp = LcdMenuSelectedItemIndex;
	const int LastIndex = 7;
	if (add > 0) {
		tmp++;
	} else if (add < 0) {
		tmp--;
	}

	if (tmp < 0) {
		tmp = LastIndex;
	} else if (tmp > LastIndex) {
		tmp = 0;
	}

	LcdMenuSelectedItemIndex = tmp;

	switch (LcdMenuSelectedItemIndex) {
	case ITEM_INDEX_SEQUENCER:
		lcdWriteText(0,"~Sequencer       ",16);
		lcdWriteText(1," Sync            ",16);
		break;
	case ITEM_INDEX_SYNC:
		lcdWriteText(0," Sequencer       ",16);
		lcdWriteText(1,"~Sync            ",16);
		break;
	case ITEM_INDEX_MIDI_MAPPING:
		lcdWriteText(0,"~MIDI Mapping    ",16);
		lcdWriteText(1," Velocity Curve  ",16);
		break;
	case ITEM_INDEX_VELOCITY_CURVE:
		lcdWriteText(0," MIDI Mapping    ",16);
		lcdWriteText(1,"~Velocity Curve  ",16);
		break;
	case ITEM_INDEX_CV_INPUT_SETTINGS:
		lcdWriteText(0,"~CVIN Assignment",16);
		lcdWriteText(1," EchoBack       ",16);
		break;
	case ITEM_INDEX_ECHO_BACK:
		lcdWriteText(0," CVIN Assignment",16);
		lcdWriteText(1,"~EchoBack       ",16);
		break;
	case ITEM_INDEX_SYSTEM_INFO:
		lcdWriteText(0,"~System Info    ",16);
		lcdWriteText(1," Factory Reset  ",16);
		break;
	case ITEM_INDEX_FACTORY_RESET:
		lcdWriteText(0," System Info    ",16);
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
	if (nn == NOTEMAP_SCALE) {
		sprintf(buff2, "         %2d  SCL", ch + 1);
	} else {
		sprintf(buff2, "         %2d  %3d", ch + 1, nn);
	}
	lcdWriteText(0, buff1, 16);
	lcdWriteText(1, buff2, 16);

}


static int loop(int a, int b) {
	int tmp = a + b;
	if (tmp < 0) {
		tmp = NOTEMAP_SCALE;
	} else if (tmp > NOTEMAP_SCALE) {
		tmp = 0;
	}
	return tmp;
}

void MIDIConfig_ChangeNt(MidiConfig* midiConfig, int add) {
	switch (displayChannel) {
	case 'A':
		midiConfig->noteNo_A = loop(midiConfig->noteNo_A, add);
		break;
	case 'B':
		midiConfig->noteNo_B = loop(midiConfig->noteNo_B, add);
		break;
	case 'C':
		midiConfig->noteNo_C = loop(midiConfig->noteNo_C, add);
		break;
	case 'D':
		midiConfig->noteNo_D = loop(midiConfig->noteNo_D, add);
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

void MIDIConfig_VelocityCurve(MidiConfig* midiConfig, int add) {
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

void ShowSequencerEditMode(Sequencer* seq, int moveStep, SyncMode syncMode) {
	LcdMenuState = LCD_STATE_SEQ_EDIT;
	char str[17] = { '\0' };
	if (moveStep != 0) {
		seq->cursor_index += (moveStep > 0 ? 1 : -1);
		if (seq->cursor_index > 15) {
			seq->cursor_index = 15;
		} else if (seq->cursor_index <= 0) {
			seq->cursor_index = 0;
		}
	}

	//1st Line
	if(syncMode == ExternalClock){
		sprintf(str, "STEP:%02d  BPM:Ext", seq->cursor_index + 1);
	}else{
		sprintf(str, "STEP:%02d  BPM:%03d", seq->cursor_index + 1, seq->bpm);
	}
	lcdWriteText(0, str, 16);

	//2nd Line
	Notes *n = &(seq->sequenceData[seq->cursor_index]);
	sprintf(str, "A:%c B:%c C:%c D:%c ",
			n->a ? '*' : '-',
			n->b ? '*' : '-',
			n->c ? '*' : '-',
			n->d ? '*' : '-');
	lcdWriteText(1, str, 16);
}

void showSequencerStepConfig(Sequencer* seq, int knob, int add) {
	LcdMenuState = LCD_STATE_SEQ_STEP_CFG;

	if (add > 0) {
		add = 1;
	} else if (add < 0) {
		add = -1;
	}

	int tmp = 0;

	switch (knob) {
	case 0:
		tmp = seq->step_length_array[0] + add;
		seq->step_length_array[0] = (uint8_t)LIMIT(tmp, 16, 1);
		break;
	case 1:
		tmp = seq->step_length_array[1] + add;
		seq->step_length_array[1] = (uint8_t)LIMIT(tmp, 16, 1);
		break;
	case 2:
		tmp = seq->step_length_array[2] + add;
		seq->step_length_array[2] = (uint8_t)LIMIT(tmp, 16, 1);
		break;
	case 3:
		tmp = seq->step_length_array[3] + add;
		seq->step_length_array[3] = (uint8_t)LIMIT(tmp, 16, 1);
		break;
	}

	char str[17] = { '\0' };
	sprintf(str,"LEN  %2d %2d %2d %2d",
			seq->step_length_array[0],
			seq->step_length_array[1],
			seq->step_length_array[2],
			seq->step_length_array[3]
			);
	lcdWriteText(0, "STEP  A  B  C  D", 16);
	lcdWriteText(1, str, 16);
}

void showSequencerBeatRepeatConfig(Sequencer* seq, int knob, int add) {
	LcdMenuState = LCD_STATE_SEQ_BEAT_REPEAT;

	if (add > 0) {
		add = 1;
	} else if (add < 0) {
		add = -1;
	}

	if (knob >= 0) {
		if(knob==4){ //ENTRY KNOB
			seq->playFxEnabled = LIMIT( ((int)seq->playFxEnabled) + add, 1, 0);
		}else{
			seq->playfx[knob].chance = (uint8_t) LIMIT(
					seq->playfx[knob].chance + add, 7, 0);
		}
	}
	char str[17] = { '\0' };
	sprintf(str, "%s      %1d %1d %1d %1d",
			seq->playFxEnabled ? "ON ":"OFF",
			seq->playfx[0].chance,
			seq->playfx[1].chance,
			seq->playfx[2].chance,
			seq->playfx[3].chance);
	lcdWriteText(0, "BeatRpt  A B C D", 16);
	lcdWriteText(1, str, 16);
}

void ShowProgramMenu(int add) {

	LcdMenuState = LCD_STATE_PROGRAM_MENU;

	ProgramMenuSelectedItemIndex += add;
	if (ProgramMenuSelectedItemIndex > 3) {
		ProgramMenuSelectedItemIndex = 0;
	} else if (ProgramMenuSelectedItemIndex < 0) {
		ProgramMenuSelectedItemIndex = 3;
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
		lcdWriteText(1, " Revert         ", 16);
		break;
	case 3:
		lcdWriteText(0, " Load  Program  ", 16);
		lcdWriteText(1, "~Revert         ", 16);
		break;
	}

}

void showConfirmRevert() {
		lcdWriteText(0, "Revert?         ", 16);
		lcdWriteText(1, "N:EXIT  Y:ENTER ", 16);
}

void showConfirmResetAssignment() {
		lcdWriteText(0, "ResetAssignment?", 16);
		lcdWriteText(1, "N:EXIT  Y:ENTER ", 16);
}

void showSystemVersion() {
	LcdMenuState = LCD_STATE_SYSTEM_INFO;
	lcdWriteText(0, "NAAD LD4        ", 16);
	lcdWriteText(1, FIRMWARE_VERSION, 16);
}

void MIDIConfig_SyncMode(MidiConfig* config){
	lcdWriteText(0,"Sync Mode       ",16);
	switch(config->syncMode){
	case InternalClock:
		lcdWriteText(1,"~Internal       ",16);
		break;
	case ExternalClock:
		lcdWriteText(1,"~External       ",16);
		break;
	}
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

void MIDIConfig_EchoBack(MidiConfig* config){
	lcdWriteText(0, "Echo Back       ", 16);
	if(config->echoBack){
		lcdWriteText(1, "~ON             ", 16);
	}else{
		lcdWriteText(1, "~OFF            ", 16);
	}
}

void CV_Assignment_Settings_Show(CV_ASSIGN* array, int size, int add_input, int add_output, int add_param) {

	static const int num_of_paramtype = 6;
	static uint8_t selected = 0;
	selected = LIMIT(selected + add_input, size-1, 0);

	static char str1[17] = {'\0'};
	sprintf(str1, "CV IN:%c          ", 'A' + selected);
	lcdWriteText(0, str1, 16);

	CV_ASSIGN* cva = &array[selected];
	cva->target_channel = LIMIT(cva->target_channel + add_output, size - 1, 0);
	cva->assign = LIMIT(cva->assign + add_param, num_of_paramtype, 0);

	static char str2[17] = {'\0'};

	char *p_name = 0;
	switch (cva->assign) {
	case CV_NONE:
		p_name = "Disconnect ";
		break;
	case CV_PITCH:
		p_name = "OSC-PITCH  ";
		break;
	case CV_MOD_FREQ:
		p_name = "Mod-Freq   ";
		break;
	case CV_CUTOFF:
		p_name = "CUTOFF     ";
		break;
	case CV_MOD_DEPTH:
		p_name = "Mod-Depth  ";
		break;
	case CV_BEND_AMT:
		p_name = "Bend-Amount";
		break;
	case CV_AMP_REL:
		p_name = "AEG-Release ";
		break;
	default:
		p_name = "";
	}
	sprintf(str2, "Ch.%c:%s", 'A'+cva->target_channel,  p_name);
	lcdWriteText(1, str2, 16);
}

void CV_Monitor_Show(){

	static char str1[17] = {'\0'};
	static char str2[17] = {'\0'};

	float iv1, iv2, iv3, iv4;


	//if (HAL_GPIO_ReadPin(GPIO_INPUT_ADCSW1_GPIO_Port, GPIO_INPUT_ADCSW1_Pin)
	//		== GPIO_PIN_RESET) {
	//	iv1 = 0;
	//} else {
		iv1 = adcResultA / 500.0f;
	//}

	//if (HAL_GPIO_ReadPin(GPIO_INPUT_ADCSW2_GPIO_Port, GPIO_INPUT_ADCSW2_Pin)
	//		== GPIO_PIN_RESET) {
	//	iv2 = 0;
	//} else {
		iv2 = adcResultB / 500.0f;
	//}

	//if (HAL_GPIO_ReadPin(GPIO_INPUT_ADCSW3_GPIO_Port, GPIO_INPUT_ADCSW3_Pin)
	//		== GPIO_PIN_RESET) {
	//	iv3 = 0;
	//} else {
		iv3 = adcResultC / 500.0f;
	//}

	//if (HAL_GPIO_ReadPin(GPIO_INPUT_ADCSW4_GPIO_Port, GPIO_INPUT_ADCSW4_Pin)
	//		== GPIO_PIN_RESET) {
	//	iv4 = 0;
	//} else {
		iv4 = adcResultD / 500.0f;
	//}


	sprintf(str1, "A:%2.2fV B:%2.2fV ", iv1, iv2);
	sprintf(str2, "C:%2.2fV D:%2.2fV ", iv3, iv4);

	lcdWriteText(0, &str1[0], 16);
	lcdWriteText(1, &str2[0], 16);
}
