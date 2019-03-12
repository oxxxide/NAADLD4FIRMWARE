/*
 * cui.h
 *
 *  Created on: 2018/06/28
 *      Author: NishiAsakusa Audio Developments / oxxxide / Akikazu Iwasa
 */

#ifndef CUI_H_
#define CUI_H_

#include "stm32f4xx.h"
#include "MidiConfig.h"
#include "Sequencer.h"
#include "cvconfig.h"

typedef enum {
	LCD_STATE_DEFAULT = 0,
	LCD_STATE_MENU = 1,
	LCD_STATE_SYNC = 4,
	LCD_STATE_VELC = 5,
	LCD_STATE_LOAD_PROGRAM = 6,
	LCD_STATE_SAVE_PROGRAM = 7,
	LCD_STATE_CV_INPUT_CONFIG = 8,
	LCD_STATE_FACTORY_RESET_CONFIRM = 9,
	LCD_STATE_MIDI_RECEIVE_CONFIG = 10,
	LCD_STATE_PROGRAM_MENU = 11,
	LCD_STATE_ECHOBACK = 12,
	LCD_STATE_SEQ_EDIT = 13,
	LCD_STATE_CONFIRM_REVERT = 14,
	LCD_STATE_SEQ_STEP_CFG = 15,
	LCD_STATE_SEQ_BEAT_REPEAT = 16,
	LCD_STATE_SYSTEM_INFO = 17,
	LCD_STATE_CV_MONTORING_INPUTS = 18,
} LCD_STATE;

typedef enum {
	ITEM_INDEX_SEQUENCER = 0,
	ITEM_INDEX_SYNC = 1,
	ITEM_INDEX_MIDI_MAPPING = 2,
	ITEM_INDEX_VELOCITY_CURVE = 3,
	ITEM_INDEX_CV_INPUT_SETTINGS = 4,
	ITEM_INDEX_ECHO_BACK = 5,
	ITEM_INDEX_SYSTEM_INFO = 6,
	ITEM_INDEX_FACTORY_RESET = 7
} MENU_ITEM;

typedef enum {
	ITEM_INDEX_TEMPSAVE = 0, ITEM_INDEX_STORE = 1, ITEM_INDEX_LOAD = 2, ITEM_INDEX_REVERT = 3,
} PROGRAM_MENU_ITEM;

#define MENU_TEXTS_1 " SEQ    SYNC    "
#define MENU_TEXTS_2 " MIDI   VELC    "
#define MENU_TEXTS_3 " MONITOR CV-IN  "
#define MENU_TEXTS_4 " RESTORE FS     "

#define MENU_TRIG_TEXT_1 "Trig Threshold  "
#define MENU_TRIG_TEXT_2 "           %3dV"

#define MENU_SEQ "INS Step Prt Vel"
#define MENU_SEQ_INFO "     %2d   %c %3"
#define MENU_SEQ_CMD "INSERT DELETE"


#define CLOCK_SOURCE_INTERNAL 1
#define CLOCK_SOURCE_EXTERNAL 2
#define CLOCK_OUT_ENABlE 1
#define CLOCK_OUT_DISABLE 0

typedef struct {
	uint8_t clockSource;
	uint8_t outClock;
	uint16_t BPM;
} MidiSyncConfig;

extern uint8_t LcdMenuSelectedItemIndex;
extern uint8_t seq_menu_item_index;
extern volatile LCD_STATE LcdMenuState;
extern int ProgramMenuSelectedItemIndex;
extern uint8_t is_pressed_key_SHIFT;
extern MidiSyncConfig midiSyncConfig;
extern char ChPtName[4];
extern const char* PresetTones[16];

void SelectMenu(int add);

void MIDIConfig_Show(MidiConfig* midiConfig);

void MIDIConfig_ChangeNt(MidiConfig* midiConfig, int add);
void MIDIConfig_ChangeCh(MidiConfig* midiConfig, int add);
void MIDIConfig_DisplayChannel(MidiConfig* midiConfig ,int add);
void MIDIConfig_VelocityCurve(MidiConfig* midiConfig, int add);
void MIDIConfig_EchoBack(MidiConfig* config);
void MIDIConfig_SyncMode(MidiConfig* config);

void TriggerConfig_Show(void);
void TriggerConfig_Change(int add);
void ShowProgramMenu(int add);
void showConfirmRevert(void);
void showSystemVersion(void);
void CV_Monitor_Show(void);
void CV_Assignment_Settings_Show(CV_ASSIGN* array, int size, int add_input,
		int add_output, int add_param);

void ShowSequencerEditMode(Sequencer* seq, int moveStep, SyncMode syncMode);
void showSequencerStepConfig(Sequencer* seq, int konb, int add);
void showSequencerBeatRepeatConfig(Sequencer* seq, int knob, int add);

void ConfirmFactoryReset(void);

#endif /* CUI_H_ */
