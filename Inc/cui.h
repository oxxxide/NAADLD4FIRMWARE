/*
 * cui.h
 *
 *  Created on: 2018/06/28
 *      Author: devox
 */

#ifndef CUI_H_
#define CUI_H_

#include "stm32f4xx.h"
#include "MidiConfig.h"

#define LCD_STATE_DEFAULT 0
#define LCD_STATE_MENU 1
#define LCD_STATE_SEQ 2
#define LCD_STATE_MIDI 3
#define LCD_STATE_SYNC 4
#define LCD_STATE_TRIG 5
#define LCD_STATE_LOAD_PROGRAM 6
#define LCD_STATE_SAVE_PROGRAM 7

#define ITEM_INDEX_SEQ 0
#define ITEM_INDEX_SYNC 1
#define ITEM_INDEX_MIDI 2
#define ITEM_INDEX_TRIG 3

#define MENU_TEXTS_1 " SEQ    SYNC    "
#define MENU_TEXTS_2 " MIDI   TRIG    "
#define MENU_MIDI_TEXT_1 "MIDI Prt Ch Note"
#define MENU_MIDI_TEXT_2 "RECV   %c %2d  %3d"

#define MENU_SYNC_TEXT_1 "SYNC MODE:      "

#define MENU_SYNC_TEXT_Internal "Internal       "
#define MENU_SYNC_TEXT_External "External       "

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
	uint8_t channel;
	uint8_t noteNo;
} ChannelConfig;

typedef struct {
	uint8_t clockSource;
	uint8_t outClock;
	uint16_t BPM;
	/*ChannelConfig channel[4];*/
} MidiSyncConfig;

extern uint8_t LcdMenuSelectedItemIndex;
extern uint8_t LcdMenuState;
extern uint8_t SelectedPartNo;
extern uint8_t is_pressed_key_SHIFT;
extern ChannelConfig ChannelConfigs[4];
extern MidiSyncConfig midiSyncConfig;


extern char ChPtName[4];

extern const char* PresetTones[16];

void SelectMenu(int add);

void MIDIConfig_Show(void);
void MIDIConfig_ChangeNt(int add);
void MIDIConfig_ChangeCh(int add);
void MIDIConfig_ChangePt(int add);

void SyncConfig_Show(void);
void SyncConfig_ChangeSync(void);

void TriggerConfig_Show(void);
void TriggerConfig_Change(int add);

#endif /* CUI_H_ */
