//////////////////////////////////////////////////////////////////////////////
//
// Filename: eeprom_app.h
//
// Description: Implements application specific needs for the ASL110 related to
//		application NVM storage needs.
//
// Author(s): Trevor Parsh (Embedded Wizardry, LLC)
//
// Modified for ASL on Date: 
//
//////////////////////////////////////////////////////////////////////////////

#ifndef EEPROM_APP_H
#define EEPROM_APP_H

/* ***************************    Includes     **************************** */

// from stdlib
#include <stdint.h>
#include <stdbool.h>

// from project
#include "user_button.h"
#include "head_array.h"

/* ******************************   Version   ***************************** */

// Anytime the data structure is altered in EEPROM, bump this version.
// This identifies the structure of the EEPROM items. If you change the 
// structure, increment this value in order to automatically update the 
// EEPROM data.
// 
// Version History
// 4 = Original plus some stuff, supported by 1.6.x
// 5 = Changed to support Minimum Drive Speed for all 3 pads.
// 6 = [9/18/20] Added RNet Sleep feature and Mode Switch Schema feature.
#define EEPROM_DATA_STRUCTURE_VERSION				((uint8_t)0x06)

/* ******************************   Types   ******************************* */
#ifdef ASL110

typedef enum
{
	EEPROM_STORED_ITEM_EEPROM_INITIALIZED,

	EEPROM_STORED_ITEM_LEFT_PAD_INPUT_TYPE,
	EEPROM_STORED_ITEM_RIGHT_PAD_INPUT_TYPE,
	EEPROM_STORED_ITEM_CTR_PAD_INPUT_TYPE,

	EEPROM_STORED_ITEM_LEFT_PAD_OUTPUT_MAP,
	EEPROM_STORED_ITEM_RIGHT_PAD_OUTPUT_MAP,
	EEPROM_STORED_ITEM_CTR_PAD_OUTPUT_MAP,

	EEPROM_STORED_ITEM_USER_BTN_LONG_PRESS_ACT_TIME,

	EEPROM_STORED_ITEM_ENABLED_FEATURES,
	EEPROM_STORED_ITEM_CURRENT_ACTIVE_FEATURE,
	
	EEPROM_STORED_ITEM_LEFT_PAD_MIN_ADC_VAL,
	EEPROM_STORED_ITEM_LEFT_PAD_MAX_ADC_VAL,
	EEPROM_STORED_ITEM_LEFT_PAD_MIN_THRESH_PERC,
	EEPROM_STORED_ITEM_LEFT_PAD_MAX_THRESH_PERC,
	
	EEPROM_STORED_ITEM_RIGHT_PAD_MIN_ADC_VAL,
	EEPROM_STORED_ITEM_RIGHT_PAD_MAX_ADC_VAL,
	EEPROM_STORED_ITEM_RIGHT_PAD_MIN_THRESH_PERC,
	EEPROM_STORED_ITEM_RIGHT_PAD_MAX_THRESH_PERC,
	
	EEPROM_STORED_ITEM_CTR_PAD_MIN_ADC_VAL,
	EEPROM_STORED_ITEM_CTR_PAD_MAX_ADC_VAL,
	EEPROM_STORED_ITEM_CTR_PAD_MIN_THRESH_PERC,
	EEPROM_STORED_ITEM_CTR_PAD_MAX_THRESH_PERC,
	
    EEPROM_STORED_ITEM_MM_NEUTRAL_DAC_COUNTS,
    EEPROM_STORED_ITEM_MM_NEUTRAL_DAC_SETTING,
    EEPROM_STORED_ITEM_MM_NEUTRAL_DAC_RANGE,

    EEPROM_STORED_ITEM_MM_EEPROM_VERSION,

    EEPROM_STORED_ITEM_MM_CENTER_PAD_MINIMUM_DRIVE_OFFSET,
    EEPROM_STORED_ITEM_MM_LEFT_PAD_MINIMUM_DRIVE_OFFSET,
    EEPROM_STORED_ITEM_MM_RIGHT_PAD_MINIMUM_DRIVE_OFFSET,
            
    EEPROM_STORED_ITEM_ENABLED_FEATURES_2,
	// Nothing else may be defined past this point!
	EEPROM_STORED_ITEM_EOL
} EepromItemId_t;

typedef uint8_t EepromStoredEnumType_t;
#endif // #ifdef ASL110

/* ***********************   Function Prototypes   ************************ */

#ifdef ASL110

bool eepromAppInit(void);
void SetDefaultValues(void);
void eepromFlush(bool force_save_all);
uint8_t eepromAppNumTimesAnyDataHasBeenUpdated(void);

void eepromBoolSet(EepromItemId_t item_id, bool val);
bool eepromBoolGet(EepromItemId_t item_id);

void eepromEnumSet(EepromItemId_t item_id, EepromStoredEnumType_t val);
EepromStoredEnumType_t eepromEnumGet(EepromItemId_t item_id);

void eeprom8bitSet(EepromItemId_t item_id, uint8_t val);
uint8_t eeprom8bitGet(EepromItemId_t item_id);

void eeprom16bitSet(EepromItemId_t item_id, uint16_t val);
uint16_t eeprom16bitGet(EepromItemId_t item_id);

#endif // #ifdef ASL110

#endif // EEPROM_APP_H

// end of file.
//-------------------------------------------------------------------------
