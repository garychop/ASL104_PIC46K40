//////////////////////////////////////////////////////////////////////////////
//
// Filename: eeprom_app.c
//
// Description: Implements application specific needs for the ASL110 related to
//		application NVM storage needs.
//  
//  Reference the following web page to create this driver:
//  https://deepbluembedded.com/eeproms-internal-in-pic-microcontrollers/
//
// Author(s): Trevor Parsh (Embedded Wizardry, LLC)
//
// Modified for ASL on Date: 
//
//////////////////////////////////////////////////////////////////////////////


/* **************************   Header Files   *************************** */

// NOTE: This must ALWAYS be the first include in a file.
#include "device.h"

#include <stdint.h>
#include <stdbool.h>
#include "user_assert.h"

// from RTOS
#include "cocoos.h"

// from project
#include "config.h"
#include "common.h"
#include "user_button.h"
#include "head_array_bsp.h" // TODO: Expose MIN/MAX values in head_array driver module
#include "head_array.h"
#include "app_common.h"

// from local
#include "eeprom_bsp.h"
#include "eeprom_app.h"

/* ******************************   Macros   ****************************** */

#ifdef ASL110

#define EEPROM_INITIALIZED_VAL						((uint8_t)0xA5)

// Size of supported data types
#define ITEM_TYPE_BOOL_SIZE_BYTES					((uint8_t)1)
#define ITEM_TYPE_UINT8_SIZE_BYTES					((uint8_t)1)
#define ITEM_TYPE_ENUM_SIZE_BYTES					((uint8_t)1)
#define ITEM_TYPE_UINT16_SIZE_BYTES					((uint8_t)2)

#endif // #ifdef ASL110

/* **************************    Memory Map     *************************** */

#ifdef ASL110

#define MM_EEPROM_INITIALIZED 						((uint8_t)0x00)

#define MM_LEFT_PAD_INPUT_TYPE 						((uint8_t)MM_EEPROM_INITIALIZED + ITEM_TYPE_UINT8_SIZE_BYTES)
#define MM_RIGHT_PAD_INPUT_TYPE 					((uint8_t)MM_LEFT_PAD_INPUT_TYPE + ITEM_TYPE_ENUM_SIZE_BYTES)
#define MM_CENTER_PAD_INPUT_TYPE 					((uint8_t)MM_RIGHT_PAD_INPUT_TYPE + ITEM_TYPE_ENUM_SIZE_BYTES)

#define MM_LEFT_PAD_OUTPUT_MAP 						((uint8_t)MM_CENTER_PAD_INPUT_TYPE + ITEM_TYPE_ENUM_SIZE_BYTES)
#define MM_RIGHT_PAD_OUTPUT_MAP 					((uint8_t)MM_LEFT_PAD_OUTPUT_MAP + ITEM_TYPE_ENUM_SIZE_BYTES)
#define MM_CENTER_PAD_OUTPUT_MAP 					((uint8_t)MM_RIGHT_PAD_OUTPUT_MAP + ITEM_TYPE_ENUM_SIZE_BYTES)

#define MM_USER_BTN_LONG_PRESS_ACT_TIME 			((uint8_t)MM_CENTER_PAD_OUTPUT_MAP + ITEM_TYPE_ENUM_SIZE_BYTES)

#define MM_ENABLED_FEATURES							((uint8_t)MM_USER_BTN_LONG_PRESS_ACT_TIME + ITEM_TYPE_UINT16_SIZE_BYTES)
#define MM_CURRENT_ACTIVE_FEATURE					((uint8_t)MM_ENABLED_FEATURES + ITEM_TYPE_UINT8_SIZE_BYTES)

#define MM_LEFT_PAD_MIN_ADC_VAL						((uint8_t)MM_CURRENT_ACTIVE_FEATURE + ITEM_TYPE_ENUM_SIZE_BYTES)
#define MM_LEFT_PAD_MAX_ADC_VAL						((uint8_t)MM_LEFT_PAD_MIN_ADC_VAL + ITEM_TYPE_UINT16_SIZE_BYTES)
#define MM_LEFT_PAD_MIN_THRESH_PERC                 ((uint8_t)MM_LEFT_PAD_MAX_ADC_VAL + ITEM_TYPE_UINT16_SIZE_BYTES)
#define MM_LEFT_PAD_MAX_THRESH_PERC             	((uint8_t)MM_LEFT_PAD_MIN_THRESH_PERC + ITEM_TYPE_UINT16_SIZE_BYTES)

#define MM_RIGHT_PAD_MIN_ADC_VAL					((uint8_t)MM_LEFT_PAD_MAX_THRESH_PERC + ITEM_TYPE_UINT16_SIZE_BYTES)
#define MM_RIGHT_PAD_MAX_ADC_VAL					((uint8_t)MM_RIGHT_PAD_MIN_ADC_VAL + ITEM_TYPE_UINT16_SIZE_BYTES)
#define MM_RIGHT_PAD_MIN_THRESH_PERC                ((uint8_t)MM_RIGHT_PAD_MAX_ADC_VAL + ITEM_TYPE_UINT16_SIZE_BYTES)
#define MM_RIGHT_PAD_MAX_THRESH_PERC            	((uint8_t)MM_RIGHT_PAD_MIN_THRESH_PERC + ITEM_TYPE_UINT16_SIZE_BYTES)

#define MM_CTR_PAD_MIN_ADC_VAL						((uint8_t)MM_RIGHT_PAD_MAX_THRESH_PERC + ITEM_TYPE_UINT16_SIZE_BYTES)
#define MM_CTR_PAD_MAX_ADC_VAL						((uint8_t)MM_CTR_PAD_MIN_ADC_VAL + ITEM_TYPE_UINT16_SIZE_BYTES)
#define MM_CTR_PAD_MIN_THRESH_PERC          		((uint8_t)MM_CTR_PAD_MAX_ADC_VAL + ITEM_TYPE_UINT16_SIZE_BYTES)
#define MM_CTR_PAD_MAX_THRESH_PERC      			((uint8_t)MM_CTR_PAD_MIN_THRESH_PERC + ITEM_TYPE_UINT16_SIZE_BYTES)

// Added in Version 2 of EEPROM
#define MM_NEUTRAL_DAC_COUNTS       				((uint8_t)MM_CTR_PAD_MAX_THRESH_PERC + ITEM_TYPE_UINT16_SIZE_BYTES) // 16bits
#define MM_NEUTRAL_DAC_SETTING      				((uint8_t)MM_NEUTRAL_DAC_COUNTS + ITEM_TYPE_UINT16_SIZE_BYTES)      // 16bits
#define MM_NEUTRAL_DAC_RANGE          				((uint8_t)MM_NEUTRAL_DAC_SETTING + ITEM_TYPE_UINT16_SIZE_BYTES)     // 16bits

// Added in Version 3 of EEPROM
#define MM_EEPROM_VERSION                   		((uint8_t)MM_NEUTRAL_DAC_RANGE + ITEM_TYPE_UINT16_SIZE_BYTES)     // 16bits

// Added in Version 4 of EEPROM and changed the name in Version 5
//#define MM_DRIVE_OFFSET                             ((uint8_t)MM_EEPROM_VERSION + ITEM_TYPE_UINT8_SIZE_BYTES)
#define MM_CENTER_PAD_MIN_DRIVE_SPEED               ((uint8_t)MM_EEPROM_VERSION + ITEM_TYPE_UINT8_SIZE_BYTES)

// Added in Version 5. Added minimum Drive Speed for all 3 speed but reusng the minimum drive speed for the Center Pad Minimum Drive Speed.
#define MM_LEFT_PAD_MIN_DRIVE_SPEED                 ((uint8_t)MM_CENTER_PAD_MIN_DRIVE_SPEED + ITEM_TYPE_UINT8_SIZE_BYTES)
#define MM_RIGHT_PAD_MIN_DRIVE_SPEED                ((uint8_t)MM_LEFT_PAD_MIN_DRIVE_SPEED + ITEM_TYPE_UINT8_SIZE_BYTES)

// Version 6. Added Feature Byte 2 to accomodate RNet Sleep and Mode Switch schema
#define MM_ENABLED_FEATURES_2						((uint8_t)MM_RIGHT_PAD_MIN_DRIVE_SPEED + ITEM_TYPE_UINT8_SIZE_BYTES)

// Must be last item in this list. Denotes the total amount of real estate taken up in EEPROM.
#define MM_NUM_BYTES								((uint8_t)MM_RIGHT_PAD_MIN_DRIVE_SPEED + ITEM_TYPE_UINT8_SIZE_BYTES)

#endif // #ifdef ASL110

/* ******************************   Types   ******************************* */

#ifdef ASL110

typedef enum
{
	ITEM_TYPE_BOOL,
	ITEM_TYPE_ENUM,
	ITEM_TYPE_UINT8,
	ITEM_TYPE_UINT16,

	// Nothing else may be defined past this point!
	ITEM_TYPE_EOL
} ItemType_t;
#endif // #ifdef ASL110

#ifdef ASL110

typedef struct
{
	ItemType_t type;

	// Position in RAM and physical memory address in EEPROM that the item lives.
	uint8_t start_addr;

	// True when an item has been updated in RAM but not in EEPROM.
	bool need_to_save;
} ItemInfo_t;

#endif // #ifdef ASL110

#ifdef ASL110

// No reason to pack the structure, the MCU is 8-bit.
// This structure can only be ADDED TO or APPENDED. It must never be changed.
// This is important to allow future proving of firmware updating and preserving
// existing settings.
typedef struct
{
	uint8_t eeprom_intiailized;
	
	EepromStoredEnumType_t left_pad_input_type;
	EepromStoredEnumType_t right_pad_input_type;
	EepromStoredEnumType_t center_pad_input_type;

	EepromStoredEnumType_t left_pad_output_map;
	EepromStoredEnumType_t right_pad_output_map;
	EepromStoredEnumType_t center_pad_output_map;

	uint16_t user_btn_long_press_act_time;

	uint8_t enabled_features;
	EepromStoredEnumType_t current_active_feature;

	uint16_t left_pad_min_adc_val;
	uint16_t left_pad_max_adc_val;
	uint16_t left_pad_min_thresh_perc;
	uint16_t left_pad_max_thresh_perc;

	uint16_t right_pad_min_adc_val;
	uint16_t right_pad_max_adc_val;
	uint16_t right_pad_min_thresh_perc;
	uint16_t right_pad_max_thresh_perc;

	uint16_t ctr_pad_min_adc_val;
	uint16_t ctr_pad_max_adc_val;
	uint16_t ctr_pad_min_thresh_perc;
	uint16_t ctr_pad_max_thresh_perc;
    
    // Added in eeprom version 2
    uint16_t neutral_DAC_counts; // TODO: What is the purpose of this variable?  It is not used.
    uint16_t neutral_DAC_setting;
    uint16_t neutral_DAC_range;
    
    // Added in EEPROM version 3
    uint8_t EEPROM_Version;     // Identifies the version/makeup of the stored data.
    
    // Added in EEPROM version 4
    // Changed to "Center" in EERPOM version 5
    uint8_t CenterPad_MinimumDriveSpeed;       // this is the drive percentage when in proportional
                                // .. and the digital sensor is active.
    
    // Added in EEPROM version 5
    uint8_t LeftPad_MinimumDriveSpeed;
    uint8_t RightPad_MinimumDriveSpeed;
    
    // Added in EEPROM Version 6
    uint8_t enabled_features2;
    
} EepromDataItems_t;

typedef union
{
	EepromDataItems_t items;
	uint8_t bytes[MM_NUM_BYTES];
} EepromData_t;
#endif // #ifdef ASL110

/* ***********************   File Scope Variables   *********************** */

#ifdef ASL110

static volatile EepromData_t eeprom_data;
static volatile uint8_t num_times_any_item_has_updated;

// Must match EepromItemId_t exactly!
static ItemInfo_t items_info[] =
{
	// EEPROM_STORED_ITEM_EEPROM_INITIALIZED
	{ITEM_TYPE_UINT8,		MM_EEPROM_INITIALIZED,						false},

	// EEPROM_STORED_ITEM_LEFT_PAD_INPUT_TYPE
	{ITEM_TYPE_ENUM,		MM_LEFT_PAD_INPUT_TYPE,						false},

	// EEPROM_STORED_ITEM_RIGHT_PAD_INPUT_TYPE
	{ITEM_TYPE_ENUM,		MM_RIGHT_PAD_INPUT_TYPE,					false},

	// EEPROM_STORED_ITEM_CTR_PAD_INPUT_TYPE
	{ITEM_TYPE_ENUM,		MM_CENTER_PAD_INPUT_TYPE,					false},

	// EEPROM_STORED_ITEM_LEFT_PAD_OUTPUT_MAP
	{ITEM_TYPE_ENUM,		MM_LEFT_PAD_OUTPUT_MAP,						false},

	// EEPROM_STORED_ITEM_RIGHT_PAD_OUTPUT_MAP
	{ITEM_TYPE_ENUM,		MM_RIGHT_PAD_OUTPUT_MAP,					false},

	// EEPROM_STORED_ITEM_CTR_PAD_OUTPUT_MAP
	{ITEM_TYPE_ENUM,		MM_CENTER_PAD_OUTPUT_MAP,					false},
	
	// EEPROM_STORED_ITEM_USER_BTN_LONG_PRESS_ACT_TIME
	{ITEM_TYPE_UINT16,		MM_USER_BTN_LONG_PRESS_ACT_TIME,			false},

	// EEPROM_STORED_ITEM_ENABLED_FEATURES
	{ITEM_TYPE_UINT8,		MM_ENABLED_FEATURES,						false},
	
	// EEPROM_STORED_ITEM_CURRENT_ACTIVE_FEATURE
	{ITEM_TYPE_ENUM,		MM_CURRENT_ACTIVE_FEATURE,					false},

	// EEPROM_STORED_ITEM_LEFT_PAD_MIN_ADC_VAL
	{ITEM_TYPE_UINT16,		MM_LEFT_PAD_MIN_ADC_VAL,					false},

	// EEPROM_STORED_ITEM_LEFT_PAD_MAX_ADC_VAL
	{ITEM_TYPE_UINT16,		MM_LEFT_PAD_MAX_ADC_VAL,					false},

	// EEPROM_STORED_ITEM_LEFT_PAD_MIN_THRESH_PERC
	{ITEM_TYPE_UINT16,		MM_LEFT_PAD_MIN_THRESH_PERC,        		false},

	// EEPROM_STORED_ITEM_LEFT_PAD_MAX_THRESH_PERC
	{ITEM_TYPE_UINT16,		MM_LEFT_PAD_MAX_THRESH_PERC,        		false},

	// EEPROM_STORED_ITEM_RIGHT_PAD_MIN_ADC_VAL
	{ITEM_TYPE_UINT16,		MM_RIGHT_PAD_MIN_ADC_VAL,					false},

	// EEPROM_STORED_ITEM_RIGHT_PAD_MAX_ADC_VAL
	{ITEM_TYPE_UINT16,		MM_RIGHT_PAD_MAX_ADC_VAL,					false},

	// EEPROM_STORED_ITEM_RIGHT_PAD_MIN_THRESH_PERC
	{ITEM_TYPE_UINT16,		MM_RIGHT_PAD_MIN_THRESH_PERC,       		false},

	// EEPROM_STORED_ITEM_RIGHT_PAD_MAX_THRESH_PERC
	{ITEM_TYPE_UINT16,		MM_RIGHT_PAD_MAX_THRESH_PERC,       		false},

	// EEPROM_STORED_ITEM_CTR_PAD_MIN_ADC_VAL
	{ITEM_TYPE_UINT16,		MM_CTR_PAD_MIN_ADC_VAL,						false},

	// EEPROM_STORED_ITEM_CTR_PAD_MAX_ADC_VAL
	{ITEM_TYPE_UINT16,		MM_CTR_PAD_MAX_ADC_VAL,						false},

	// EEPROM_STORED_ITEM_CTR_PAD_MIN_THRESH_PERC
	{ITEM_TYPE_UINT16,		MM_CTR_PAD_MIN_THRESH_PERC,         		false},

	// EEPROM_STORED_ITEM_CTR_PAD_MAX_THRESH_PERC
	{ITEM_TYPE_UINT16,		MM_CTR_PAD_MAX_THRESH_PERC,         		false},

    // Added in eeprom version 2

	// EEPROM_STORED_ITEM_MM_NEUTRAL_DAC_COUNTS
	{ITEM_TYPE_UINT16,		MM_NEUTRAL_DAC_COUNTS,                      false},

	// EEPROM_STORED_ITEM_MM_NEUTRAL_DAC_SETTING
	{ITEM_TYPE_UINT16,		MM_NEUTRAL_DAC_SETTING,                 	false},

	// EEPROM_STORED_ITEM_MM_NEUTRAL_DAC_RANGE
	{ITEM_TYPE_UINT16,		MM_NEUTRAL_DAC_RANGE,               		false},
    
    // Added in EEPROM Version 3
    // EEPROM_STORED_ITEM_MM_EEPROM_VERSION
    {ITEM_TYPE_UINT8, MM_EEPROM_VERSION,                                false},

    // Added in EEPROM Version 4
    // EEPROM_STORED_ITEM_MM_MINIMUM_DRIVE_OFFSET
    //{ITEM_TYPE_UINT8, MM_DRIVE_OFFSET,                                false},

    // Changes to EEPROM Version 5
    // EEPROM_STORED_ITEM_MM_CENTER_PAD_MINIMUM_DRIVE_OFFSET
    {ITEM_TYPE_UINT8, MM_CENTER_PAD_MIN_DRIVE_SPEED,                    false},
    // EEPROM_STORED_ITEM_MM_LEFT_PAD_MINIMUM_DRIVE_OFFSET
    {ITEM_TYPE_UINT8, MM_LEFT_PAD_MIN_DRIVE_SPEED,                      false},
    // EEPROM_STORED_ITEM_MM_RIGHT_PAD_MINIMUM_DRIVE_OFFSET
    {ITEM_TYPE_UINT8, MM_RIGHT_PAD_MIN_DRIVE_SPEED,                     false},

    // Added in Version 6
    // EEPROM_STORED_ITEM_ENABLED_FEATURES_2
	{ITEM_TYPE_UINT8,		MM_ENABLED_FEATURES_2,						false}
};
#endif // #ifdef ASL110

#ifdef ASL110

static volatile bool at_least_one_item_requires_saving;

#endif // #ifdef ASL110

/* ***********************   Function Prototypes   ************************ */

#ifdef ASL110
static bool SyncWithEeprom(void);
static void EepromIsProgrammedValWrite(void);

inline static void Write8bitVal(uint8_t address, uint8_t new_val);
inline static uint8_t Read8bitVal(uint8_t address);

inline static void Write16bitVal(uint8_t address, uint16_t new_val);
inline static uint16_t Read16bitVal(uint8_t address);

inline static void Write32bitVal(uint8_t address, uint32_t new_val);
inline static uint32_t Read32bitVal(uint8_t address);
#endif // #ifdef ASL110

/* *******************   Public Function Definitions   ******************** */

//-------------------------------
// Function: eepromAppInit
//
// Description: Intializes this module.
//
// return: true if EEPROM was initialized prior to now, false if it was initialized here.
//
//-------------------------------

#ifdef ASL110

bool eepromAppInit(void)
{
    volatile int8_t EEPROM_Version;
    int8_t MinimumDriveSpeed;
    
	at_least_one_item_requires_saving = false;
	num_times_any_item_has_updated = 0;

	eepromBspInit();

    SetDefaultValues(); // Load all eeprom items with default values.

#if !defined(SPECIAL_EEPROM_TO_DEFAULT_VALUES)
	bool eeprom_has_been_initialized = SyncWithEeprom();
#else
	bool eeprom_has_been_initialized = false;
#endif

	if (!eeprom_has_been_initialized)
	{
		//SetDefaultValues();
		eepromFlush(true);
	}
    else
    {
        // Check to see if the read/retrieved EEPROM data is different than
        // the data that this firmware version understands. If so, we need to
        // process any changes and then update the stored EEPROM version
        // and then write the new stuff to the EEPROM.
        EEPROM_Version = eeprom8bitGet (EEPROM_STORED_ITEM_MM_EEPROM_VERSION);
        if (EEPROM_Version != EEPROM_DATA_STRUCTURE_VERSION)
        {
            if (EEPROM_Version == 3)
            {
                // Need to add items since Version 3.
                //eeprom8bitSet (EEPROM_STORED_ITEM_MM_MINIMUM_DRIVE_OFFSET, 20); // Add minimum drive offset
                // The above got replaced by the following 3 pad minimum speed schema
                eeprom8bitSet (EEPROM_STORED_ITEM_MM_CENTER_PAD_MINIMUM_DRIVE_OFFSET, 20); // Add minimum drive offset
                EEPROM_Version = 4; // This allows any further updating to occur
            }
            if (EEPROM_Version == 4)
            {
                // We changed the one Minimum Drive Speed to Center Pad Minimum Speed
                // ... and we will leave it at the same value.
                // In fact, we will use this value as the default value for the
                // other 2 pads.
                MinimumDriveSpeed = eeprom8bitGet (EEPROM_STORED_ITEM_MM_CENTER_PAD_MINIMUM_DRIVE_OFFSET);
                eeprom8bitSet (EEPROM_STORED_ITEM_MM_LEFT_PAD_MINIMUM_DRIVE_OFFSET, MinimumDriveSpeed); // Add minimum drive offset
                eeprom8bitSet (EEPROM_STORED_ITEM_MM_RIGHT_PAD_MINIMUM_DRIVE_OFFSET, MinimumDriveSpeed); // Add minimum drive offset
                EEPROM_Version = 5;
            }
            if (EEPROM_Version == 5)
            {
                eeprom8bitSet (EEPROM_STORED_ITEM_ENABLED_FEATURES_2, 0); // create the space to hold the New Features, i.e. RNet Sleep and Mode Switch Schema
                EEPROM_Version = 6; // This allows any further updating to occur
            }
            if (EEPROM_Version == 6)
            {
                ; // Nothing to do
            }
            eeprom8bitSet (EEPROM_STORED_ITEM_MM_EEPROM_VERSION, EEPROM_DATA_STRUCTURE_VERSION);
    		eepromFlush(true);
        }
    }
    
	return eeprom_has_been_initialized;
}
#endif // #ifdef ASL110

//-------------------------------
// Function: eepromAppNumTimesAnyDataHasBeenUpdated
//
// Description: Let's caller know how many times any data has been updated since boot.
//		The value may roll over, this is intended. The point is to have other processes simply notice
//		when any data has been updated, ensuring that this value is checked quicker than this value may be updated >256 times.
//
//-------------------------------

#ifdef ASL110

uint8_t eepromAppNumTimesAnyDataHasBeenUpdated(void)
{
	return num_times_any_item_has_updated;
}

#endif // #ifdef ASL110

//-------------------------------
// Function: eepromFlush
//
// Description: Flushes values currently stored in RAM to flash. Only if required though.  No reason to needlessly
// 	write to EEPROM and wear it out.
//
//-------------------------------
#ifdef ASL110

void eepromFlush(bool force_save_all)
{
	if (force_save_all || at_least_one_item_requires_saving)
	{
		for (int item = (int)EEPROM_STORED_ITEM_EEPROM_INITIALIZED; item < (int)EEPROM_STORED_ITEM_EOL; item++)
		{
			ItemInfo_t *item_info = &items_info[item];

			if (item_info->need_to_save || force_save_all)
			{
 				item_info->need_to_save = false;

				switch (item_info->type)
				{
					case ITEM_TYPE_BOOL:
					case ITEM_TYPE_ENUM:
					case ITEM_TYPE_UINT8:
						Write8bitVal(item_info->start_addr, eeprom_data.bytes[item_info->start_addr]);
						break;
					
					case ITEM_TYPE_UINT16:
						Write16bitVal(item_info->start_addr, *((uint16_t *)&eeprom_data.bytes[item_info->start_addr]));
						break;
					
					case ITEM_TYPE_EOL:
					default:
						ASSERT(item_info->type == ITEM_TYPE_UINT16);
						break;
				}
			}
		}

		at_least_one_item_requires_saving = false;
	}
}
#endif // #ifdef ASL110

//-------------------------------
// Function: eepromBoolSet
//
// Description: Sets a boolean type item's value.
//
//-------------------------------
#ifdef ASL110

void eepromBoolSet(EepromItemId_t item_id, bool val)
{
	ItemInfo_t *item_info = &items_info[(int)item_id];
	ASSERT(item_info->type == ITEM_TYPE_BOOL);

	if (eeprom_data.bytes[item_info->start_addr] != (uint8_t)val)
	{
		at_least_one_item_requires_saving = true;
		item_info->need_to_save = true;
		eeprom_data.bytes[item_info->start_addr] = (uint8_t)val;
		num_times_any_item_has_updated++;
	}
}
#endif // #ifdef ASL110

//-------------------------------
// Function: eepromBoolGet
//
// Description: Gets a boolean type item's value.
//
//-------------------------------
#ifdef ASL110

bool eepromBoolGet(EepromItemId_t item_id)
{
	ASSERT(items_info[(int)item_id].type == ITEM_TYPE_BOOL);
	return (bool)eeprom_data.bytes[items_info[(int)item_id].start_addr];
}
#endif // #ifdef ASL110

//-------------------------------
// Function: eepromEnumSet
//
// Description: Sets an enumerated type item's value.
//
//-------------------------------
#ifdef ASL110

void eepromEnumSet(EepromItemId_t item_id, EepromStoredEnumType_t val)
{
	ItemInfo_t *item_info = &items_info[(int)item_id];
	ASSERT(item_info->type == ITEM_TYPE_ENUM);

	if (eeprom_data.bytes[item_info->start_addr] != (uint8_t)val)
	{
		at_least_one_item_requires_saving = true;
		item_info->need_to_save = true;
		eeprom_data.bytes[item_info->start_addr] = (uint8_t)val;
		num_times_any_item_has_updated++;
	}
}
#endif // #ifdef ASL110

//-------------------------------
// Function: eepromEnumGet
//
// Description: Gets an enumerated type item's value.
//
//-------------------------------
#ifdef ASL110

EepromStoredEnumType_t eepromEnumGet(EepromItemId_t item_id)
{
	ASSERT(items_info[(int)item_id].type == ITEM_TYPE_ENUM);
	return (EepromStoredEnumType_t)eeprom_data.bytes[items_info[(int)item_id].start_addr];
}
#endif // #ifdef ASL110

//-------------------------------
// Function: eeprom8bitSet
//
// Description: Sets an 8-bit type item's value.
//
//-------------------------------
#ifdef ASL110

void eeprom8bitSet(EepromItemId_t item_id, uint8_t val)
{
	ItemInfo_t *item_info = &items_info[(int)item_id];
	ASSERT(item_info->type == ITEM_TYPE_UINT8);

	if (eeprom_data.bytes[item_info->start_addr] != val)
	{
		at_least_one_item_requires_saving = true;
		item_info->need_to_save = true;
		eeprom_data.bytes[item_info->start_addr] = val;
		num_times_any_item_has_updated++;
	}
}
#endif // #ifdef ASL110

//-------------------------------
// Function: eeprom8bitGet
//
// Description: Gets an 8-bit type item's value.
//
//-------------------------------
#ifdef ASL110

uint8_t eeprom8bitGet(EepromItemId_t item_id)
{
	ASSERT(items_info[(int)item_id].type == ITEM_TYPE_UINT8);
	return (EepromStoredEnumType_t)eeprom_data.bytes[items_info[(int)item_id].start_addr];
}
#endif // #ifdef ASL110

//-------------------------------
// Function: eeprom16bitSet
//
// Description: Sets a 16-bit type item's value.
//
//-------------------------------
#ifdef ASL110

void eeprom16bitSet(EepromItemId_t item_id, uint16_t val)
{
	ItemInfo_t *item_info = &items_info[(int)item_id];
	ASSERT(item_info->type == ITEM_TYPE_UINT16);

	if (*((uint16_t *)&eeprom_data.bytes[item_info->start_addr]) != val)
	{
		at_least_one_item_requires_saving = true;
		item_info->need_to_save = true;
		*((uint16_t *)&eeprom_data.bytes[item_info->start_addr]) = val;
		num_times_any_item_has_updated++;
	}
}
#endif // #ifdef ASL110

//-------------------------------
// Function: eeprom16bitGet
//
// Description: Gets a 16-bit type item's value.
//
//-------------------------------
#ifdef ASL110

uint16_t eeprom16bitGet(EepromItemId_t item_id)
{
	ASSERT(items_info[(int)item_id].type == ITEM_TYPE_UINT16);
	return *((uint16_t *)&eeprom_data.bytes[items_info[(int)item_id].start_addr]);
}
#endif // #ifdef ASL110

/* ********************   Private Function Definitions   ****************** */

//-------------------------------
// Function: SyncWithEeprom
//
// Description: Synchronizes data in RAM with data in EEPROM.
//
// return: True if EEPROM has been written to before, false if it has not.  When true, all values in RAM
// 	are synced with the corresponding item value in EEPROM.
//
//-------------------------------

#ifdef ASL110

static bool SyncWithEeprom(void)
{
	uint8_t is_programmed_val = Read8bitVal(items_info[EEPROM_STORED_ITEM_EEPROM_INITIALIZED].start_addr);
	
	if (is_programmed_val == EEPROM_INITIALIZED_VAL)
	{
		eeprom_data.bytes[EEPROM_STORED_ITEM_EEPROM_INITIALIZED] = is_programmed_val;
		
		for (int item = (int)EEPROM_STORED_ITEM_EEPROM_INITIALIZED + 1; item < (int)EEPROM_STORED_ITEM_EOL; item++)
		{
			ItemInfo_t *item_info = &items_info[item];
			item_info->need_to_save = false;

			switch (item_info->type)
			{
				case ITEM_TYPE_BOOL:
				case ITEM_TYPE_ENUM:
				case ITEM_TYPE_UINT8:
					eeprom_data.bytes[item_info->start_addr] = Read8bitVal(item_info->start_addr);
					break;
				
				case ITEM_TYPE_UINT16:
					*((uint16_t *)&eeprom_data.bytes[item_info->start_addr]) = Read16bitVal(item_info->start_addr);
					break;
					
				case ITEM_TYPE_EOL:
				default:
					ASSERT(item_info->type == ITEM_TYPE_UINT16);
					break;
			}
		}

		return true;
	}
	else
	{
		return false;
	}
}
#endif // #ifdef ASL110

//-------------------------------
// Function: SetDefaultValues
//
// Description: Sets all data stored in RAM to default values.
//
// NOTE: When adding/removing an item, be sure to update this function as well!
//
//-------------------------------
#ifdef ASL110

void SetDefaultValues(void)
{
	eeprom_data.items.eeprom_intiailized = EEPROM_INITIALIZED_VAL;
	
	eeprom_data.items.left_pad_input_type = (EepromStoredEnumType_t)HEAD_ARR_INPUT_PROPORTIONAL;
	eeprom_data.items.right_pad_input_type = (EepromStoredEnumType_t)HEAD_ARR_INPUT_PROPORTIONAL;
	eeprom_data.items.center_pad_input_type = (EepromStoredEnumType_t)HEAD_ARR_INPUT_PROPORTIONAL;

	eeprom_data.items.left_pad_output_map = (EepromStoredEnumType_t)HEAD_ARRAY_OUT_FUNC_LEFT;
	eeprom_data.items.right_pad_output_map = (EepromStoredEnumType_t)HEAD_ARRAY_OUT_FUNC_RIGHT;
	eeprom_data.items.center_pad_output_map = (EepromStoredEnumType_t)HEAD_ARRAY_OUT_FUNC_FWD;

	eeprom_data.items.user_btn_long_press_act_time = 1000;
	
	// Enable all features.
	eeprom_data.items.enabled_features = //FUNC_FEATURE_POWER_ON_OFF_BIT_MASK; | 
                                        //FUNC_FEATURE_OUT_CTRL_TO_BT_MODULE_BIT_MASK;
                                        //FUNC_FEATURE_NEXT_FUNCTION_BIT_MASK | 
                                        //FUNC_FEATURE_NEXT_PROFILE_BIT_MASK |
                                        //FUNC_FEATURE_SOUND_ENABLED_BIT_MASK;
            0;
            
	eeprom_data.items.current_active_feature = (EepromStoredEnumType_t)FUNC_FEATURE_POWER_ON_OFF;

	eeprom_data.items.left_pad_min_adc_val = ADC_LEFT_PAD_MIN_VAL;
	eeprom_data.items.left_pad_max_adc_val = ADC_LEFT_PAD_MAX_VAL;
	eeprom_data.items.left_pad_min_thresh_perc = 2;
	eeprom_data.items.left_pad_max_thresh_perc = 30;

	eeprom_data.items.right_pad_min_adc_val = ADC_RIGHT_PAD_MIN_VAL;
	eeprom_data.items.right_pad_max_adc_val = ADC_RIGHT_PAD_MAX_VAL;
	eeprom_data.items.right_pad_min_thresh_perc = 2;
	eeprom_data.items.right_pad_max_thresh_perc = 30;

	eeprom_data.items.ctr_pad_min_adc_val = ADC_CTR_PAD_MIN_VAL;
	eeprom_data.items.ctr_pad_max_adc_val = ADC_CTR_PAD_MAX_VAL;
	eeprom_data.items.ctr_pad_min_thresh_perc = 2;
	eeprom_data.items.ctr_pad_max_thresh_perc = 30;

#ifdef USE_12VOLT_REGULATOR
    eeprom_data.items.neutral_DAC_counts = 2048+212;        // Mid-point of 12-bit DAC
    eeprom_data.items.neutral_DAC_setting = 2040+212;       // Mid-point of 12-bit DAC
#else
    eeprom_data.items.neutral_DAC_counts = 2048;        // Mid-point of 12-bit DAC
    eeprom_data.items.neutral_DAC_setting = 2032;       // Mid-point of 12-bit DAC
#endif
    eeprom_data.items.neutral_DAC_range = 410;          // Allowable range for 1.2 V swing.
    
    // Added in Version 3
    eeprom_data.items.EEPROM_Version = EEPROM_DATA_STRUCTURE_VERSION;
    
    // Added in Version 4
    // Obsoleted/replaced in Version 5
    //eeprom_data.items.Drive_Offset = 20;                // Default drive offset is 20%

    // Changed and/or added in Version 5
    eeprom_data.items.CenterPad_MinimumDriveSpeed = 20;     // Center Pad minimum speed
    eeprom_data.items.LeftPad_MinimumDriveSpeed = 20;       // Left Pad minimum speed
    eeprom_data.items.RightPad_MinimumDriveSpeed = 20;      // Right Pad minimum speed
    
    // Added in Version 6
    eeprom_data.items.enabled_features2 = 0;                // Feature set 2 is all disabled.
}
#endif // #ifdef ASL110

//-------------------------------
// Function: Write8bitVal
//
// Description: Writes a 8-bit value to EEPROM
//
// 		Function calls are computationally expensive, inlining this function in favor of increased speed.
//
//-------------------------------
#ifdef ASL110

inline static void Write8bitVal(uint8_t address, uint8_t new_val)
{
	bool op_success = eepromBspWriteByte(address, new_val, 0);
	ASSERT(op_success);
}
#endif // #ifdef ASL110

//-------------------------------
// Function: Read8bitVal
//
// Description: Reads a 8-bit value from EEPROM
//
// 		Function calls are computationally expensive, inlining this function in favor of increased speed.
//
//-------------------------------
#ifdef ASL110
inline static uint8_t Read8bitVal(uint8_t address)
{
	uint8_t read_val;
	bool op_success = eepromBspReadSection(address, 1, &read_val, 0);
	ASSERT(op_success);

	if (!op_success)
	{
		// Just set to a known value so there is little chance of total crash...though this case can currently not occur
		read_val = 0;
	}

	return read_val;
}
#endif // #ifdef ASL110

//-------------------------------
// Function: Write16bitVal
//
// Description: Writes a 16-bit value to EEPROM
//
// 		Function calls are computationally expensive, inlining this function in favor of increased speed.
//
//-------------------------------
#ifdef ASL110
inline static void Write16bitVal(uint8_t address, uint16_t new_val)
{
	TypeAccess16Bit_t write_val;
	write_val.val = new_val;
	bool op_success = eepromBspWriteBuffer(address, 2, write_val.bytes, 0);
	ASSERT(op_success);
}
#endif // #ifdef ASL110

//-------------------------------
// Function: Read16bitVal
//
// Description: Reads a 16-bit value from EEPROM
//
// 		Function calls are computationally expensive, inlining this function in favor of increased speed.
//
//-------------------------------
#ifdef ASL110
inline static uint16_t Read16bitVal(uint8_t address)
{
	TypeAccess16Bit_t read_val;
	bool op_success = eepromBspReadSection(address, 2, read_val.bytes, 0);
	ASSERT(op_success);

	if (!op_success)
	{
		ASSERT(op_success);
		read_val.val = 0; // Just set to a known value so there is little chance of total crash...
	}

	return read_val.val;
}
#endif // #ifdef ASL110

//-------------------------------
// Function: Write32bitVal
//
// Description: Writes a 32-bit value to EEPROM
//
// 		Function calls are computationally expensive, inlining this function in favor of increased speed.
//
//-------------------------------
#ifdef ASL110
inline static void Write32bitVal(uint8_t address, uint32_t new_val)
{
	TypeAccess32Bit_t write_val;
	write_val.val = new_val;
	bool op_success = eepromBspWriteBuffer(address, 4, write_val.bytes, 0);
	ASSERT(op_success);
}
#endif // #ifdef ASL110

//-------------------------------
// Function: Read32bitVal
//
// Description: Reads a 32-bit value from EEPROM
//
// 		Function calls are computationally expensive, inlining this function in favor of increased speed.
//
//-------------------------------
#ifdef ASL110
inline static uint32_t Read32bitVal(uint8_t address)
{
	TypeAccess32Bit_t read_val;
	bool op_success = eepromBspReadSection(address, 4, read_val.bytes, 0);
	ASSERT(op_success);

	if (!op_success)
	{
		read_val.val = 0; // Just set to a known value so there is little chance of total crash...
	}

	return read_val.val;
}
#endif // #ifdef ASL110

// end of file.
//-------------------------------------------------------------------------
