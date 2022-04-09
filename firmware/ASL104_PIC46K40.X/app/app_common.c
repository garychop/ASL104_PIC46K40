//////////////////////////////////////////////////////////////////////////////
//
// Filename: app_common.c
//
// Description: Provides generic/common application functionality
//
// Author(s): Trevor Parsh (Embedded Wizardry, LLC)
//
// Modified for ASL on Date: 
//
//////////////////////////////////////////////////////////////////////////////


/* **************************   Header Files   *************************** */

// NOTE: This must ALWAYS be the first include in a file.
#include "device.h"

// from RTOS
#include "cocoos.h"

// from stdlib
#include <stdint.h>
#include <stdbool.h>
#include "user_assert.h"

// from system

// from project
#include "rtos_task_priorities.h"
#include "eeprom_app.h"

// from local
#include "app_common.h"


/* ******************************   Macros   ****************************** */

// Execution rate for this module's task, in milliseconds.
#define SYS_SUPERVISOR_TASK_EXECUTION_RATE_ms (20)

/* ******************************   Types   ******************************* */



/* ***********************   File Scope Variables   *********************** */

static volatile bool device_is_active;
//static volatile bool device_in_calibration;
static bool g_BT_SequnceActive;

/* ***********************   Global Variables   *********************** */


/* ***********************   Function Prototypes   ************************ */

static void SystemSupervisorTask(void);
inline static void ManageEepromDataFlush(void);

/* *******************   Public Function Definitions   ******************** */

//-------------------------------
// Function: AppCommonInit
//
// Description: Initializes this module
//
//-------------------------------
void AppCommonInit(void)
{
    (void)task_create(SystemSupervisorTask , NULL, SYSTEM_SUPERVISOR_TASK_PRIO, NULL, 0, 0);
}

//-------------------------------
// Function: appCommonFeatureIsEnabled
//
// Description: Determines whether or not a given feature is enabled.
//
//-------------------------------
bool appCommonFeatureIsEnabled(FunctionalFeature_t feature)
{
#if 0
	uint8_t feature_mask;

	switch (feature)
	{
		case FUNC_FEATURE_POWER_ON_OFF:
			feature_mask = FUNC_FEATURE_POWER_ON_OFF_BIT_MASK;
			break;

		case FUNC_FEATURE_OUT_CTRL_TO_BT_MODULE:
			feature_mask = FUNC_FEATURE_OUT_CTRL_TO_BT_MODULE_BIT_MASK;
			break;

		case FUNC_FEATURE_OUT_NEXT_FUNCTION:
			feature_mask = FUNC_FEATURE_NEXT_FUNCTION_BIT_MASK;
			break;

        case FUNC_FEATURE_RNET_SEATING:
            feature_mask = FUNC_FEATURE_RNET_SEATING_MASK;
            break;

        // The following feature is set the 2nd feature byte, so we are making an "exception" to the mask setting.
        case FUNC_FEATURE2_RNET_SLEEP:
            return ((eeprom8bitGet (EEPROM_STORED_ITEM_ENABLED_FEATURES_2) & FUNC_FEATURE2_RNET_SLEEP_BIT_MASK) == FUNC_FEATURE2_RNET_SLEEP_BIT_MASK);
            break;
            
		case FUNC_FEATURE_OUT_NEXT_PROFILE:
		default:
			feature_mask = FUNC_FEATURE_NEXT_PROFILE_BIT_MASK;
			break;
	}

	return ((eeprom8bitGet(EEPROM_STORED_ITEM_ENABLED_FEATURES) & feature_mask) > 0);
#endif
    return (false);
}

//-------------------------------
// Function: appCommonSoundEnabled
//
// Description: Determines whether or not a sound/beeper is enabled.
//
//-------------------------------
bool appCommonSoundEnabled(void)
{
//	return ((eeprom8bitGet(EEPROM_STORED_ITEM_ENABLED_FEATURES) & FUNC_FEATURE_SOUND_ENABLED_BIT_MASK) > 0);
    return true;
}

//-------------------------------
// Function: appCommonGetCurrentFeature
// This functions returns an index value representing the active feature. 
//-------------------------------

FunctionalFeature_t appCommonGetCurrentFeature(void)
{
//    return (FunctionalFeature_t)eepromEnumGet(EEPROM_STORED_ITEM_CURRENT_ACTIVE_FEATURE);
    return (FunctionalFeature_t) 0;
}

//-------------------------------
// Function: appCommonGetPreviousEnabledFeature
//
// Description: Gets the previous feature in line given the currently active feature
//
//-------------------------------
FunctionalFeature_t appCommonGetPreviousEnabledFeature(void)
{
#if 0
	FunctionalFeature_t feature = (FunctionalFeature_t)eepromEnumGet(EEPROM_STORED_ITEM_CURRENT_ACTIVE_FEATURE);
	uint8_t numberFeaturesChecked;

    for (numberFeaturesChecked = 0; numberFeaturesChecked < FUNC_FEATURE_EOL; ++ numberFeaturesChecked)
	{
        // Wrap from lowest to highest.
        if (feature == 0)
            feature = (FunctionalFeature_t)((uint8_t)FUNC_FEATURE_EOL - 1); // Point at last feature.
        else
            --feature;

		if (appCommonFeatureIsEnabled(feature))
		{
			return feature;
		}
	}

	return (FunctionalFeature_t)eepromEnumGet(EEPROM_STORED_ITEM_CURRENT_ACTIVE_FEATURE);
#endif
    return (FunctionalFeature_t) 0;
}

//-------------------------------
// Function: appCommonGetNextFeature
//
// Description: Gets the next feature in line given the currently active feature
//
//-------------------------------
FunctionalFeature_t appCommonGetNextFeature (void)
{
#if 0
	FunctionalFeature_t next_feature = (FunctionalFeature_t)eepromEnumGet(EEPROM_STORED_ITEM_CURRENT_ACTIVE_FEATURE);
	uint8_t numberFeaturesChecked;

    for (numberFeaturesChecked = 0; numberFeaturesChecked < FUNC_FEATURE_EOL; ++ numberFeaturesChecked)
	{
		next_feature = (FunctionalFeature_t)((next_feature >= (FunctionalFeature_t)((uint8_t)FUNC_FEATURE_EOL - 1)) ? 0 : (uint8_t)next_feature + 1);

		if (appCommonFeatureIsEnabled(next_feature))
		{
			return next_feature;
		}
	}
#endif
	return FUNC_FEATURE_EOL;
}

//-------------------------------
// This sets the system var that indicates the Bluetooth module is
// being Enabled or Disabled and commands to the driver demands should be
// inhibit.
//-------------------------------

void AppCommonSetBTSequenceActive (bool active)   // true to indicate BT control is active.
{
    g_BT_SequnceActive = active;
}

bool AppCommonGetBTSequenceActive (void)
{
    return (g_BT_SequnceActive);
}

/* ********************   Private Function Definitions   ****************** */

//-------------------------------
// Function: SystemSupervisorTask
//
// Description: System monitoring duties.
//
//-------------------------------
static void SystemSupervisorTask(void)
{
    task_open();
	while (1)
	{
		task_wait(MILLISECONDS_TO_TICKS(SYS_SUPERVISOR_TASK_EXECUTION_RATE_ms));
		
	}
	task_close();
}

// end of file.
//-------------------------------------------------------------------------
