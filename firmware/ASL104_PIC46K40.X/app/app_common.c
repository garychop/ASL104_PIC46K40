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
#include "inc/beeper.h"

/* ******************************   Macros   ****************************** */

// Execution rate for this module's task, in milliseconds.
#define SYS_SUPERVISOR_TASK_EXECUTION_RATE_ms (20)

/* ******************************   Types   ******************************* */



/* ***********************   File Scope Variables   *********************** */

static volatile bool device_is_active;
//static volatile bool device_in_calibration;

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
    device_is_active = false;
    
    //device_in_calibration = false;

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
//	uint8_t feature_mask;
//
//	switch (feature)
//	{
//		case FUNC_FEATURE_POWER_ON_OFF:
//			feature_mask = FUNC_FEATURE_POWER_ON_OFF_BIT_MASK;
//			break;
//
//		case FUNC_FEATURE_OUT_CTRL_TO_BT_MODULE:
//			feature_mask = FUNC_FEATURE_OUT_CTRL_TO_BT_MODULE_BIT_MASK;
//			break;
//
//		case FUNC_FEATURE_OUT_NEXT_FUNCTION:
//			feature_mask = FUNC_FEATURE_NEXT_FUNCTION_BIT_MASK;
//			break;
//
//        case FUNC_FEATURE_RNET_SEATING:
//            feature_mask = FUNC_FEATURE_RNET_SEATING_MASK;
//            break;
//
//		case FUNC_FEATURE_OUT_NEXT_PROFILE:
//		default:
//			feature_mask = FUNC_FEATURE_NEXT_PROFILE_BIT_MASK;
//			break;
//	}

//	return ((eeprom8bitGet(EEPROM_STORED_ITEM_ENABLED_FEATURES) & feature_mask) > 0);
    return 0;
}

//-------------------------------
// Function: appCommonSoundEnabled
//
// Description: Determines whether or not a sound/beeper is enabled.
//
//-------------------------------
bool appCommonSoundEnabled(void)
{
    return IsBeepEnabled();
}

//-------------------------------
// Function: appCommonIsPowerUpInIdleEnabled
//
// Description: Evaluates the Power Up In Idle feature.
// Returns: true if Power Up In Idle feature is enabled else false.
//-------------------------------
#ifdef ASL110
bool appCommonIsPowerUpInIdleEnabled (void)
{
	//return ((eeprom8bitGet(EEPROM_STORED_ITEM_ENABLED_FEATURES) & FUNC_FEATURE_POWER_UP_IN_IDLE_BIT_MASK) > 0);
    return false;
}
#endif

//-------------------------------
// Function: appCommonGetCurrentFeature
// This functions returns an index value representing the active feature. 
//-------------------------------

FunctionalFeature_t appCommonGetCurrentFeature(void)
{
    //return (FunctionalFeature_t)eepromEnumGet(EEPROM_STORED_ITEM_CURRENT_ACTIVE_FEATURE);
    return (FunctionalFeature_t)0;
}

//-------------------------------
// Function: appCommonGetPreviousEnabledFeature
//
// Description: Gets the previous feature in line given the currently active feature
//
//-------------------------------
FunctionalFeature_t appCommonGetPreviousEnabledFeature(void)
{
    FunctionalFeature_t feature = FUNC_FEATURE_DRIVING;
    
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
    return feature;
}

//-------------------------------
// Function: appCommonGetNextFeature
//
// Description: Gets the next feature in line given the currently active feature
//
//-------------------------------
FunctionalFeature_t appCommonGetNextFeature (void)
{
	FunctionalFeature_t next_feature = FUNC_FEATURE_DRIVING;
	uint8_t numberFeaturesChecked;

    for (numberFeaturesChecked = 0; numberFeaturesChecked < FUNC_FEATURE_EOL; ++ numberFeaturesChecked)
	{
		next_feature = (FunctionalFeature_t)((next_feature >= (FunctionalFeature_t)((uint8_t)FUNC_FEATURE_EOL - 1)) ? 0 : (uint8_t)next_feature + 1);

		if (appCommonFeatureIsEnabled(next_feature))
		{
			return next_feature;
		}
	}

	return FUNC_FEATURE_EOL;
}

//-------------------------------
// Function: AppCommonForceActiveState
//
// Description: Sets the 'active/enabled' state of the system.
//
//-------------------------------
void AppCommonForceActiveState (bool is_active)
{
    device_is_active = is_active;
}

//-------------------------------
// Function: AppCommonDeviceActiveSet
//
// Description: Sets the 'active/enabled' state of the system.
//		When inactive, all outputs are shutoff (sound, control, etc).
//      If going from Inactive to Active, the Neutral Test is performed.
//-------------------------------
void AppCommonDeviceActiveSet(bool is_active)
{
    // Determine if we need to check for Neutral.
//    if (device_is_active == false)      // We are NOT active
//    {
//        if (is_active)                  // We are going to ACTIVE
//            SetNeedForNeutralTest ();   // Tell the Head Array task to perform a Neutral Test
//    }
	device_is_active = is_active;

}

//-------------------------------
// Function: AppCommonDeviceActiveGet
//
// Description: Gets the 'active/enabled' state of the system.
//		When inactive, all outputs are shutoff (sound, control, etc).
//
//-------------------------------
bool AppCommonDeviceActiveGet(void)
{
	return device_is_active;
}

//-------------------------------
// Function: AppCommonCalibrationActiveSet
//
// Description: Sets the 'calibration/normal' state of the system.
//		When true, all outputs are shutoff (sound, control, etc).
//
//-------------------------------
//void AppCommonCalibrationActiveSet(bool put_into_calibration)
//{
//	//device_in_calibration = put_into_calibration;
//}

//-------------------------------
// Function: AppCommonCalibrationActiveGet
//
// Description: Gets the 'calibration/normal' state of the system.
//		When true, all outputs are shutoff (sound, control, etc).
//
//-------------------------------
//bool AppCommonCalibrationActiveGet(void)
//{
//	//return device_in_calibration;
//}

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
		
		//ManageEepromDataFlush();
	}
	task_close();
}

//-------------------------------
// Function: ManageEepromDataFlush
//
// Description: Monitor EEPROM and flush changes, stored in RAM, to EEPROM after waiting for a user(s) to
// 		finish updating all values of interest to them to help manage wear on the EEPROM.
//
//-------------------------------
#ifdef ASL110

inline static void ManageEepromDataFlush(void)
{
// This is the time to wait for new updates to persistent data (stored in RAM) before flushing to the EEPROM.
// This is to reduce wear on EEPROM.
#define TIME_TO_WAIT_BEFORE_EEPROM_FLUSH_ms ((uint32_t)10 * (uint32_t)1000)
#define TIME_TO_WAIT_BEFORE_EEPROM_FLUSH_PERIODS (uint16_t)(TIME_TO_WAIT_BEFORE_EEPROM_FLUSH_ms / (uint32_t)SYS_SUPERVISOR_TASK_EXECUTION_RATE_ms)

	static uint16_t num_periods_before_eeprom_flush = 0;
	static uint8_t num_times_any_item_has_updated = 0;

	if (num_times_any_item_has_updated != eepromAppNumTimesAnyDataHasBeenUpdated())
	{
		num_times_any_item_has_updated = eepromAppNumTimesAnyDataHasBeenUpdated();
		num_periods_before_eeprom_flush = TIME_TO_WAIT_BEFORE_EEPROM_FLUSH_PERIODS;
	}
	else if (num_periods_before_eeprom_flush > 0)
	{
		num_periods_before_eeprom_flush--;

		if (num_periods_before_eeprom_flush == 0)
		{
			eepromFlush(false);
		}
	}
}
#endif // #ifdef ASL110

// end of file.
//-------------------------------------------------------------------------
