//////////////////////////////////////////////////////////////////////////////
//
// Filename: app_common.h
//
// Description: Provides generic/common application functionality
//
// Author(s): Trevor Parsh (Embedded Wizardry, LLC)
//
// Modified for ASL on Date: 
//
//////////////////////////////////////////////////////////////////////////////

#ifndef APP_COMMON_H
#define APP_COMMON_H

/* ******************************   Macros   ****************************** */

// Bit masks pertaining to the REQUEST FEATURE SETTNG CMD HA<->HHP comms command.
#define FUNC_FEATURE_POWER_ON_OFF_BIT_MASK				(0x01)
#define FUNC_FEATURE_OUT_CTRL_TO_BT_MODULE_BIT_MASK		(0x02)
#define FUNC_FEATURE_NEXT_FUNCTION_BIT_MASK				(0x04)
#define FUNC_FEATURE_NEXT_PROFILE_BIT_MASK				(0x08)
#define FUNC_FEATURE_SOUND_ENABLED_BIT_MASK				(0x10)
#define FUNC_FEATURE_POWER_UP_IN_IDLE_BIT_MASK			(0x20)
#define FUNC_FEATURE_RNET_SEATING_MASK                  (0x80)

// Bit bask for Feature set 2
#define FUNC_FEATURE2_RNET_SLEEP_BIT_MASK               (0x01)
#define FUNC_FEATURE2_MODE_REVERSE_BIT_MASK             (0x02)

#define FUNC_FEATURE_ALL (FUNC_FEATURE_POWER_ON_OFF_BIT_MASK | FUNC_FEATURE_OUT_CTRL_TO_BT_MODULE_BIT_MASK | \
							FUNC_FEATURE_NEXT_FUNCTION_BIT_MASK | FUNC_FEATURE_NEXT_PROFILE_BIT_MASK | \
                            FUNC_FEATURE_SOUND_ENABLED_BIT_MASK | FUNC_FEATURE_RNET_SEATING_MASK)

/* ******************************   Types   ******************************* */

typedef enum
{
	//FUNC_FEATURE_POWER_ON_OFF,
    FUNC_FEATURE_DRIVING,
	FUNC_FEATURE_OUT_CTRL_TO_BT_MODULE,
	//FUNC_FEATURE_OUT_NEXT_FUNCTION,
	//FUNC_FEATURE_OUT_NEXT_PROFILE,
	//FUNC_FEATURE_RNET_SEATING,
    //FUNC_FEATURE2_RNET_SLEEP,
	// Nothing else may be defined past this point!
	FUNC_FEATURE_EOL
} FunctionalFeature_t;

/* ***********************   Function Prototypes   ************************ */

void AppCommonInit(void);
bool appCommonFeatureIsEnabled(FunctionalFeature_t feature);
bool appCommonSoundEnabled(void);
//bool appCommonIsPowerUpInIdleEnabled (void);
FunctionalFeature_t appCommonGetNextFeature(void);
FunctionalFeature_t appCommonGetCurrentFeature(void);
FunctionalFeature_t appCommonGetPreviousEnabledFeature(void);

void AppCommonDeviceActiveSet(bool is_active);
bool AppCommonDeviceActiveGet(void);
void AppCommonForceActiveState (bool is_active);

//void AppCommonCalibrationActiveSet(bool put_into_calibration);
//bool AppCommonCalibrationActiveGet(void);

#endif // APP_COMMON_H

// end of file.
//-------------------------------------------------------------------------
