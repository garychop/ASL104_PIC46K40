//////////////////////////////////////////////////////////////////////////////
//
// Filename: led_app.c
//
// Description: Special app level control of the on-board LEDs.
//
// Author(s): Trevor Parsh (Embedded Wizardry, LLC)
//
// Modified for ASL on Date: 
//
//////////////////////////////////////////////////////////////////////////////

/* ***************************    Includes     **************************** */

// from stdlib
#include <stdbool.h>

// from project
#include "app_common.h"
#include "general_output_ctrl.h"
#include "general_output_ctrl_app.h"

// from local
#include "led_app.h"


/* *******************   Public Function Definitions   ******************** */

//-------------------------------
// Function: LedAppControlLeds
//
// Description: Given the very "hands-on" nature of controlling the LEDs, circumvent the normal control scheme
//      here for most LED control cases.
//
// return:  true: calling tasks needs to wake up general output controller to service event
//          false: calling task does not need to take any further action
//
// Design:
//   Amber light for Proportional
//   Power up – Amber solid
//   Power off – No light
//   Function next - Amber solid
//   Profile Next – Amber solid 
//   Bluetooth – Green flashing 
//    
//   Green light Digital
//   Power up – Green light solid
//   Power off – No light
//   Function Next – Green solid
//   Profile Next – Green solid
//   Bluetooth Green flashing.
//
//-------------------------------
bool LedAppControlLeds(void)
{
    bool ret_val = false;
    bool turn_led_on = false;
    
	FunctionalFeature_t curr_feature = AppCommonFeatureIsValid(false);
	if (curr_feature != FUNC_FEATURE_EOL)
	{
		switch (curr_feature)
		{
			case FUNC_FEATURE_POWER_ON_OFF:
                if (AppCommonDeviceActiveGet())
                {
                    turn_led_on = true;
                }
                else
                {
                    GenOutCtrl_Stop(GEN_OUT_CTRL_ID_LED0);
                    GenOutCtrl_Stop(GEN_OUT_CTRL_ID_LED1);
                    GenOutCtrl_SetInactive(GEN_OUT_CTRL_ID_LED0);
                    GenOutCtrl_SetInactive(GEN_OUT_CTRL_ID_LED1);
                }
				break;

			case FUNC_FEATURE_OUT_CTRL_TO_BT_MODULE:
                GenOutCtrl_Start(GEN_OUT_CTRL_ID_LED0);
                GenOutCtrl_Start(GEN_OUT_CTRL_ID_LED1);
				GenOutCtrlApp_SetStateAll(GEN_OUT_CTRL_STATE_BLUETOOTH_OUTPUT);
                ret_val = true;
				break;

			case FUNC_FEATURE_OUT_NEXT_FUNCTION:
                turn_led_on = true;
				break;

			case FUNC_FEATURE_OUT_NEXT_PROFILE:
                turn_led_on = true;
				break;

			default:
				break;
		}
	}

    // If we need to force an LED into an on state, then do so.
    if (turn_led_on)
    {
        // Stop automatic control so that the program may control the LEDs manually.
        GenOutCtrl_Stop(GEN_OUT_CTRL_ID_LED0);
        GenOutCtrl_Stop(GEN_OUT_CTRL_ID_LED1);
        
        // TODO: Actually figure out what mode we are in. Currently, there is no way of doing this and
        // TODO: how we do this is unknown.
        if (true)
        {
            // Pads are set to be read as proportional inputs
            GenOutCtrl_SetActive(GEN_OUT_CTRL_ID_LED0);
            GenOutCtrl_SetInactive(GEN_OUT_CTRL_ID_LED1);
        }
        else
        {
            GenOutCtrl_SetInactive(GEN_OUT_CTRL_ID_LED0);
            GenOutCtrl_SetActive(GEN_OUT_CTRL_ID_LED1);
        }
    }

    return ret_val;
}

// end of file.
//-------------------------------------------------------------------------
