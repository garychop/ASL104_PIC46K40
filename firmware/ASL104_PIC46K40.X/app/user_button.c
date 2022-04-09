//////////////////////////////////////////////////////////////////////////////
//
// Filename: user_button.c
//
// Description: App level control and feedback for the user button.
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

// from project
#include "rtos_task_priorities.h"
#include "beeper.h"
#include "stopwatch.h"
#include "eeprom_app.h"
#include "app_common.h"
#include "general_output_ctrl_app.h"

// from local
#include "user_button_bsp.h"
#include "user_button.h"

/* ******************************   Macros   ****************************** */

// Amount of time, in milliseconds, to wait from the time a button press action is signaled
// to the time that we can start monitoring the button again. The purpose is to prevent
// "double-tapping" from occurring.
#define USER_BTN_hold_off_monitoring_PERIOD_ms (100)

//#define USER_SWITCH 0x01
//#define MODE_SWITCH 0x02

// This is used as a State Engine of sorts.
typedef enum BUTTUN_STATE_ENUM
{
    INIT_BUTTON_STATE = 0,
    WAIT_FOR_ACITVE_SWTICH,
    DEBOUCE_SWITCH,
    PROCESS_ACTIVE_USER_SWTICH,
    PROCESS_ACTIVE_MODE_SWTICH,
    WAIT_LONGER,
    WAIT_FOR_NO_SWITCHES
} BUTTUN_STATE_E;

/* ***********************   File Scope Variables   *********************** */

static volatile UserButtonPress_t curr_thresh_hit;
BUTTUN_STATE_E g_ButtonState = 0;
uint8_t g_ButtonPattern = 0;

// Protects access to critical sections of code in this module
static volatile Sem_t data_lock_mutex;

// Variables used by the button monitor.
static volatile bool hold_off_monitoring = false;
static StopWatch_t btn_mon_stopwatch = {0, false};

// NOTE: Must match UserButtonPress_t exactly
// Default values are set in eeprom_app.c
static volatile uint16_t time_for_func_to_trigger_ms[] =
{
//	BTN_ACTION_NONE	USER_BTN_PRESS_SHORT	USER_BTN_PRESS_LONG
	0,				0,						0
};

// NOTE: Must match UserButtonPress_t exactly
static volatile BeepPattern_t beep_pattern_for_press_type[] =
{
//	USER_BTN_PRESS_NONE,		USER_BTN_PRESS_SHORT					USER_BTN_PRESS_LONG
	BEEPER_PATTERN_EOL,			BEEPER_PATTERN_USER_BUTTON_SHORT_PRESS,	BEEPER_PATTERN_USER_BUTTON_LONG_PRESS
};

/* ***********************   Function Prototypes   ************************ */

static void UserButtonMonitorTask (void);
static void ResetButtonMonitoring(void);
static void CarryOutShortPressAction(void);
static void CarryOutLongPressAction(void);
static FunctionalFeature_t FeatureIsValid(bool set_to_next);
//uint8_t GetSwitchStatus(void);

/* *******************   Public Function Definitions   ******************** */

//-------------------------------
// Function: userButtonInit
//
// Description: Initializes this module.
//
//-------------------------------
void userButtonInit(void)
{
	ButtonBspInit();

    g_ButtonState = INIT_BUTTON_STATE;
    // Check for buttons to be active. If so, beep an error and enter
    // then wait for all buttons to be released. This catches a "stuck button".
    if (GetSwitchStatus())
    {
        g_ButtonState = WAIT_FOR_NO_SWITCHES;  // Go to the state where we are looking for no switches
    }

    
	curr_thresh_hit = USER_BTN_PRESS_NONE;
    data_lock_mutex = sem_bin_create(1); // Set up so the first task to try and take the semaphore succeeds
	
    // People were indicating that the switch response is sluggish. I reduced the time from 100 to 50 and
    // it is much more responsive and still provides sufficient debounce time.
	time_for_func_to_trigger_ms[(int)USER_BTN_PRESS_SHORT] = 50; // 100;
	time_for_func_to_trigger_ms[(int)USER_BTN_PRESS_LONG] = 1500; // eeprom16bitGet(EEPROM_STORED_ITEM_USER_BTN_LONG_PRESS_ACT_TIME);

    (void)task_create(UserButtonMonitorTask , NULL, USER_BTN_MGMT_TASK_PRIO, NULL, 0, 0);
}

/* ********************   Private Function Definitions   ****************** */

//-------------------------------
// Function: GetSwitchStatus
// Description: Get the status of the switches where:
//      D0 represents the User Port switch
//      D1 represents the Mode Port switch.
// Returns: a byte as represented above.
//-------------------------------

uint8_t GetSwitchStatus(void)
{
    uint8_t mySwitches = 0;
    
    if (userButtonBspIsActive())
        mySwitches |= USER_SWITCH;
    if (ModeButtonBspIsActive())
        mySwitches |= MODE_SWITCH;
    
    return mySwitches;
}

//-------------------------------
// Function: userButtonMonitorTick
//
// Description: Button monitoring process.
//
//-------------------------------

static void UserButtonMonitorTask (void)
{
    Evt_t event_to_send_beeper_task;
    uint8_t currentFeature;
    uint8_t currentButtonPattern = 0;
    uint8_t feature2;

    task_open();

    while (1)
    {
#if 0
        feature2 = eeprom8bitGet (EEPROM_STORED_ITEM_ENABLED_FEATURES_2);
#endif
        // Get the Long Press time each cycle to ensure that if it's changed during programming, it is used before power cycle.
    	time_for_func_to_trigger_ms[(int)USER_BTN_PRESS_LONG] = 1500; // eeprom16bitGet(EEPROM_STORED_ITEM_USER_BTN_LONG_PRESS_ACT_TIME);
        currentButtonPattern = GetSwitchStatus();   // Get current switch pattern.

        // State INIT_BUTTON_STATE ---------------------------------------------------
        // Just reset/stop the timer then change to look for switch activation.
        if (g_ButtonState == INIT_BUTTON_STATE)
        {
            // Initialize state
            stopwatchStop(&btn_mon_stopwatch);      // Stop the timer
            g_ButtonState = WAIT_FOR_ACITVE_SWTICH;          // Goto wait for button to be pushed
        }
        // State WAIT_FOR_ACITVE_SWTICH ---------------------------------------------------
        // Look for switch or switches to become active.
        else if (g_ButtonState == WAIT_FOR_ACITVE_SWTICH)
        {
            // Ready state. We're waiting for a button to be pushed.
            if (currentButtonPattern)       // Non-Zero means a button is pressed.
            {
                g_ButtonPattern = currentButtonPattern;   // Save for later evaluation,
                stopwatchStart(&btn_mon_stopwatch);
                g_ButtonState = DEBOUCE_SWITCH;
            }
        }
        // State DEBOUCE_SWITCH ---------------------------------------------------
        // At least one button/switch is active, let's debounce it.
        else if (g_ButtonState == DEBOUCE_SWITCH)
        {
            // Button is pushed, we're going to debounce.
            if (currentButtonPattern == g_ButtonPattern)
            {
                if (stopwatchTimeElapsed (&btn_mon_stopwatch, false) >= time_for_func_to_trigger_ms[USER_BTN_PRESS_SHORT]) // debounce
                {
                    // Yep, we waited long enough,
                    stopwatchZero (&btn_mon_stopwatch); // Restart the timer
                    // Beep that the button was pushed.
                    if (currentButtonPattern & USER_SWITCH)
                    {
                        event_to_send_beeper_task = beeperBeep(BEEPER_PATTERN_USER_BUTTON_SHORT_PRESS);
                        if (event_to_send_beeper_task != NO_EVENT)
                        {
                            event_signal(event_to_send_beeper_task);
                        }
                        g_ButtonState = PROCESS_ACTIVE_USER_SWTICH;
                    }
                    else if (currentButtonPattern & MODE_SWITCH)
                    {
                        // To make it beep, uncomment the following lines.
                        //event_to_send_beeper_task = beeperBeep(BEEPER_PATTERN_MODE_ACTIVE);
                        //if (event_to_send_beeper_task != NO_EVENT)
                        //{
                        //    event_signal(event_to_send_beeper_task);
                        //}
                        // We added "Mode Switch means Reverse" feature. If it's active, don't active the Mode Pin on the 9-Pin DSub
//                        if ((feature2 & FUNC_FEATURE2_MODE_REVERSE_BIT_MASK) == 0)
                            GenOutCtrlApp_SetStateAll(GEN_OUT_CTRL_STATE_MODE_ACTIVE);
                        g_ButtonState = PROCESS_ACTIVE_MODE_SWTICH;
                    }
                }
            }
            else // Button was released before debounce time expired.
            {
                g_ButtonState = WAIT_FOR_NO_SWITCHES;  // I don't like when thing change!
            }
        }
        // State PROCESS_ACTIVE_USER_SWTICH ---------------------------------------------------
        else if (g_ButtonState == PROCESS_ACTIVE_USER_SWTICH)
        {
            // Wait for the timer to reach the long press. If it doesn't
            // then perform the Short Press operation.
            if (currentButtonPattern & USER_SWITCH) // I'm looking only at the User Switch.
            {
                if (stopwatchTimeElapsed (&btn_mon_stopwatch, false) >= time_for_func_to_trigger_ms[USER_BTN_PRESS_LONG])
                {
                    // Change to next function
                    if (1) // (AppCommonDeviceActiveGet())     // Is this thing active?
                    {
                        CarryOutLongPressAction();
                        // Beep that a feature change is happening
                        currentFeature = appCommonGetCurrentFeature();
                        event_to_send_beeper_task = beeperBeep ((BeepPattern_t) currentFeature); // (BEEPER_PATTERN_USER_BUTTON_LONG_PRESS);
                        if (event_to_send_beeper_task != NO_EVENT)
                        {
                            event_signal(event_to_send_beeper_task);
                        }
                    }
                    g_ButtonState = WAIT_FOR_NO_SWITCHES;
                }

            }
            else // button was released.
            {
                CarryOutShortPressAction();
                event_signal(genOutCtrlAppWakeEvent());
                g_ButtonState = WAIT_FOR_NO_SWITCHES;
            }
        }
        // State PROCESS_ACTIVE_MODE_SWTICH ---------------------------------------------------
        else if (g_ButtonState == PROCESS_ACTIVE_MODE_SWTICH)
        {
            // Wait for the Mode Switch to be released.
            if ((currentButtonPattern & MODE_SWITCH) == 0)
            {
                // We added a "Mode Swith is Reverse" feature. If it's active, don't affect the Mode Pin.
//                if ((feature2 & FUNC_FEATURE2_MODE_REVERSE_BIT_MASK) == 0)
                    GenOutCtrlApp_SetStateAll(GEN_OUT_CTRL_STATE_MODE_INACTIVE);
                // To annunciate Mode Switch release, uncomment the following.
                //event_to_send_beeper_task = beeperBeep (BEEPER_PATTERN_USER_BUTTON_SHORT_PRESS); // (BEEPER_PATTERN_USER_BUTTON_LONG_PRESS);
                //if (event_to_send_beeper_task != NO_EVENT)
                //{
                //    event_signal(event_to_send_beeper_task);
                //}
                g_ButtonState = WAIT_FOR_NO_SWITCHES;
            }
        }
        // State WAIT_FOR_NO_SWITCHES ---------------------------------------------------
        else if (g_ButtonState == WAIT_FOR_NO_SWITCHES)
        {
            // wait for No button
            if (currentButtonPattern)    // Nope the switch became active
            {
                stopwatchZero (&btn_mon_stopwatch); // Restart the timer
            }
            else
            {
                if (stopwatchTimeElapsed (&btn_mon_stopwatch, false) >= time_for_func_to_trigger_ms[0]) // debounce
                {
                    g_ButtonState = INIT_BUTTON_STATE; // essentially, start from scratch.
                    stopwatchStop(&btn_mon_stopwatch);      // Stop the timer
                }
            }
        }

		task_wait(MILLISECONDS_TO_TICKS (10));

    }  // End while

    task_close();
}

//-------------------------------
// Function: HoldOffOnButtonMonitoring
//
// Description: Has monitoring reset and wait for a small amount of time to start monitoring again.
//
//-------------------------------
static void ResetButtonMonitoring(void)
{
	curr_thresh_hit = USER_BTN_PRESS_NONE;
	hold_off_monitoring = true;
	stopwatchZero(&btn_mon_stopwatch);
}

//-------------------------------
// Function: CarryOutShortPressAction
//
// Description: Carries out an action based on a short button press being detected.
//
//-------------------------------
static void CarryOutShortPressAction(void)
{
#if 0
	FunctionalFeature_t feature = FeatureIsValid(false);
	if (feature != FUNC_FEATURE_EOL)
	{
		switch (feature)
		{
			case FUNC_FEATURE_POWER_ON_OFF:
				AppCommonForceActiveState(AppCommonDeviceActiveGet() ? false : true);
				break;

			case FUNC_FEATURE_OUT_CTRL_TO_BT_MODULE:
                if (AppCommonDeviceActiveGet() == false)    // This occurs if Power Up In Idle is on and
                                                            // .. we just turned on the system.
                    AppCommonForceActiveState(true);
                else
				{
                    // Nothing to do here.
                }
				break;

			case FUNC_FEATURE_OUT_NEXT_FUNCTION:
                if (AppCommonDeviceActiveGet() == false)    // This occurs if Power Up In Idle is on and
                                                            // .. we just turned on the system.
                    AppCommonForceActiveState(true);
                else
                    GenOutCtrlApp_SetStateAll(GEN_OUT_CTRL_STATE_STATE_USER_BTN_NEXT_FUNCTION);
				break;

			case FUNC_FEATURE_OUT_NEXT_PROFILE:
                if (AppCommonDeviceActiveGet() == false)    // This occurs if Power Up In Idle is on and
                                                            // .. we just turned on the system.
                    AppCommonForceActiveState(true);
                else
    				GenOutCtrlApp_SetStateAll(GEN_OUT_CTRL_STATE_USER_BTN_NEXT_PROFILE);
				break;

       		case FUNC_FEATURE_RNET_SEATING:
                if (AppCommonDeviceActiveGet() == false)    // This occurs if Power Up In Idle is on and
                                                            // .. we just turned on the system.
                    AppCommonForceActiveState(true);
                else
    			{
                   //eepromEnumSet(EEPROM_STORED_ITEM_CURRENT_ACTIVE_FEATURE, FUNC_FEATURE_POWER_ON_OFF);
                    feature = appCommonGetPreviousEnabledFeature();
                    eepromEnumSet(EEPROM_STORED_ITEM_CURRENT_ACTIVE_FEATURE, (EepromStoredEnumType_t)feature);
                }
				break;

            case FUNC_FEATURE2_RNET_SLEEP:
                if (AppCommonDeviceActiveGet() == false)    // This occurs if Power Up In Idle is on and
                                                            // .. we just turned on the system.
                    AppCommonForceActiveState(true);
                else
    			{
    				GenOutCtrlApp_SetStateAll(GEN_OUT_CTRL_RNET_SLEEP);
                }
                break;
                
			default:
				break;
		}
	}
#endif
}

//-------------------------------
// Function: CarryOutLongPressAction
//
// Description: Carries out an action based on a long button press being detected.
//
//-------------------------------
static void CarryOutLongPressAction(void)
{
#if 0
	// When powered off, the ONLY thing that the system does is respond to HHP requests and
	// short button presses.
	if (AppCommonDeviceActiveGet())
	{
		(void)FeatureIsValid(true); // This sets a global variable and the EEPROM parameter
                                    // .. to the next available feature.
	}
#endif
}

//------------------------------
// Function: IsModeSwtichActive
//
// Description: This function returns "true" if the Mode Switch is determined 
//  to be active based upon the button state engine processing.
//------------------------------

bool IsModeSwitchActive(void)
{
    return (g_ButtonState == PROCESS_ACTIVE_MODE_SWTICH);
}

//-------------------------------
// Function: FeatureIsValid
//
// Description: Determines whether or not the current/next control feature is valid.
//
// Other tasks will act on changes made here. E.g. the head array controller may
// change where it is physically outputting signals.
//
//-------------------------------
static FunctionalFeature_t FeatureIsValid(bool set_to_next)
{
	FunctionalFeature_t curr_active_feature = (FunctionalFeature_t)0; // eepromEnumGet(EEPROM_STORED_ITEM_CURRENT_ACTIVE_FEATURE);
	
//	if (set_to_next || !appCommonFeatureIsEnabled(curr_active_feature))
//	{
//		// Either forcing to change to next feature, or feature set has changed and we need to correct the invalid system state.
//		curr_active_feature = appCommonGetNextFeature();
//		eepromEnumSet(EEPROM_STORED_ITEM_CURRENT_ACTIVE_FEATURE, (EepromStoredEnumType_t)curr_active_feature);
//	}

	// This will already be of value FUNC_FEATURE_EOL if we do not have an available feature to use.
	return curr_active_feature;
}

// end of file.
//-------------------------------------------------------------------------
