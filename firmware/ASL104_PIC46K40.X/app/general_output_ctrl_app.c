/*
    MIT License

    Copyright (c) 2018 Trevor Parsh

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
 */

/**
 **************************************************************************************************
 *
 * @brief       App level general output control
 * @file        general_output_ctrl_app.c
 * @author      tparsh
 * @ingroup     APP
 *
 **************************************************************************************************
 */

/**
 * @addtogroup APP
 * @{
 */

/*
 *************************************************************************************************
 *                                           INCLUDE FILES
 *************************************************************************************************
 */

// NOTE: This must ALWAYS be the first include in a file.
#include "device.h"

/********************************************** System *******************************************/
#include <stdbool.h>
#include <stdint.h>
#include "user_assert.h"

/********************************************    RTOS   ******************************************/
#include "cocoos.h"

/********************************************    User   ******************************************/
#include "rtos_task_priorities.h"
#include "common.h"
#include "bsp.h"
#include "app_common.h"
#include "stopwatch.h"
#include "general_output_ctrl.h"
#include "general_output_ctrl_app.h"
#include "eeprom_app.h"

/*
 **************************************************************************************************
 *                                        DEFINES
 **************************************************************************************************
 */
/************************************ Symbolic Constants **********************************/
#define MIN_TIME_FOR_SUBSTATE_ms (GENERAL_OUTPUT_CTRL_UPDATE_RATE_ms)

#define STATE_CHANGE_REQ_CIRC_BUF_NUM_SLOTS (3)

/*
 **************************************************************************************************
 *                                       DATA TYPES
 **************************************************************************************************
 */

/**
 * Structure used to store general output controller state requests.
 */
typedef struct
{
    bool set_all;
    GenOutCtrlId_t id;
    GenOutState_t state;
} StateCtrl_t;

/*
 **************************************************************************************************
 *                                       VARIABLES
 **************************************************************************************************
 */

#ifdef OK_TO_USE_CONTROL_SCHEME

static volatile unsigned int state_change_req_circ_buf_pos_head = 0;
static volatile unsigned int state_change_req_circ_buf_pos_tail = 0;
static volatile unsigned int state_change_req_circ_buf_slots_in_use = 0;
static StateCtrl_t state_change_req_circ_buf[STATE_CHANGE_REQ_CIRC_BUF_NUM_SLOTS];

/// @brief Event that wakes up this module's task if it needs to to something useful.
static volatile Evt_t os_event_wake_task_id;

static GenOutCtrlStateStepDef_t control_disabled_in_state[] =
{
    GEN_OUT_CTRL_END_OF_STATE
};

static bool g_BT_State = GPIO_LOW;

// LED state processing definitions
static GenOutCtrlStateStepDef_t LED_Steady_Off_StateDefinition[] =
{
    GEN_OUT_CTRL_ALWAYS_OFF,        // Turn off LED
    GEN_OUT_CTRL_END_OF_STATE
};
static GenOutCtrlStateStepDef_t LED_Steady_On_StateDefinition[] =
{
    GEN_OUT_CTRL_ALWAYS_ON,        // Turn on LED
    GEN_OUT_CTRL_END_OF_STATE
};


/******************************************* LED0 ****************************************/
// LED0 ON = Green color
//
// GEN_OUT_CTRL_STATE_BLUETOOTH_OUTPUT
static GenOutCtrlStateStepDef_t led0_state_BLUETOOTH_OUTPUT[] =
{
    //{250, 750, GEN_OUT_CTRL_REPEAT_PATTERN_FOREVER_VAL}, // This blinks the LED green.
    GEN_OUT_CTRL_ALWAYS_OFF,        // Turn off Green LED
    GEN_OUT_CTRL_END_OF_STATE
};

// GEN_OUT_CTRL_STATE_HEAD_ARRAY_ACTIVE
static GenOutCtrlStateStepDef_t led0_state_HEAD_ARRAY_ACTIVE[] =
{
    GEN_OUT_CTRL_ALWAYS_ON,         // Turn ON Green LED
    GEN_OUT_CTRL_END_OF_STATE
};

// GEN_OUT_CTRL_STATE_NO_OUTPUT
static GenOutCtrlStateStepDef_t led0_state_NO_OUTPUT[] =
{
    GEN_OUT_CTRL_ALWAYS_OFF,
    GEN_OUT_CTRL_END_OF_STATE
};

// GEN_OUT_STATE_CTRL_TEST
// control_disabled_in_state

/******************************************** LED1 *******************************************/
// LED1 ON = Amber color of the single LED.
//
// GEN_OUT_CTRL_STATE_BLUETOOTH_OUTPUT
// control_disabled_in_state

// GEN_OUT_CTRL_STATE_BLUETOOTH_OUTPUT
static GenOutCtrlStateStepDef_t led1_state_BLUETOOTH_OUTPUT[] =
{
    //{250, 750, GEN_OUT_CTRL_REPEAT_PATTERN_FOREVER_VAL}, // This blinks LED orange.
    GEN_OUT_CTRL_ALWAYS_ON,     // Turn on Orange LED
    GEN_OUT_CTRL_END_OF_STATE
};

// GEN_OUT_CTRL_STATE_HEAD_ARRAY_ACTIVE
static GenOutCtrlStateStepDef_t led1_state_HEAD_ARRAY_ACTIVE[] =
{
    GEN_OUT_CTRL_ALWAYS_OFF,    // Turn off Orange LED
    GEN_OUT_CTRL_END_OF_STATE
};

// GEN_OUT_CTRL_STATE_NO_OUTPUT
// control_disabled_in_state

// GEN_OUT_STATE_CTRL_TEST
// control_disabled_in_state

/**************************** Internal System Action (reset line) ****************************/
// GEN_OUT_CTRL_STATE_HEAD_ARRAY_RESETTING
static GenOutCtrlStateStepDef_t int_sys_act_HEAD_ARRAY_RESETTING[] =
{
    { 100, MIN_TIME_FOR_SUBSTATE_ms, 1 },
    GEN_OUT_CTRL_END_OF_STATE
};

// GEN_OUT_CTRL_STATE_STATE_USER_BTN_NEXT_FUNCTION
static GenOutCtrlStateStepDef_t int_sys_act_STATE_USER_BTN_NEXT_FUNCTION[] =
{
    { 500, MIN_TIME_FOR_SUBSTATE_ms, 1 },
    GEN_OUT_CTRL_END_OF_STATE
};

// GEN_OUT_CTRL_STATE_USER_BTN_NEXT_PROFILE
static GenOutCtrlStateStepDef_t int_sys_act_STATE_USER_BTN_NEXT_PROFILE[] =
{
    { 1500, MIN_TIME_FOR_SUBSTATE_ms, 1 },
    GEN_OUT_CTRL_END_OF_STATE
};

// GEN_OUT_CTRL_RNET_SLEEP
static GenOutCtrlStateStepDef_t int_sys_act_STATE_USER_RNET_SLEEP[] =
{
    { 3000, MIN_TIME_FOR_SUBSTATE_ms, 1 },
    GEN_OUT_CTRL_END_OF_STATE
};


// GEN_OUT_CTRL_STATE_MODE_ACTIVE
static GenOutCtrlStateStepDef_t int_sys_act_STATE_MODE_ACTIVE[] =
{
    GEN_OUT_CTRL_ALWAYS_ON,     // Active the Pin 5 & 6.
    GEN_OUT_CTRL_END_OF_STATE
};

// GEN_OUT_CTRL_STATE_MODE_INACTIVE
static GenOutCtrlStateStepDef_t int_sys_act_STATE_MODE_INACTIVE[] =
{
    GEN_OUT_CTRL_ALWAYS_OFF,     // Inactivate the Pin 5 & 6.
    GEN_OUT_CTRL_END_OF_STATE
};

//**************************** Internal System Action (reset line) ****************************

// GEN_OUT_BLUETOOTH_ON
static GenOutCtrlStateStepDef_t bluetooth_enable_BLUETOOTH_LED_CTRL[] =
{
    GEN_OUT_CTRL_ALWAYS_OFF,     // Turn on the LED
    GEN_OUT_CTRL_END_OF_STATE
};

// GEN_OUT_BLUETOOTH_OFF
static GenOutCtrlStateStepDef_t bluetooth_disable_BLUETOOTH_LED_CTRL[] =
{
    GEN_OUT_CTRL_ALWAYS_OFF,     // Turn off the LED.
    GEN_OUT_CTRL_END_OF_STATE
};

// GEN_OUT_STATE_CTRL_TEST
// control_disabled_in_state

/*
 **************************************************************************************************
 *                                       FILE LOCAL FUNCTIONS DECLARATIONS
 **************************************************************************************************
 */

static void ControlTask(void);
static void AddStates(void);
static void SetOutputControllersToNewState(StateCtrl_t *stateData);

/*
 **************************************************************************************************
 *                                         PUBLIC FUNCTIONS DEFINITIONS
 **************************************************************************************************
 */

#endif // #ifdef OK_TO_USE_CONTROL_SCHEME

/**
 * This function initializes the data structures and drivers needed to run the LED's on the board.
 *
 * @return true if everything was successful or no initialization required, false if something
 * failed
 *
 * @note use this function to init using the RTOS init method.
 */
bool GenOutCtrlApp_Init(void)
{
    if (!GenOutCtrl_Init(false))
    {
        return false;
    }

#ifdef OK_TO_USE_CONTROL_SCHEME

    AddStates();
    
#if defined(GENERAL_OUTPUT_CTRL_USE_MUTEX)
    /// @todo actually fill these in!
    GenOutCtrl_AppRtosCb_MutexLock_Set((GenOutCtrl_AppCbFunc_t)NULL);
    GenOutCtrl_AppRtosCb_MutexUnlock_Set((GenOutCtrl_AppCbFunc_t)NULL);
#endif

    os_event_wake_task_id = event_create();

    // Create the state update and control task
    (void)task_create(ControlTask, NULL, GEN_OUT_CTRL_MGMT_TASK_PRIO, NULL, 0, 0);
#endif // #ifdef OK_TO_USE_CONTROL_SCHEME

    return true;
}

#ifdef OK_TO_USE_CONTROL_SCHEME

/**
 * @brief Makes a request to set the state of all output controllers.
 *
 * @param ctrlr_state State to set the output controller to.
 */
void GenOutCtrlApp_SetStateAll(GenOutState_t ctrlr_state)
{
    if (state_change_req_circ_buf_slots_in_use < STATE_CHANGE_REQ_CIRC_BUF_NUM_SLOTS)
    {
        state_change_req_circ_buf[state_change_req_circ_buf_pos_tail].set_all = true;
        state_change_req_circ_buf[state_change_req_circ_buf_pos_tail].id = GEN_OUT_CTRL_ID_MAX;
        state_change_req_circ_buf[state_change_req_circ_buf_pos_tail].state = ctrlr_state;

        state_change_req_circ_buf_slots_in_use++;
        state_change_req_circ_buf_pos_tail = (state_change_req_circ_buf_pos_tail == (STATE_CHANGE_REQ_CIRC_BUF_NUM_SLOTS - 1)) ? 0 : (state_change_req_circ_buf_pos_tail + 1);
    }
    else
    {
        ASSERT(false);
    }
}

/**
 * @brief Makes a request to set the state of a single output controller.
 *
 * @param item_id ID of the output control item that is to be modified
 * @param ctrlr_state State to set the output controller to.
 */
void GenOutCtrlApp_SetState(GenOutCtrlId_t item_id, GenOutState_t ctrlr_state)
{
    if (state_change_req_circ_buf_slots_in_use < STATE_CHANGE_REQ_CIRC_BUF_NUM_SLOTS)
    {
        state_change_req_circ_buf[state_change_req_circ_buf_pos_tail].set_all = false;
        state_change_req_circ_buf[state_change_req_circ_buf_pos_tail].id = item_id;
        state_change_req_circ_buf[state_change_req_circ_buf_pos_tail].state = ctrlr_state;

        state_change_req_circ_buf_slots_in_use++;
        state_change_req_circ_buf_pos_tail = (state_change_req_circ_buf_pos_tail == (STATE_CHANGE_REQ_CIRC_BUF_NUM_SLOTS - 1)) ? 0 : (state_change_req_circ_buf_pos_tail + 1);
    }
    else
    {
        ASSERT(false);
    }
}

/**
 * @brief Let's caller know whether or not this module's task needs to be woken up with an event to carry out able
 *        state change request. 
 * 
 * @return true: need to send event to have task carry out state change request, false: no need to send event.
 */
bool genOutCtrlAppNeedSendEvent(void)
{
    return !GenOutCtrl_AtLeastOneOutputCtrlrIsActive();
}

/**
 * @brief Returns the event that may be used to wake up this module's task.
 * 
 * @return Event that may be used to wake up the this module's task.
 */
Evt_t genOutCtrlAppWakeEvent(void)
{
    return os_event_wake_task_id;
}

/*
 **************************************************************************************************
 *                                       FILE LOCAL FUNCTIONS DEFINITIONS
 **************************************************************************************************
 */

/**
 * Takes in output controller state change requests and provides timing mechanism
 * for all output controllers.
 */

static void ControlTask(void)
{
    task_open();
    
    // Service any state change requests that may be pending.
    // We'll have control of the OS through this entire loop. Run through all requested state changes
    //
    // NOTE: We do this check here as tasks that run before this one on boot
    // NOTE: may request state changes where the event will be "sent" but thrown away by the
    // NOTE: OS kernel.
    while (state_change_req_circ_buf_slots_in_use > 0)
    {
        SetOutputControllersToNewState(&state_change_req_circ_buf[state_change_req_circ_buf_pos_head]);

        state_change_req_circ_buf_slots_in_use--;
        state_change_req_circ_buf_pos_head = (state_change_req_circ_buf_pos_head == (STATE_CHANGE_REQ_CIRC_BUF_NUM_SLOTS - 1)) ? 0 : (state_change_req_circ_buf_pos_head + 1);
    }
    
    StopWatch_t task_time_elapsed_sw;
    stopwatchStart(&task_time_elapsed_sw);

//    GenOutCtrl_Stop (GEN_OUT_CTRL_ID_BT_LED);

	while (1)
	{
        if (!GenOutCtrl_AtLeastOneOutputCtrlrIsActive())
        {
            // Wait forever as no output controllers are active and we want to use as
            // little energy and processing power as possible at all times.
			event_wait(os_event_wake_task_id);
            stopwatchZero(&task_time_elapsed_sw);
        }
        
        // Service any state change requests that may be pending.
        // We'll have control of the OS through this entire loop. Run through all requested state changes
        while (state_change_req_circ_buf_slots_in_use > 0)
        {
            SetOutputControllersToNewState(&state_change_req_circ_buf[state_change_req_circ_buf_pos_head]);

            state_change_req_circ_buf_slots_in_use--;
            state_change_req_circ_buf_pos_head = (state_change_req_circ_buf_pos_head == (STATE_CHANGE_REQ_CIRC_BUF_NUM_SLOTS - 1)) ? 0 : (state_change_req_circ_buf_pos_head + 1);
        }

        // Turn the BLUE Bluetooth LED on/off based upon the Bluetooth feature
        // being enabled.
#ifdef ASL110
        if ((eeprom8bitGet(EEPROM_STORED_ITEM_ENABLED_FEATURES) & FUNC_FEATURE_OUT_CTRL_TO_BT_MODULE_BIT_MASK) > 0)
        {
            GenOutCtrl_Disable (GEN_OUT_CTRL_ID_BT_LED);
            GenOutCtrlApp_SetStateAll (GEN_OUT_BLUETOOTH_ENABLED);
        }
        else 
        {
            GenOutCtrl_Enable (GEN_OUT_CTRL_ID_BT_LED);
            GenOutCtrlApp_SetStateAll (GEN_OUT_BLUETOOTH_DISABLED);
        }
#endif 
        
		task_wait(MILLISECONDS_TO_TICKS(GENERAL_OUTPUT_CTRL_UPDATE_RATE_ms));
        GenOutCtrl_TickUpdateAll_ms(stopwatchTimeElapsed(&task_time_elapsed_sw, true));
    }
    task_close();
}

/**
 * Adds all the states for all the output controllers.
 *
 * @note    After a new state is defined at the top of this file, a new line of code must be added
 *          here in order for it to be able to be used!
 */
static void AddStates(void)
{
    // Add state definitions for LED1, Forward Pad LED
    if (!GenOutCtrl_AddState(GEN_OUT_CTRL_ID_FORWARD_PAD_LED, GEN_OUT_FORWARD_PAD_INACTIVE, true, 0, LED_Steady_Off_StateDefinition) ||
        !GenOutCtrl_AddState(GEN_OUT_CTRL_ID_FORWARD_PAD_LED, GEN_OUT_FORWARD_PAD_ACTIVE, true, 0, LED_Steady_On_StateDefinition))
    {
        ASSERT(false);
    }

    // Add state definitions for LED2, Reverse Pad LED
    if (!GenOutCtrl_AddState(GEN_OUT_CTRL_ID_REVERSE_PAD_LED, GEN_OUT_REVERSE_PAD_INACTIVE, true, 0, LED_Steady_Off_StateDefinition) ||
        !GenOutCtrl_AddState(GEN_OUT_CTRL_ID_REVERSE_PAD_LED, GEN_OUT_REVERSE_PAD_ACTIVE, true, 0, LED_Steady_On_StateDefinition))
    {
        ASSERT(false);
    }

//    // Add state definitions for LED3, Left Pad LED
    if (!GenOutCtrl_AddState(GEN_OUT_CTRL_ID_LEFT_PAD_LED, GEN_OUT_LEFT_PAD_INACTIVE, true, 0, LED_Steady_Off_StateDefinition) ||
        !GenOutCtrl_AddState(GEN_OUT_CTRL_ID_LEFT_PAD_LED, GEN_OUT_LEFT_PAD_ACTIVE, true, 0, LED_Steady_On_StateDefinition))
    {
        ASSERT(false);
    }

    // Add state definitions for LED4, Right Pad LED
    if (!GenOutCtrl_AddState(GEN_OUT_CTRL_ID_RIGHT_PAD_LED, GEN_OUT_RIGHT_PAD_INACTIVE, true, 0, LED_Steady_Off_StateDefinition) || 
        !GenOutCtrl_AddState(GEN_OUT_CTRL_ID_RIGHT_PAD_LED, GEN_OUT_RIGHT_PAD_ACTIVE, true, 0, LED_Steady_On_StateDefinition))
    {
        ASSERT(false);
    }

    // NOTE: The first entry is executed at Power Up. So it's the one to set the initial state of the item.
    // Add state definitions for LEDx, Power LED
    if (!GenOutCtrl_AddState(GEN_OUT_CTRL_ID_POWER_LED, GEN_OUT_POWER_LED_ON, true, 0, LED_Steady_On_StateDefinition) || 
        !GenOutCtrl_AddState(GEN_OUT_CTRL_ID_POWER_LED, GEN_OUT_POWER_LED_OFF, true, 0, LED_Steady_Off_StateDefinition))
    {
        ASSERT(false);
    }

    // Add state definitions for LED1. (Amber Color)
//    if (!GenOutCtrl_AddState(GEN_OUT_CTRL_ID_LED1, GEN_OUT_CTRL_STATE_BLUETOOTH_OUTPUT, false, 0, led1_state_BLUETOOTH_OUTPUT) ||
//        !GenOutCtrl_AddState(GEN_OUT_CTRL_ID_LED1, GEN_OUT_CTRL_STATE_HEAD_ARRAY_ACTIVE, false, 0, control_disabled_in_state) ||
//        !GenOutCtrl_AddState(GEN_OUT_CTRL_ID_LED1, GEN_OUT_CTRL_STATE_NO_OUTPUT, false, 0, control_disabled_in_state) ||
//        !GenOutCtrl_AddState(GEN_OUT_CTRL_ID_LED1, GEN_OUT_STATE_CTRL_TEST, false, 0, control_disabled_in_state))
//    {
//        ASSERT(false);
//    }

    // Add state definitions for internal system action output control.
//    if (!GenOutCtrl_AddState(GEN_OUT_CTRL_ID_INTERNAL_SYS_ACTION, GEN_OUT_CTRL_STATE_HEAD_ARRAY_RESETTING, true, 0, int_sys_act_HEAD_ARRAY_RESETTING) ||
//        !GenOutCtrl_AddState(GEN_OUT_CTRL_ID_INTERNAL_SYS_ACTION, GEN_OUT_CTRL_STATE_STATE_USER_BTN_NEXT_FUNCTION, true, 0, int_sys_act_STATE_USER_BTN_NEXT_FUNCTION) ||
//        !GenOutCtrl_AddState(GEN_OUT_CTRL_ID_INTERNAL_SYS_ACTION, GEN_OUT_CTRL_STATE_USER_BTN_NEXT_PROFILE, true, 0, int_sys_act_STATE_USER_BTN_NEXT_PROFILE) ||
//        !GenOutCtrl_AddState(GEN_OUT_CTRL_ID_INTERNAL_SYS_ACTION, GEN_OUT_STATE_CTRL_TEST, false, 0, control_disabled_in_state) ||
//        !GenOutCtrl_AddState(GEN_OUT_CTRL_ID_INTERNAL_SYS_ACTION, GEN_OUT_CTRL_STATE_MODE_ACTIVE, true, 0, int_sys_act_STATE_MODE_ACTIVE) ||
//        !GenOutCtrl_AddState(GEN_OUT_CTRL_ID_INTERNAL_SYS_ACTION, GEN_OUT_CTRL_STATE_MODE_INACTIVE, true, 0, int_sys_act_STATE_MODE_INACTIVE) ||
//        !GenOutCtrl_AddState(GEN_OUT_CTRL_ID_INTERNAL_SYS_ACTION, GEN_OUT_CTRL_RNET_SLEEP, true, 0, int_sys_act_STATE_USER_RNET_SLEEP))
//    {
//        ASSERT(false);
//    }

    // Add state definitions for Bluetooth Blue LED control.
//    if (!GenOutCtrl_AddState(GEN_OUT_CTRL_ID_BT_LED, GEN_OUT_BLUETOOTH_ENABLED, true, 0, bluetooth_enable_BLUETOOTH_LED_CTRL) ||
//        !GenOutCtrl_AddState(GEN_OUT_CTRL_ID_BT_LED, GEN_OUT_BLUETOOTH_DISABLED, true, 0, bluetooth_disable_BLUETOOTH_LED_CTRL))
//    {
//        ASSERT(false);
//    }
}

/**
 * @brief Sets the state of the one or all output controllers to a newly requested state.
 *
 * @param stateData State information.
 */
static void SetOutputControllersToNewState(StateCtrl_t *stateData)
{
    if (stateData->set_all)
    {
        GenOutCtrl_StateSetAll(stateData->state);
        GenOutCtrl_StartAll(); // Start all if they're not already started under automatic control.
    }
    else
    {
        GenOutCtrl_StateSet(stateData->id, stateData->state);
        GenOutCtrl_Start(stateData->id); // Start it if it's not already started under automatic control.
    }
}

#endif // #ifdef OK_TO_USE_CONTROL_SCHEME

// End of Doxygen grouping
/** @} */

/**************************************************************************************************
---          End of File          ------          End of File          ------
**************************************************************************************************/
