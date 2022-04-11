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
 ********************************************************************************************************
 *
 * @brief       Defines general output pin control API.
 * @file        general_output_ctrl.c
 * @author      tparsh
 * @ingroup		USER_DRIVER
 *
 ********************************************************************************************************
 */

/**
 * @addtogroup USER_DRIVER
 * @{
 */

/*
 ********************************************************************************************************
 *                                         Lint Begin File
 ********************************************************************************************************
 */

// The lint ignores below are there because the mutex lock/unlock mechanism is abstracted
// out on purpose so as to support both RTOS and bare metal systems.
//lint -efunc(454,LockMutex,UnlockMutex)
//lint -efunc(455,LockMutex,UnlockMutex)
   
/*
 ********************************************************************************************************
 *                                           INCLUDE FILES
 ********************************************************************************************************
 */
/********************************************** System *************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

/********************************************    User   ************************************************/
#include "user_assert.h"
#include "general_output_ctrl_bsp.h"
#include "general_output_ctrl.h"

#if defined(GENERAL_OUTPUT_CTRL_MODULE_ENABLE)

#ifdef OK_TO_USE_OUTPUT_CONTROL

/*
 ********************************************************************************************************
 *                                               DATA TYPES
 ********************************************************************************************************
 */

/// Information for an output controller state definition
typedef struct
{
    /// Unique ID of the state.
    GenOutState_t state;

    /// If true, the state will be run through once and then stop.  Otherwise, it will loop forever.
    bool one_shot;

    /// Priority of the state. 0 is lowest, 255 is highest. e.g. if a state of prioirity 2 is running,
    /// then any requested state change where the state is <2 priority will be denied.
    uint8_t priority;

    /// The different on/off steps for the output controller for this state.
    GenOutCtrlStateStepDef_t *steps;
} OutCtrlrStateDef_t;

/**
 * State information for an output controller's test mode. Test mode is used to do just that:
 * test output functionality and possibly sanity check to make sure a board works.
 */
typedef struct
{
    /// True if the test state is active.
    bool is_active;

    /**
     * Contains the most recent operational state object of the system since entering test test mode.
     * Means nothing when not in test mode.
    */
    OutCtrlrStateDef_t *curr_state_obj;
} OutCtrlrTestState_t;

/// Stores all state and control information related to an output controller for a single general output line.
typedef struct
{
    /// Unique ID for the output controller
    GenOutCtrlId_t id;

    /// Holds test mode information.
    OutCtrlrTestState_t test_state;

    /// Current state's control object.
    OutCtrlrStateDef_t *curr_state_obj;

    /// Current step for the operational state of the output controller.
    uint16_t curr_index;

    /// Time elapsed, in ms, since entering the current step.
    GenOutCtrlTime_t time_elapsed_ms;

    /// Let's the controller know whether the output is currently active or inactive.
    bool output_is_active;

    /// True if under automatic timing control by this module, false if timer control exists outside of this module.
    bool is_active;

    /// The total number of states that are defined for this output control state.
    /// Cannot exceed the value of GEN_OUT_CTRL_STATE_MAX!
    uint8_t num_states;

    /// Only used when a state is one shot.  When false, the state is still running, when true, it is not.
    bool one_shot_complete;

    /// Number of times repeated at the current stage.
    uint8_t num_times_run;

    /// Definitions for all states that an output controller can be in
    /// @todo Turn this into a singly-linked list so as to reduce the RAM footprint of this module
    OutCtrlrStateDef_t states[GEN_OUT_CTRL_STATE_MAX];
} OutputCtrlrState_t;

/*
 ********************************************************************************************************
 *                                                VARIABLES
 ********************************************************************************************************
 */
/************************************************* Local ***********************************************/
/// True when operating in bare metal mode (no RTOS)
static bool bare_metal = false;

/// Function called at module initialization when the driver is initialized in bare-metal mode.
static GenOutCtrl_AppCbFunc_t AppBaremetalCb_Init = NULL;

/// Function called at module de-initialization when the driver is de-initialized in bare-metal mode.
static GenOutCtrl_AppCbFunc_t AppBaremetalCb_Deinit = NULL;

/// Function called at module initialization when the driver is initialized in RTOS-metal mode.
static GenOutCtrl_AppCbFunc_t AppRtosCb_Init = NULL;

/// Function called at de-initialization when the driver is de-initialized in RTOS-metal mode.
static GenOutCtrl_AppCbFunc_t AppRtosCb_Deinit = NULL;

#if defined(GENERAL_OUTPUT_CTRL_USE_MUTEX)
	/// Function called to lock the module control mutex when in RTOS mode.
	static GenOutCtrl_AppCbFunc_t AppRtosCb_MutexLock = NULL;

	/// Function called to unlock the module control mutex when in RTOS mode.
	static GenOutCtrl_AppCbFunc_t AppRtosCb_MutexUnlock = NULL;
#endif

/// True when all module components are initialized
static bool module_is_initialized = false;

/// Master control structure array for all output state controllers
OutputCtrlrState_t state_ctrl[(int)GEN_OUT_CTRL_ID_MAX];

/*
 ********************************************************************************************************
 *                                       FILE LOCAL FUNCTIONS DECLARATIONS
 ********************************************************************************************************
 */

static bool StateCtrlr_NextControlSubstep(OutputCtrlrState_t *ctrl);
static bool StateCtrlr_EndOfList(OutputCtrlrState_t *ctrl);
static bool IsStateDefined(GenOutCtrlId_t item_id, GenOutState_t state);
static OutCtrlrStateDef_t *StateControlObject_Get(GenOutCtrlId_t item_id, GenOutState_t state);
static void InitControlData(void);
static void ResetControlData(GenOutCtrlId_t item_id);

#if defined(GENERAL_OUTPUT_CTRL_USE_MUTEX)
    static void LockMutex(void);
    static void UnlockMutex(void);
#else
    // Do this so execution time isn't taken up when mutexes are unavailable.
    #define LockMutex()
    #define UnlockMutex()
#endif

static bool DefaultCbFunc(void);

/*
 ********************************************************************************************************
 *                                         PUBLIC FUNCTIONS DEFINITIONS
 ********************************************************************************************************
 */

/**
 * @brief This function initializes the data structures and drivers needed to run the output's on the board.
 *
 * @param is_bare_metal true if not using an RTOS.
 *
 * @return True if everything was successful or no initialization required, false if something failed
 */
bool GenOutCtrl_Init(bool is_bare_metal)
{
    bool ret_val = true;
    
    if (AppBaremetalCb_Init == NULL)
    {
        AppBaremetalCb_Init = DefaultCbFunc;
        AppBaremetalCb_Deinit = DefaultCbFunc;
        AppRtosCb_Init = DefaultCbFunc;
        AppRtosCb_Deinit = DefaultCbFunc;
        
#if defined(GENERAL_OUTPUT_CTRL_USE_MUTEX)
        AppRtosCb_MutexLock = DefaultCbFunc;
        AppRtosCb_MutexUnlock = DefaultCbFunc;
#endif
    }

    if ((bare_metal != is_bare_metal) || !module_is_initialized)
    {
        if (is_bare_metal)
        {
            if (module_is_initialized)
            {
                if (AppRtosCb_Deinit != NULL)
                {
                    if (!AppRtosCb_Deinit())
                    {
                        ret_val = false;
                    }
                }
            }

            bare_metal = true;
            InitControlData();

            if (AppBaremetalCb_Init != NULL)
            {
                if (!AppBaremetalCb_Init())
                {
                    ret_val = false;
                }
            }

            if (!module_is_initialized)
            {
                module_is_initialized = true;
                ret_val = GenOutCtrl_EnableAll();
            }
        }
        else
        {
            if (module_is_initialized)
            {
                if (AppBaremetalCb_Deinit != NULL)
                {
                    if (!AppBaremetalCb_Deinit())
                    {
                        ret_val = false;
                    }
                }
            }

            bare_metal = false;
            InitControlData();

            if (AppRtosCb_Init != NULL)
            {
                if (!AppRtosCb_Init())
                {
                    ret_val = false;
                }
            }

            if (!module_is_initialized)
            {
                module_is_initialized = true;
                ret_val = GenOutCtrl_EnableAll();
            }
        }
    }
    
    return ret_val;
}

/**
 * This function de-initializes the data structures and drivers needed to run the outputs desired
 *
 * @return true if everything was successful or no de-initialization required, false if something failed
 */
bool GenOutCtrl_Deinit(void)
{
    bool ret_val = true;

    if (module_is_initialized)
    {
        GenOutCtrl_StopAll();

        if (!bare_metal)
        {
            if (!AppRtosCb_Deinit())
            {
                ret_val = false;
            }
        }
        else if (!AppBaremetalCb_Deinit())
        {
            ret_val = false;
        }

        if (!GenOutCtrl_DisableAll())
        {
            ret_val = false;
        }

        // Wipe out all assigned states and state data for all output controllers.
        InitControlData();

        module_is_initialized = false;
    }

    return ret_val;
}

/**
 * Add a new state to an output controller.
 *
 * @param item_id   ID of the output control item that is to be modified
 * @param state 	State to add to the controller.
 * @param one_shot 	If true, the state is run through one and then control is stopped, else
 * 					control lasts until told to stop.
 * @param priority  State priority. 0 is lowest and 255 is highest.
 * @param steps 	Contains the on and off time definitions for the state.
 *
 * @return  		True if the state was added successfully false if state has already been defined.
 */
bool GenOutCtrl_AddState(GenOutCtrlId_t item_id, GenOutState_t state, bool one_shot, uint8_t priority, GenOutCtrlStateStepDef_t *steps)
{
	if (IsStateDefined(item_id, state) || (state_ctrl[item_id].num_states >= GEN_OUT_CTRL_STATE_MAX))
	{
		return false;
	}
	else
	{
		state_ctrl[item_id].states[state_ctrl[item_id].num_states].steps = steps;
		state_ctrl[item_id].states[state_ctrl[item_id].num_states].one_shot = one_shot;
        state_ctrl[item_id].states[state_ctrl[item_id].num_states].state = state;
        state_ctrl[item_id].states[state_ctrl[item_id].num_states].priority = priority;

        // Set initial state to prevent system crash if user forgets to set a valid state
        // before enabling the module to start running.
        if (state_ctrl[item_id].curr_state_obj == (OutCtrlrStateDef_t *)NULL)
        {
            state_ctrl[item_id].curr_state_obj = &state_ctrl[item_id].states[state_ctrl[item_id].num_states];
            state_ctrl[item_id].test_state.curr_state_obj = state_ctrl[item_id].curr_state_obj;
        }

        state_ctrl[item_id].num_states++;

		return true;
	}
}

/**
 * Heart of this control module.  Updates all the output controller states based on time passed.
 * This function may be called internally during automatic control (through a timer resource) or
 * through manual control from an outside entity.
 *
 * @param time_elapsed_ms 	Amount of time passed since the last time this function has been called.
 *        					Must be accurate and precise!
 *
 * @return true if the update succeeded, false if it failed
 */
bool GenOutCtrl_TickUpdateAll_ms(GenOutCtrlTime_t time_elapsed_ms)
{
    bool ret_val = true;

    for (unsigned int i = 0; i < (uint8_t)GEN_OUT_CTRL_ID_MAX; i++)
    {
        if (!GenOutCtrl_TickUpdate_ms(time_elapsed_ms, (GenOutCtrlId_t)i))
        {
            ret_val = false;
        }
    }

    return ret_val;
}

/**
 * Heart of this control module.  Updates all the output controller states based on time passed.
 * This function may be called internally during automatic control (through a timer resource) or
 * through manual control from an outside entity.
 *
 * @param time_elapsed_ms 	Amount of time passed since the last time this function has been called.
 * 							Must be accurate and precise!
 * @param item_id           ID of the output control item that is to be modified
 *
 * @return true if the update succeeded, false if it failed
 */
bool GenOutCtrl_TickUpdate_ms(GenOutCtrlTime_t time_elapsed_ms, GenOutCtrlId_t item_id)
{
    LockMutex();

    // Make sure the output controller is active and that if it is a state with one-shot enabled that
    // that the sequence has not yet finished with it's single execution sequence yet.
    if (state_ctrl[item_id].is_active && !state_ctrl[item_id].one_shot_complete)
    {
        bool go_to_next_substep = false;
        bool go_to_next_step = false;

        // Do the below for code clarity in the decision logic below these two definitions.
        GenOutCtrlTime_t on_time = state_ctrl[item_id].curr_state_obj->steps[state_ctrl[item_id].curr_index].on_time_ms;
        GenOutCtrlTime_t off_time = state_ctrl[item_id].curr_state_obj->steps[state_ctrl[item_id].curr_index].off_time_ms;

        state_ctrl[item_id].time_elapsed_ms += time_elapsed_ms;

        if (state_ctrl[item_id].output_is_active)
        {
            if ((state_ctrl[item_id].time_elapsed_ms >= on_time) &&
                (on_time != GEN_OUT_CTRL_ALWAYS_IN_STATE_VAL))
            {
            	state_ctrl[item_id].time_elapsed_ms = 0;
                go_to_next_substep = true;
            }
        }
        else if ((state_ctrl[item_id].time_elapsed_ms >= off_time) &&
                 (off_time != GEN_OUT_CTRL_ALWAYS_IN_STATE_VAL))
        {
            state_ctrl[item_id].time_elapsed_ms = 0;
            go_to_next_step = true;
        }

        // See if we need to repeat this current step.
        if (go_to_next_step)
        {
            state_ctrl[item_id].num_times_run++;
            if (state_ctrl[item_id].num_times_run >= state_ctrl[item_id].curr_state_obj->steps[state_ctrl[item_id].curr_index].num_times_to_run)
            {
                (state_ctrl[item_id].curr_index)++;
                state_ctrl[item_id].num_times_run = 0;
            }
        }

        // Go to the next substep if required.
        if (go_to_next_substep || go_to_next_step)
        {
            (void)StateCtrlr_NextControlSubstep(&state_ctrl[item_id]);
        }
    }

    UnlockMutex();

    return true;
}

/**
 * Starts all output controllers in the current output control state assigned to each controller.
 */
bool GenOutCtrl_StartAll(void)
{
    bool ret_val = true;

    for (unsigned int i = 0; i < (unsigned int)GEN_OUT_CTRL_ID_MAX; i++)
    {
        if (!GenOutCtrl_Start((GenOutCtrlId_t)i))
        {
            ret_val = false;
        }
    }

    return ret_val;
}

/**
 * Stops the current output control state of all output controllers.
 */
bool GenOutCtrl_StopAll(void)
{
    bool ret_val = true;

    for (unsigned int i = 0; i < (unsigned int)GEN_OUT_CTRL_ID_MAX; i++)
    {
        if (!GenOutCtrl_Stop((GenOutCtrlId_t)i))
        {
            ret_val = false;
        }
    }

    return ret_val;
}

/**
 * Kicks off automatic control for one output controller.
 *
 * @param item_id   ID of the output control item that is to be modified
 *
 * @return true if the update succeeded or the RTOS is not running, false if the output
 *         controller is set to an undefined state.
 */
bool GenOutCtrl_Start(GenOutCtrlId_t item_id)
{
    bool ret_val = false;

    LockMutex();

    if (!state_ctrl[item_id].is_active)
    {
        if (!StateCtrlr_EndOfList(&state_ctrl[item_id]))
        {
            ResetControlData(item_id);
            state_ctrl[item_id].is_active = true;
            (void)StateCtrlr_NextControlSubstep(&state_ctrl[item_id]);
            ret_val = true;
        }
    }

    UnlockMutex();

    return ret_val;
}

/**
 * Stops automatic output control of desired controller.
 *
 * @param item_id   ID of the output control item that is to be modified
 *
 * @return true if the update succeeded or the RTOS is not running, false if it failed
 */
bool GenOutCtrl_Stop(GenOutCtrlId_t item_id)
{
    LockMutex();

    if (state_ctrl[item_id].is_active)
    {
        state_ctrl[item_id].is_active = false; // Must be before the reset data call below
        ResetControlData(item_id);
    }

    UnlockMutex();

    return true;
}

/**
 * Sets the state of all output controllers.
 */
bool GenOutCtrl_StateSetAll(GenOutState_t new_state)
{
    bool ret_val = true;

    for (unsigned int i = 0; i < (unsigned int)GEN_OUT_CTRL_ID_MAX; i++)
    {
        if (!GenOutCtrl_StateSet((GenOutCtrlId_t)i, new_state))
        {
            ret_val = false;
        }
    }

    return ret_val;
}

/**
 * Sets the operating state of a specific output controller.
 *
 * @param item_id   ID of the output control item that is to be modified
 * @param new_state  The state to enter.
 *
 * @return true if the update succeeded, false if it failed. Right now, it always succeeds.
 *
 * @note All state control variables are reset.  Also, if the output controller was running before this
 *       function was called, then it remains running after the call.
 */
bool GenOutCtrl_StateSet(GenOutCtrlId_t item_id, GenOutState_t new_state)
{
    bool ret_val = true;

    LockMutex();

    if (IsStateDefined(item_id, new_state))
    {
        OutCtrlrStateDef_t *temp_state_obj_ptr = StateControlObject_Get(item_id, new_state);

        if ((state_ctrl[item_id].curr_state_obj->priority <= temp_state_obj_ptr->priority) &&
            ((state_ctrl[item_id].curr_state_obj->state != new_state) || state_ctrl[item_id].one_shot_complete))
        {
            if (state_ctrl[item_id].test_state.is_active)
            {
                // In test mode, so we will set the state that the system will jump to after kicking out
                // of test mode.
                state_ctrl[item_id].test_state.curr_state_obj = StateControlObject_Get(item_id, new_state);
            }
            else
            {
                bool temp_is_active = state_ctrl[item_id].is_active;

                // Turn off state control timer and reset state control data.
                // Ordering below is important! Do not change!
                state_ctrl[item_id].is_active = false; // Must be before the reset data call below
                ResetControlData(item_id);
                state_ctrl[item_id].curr_state_obj = temp_state_obj_ptr;
                
                if (temp_is_active)
                {
                    if (StateCtrlr_EndOfList(&state_ctrl[item_id]))
                    {
                        // If in a "do nothing at all" state, simply ensure that the output is turned off.
                        GenOutCtrlBsp_SetInactive(state_ctrl[item_id].id);
                    }
                    else
                    {
                        // Re-enable the timer module after state data reset.
                        state_ctrl[item_id].is_active = true;

                        // Sets the state machine off with the output in the right state (on or off)
                        (void)StateCtrlr_NextControlSubstep(&state_ctrl[item_id]);
                    }
                }
            }
        }
    }

    UnlockMutex();

    return ret_val;
}

/**
 * Lets the caller know whether or not any of the output controllers are active.
 *
 * @return true if at least one output controller is active, false if none are active.
 */
bool GenOutCtrl_AtLeastOneOutputCtrlrIsActive(void)
{
    for (unsigned int i = 0; i < (unsigned int)GEN_OUT_CTRL_ID_MAX; i++)
    {
        if (GenOutCtrl_OutputCtrlrIsActive((GenOutCtrlId_t)i))
        {
            return true;
        }
    }

    return false;
}

/**
 * Lets the caller know whether or not a particular output controller is active.
 *
 * @param item_id   ID of the output control item that is to be modified
 *
 * @return true if the output controller is active, false if it is not active.
 */
bool GenOutCtrl_OutputCtrlrIsActive(GenOutCtrlId_t item_id)
{
    if (state_ctrl[item_id].is_active && !state_ctrl[item_id].one_shot_complete &&
       (state_ctrl[item_id].curr_state_obj->steps[state_ctrl[item_id].curr_index].off_time_ms != GEN_OUT_CTRL_ALWAYS_IN_STATE_VAL))
    {
        return true;
    }
    else
    {
        return false;
    }
}

/**
 * Gets the operating state of a specific output controller.
 *
 * @param item_id Id of the output controller.
 *
 * @return ID of the current operating state of the output controller
 */
GenOutState_t GenOutCtrl_OutputCtrlrStateGet(GenOutCtrlId_t item_id)
{
    GenOutState_t ret_val;

    LockMutex();

    ret_val = state_ctrl[item_id].curr_state_obj->state;

    UnlockMutex();

    return ret_val;
}

/**
 * Sets up the GPIO to control all defined output controllers.
 *
 * @param item_id   ID of the output control item that is to be modified
 *
 * @return true if the update succeeded, false if the output controller is currently being actively controlled.
 *
 * @note This function is locked if the output controllers are not under manual control.
 */
bool GenOutCtrl_EnableAll(void)
{
    bool ret_val = true;

    for (unsigned int  i = 0; i < (unsigned int )GEN_OUT_CTRL_ID_MAX; i++)
    {
        if (!GenOutCtrl_Enable((GenOutCtrlId_t)i))
        {
            ret_val = false;
        }
    }

    return ret_val;
}

/**
 * Sets up the GPIO to control an output controller.
 *
 * @param item_id   ID of the output control item that is to be modified
 *
 * @return true if the update succeeded, false if the output is currently being actively controlled.
 *
 * @note This function is locked if the output is not under manual control.
 */
bool GenOutCtrl_Enable(GenOutCtrlId_t item_id)
{
    bool ret_val = false;

    LockMutex();

    if (!state_ctrl[item_id].is_active)
    {
        ret_val = GenOutCtrlBsp_Enable(item_id);
    }

    UnlockMutex();

    return ret_val;
}

/**
 * Puts all defined output controller's GPIO in a state that interferes as minimally as possible with the rest of the hardware.
 *
 * @param item_id   ID of the output control item that is to be modified
 *
 * @return true if the update succeeded, false if the output controller is currently being actively controlled.
 *
 * @note This function is locked if the output controllers are not under manual control.
 *       Also, mutex control is in GenOutCtrl_Disable().
 */
bool GenOutCtrl_DisableAll(void)
{
    bool ret_val = true;

    for (unsigned int  i = 0; i < (unsigned int )GEN_OUT_CTRL_ID_MAX; i++)
    {
        if (!GenOutCtrl_Disable((GenOutCtrlId_t)i))
        {
            ret_val = false;
        }
    }

    return ret_val;
}

/**
 * Puts defined output controller's GPIO in a state that interferes as minimally as possible with the rest of the hardware.
 *
 * @param item_id   ID of the output control item that is to be modified
 *
 * @return true if the update succeeded, false if the output controller is currently being actively controlled.
 *
 * @note This function is locked if the output controller is not under manual control.
 */
bool GenOutCtrl_Disable(GenOutCtrlId_t item_id)
{
    bool ret_val = false;

    if (!state_ctrl[item_id].is_active)
    {
        ret_val = GenOutCtrlBsp_Disable(item_id);
    }

    return ret_val;
}

/**
 * This function is used to manually set an output to an active state.
 *
 * @param item_id   ID of the output control item that is to be modified
 *
 * @return true if the update succeeded, false if the output is currently being actively controlled.
 *
 * @note This function is locked if the output is under automatic control in this module.
 */
bool GenOutCtrl_SetActive(GenOutCtrlId_t item_id)
{
    bool ret_val = false;

    if (!state_ctrl[item_id].is_active)
    {
        (void)GenOutCtrlBsp_SetActive(item_id);
        ret_val = true;
    }

    return ret_val;
}

/**
 * This function is used to manually set an output to an inactive state.
 *
 * @param item_id   ID of the output control item that is to be modified
 *
 * @return true if the update succeeded, false if the output is currently being actively controlled.
 *
 * @note This function is locked if the output is under automatic control in this module.
 */
bool GenOutCtrl_SetInactive(GenOutCtrlId_t item_id)
{
    bool ret_val = false;

    if (!state_ctrl[item_id].is_active)
    {
        (void)GenOutCtrlBsp_SetInactive(item_id);
        ret_val = true;
    }

    return ret_val;
}

/**
 * This function is used to toggle an output's state.
 *
 * @param item_id   ID of the output control item that is to be modified
 *
 * @return true if the update succeeded, false if the output is currently being actively controlled.
 *
 * @note This function is locked if the output is under automatic control in this module.
 */
bool GenOutCtrl_ToggleActiveState(GenOutCtrlId_t item_id)
{
    bool ret_val = false;

    if (!state_ctrl[item_id].is_active)
    {
        (void)GenOutCtrlBsp_Toggle(item_id);
        ret_val = true;
    }

    return ret_val;
}

/**
 * Function called during initialization of this module when in bare-metal mode.
 *
 * @param callback Callback function defined by the application.
 */
void GenOutCtrl_AppBaremetalCb_Init_Set(GenOutCtrl_AppCbFunc_t callback)
{
    AppBaremetalCb_Init = callback;
}

/**
 * Function called during de-initialization of this module when in bare-metal mode.
 *
 * @param callback Callback function defined by the application.
 */
void GenOutCtrl_AppBaremetalCb_Deinit_Set(GenOutCtrl_AppCbFunc_t callback)
{
    AppBaremetalCb_Deinit = callback;
}

/**
 * Function called during initialization when this module is initialized in RTOS mode.
 *
 * @param callback Callback function defined by the application.
 */
void GenOutCtrl_AppRtosCb_Init_Set(GenOutCtrl_AppCbFunc_t callback)
{
    AppRtosCb_Init = callback;
}

/**
 * Function called during de-initialization when this module has been initialized in RTOS mode.
 *
 * @param callback Callback function defined by the application.
 */
void GenOutCtrl_AppRtosCb_Deinit_Set(GenOutCtrl_AppCbFunc_t callback)
{
    AppRtosCb_Deinit = callback;
}

#if defined(GENERAL_OUTPUT_CTRL_USE_MUTEX)
/**
 * Called to lock a mutex when this module is initialied in RTOS mode.
 *
 * @param callback Callback function defined by the application.
 */
void GenOutCtrl_AppRtosCb_MutexLock_Set(GenOutCtrl_AppCbFunc_t callback)
{
    AppRtosCb_MutexLock = callback;
}

/**
 * Called to unlock a mutex when this module is initialied in RTOS mode.
 *
 * @param callback Callback function defined by the application.
 */
void GenOutCtrl_AppRtosCb_MutexUnlock_Set(GenOutCtrl_AppCbFunc_t callback)
{
    AppRtosCb_MutexUnlock = callback;
}
#endif

/**
 * Puts all output controllers into test mode or takes them out of output test mode.
 *
 * @param make_active if true then the output controllers go into test mode, otherwise kick out of test mode.
 *
 * @return true if the update succeeded, false if it failed.
 *
 * @note Mutex control is handled in GenOutCtrl_TestModeSet
 */
bool GenOutCtrl_TestModeSetAll(bool make_active)
{
    LockMutex();

    for (unsigned int  i = 0; i < (unsigned int )GEN_OUT_CTRL_ID_MAX; i++)
    {
        (void)GenOutCtrl_TestModeSet((GenOutCtrlId_t)i, make_active);
    }

    UnlockMutex();

    return true;
}

/**
 * Put a single output controller into test mode or take it out of test mode and return to whatever the current
 * normal mode operation is.
 *
 * @param item_id       ID of the output control item that is to be modified
 * @param make_active    If true then the output controller goes into test mode, otherwise kick out of test mode.
 *
 * @return true if the update succeeded, false if it failed.
 *
 * @note All state control variables are reset.  Also, if the output controller was running before this function
 *       was called, then it remains running after the call.
 */
bool GenOutCtrl_TestModeSet(GenOutCtrlId_t item_id, bool make_active)
{
    // Only put the output controller into test mode if it is not already in test mode.
    if (state_ctrl[item_id].test_state.is_active != make_active)
    {
        state_ctrl[item_id].test_state.is_active = make_active;

        bool temp_is_active = state_ctrl[item_id].is_active;
        state_ctrl[item_id].is_active = false; // Must be before the reset data call below
        ResetControlData(item_id); // Put controller into test idle state

        if (state_ctrl[item_id].test_state.is_active)
        {
            state_ctrl[item_id].test_state.curr_state_obj = state_ctrl[item_id].curr_state_obj;
            state_ctrl[item_id].curr_state_obj = StateControlObject_Get(item_id, GEN_OUT_STATE_CTRL_TEST);
        }
        else
        {
            state_ctrl[item_id].curr_state_obj = state_ctrl[item_id].test_state.curr_state_obj;
        }

        // Re-enable the output controller if it was active before the state change.
        if (temp_is_active)
        {
            state_ctrl[item_id].is_active = true;
        }
    }

    return true;
}

/*
 ********************************************************************************************************
 *                                       FILE LOCAL FUNCTIONS DEFINITIONS
 ********************************************************************************************************
 */

/**
 * Goes to the next valid sub-step in the state step-list.
 *
 * @param ctrl Pointer to the output controller state object
 *
 * @return true if the update succeeded, false if it failed
 *
 * @note Mutex control must be handled outside of this function
 */
static bool StateCtrlr_NextControlSubstep(OutputCtrlrState_t *ctrl)
{
    if (ctrl->output_is_active)
    {
        if (ctrl->curr_state_obj->steps[ctrl->curr_index].off_time_ms > 0)
        {
            ctrl->output_is_active = false;
            (void)GenOutCtrlBsp_SetInactive(ctrl->id);
        }
        else
        {
            (ctrl->curr_index)++;
            if (StateCtrlr_EndOfList(ctrl))
            {
                // At end of the list
                ctrl->curr_index = 0;
                ASSERT(!StateCtrlr_EndOfList(ctrl)); // Critical error...this should never be allowed to happen.
            }

            if (ctrl->curr_state_obj->one_shot)
            {
                ctrl->one_shot_complete = true;
            }
            else if (ctrl->curr_state_obj->steps[ctrl->curr_index].on_time_ms > 0)
            {
                (void)GenOutCtrlBsp_SetActive(ctrl->id);
                ctrl->output_is_active = true;
            }
            else
            {
                (void)GenOutCtrlBsp_SetInactive(ctrl->id);
                ctrl->output_is_active = false;
            }
        }
    }
    else
    {
        if (StateCtrlr_EndOfList(ctrl))
        {
            if (ctrl->curr_state_obj->one_shot)
            {
                ctrl->one_shot_complete = true;
            }

            // At end of the list
            ctrl->curr_index = 0;
        }

        if (ctrl->is_active && !ctrl->one_shot_complete)
        {
            if (ctrl->curr_state_obj->steps[ctrl->curr_index].on_time_ms > 0)
            {
                ctrl->output_is_active = true;
                (void)GenOutCtrlBsp_SetActive(ctrl->id);
            }
            else
            {
                // The below should already be set. Set it again for good measure...
                ctrl->output_is_active = false;
                (void)GenOutCtrlBsp_SetInactive(ctrl->id);
            }
        }
    }

    return true;
}

/**
 * Checks to see if we are at the end of the output state controller step list.
 *
 * @param ctrl Pointer to the output state control & definition structure
 *
 * @return true if at the end of the step list, false if not at the end of the list.
 *
 * @note Mutex control must be handled outside of this function
 */
static bool StateCtrlr_EndOfList(OutputCtrlrState_t *ctrl)
{
    if ((ctrl->curr_state_obj != NULL) &&
        (ctrl->curr_state_obj->steps[ctrl->curr_index].off_time_ms == GEN_OUT_CTRL_ALWAYS_IN_STATE_VAL) &&
        (ctrl->curr_state_obj->steps[ctrl->curr_index].on_time_ms == GEN_OUT_CTRL_ALWAYS_IN_STATE_VAL))
    {
        return true;
    }
    else
    {
        return false;
    }
}

/**
 * Figures out where new state information should go.
 *
 * @param item_id   ID of the output control item that is to be modified
 * @param state 	State to check for definition in the controller.
 *
 * @return			true if state is already defined, false if it is not defined.
 */
static bool IsStateDefined(GenOutCtrlId_t item_id, GenOutState_t state)
{
	for (uint8_t i = 0; i < state_ctrl[item_id].num_states; i++)
	{
		if (state_ctrl[item_id].states[i].state == state)
		{
			return true;
		}
	}

	return false;
}

/**
 * Grabs the state control object for a particular state for a particular output controller.
 *
 * @param item_id   ID of the output control item that is to be modified
 * @param state     State definition to pull from the controller.
 *
 * @note            The state is already checked for definition prior to this function call.
 */
static OutCtrlrStateDef_t *StateControlObject_Get(GenOutCtrlId_t item_id, GenOutState_t state)
{
    for (uint8_t i = 0; i < state_ctrl[item_id].num_states; i++)
    {
        if (state_ctrl[item_id].states[i].state == state)
        {
            return &state_ctrl[item_id].states[i];
        }
    }

    ASSERT(false);
    return (OutCtrlrStateDef_t *)NULL;
}

/**
 * Initializes all data for all output controller state objects.
 *
 * @note Mutex control must be handled outside of this function
 */
static void InitControlData(void)
{
	for (unsigned int i = 0; i < (unsigned int)GEN_OUT_CTRL_ID_MAX; i++)
	{
		state_ctrl[(GenOutCtrlId_t)i].id = (GenOutCtrlId_t)i;
		state_ctrl[(GenOutCtrlId_t)i].num_states = 0;// No states defined yet.
        state_ctrl[(GenOutCtrlId_t)i].curr_state_obj = (OutCtrlrStateDef_t *)NULL;
        state_ctrl[(GenOutCtrlId_t)i].num_times_run = 0;
        state_ctrl[(GenOutCtrlId_t)i].test_state.curr_state_obj = (OutCtrlrStateDef_t *)NULL;
		state_ctrl[(GenOutCtrlId_t)i].test_state.is_active = false; // Test mode is not active.
        state_ctrl[(GenOutCtrlId_t)i].is_active = false;

		ResetControlData((GenOutCtrlId_t)i);
	}
}

/**
 * Resets all data relevant to a particular output state controller's state control.
 *
 * @param item_id   ID of the output control item that is to be modified
 *
 * @note Mutex control must be handled outside of this function
 * @note Be conscious of the enable/disable state of the controller.  It determines what
 *       this function does.
 */
static void ResetControlData(GenOutCtrlId_t item_id)
{
    state_ctrl[item_id].curr_index = 0;
    state_ctrl[item_id].time_elapsed_ms = 0;
    state_ctrl[item_id].one_shot_complete = false;
    state_ctrl[item_id].num_times_run = 0;

    // Only want to force output inactive if the controller is inactive.
    if (!state_ctrl[item_id].is_active)
    {
        state_ctrl[item_id].output_is_active = false;
        
        // Make sure the output is inactive.
        (void)GenOutCtrlBsp_SetInactive(item_id);
    }
}

/**
 * Locks the mutex, but only if the RTOS is running.
 */
#if defined(GENERAL_OUTPUT_CTRL_USE_MUTEX)
static void LockMutex(void)
{
    if (!bare_metal)
    {
        if (!AppRtosCb_MutexLock())
        {
            assert(false);
        }
    }
} //lint !e456
#endif

/**
 * Unlocks the mutex, but only if the RTOS is running.
 */
#if defined(GENERAL_OUTPUT_CTRL_USE_MUTEX)
static void UnlockMutex(void)
{
    if (!bare_metal)
    {
        if (!AppRtosCb_MutexUnlock())
        {
            assert(false);
        }
    }
}
#endif

/**
 * Default callback function. Used for any callbacks not defined by the applications.
 */
static bool DefaultCbFunc(void)
{
    return true;
}

// End of Doxygen grouping
/** @} */

#endif // #ifdef OK_TO_USE_OUTPUT_CONTROL

#endif

/**********************************************************************************************************************
---          End of File          ------          End of File          ------          End of File          ---
**********************************************************************************************************************/
