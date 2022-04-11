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

/**1
 ********************************************************************************************************
 *
 * @brief       Defines general output pin control API.
 * @file        general_output_ctrl.h
 * @author      tparsh
 * @ingroup		USER_DRIVER
 *
 ********************************************************************************************************
 */

#ifndef GENERAL_OUTPUT_CTRL_H_
#define GENERAL_OUTPUT_CTRL_H_

/**
 * @addtogroup USER_DRIVER
 * @{
 */

/*
 ********************************************************************************************************
 *                                           INCLUDE FILES
 ********************************************************************************************************
 */
/********************************************** System *************************************************/
#include <stdbool.h>
#include <stdint.h>
/********************************************    User   ************************************************/
#include "general_output_ctrl_cfg.h"
#include "general_output_ctrl.h"

/*
 ********************************************************************************************************
 *                                               DEFINES
 ********************************************************************************************************
 */
/****************************************** Symbolic Constants ******************************************/
#define GEN_OUT_CTRL_REPEAT_PATTERN_FOREVER_VAL ((GenOutCtrlTime_t)0)

#define GEN_OUT_CTRL_END_OF_STATE { GEN_OUT_CTRL_ALWAYS_IN_STATE_VAL, GEN_OUT_CTRL_ALWAYS_IN_STATE_VAL, GEN_OUT_CTRL_REPEAT_PATTERN_FOREVER_VAL }

/// Has an output controller in an active state forever.
#define GEN_OUT_CTRL_ALWAYS_ON { GEN_OUT_CTRL_ALWAYS_IN_STATE_VAL, 0, GEN_OUT_CTRL_REPEAT_PATTERN_FOREVER_VAL }

/// Has an output controller in an inactive state forever.
#define GEN_OUT_CTRL_ALWAYS_OFF { 0, GEN_OUT_CTRL_ALWAYS_IN_STATE_VAL, GEN_OUT_CTRL_REPEAT_PATTERN_FOREVER_VAL }

/// Must be the last entry in a state steps definition list. Defines end of the list.
#define GEN_OUT_CTRL_ALWAYS_IN_STATE_VAL ((GenOutCtrlTime_t)0xFFFF)

/*
 ********************************************************************************************************
 *                                               DATA TYPES
 ********************************************************************************************************
 */

typedef bool (*GenOutCtrl_AppCbFunc_t)(void);

/**
 * On/Off time for a state entry (a step in the state).
 *
 * @note An on/off time of GEN_OUT_CTRL_ALWAYS_IN_STATE_VAL marks the end of a state
*/
typedef struct
{
    /// Time to be on for this state.
    GenOutCtrlTime_t on_time_ms;

    /// Time to be off for this state
    GenOutCtrlTime_t off_time_ms;

    /// Number of times to run this stage.
    uint8_t num_times_to_run;
} GenOutCtrlStateStepDef_t;

/*
 ********************************************************************************************************
 *                                             FUNCTION PROTOTYPES
 ********************************************************************************************************
 */

#if defined(GENERAL_OUTPUT_CTRL_MODULE_ENABLE)
	bool GenOutCtrl_Init(bool is_bare_metal);
	bool GenOutCtrl_Deinit(void);
	bool GenOutCtrl_AddState(GenOutCtrlId_t item_id, GenOutState_t state, bool one_shot, uint8_t priority, GenOutCtrlStateStepDef_t *steps);
	bool GenOutCtrl_TickUpdateAll_ms(GenOutCtrlTime_t time_elapsed_ms);
	bool GenOutCtrl_TickUpdate_ms(GenOutCtrlTime_t time_elapsed_ms, GenOutCtrlId_t item_id);
	bool GenOutCtrl_StartAll(void);
	bool GenOutCtrl_StopAll(void);
	bool GenOutCtrl_Start(GenOutCtrlId_t item_id);
	bool GenOutCtrl_Stop(GenOutCtrlId_t item_id);
	bool GenOutCtrl_StateSetAll(GenOutState_t new_state);
	bool GenOutCtrl_StateSet(GenOutCtrlId_t item_id, GenOutState_t new_state);
    bool GenOutCtrl_AtLeastOneOutputCtrlrIsActive(void);
    bool GenOutCtrl_OutputCtrlrIsActive(GenOutCtrlId_t item_id);
	GenOutState_t GenOutCtrl_OutputCtrlrStateGet(GenOutCtrlId_t item_id);
    bool GenOutCtrl_TestModeSetAll(bool make_active);
	bool GenOutCtrl_TestModeSet(GenOutCtrlId_t item_id, bool make_active);

	// The functions below may only be called if the general output that is desired to be updated
	// has not been started with GenOutCtrl_Start() or GenOutCtrl_StartAll().
	bool GenOutCtrl_EnableAll(void);
	bool GenOutCtrl_Enable(GenOutCtrlId_t item_id);
	bool GenOutCtrl_DisableAll(void);
	bool GenOutCtrl_Disable(GenOutCtrlId_t item_id);
	bool GenOutCtrl_SetActive(GenOutCtrlId_t item_id);
	bool GenOutCtrl_SetInactive(GenOutCtrlId_t item_id);
	bool GenOutCtrl_ToggleActiveState(GenOutCtrlId_t item_id);

	// The functions below must be set early in system initialization.  This module will come up in bare metal mode
	// at system boot for low level initialization. The GenOutCtrl_Init() function must be called before use no matter what.
	void GenOutCtrl_AppBaremetalCb_Init_Set(GenOutCtrl_AppCbFunc_t callback);
	void GenOutCtrl_AppBaremetalCb_Deinit_Set(GenOutCtrl_AppCbFunc_t callback);
	void GenOutCtrl_AppRtosCb_Init_Set(GenOutCtrl_AppCbFunc_t callback);
	void GenOutCtrl_AppRtosCb_Deinit_Set(GenOutCtrl_AppCbFunc_t callback);

	#if defined(GENERAL_OUTPUT_CTRL_USE_MUTEX)
		void GenOutCtrl_AppRtosCb_MutexLock_Set(GenOutCtrl_AppCbFunc_t callback);
		void GenOutCtrl_AppRtosCb_MutexUnlock_Set(GenOutCtrl_AppCbFunc_t callback);
	#endif
#else
	// Effectively compile out any function calls in the application to the this module if the
	// this module is disabled.
	#define GenOutCtrl_Init(x) true
	#define GenOutCtrl_Deinit(void) true
	#define GenOutCtrl_TickUpdateAll_ms(x) true
	#define GenOutCtrl_TickUpdate_ms(x, y) true
	#define GenOutCtrl_StartAll() true
	#define GenOutCtrl_StopAll() true
	#define GenOutCtrl_Start(x) true
	#define GenOutCtrl_Stop(x) true
	#define GenOutCtrl_StateSetAll(x) true
	#define GenOutCtrl_StateSet(x, y) true
    #define GenOutCtrl_AtLeastOneOutputCtrlrIsActive() false
    #define GenOutCtrl_OutputCtrlrIsActive(x) false

	/// @note Special note here: the output controller who's state is being retrieved may not actually implement 'normal state'.  The application
	/// must account for this when this module is disabled!
	#define GenOutCtrl_OutputCtrlrStateGet(x) GEN_OUT_CTRL_STATE_IDLE
    #define GenOutCtrl_TestModeSetAll(x, y) true
	#define GenOutCtrl_TestModeSet(x) true

	#define GenOutCtrl_EnableAll() true
	#define GenOutCtrl_Enable(x) true
	#define GenOutCtrl_DisableAll() true
	#define GenOutCtrl_Disable(x) true
	#define GenOutCtrl_SetActive(x) true
	#define GenOutCtrl_SetInactive(x) true
	#define GenOutCtrl_ToggleActiveState(x) true

	#define GenOutCtrl_AppBaremetalCb_Init_Set(x)
	#define GenOutCtrl_AppBaremetalCb_Deinit_Set(x)
	#define GenOutCtrl_AppRtosCb_Init_Set(x)
	#define GenOutCtrl_AppRtosCb_Deinit_Set(x)

	#if defined(GENERAL_OUTPUT_CTRL_USE_MUTEX)
		#define GenOutCtrl_AppRtosCb_MutexLock_Set(x)
		#define GenOutCtrl_AppRtosCb_MutexUnlock_Set(x)
	#endif
#endif
// End of Doxygen grouping
/** @} */

#endif // End of GENERAL_OUTPUT_CTRL_H_

/**********************************************************************************************************************
---          End of File          ------          End of File          ------          End of File          ---
**********************************************************************************************************************/
