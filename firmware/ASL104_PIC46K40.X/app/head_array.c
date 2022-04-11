//////////////////////////////////////////////////////////////////////////////
//
// Filename: head_array.c
//
// Description: Core head array control and feedback.
//
// Author(s): Trevor Parsh (Embedded Wizardry, LLC)
//
// Modified for ASL on Date: 
//
//////////////////////////////////////////////////////////////////////////////


/* **************************   Header Files   *************************** */

// NOTE: This must ALWAYS be the first include in a file.
#include "device.h"

// from stdlib
#include <stdint.h>
#include <stdbool.h>
#include "user_assert.h"

// from RTOS
#include "cocoos.h"

// from project
#include "rtos_task_priorities.h"
#include "config.h"
#include "common.h"
#include "bsp.h"
#include "stopwatch.h"
#include "bluetooth_simple_if_bsp.h"
#include "eeprom_app.h"
//#include "dac_bsp.h"
//#include "general_output_ctrl_app.h"
#include "general_output_ctrl_bsp.h"
#include "app_common.h"
#include "beeper.h"

// from local
#include "head_array_bsp.h"
#include "head_array.h"

/* ******************************   Macros   ****************************** */

/* ***********************   File Scope Variables   *********************** */

struct 
{
    bool m_CurrentPadStatus;
    bool m_PreviousPadStatus;
    GenOutCtrlId_t m_LED_ID;
} g_PadInfo[HEAD_ARRAY_SENSOR_EOL];


/* ***********************   Function Prototypes   ************************ */

static void HeadArrayInputControlTask(void);

//static void MirrorUpdateDigitalInputValues(void);
//static void MirrorUpdateProportionalInputValues(void);
//static uint16_t ConvertPropInToOutValue(uint8_t sensor_id);

//static bool SyncWithEeprom(void);
//static void RefreshLimits(void);

#if defined(TEST_BASIC_DAC_CONTROL)
	static void TestBasicDacControl(void);
#endif

/* *******************   Public Function Definitions   ******************** */

//------------------------------------------------------------------------------
// Function: headArrayinit
//
// Description: Initializes this module.
//
//------------------------------------------------------------------------------
void headArrayinit(void)
{
	// Initialize other data
	for (int i = 0; i < (int)HEAD_ARRAY_SENSOR_EOL; i++)
	{
        g_PadInfo[i].m_CurrentPadStatus = false;
        g_PadInfo[i].m_PreviousPadStatus = false;
	}
    g_PadInfo[HEAD_ARRAY_SENSOR_LEFT].m_LED_ID = GEN_OUT_CTRL_ID_LEFT_PAD_LED;
    g_PadInfo[HEAD_ARRAY_SENSOR_RIGHT].m_LED_ID = GEN_OUT_CTRL_ID_RIGHT_PAD_LED;
    g_PadInfo[HEAD_ARRAY_SENSOR_CENTER].m_LED_ID = GEN_OUT_CTRL_ID_FORWARD_PAD_LED;
    g_PadInfo[HEAD_ARRAY_SENSOR_BACK].m_LED_ID = GEN_OUT_CTRL_ID_REVERSE_PAD_LED;
    
	// Initialize all submodules controlled by this module.
	headArrayBspInit();
	bluetoothSimpleIfBspInit();
    
    (void)task_create(HeadArrayInputControlTask, NULL, HEAD_ARR_MGMT_TASK_PRIO, NULL, 0, 0);
}

//------------------------------------------------------------------------------
// Function: headArrayDigitalInputValue
//
// Description: Returns the value of a digital sensor input from the last reading.
//
// NOTE: Do not use this for control or reporting related features that require taking into account
// NOTE: input->output mapping.
//
//------------------------------------------------------------------------------
bool headArrayDigitalInputValue(HeadArraySensor_t sensor)
{
	return g_PadInfo[sensor].m_CurrentPadStatus;
}

//------------------------------------------------------------------------------
// Function: headArrayPadIsConnected
//
// Description: Checks to see if a pad is connected.
//
//------------------------------------------------------------------------------
bool headArrayPadIsConnected(HeadArraySensor_t sensor)
{
	// NOTE: There's currently no way to tell if a pad is connected or not given hardware specs.
	// TODO: If/when hardware supports detecting "pad disconnected", add code to handle the case as well.
	return true;
}


/* ********************   Private Function Definitions   ****************** */

//------------------------------------------------------------------------------
// Function: HeadArrayInputControlTask
//
// Description: Does as the name suggests.
//
//------------------------------------------------------------------------------
static void HeadArrayInputControlTask(void)
{
    task_open();
//	void (*myState)(void);
    
	bool outputs_are_off = false;
	StopWatch_t neutral_sw;

	while (1)
	{
        // Get the current status of all pads
        for (int sensor_id = 0; sensor_id < (int)HEAD_ARRAY_SENSOR_EOL; sensor_id++)
        {
            g_PadInfo[sensor_id].m_CurrentPadStatus = headArrayBspDigitalState((HeadArraySensor_t)sensor_id);
        }
        
        // Prevent the Forward and Reverse pads active at the same time.
        if (g_PadInfo[HEAD_ARRAY_SENSOR_CENTER].m_CurrentPadStatus && g_PadInfo[HEAD_ARRAY_SENSOR_BACK].m_CurrentPadStatus)
        {
            g_PadInfo[HEAD_ARRAY_SENSOR_CENTER].m_CurrentPadStatus = false;
            g_PadInfo[HEAD_ARRAY_SENSOR_BACK].m_CurrentPadStatus = false;
        }

        // Prevent the Right and Left pads active at the same time.
        if (g_PadInfo[HEAD_ARRAY_SENSOR_LEFT].m_CurrentPadStatus && g_PadInfo[HEAD_ARRAY_SENSOR_RIGHT].m_CurrentPadStatus)
        {
            g_PadInfo[HEAD_ARRAY_SENSOR_LEFT].m_CurrentPadStatus = false;
            g_PadInfo[HEAD_ARRAY_SENSOR_RIGHT].m_CurrentPadStatus = false;
        }

        // For all sensors....
        //      Look for a change in state.
        //      If so, change the LED appropriately and beep if turning on.
        for (int sensor_id = 0; sensor_id < (int)HEAD_ARRAY_SENSOR_EOL; sensor_id++)
        {
            if (g_PadInfo[sensor_id].m_CurrentPadStatus != g_PadInfo[sensor_id].m_PreviousPadStatus)
            {
                if (g_PadInfo[sensor_id].m_CurrentPadStatus)
                {
                    GenOutCtrlBsp_SetActive(g_PadInfo[sensor_id].m_LED_ID);
                    beeperBeep (BEEPER_PATTERN_PAD_ACTIVE);
                }
                else
                {
                    GenOutCtrlBsp_SetInactive(g_PadInfo[sensor_id].m_LED_ID);
                }
                g_PadInfo[sensor_id].m_PreviousPadStatus = g_PadInfo[sensor_id].m_CurrentPadStatus;
            }            
        }        

        task_wait(MILLISECONDS_TO_TICKS(HEAD_ARRAY_TASK_DELAY));
	}
    task_close();
}

//------------------------------------------------------------------------------
// Function: PadsInNeutralState
//
// Description: Checks to see if the wheelchair is in a neutral wheelchair control state.
//
// NOTE: Must be called from a task/ISR.
//
//------------------------------------------------------------------------------
bool PadsInNeutralState(void)
{
    bool active = true;
	for (int i = 0; i < (int) HEAD_ARRAY_SENSOR_EOL; i++)
	{
        if (g_PadInfo[i].m_CurrentPadStatus)    // non-zero is active
            active = false;                 // Indicate that it's not in neutral
	}
    return active;
}

// end of file.
//-------------------------------------------------------------------------
