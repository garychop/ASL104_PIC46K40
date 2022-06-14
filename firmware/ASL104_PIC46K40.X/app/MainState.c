/*
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

#include <xc.h>

// NOTE: This must ALWAYS be the first include in a file.
#include "device.h"

// from RTOS
#include "cocoos.h"

// from stdlib
#include <stdint.h>
#include <stdbool.h>
#include "user_assert.h"

// from project
//#include "common.h"
#include "bsp.h"
#include "test_gpio.h"
#include "eeprom_app.h"
#include "head_array.h"
#include "beeper.h"
#include "user_button_bsp.h"
#include "user_button.h"
#include "general_output_ctrl_app.h"
#include "general_output_ctrl_bsp.h"
#include "ha_hhp_interface_app.h"
#include "app_common.h"

//#include "beeper_bsp.h"
#include "bluetooth_simple_if_bsp.h"
#include "inc/eFix_Communication.h"
#include "inc/rtos_task_priorities.h"
#include "Delay_Pot.h"

//------------------------------------------------------------------------------
// Defines and Macros 
//------------------------------------------------------------------------------

#define SEQUENCE_TIME 0
#define SEQUENCE_STATE 1
//------------------------------------------------------------------------------
// Local Variables
//------------------------------------------------------------------------------

static void (*MainState)(void);
static int g_StartupDelayCounter;
static uint16_t g_SwitchDelay;
static uint8_t g_ExternalSwitchStatus;
static int g_PulseDelay;
static int g_SequenceCounter;

//static BeepPattern_t g_BeepPatternRequest = BEEPER_PATTERN_EOL;
static uint8_t g_MainTaskID = 0;
static uint8_t g_BluetoothSetupStep = 0;
static uint8_t g_BluetoothSequenceTimer = 0;

// To enable the Bluetooth module, all three pads go:
// NOTE TO SELF: The signal gets inverted in the bluetooth_simple_if_bsp function call
//
//  low for 10ms (+/-5 ms)
//  high for 10ms (+/-5 ms)
//  low for 10ms (+/-5 ms)
//  high for 10ms (+/-5 ms)
//
//  low 50ms (+/-20 ms)
//  high for  50ms (+/-20 ms)
//  low for 50ms (+/-20 ms)
//  high for  50ms (+/-20 ms)
//  high for 70 ms.
//      Bluetooth device "latches" at 100 ms.
//
// Essentially, 2 short pulses followed by 2 longer pulses
static uint8_t g_BluetoothEnableSequence [][2] = 
{
    {100, false},   // Ensure that no pad signals are active.
    {10, true},     // high for 10 milliseconds.
    {10, false},    // low
    {10, true},     // high for 10
    {10, false},    // low for 50
    {50, true},     // high for 50
    {50, false},    // low for 50
    {50, true},     // high for 50
    {50, false},    // low for at least 50.
    {0,0}   // Must be at the end
};

// 2 short pulses followed by 1 longer pulses
static uint8_t g_BluetoothDisableSequence [][2] = 
{
    {100, false},   // Ensure that no pad signals are active.
    {10, true},     // high for 10 milliseconds.
    {10, false},    // low
    {10, true},     // high for 10
    {10, false},    // low for 50
    {50, true},     // high for 50
    {50, false},    // low for at least 50.
    {0,0}   // Must be at the end
};

// VersionTable
#define LONG_PULSE (350)
#define SHORT_PULSE (100)
#define OFF_PULSE (200)

static int g_VersionTable[][2] =
{
    {GEN_OUT_CTRL_ID_MAX, LONG_PULSE},
    {GEN_OUT_CTRL_ID_FORWARD_PAD_LED, LONG_PULSE},
    {GEN_OUT_CTRL_ID_MAX, OFF_PULSE},
    {GEN_OUT_CTRL_ID_FORWARD_PAD_LED, LONG_PULSE},
    {GEN_OUT_CTRL_ID_MAX, OFF_PULSE},
    {GEN_OUT_CTRL_ID_LEFT_PAD_LED, SHORT_PULSE},
    {GEN_OUT_CTRL_ID_MAX, OFF_PULSE},
    {GEN_OUT_CTRL_ID_LEFT_PAD_LED, SHORT_PULSE},
    {GEN_OUT_CTRL_ID_MAX,0}
};

//------------------------------------------------------------------------------
// Forward Declarations
//------------------------------------------------------------------------------
//static void NewTask (void);

//static void MainTaskInitialise(void);
static void MainTask (void);
static void MirrorDigitalInputOnBluetoothOutput(void);

// The following are the states
//static void Idle_State (void);

static void StartVersionAnnunciationState (void);
static void AnnunciateVersionState (void);

// State: Startup_State
//      Stay here until 500 milliseconds lapses then switch to OONAPU_State.
static void Startup_State (void);

// State: OONAPU_State
//      Stay here until No Pads are active then change to Driving State
static void OONAPU_Setup_State (void);
static void OONAPU_State (void);

// State: Driving_Setup_State
//      Stay here until User Port switch becomes inactive then
//          switch to Driving State.
static void Driving_Setup_State (void);
static void OON_Check_State (void);

// State: Driving_State
//      Stay here while reading Pads and send pad info to eFix Task.
//      If user port switch is active then
//          - Send BT beeping sequence.
//          - switch to Bluetooth Setup state.
static void Driving_State (void);
//static void Driving_UserSwitchActivated (void);
static void Driving_UserSwitch_State (void);
static void NoSwitchesThenDisabled_State (void);
static void Idle_State (void);
static void ExitIdleState (void);

static void Driving_ModeSwitch_State (void);
static void DriveMode_ModeSwitchDelay1 (void);
static void DriveMode_ModeSwitchDelay2 (void);
static void DriveMode_ModeSwitchDelay3 (void);
static void DriveMode_ModeSwitchDelay4 (void);
static void DriveMode_ModeSwitchDelay5 (void);
static void DriveMode_ModeSwitchDelay6 (void);

// State: BluetoothSetup_State
//      Stay here and user port switch becomes inactive then
//          - Switch to DoBluetooth_State
static void BluetoothSetup_State (void);
static void BluetoothEnable_State (void);

// State: DoBluetooth_State
//      Stay here and send active pad info to Bluetooth module.
//      If user port switch is active then
//          - Beep to annunciate driving.
//          - Switch to Driving_Setup_State
static void DoBluetooth_State (void);
static void BluetoothDisable_State (void);

static void ControlAll_BT_Pads (uint8_t active);
static void SetupBluetoothStartSequence (void);
static void SetupBluetoothStopSequence (void);

//-------------------------------------------------------------------------
// Main task
//-------------------------------------------------------------------------

void MainTaskInitialise(void)
{
    //g_BeepPatternRequest = BEEPER_PATTERN_EOL;
    g_StartupDelayCounter = 1000 / MAIN_TASK_DELAY;

    MainState = StartVersionAnnunciationState; // Startup_State;

    g_MainTaskID = task_create(MainTask , NULL, MAIN_TASK_PRIO, NULL, 0, 0);


}

//-------------------------------------------------------------------------
// Function: MainTask
// Description: This is the main task that controls everything.
//-------------------------------------------------------------------------

//uint16_t minADC = 0xffff, maxADC = 0x0;

static void MainTask (void)
{
    uint16_t adcValue;
    
    task_open();

    while (1)
	{
//        adcValue = ReadDelayPot();
//        if (adcValue > maxADC)
//            maxADC = adcValue;
//        if (adcValue < minADC)
//            minADC = adcValue;
        
        // Get the User and Mode port switch status all of the time.
        g_ExternalSwitchStatus = GetSwitchStatus();
        
        MainState();

        task_wait(MILLISECONDS_TO_TICKS(MAIN_TASK_DELAY));

    }
    
    task_close();
}

//-------------------------------------------------------------------------
//static void Idle_State (void)
//{
//    ++g_StartupDelayCounter;
//}

//-------------------------------------------------------------------------
// State: AnnunciateVersionState
// Description: this function annunciates the Version by executing the 
//      "instructions" in the VersionTable
//-------------------------------------------------------------------------
static void StartVersionAnnunciationState (void)
{
    g_SequenceCounter = 0;
    g_PulseDelay = g_VersionTable[g_SequenceCounter][1] / MAIN_TASK_DELAY;
    GenOutCtrlBsp_SetActive ((GenOutCtrlId_t) g_VersionTable[g_SequenceCounter][0]);
    
    MainState = AnnunciateVersionState;
}
static void AnnunciateVersionState (void)
{
    if (g_PulseDelay > 0)
    {
        --g_PulseDelay;
        if (g_PulseDelay == 0)
        {
            // turn off all LED's
            GenOutCtrlBsp_SetInactive (GEN_OUT_CTRL_ID_FORWARD_PAD_LED);
            GenOutCtrlBsp_SetInactive (GEN_OUT_CTRL_ID_REVERSE_PAD_LED);
            GenOutCtrlBsp_SetInactive (GEN_OUT_CTRL_ID_RIGHT_PAD_LED);
            GenOutCtrlBsp_SetInactive (GEN_OUT_CTRL_ID_LEFT_PAD_LED);
            ++g_SequenceCounter;
            g_PulseDelay = g_VersionTable[g_SequenceCounter][1] / MAIN_TASK_DELAY;
            if (g_PulseDelay == 0) // All done
                MainState = Startup_State;
            else
            {
                if (g_VersionTable[g_SequenceCounter][0] != GEN_OUT_CTRL_ID_MAX)
                    GenOutCtrlBsp_SetActive ((GenOutCtrlId_t) g_VersionTable[g_SequenceCounter][0]);
            }
        }
    }
    else
        MainState = Startup_State;

}

//-------------------------------------------------------------------------
// State: Startup_State
// Description: Stay here until 500 milliseconds lapses then switch to OONAPU_State
//      or to IDLE state to wait for user to press the switch.
//-------------------------------------------------------------------------
static void Startup_State (void)
{
    // If we need to startup differently, this is where you can do it.
    if (g_StartupDelayCounter > (500 / MAIN_TASK_DELAY))
        g_StartupDelayCounter = (500 / MAIN_TASK_DELAY);

    if (--g_StartupDelayCounter < 1) // Have we waited long enough.
    {
        if (Is_SW3_ON())    // If ON, power up with the chair's power.
        {
            GenOutCtrlBsp_SetActive (GEN_OUT_CTRL_ID_POWER_LED);  // Turn on the LED
            MainState = OONAPU_Setup_State;
        }
        else    // Go to a Drive Disable state.
        {
            GenOutCtrlBsp_SetInactive (GEN_OUT_CTRL_ID_POWER_LED);  // Turn off the LED
            MainState = Idle_State; // wait for push button.
        }
    }
}

//-------------------------------------------------------------------------

static void OONAPU_Setup_State (void)
{
    g_StartupDelayCounter = (500 / MAIN_TASK_DELAY);
    
    MainState = OONAPU_State;
    
}

//-------------------------------------------------------------------------
// State: OONAPU_State (Out-Of-Neutral-At-Power-Up acronym)
// Description: Stay here until No Pads are active then change to Driving State
//-------------------------------------------------------------------------
static void OONAPU_State (void)
{
    if (PadsInNeutralState())      // Yep, we are in neutral
    {
        // The following code ensures that the BT signals are NOT active
        // if leaving BT Control State while a PAD is active.
        MirrorDigitalInputOnBluetoothOutput();

        // Guard against a painfully long time out bug where we in Out-of-neutral state.
        if (g_StartupDelayCounter > (500 / MAIN_TASK_DELAY))
            g_StartupDelayCounter = (500 / MAIN_TASK_DELAY);
        
        if (--g_StartupDelayCounter < 1) // Have we waited long enough.
        {
            MainState = Driving_Setup_State;
        }
    }
    else
    {
        g_StartupDelayCounter = (500 / MAIN_TASK_DELAY);  // reset the counter.
    }
}

//-------------------------------------------------------------------------
// State: Driving_Setup_State
// Description: Stay here until User Port switch becomes inactive then
//          switch to Driving State.
//-------------------------------------------------------------------------
static void Driving_Setup_State (void)
{
    GenOutCtrlBsp_SetActive (GEN_OUT_CTRL_ID_POWER_LED);  // Turn ON power LED
    // Check the user port for go inactive.
    // When it does, go to the Driving State.
    if ((g_ExternalSwitchStatus & (USER_SWITCH | MODE_SWITCH)) == false)
    {
        //beeperBeep (ANNOUNCE_POWER_ON);
        MainState = Driving_State;  // Set to Driving state
    }
}

//-------------------------------------------------------------------------
// State: OON_Check_State
// Description: Wait here until the Pad and Switches are inactive then
//      proceed to driving.
//-------------------------------------------------------------------------
static void OON_Check_State (void)
{
    if (PadsInNeutralState())      // Yep, we are in neutral
    {
        if ((g_ExternalSwitchStatus & (USER_SWITCH | MODE_SWITCH)) == false)
        {
            MainState = Driving_State;  // Set to Driving state
        }
    }    
}
//-------------------------------------------------------------------------
// State: Driving_State
// Description: 
//      Stay here while reading Pads and sending pad info to eFix Task.
//      If user port switch is active then
//          - Send BT beeping sequence.
//          - switch to Bluetooth Setup state.
//-------------------------------------------------------------------------
static void Driving_State (void)
{
    int speedPercentage = 0, directionPercentage = 0;

    if (!PadsInNeutralState())      // Nope we are not in neutral
    {
        // Determine which is active and set the output accordingly.
        // Note that the Left/Right override is performed at the lower level.
        if (headArrayDigitalInputValue(HEAD_ARRAY_SENSOR_LEFT)) // Is Left pad active?
        {
            directionPercentage = -100;
        }
        else if (headArrayDigitalInputValue(HEAD_ARRAY_SENSOR_RIGHT)) // Is right pad active?
        {
            directionPercentage = 100;
        }
        if (headArrayDigitalInputValue(HEAD_ARRAY_SENSOR_CENTER)) // Is center pad active?
        {
            speedPercentage = 100;
        }
        else if (headArrayDigitalInputValue(HEAD_ARRAY_SENSOR_BACK)) // Is 4th back pad active?
        {
            if (Is_SW1_ON() == false)   // Only if SW1 is OFF
                speedPercentage = -100;
        }
    }
    
    // Check the user port for active... If so, change to Bluetooth state.
    if (g_ExternalSwitchStatus & USER_SWITCH)
    {
        speedPercentage = 0;        // Force no drive demand.
        directionPercentage = 0;

        // Turn off the Power LED
        GenOutCtrlBsp_SetInactive (GEN_OUT_CTRL_ID_POWER_LED);  // Turn off the LED
        beeperBeep (BEEPER_PATTERN_GOTO_IDLE);
        g_SwitchDelay = GetDelayTime() / MAIN_TASK_DELAY;
        if (g_SwitchDelay == 0)
        {
            MainState = NoSwitchesThenDisabled_State;
        }
        else
        {
            // Setup delay time.
            MainState = Driving_UserSwitch_State;
        }
    }
    if (g_ExternalSwitchStatus & MODE_SWITCH)
    {
        speedPercentage = 0;        // Force no drive demand.
        directionPercentage = 0;

        beeperBeep (BEEPER_PATTERN_RESUME_DRIVING);
        // Setup delay time based upon Delay Pot
        g_PulseDelay = 100 / MAIN_TASK_DELAY;   // 100 milliseconds
        GenOutCtrlBsp_SetActive (GEN_OUT_CTRL_ID_RESET_OUT);  // Turn off the LED
        MainState = Driving_ModeSwitch_State;
    }

#ifdef EFIX
    SetSpeedAndDirection (speedPercentage, directionPercentage);
#else
    if (speedPercentage > 0)    // Forward?
    {
        GenOutCtrlBsp_SetActive (GEN_OUT_CTRL_ID_FORWARD_DEMAND);     // Forward Digital Output to W/C
        GenOutCtrlBsp_SetInactive (GEN_OUT_CTRL_ID_REVERSE_DEMAND);     // Reverse Digital Output to W/C
    }
    else if (speedPercentage < 0)   // Reverse
    {
        GenOutCtrlBsp_SetInactive (GEN_OUT_CTRL_ID_FORWARD_DEMAND);     // Forward Digital Output to W/C
        GenOutCtrlBsp_SetActive (GEN_OUT_CTRL_ID_REVERSE_DEMAND);     // Reverse Digital Output to W/C
    }
    else    // Must be no speed demand
    {
        GenOutCtrlBsp_SetInactive (GEN_OUT_CTRL_ID_FORWARD_DEMAND);     // Forward Digital Output to W/C
        GenOutCtrlBsp_SetInactive (GEN_OUT_CTRL_ID_REVERSE_DEMAND);     // Reverse Digital Output to W/C
    }
    
    if (directionPercentage > 0)    // Right demand?
    {
        GenOutCtrlBsp_SetInactive (GEN_OUT_CTRL_ID_LEFT_DEMAND);        // Left Digital Output to W/C
        GenOutCtrlBsp_SetActive (GEN_OUT_CTRL_ID_RIGHT_DEMAND);       // Right Digital Output to W/C
    }
    else if (directionPercentage < 0)    // Left demand?
    {
        GenOutCtrlBsp_SetActive (GEN_OUT_CTRL_ID_LEFT_DEMAND);        // Left Digital Output to W/C
        GenOutCtrlBsp_SetInactive (GEN_OUT_CTRL_ID_RIGHT_DEMAND);       // Right Digital Output to W/C
    }
    else // Must be no directional demand
    {
        GenOutCtrlBsp_SetInactive (GEN_OUT_CTRL_ID_LEFT_DEMAND);        // Left Digital Output to W/C
        GenOutCtrlBsp_SetInactive (GEN_OUT_CTRL_ID_RIGHT_DEMAND);       // Right Digital Output to W/C
    }
#endif
}

//-------------------------------------------------------------------------
// NoSwitchesThenDisabled_State
//      Stay here until the switches are released and then goto
//      standby condition.
//-------------------------------------------------------------------------
static void NoSwitchesThenDisabled_State (void)
{
    if ((g_ExternalSwitchStatus & USER_SWITCH) == false)
    {
        MainState = Idle_State;
    }
}

//-------------------------------------------------------------------------
// Driving_UserSwitch_State
//      Stay here until
//      a. The delays expires then switch to Bluetooth active state.
//      b. The switch is released prior to delay expires... goto Idle state.
//-------------------------------------------------------------------------
static void Driving_UserSwitch_State(void)
{
    if (g_ExternalSwitchStatus & USER_SWITCH)
    {
        if (g_SwitchDelay != 0)      // Sanity check.
        {
            --g_SwitchDelay;
        }
        // Did we wait long enough to switch to Bluetooth
        if (g_SwitchDelay == 0)
        {
            beeperBeep (ANNOUNCE_BLUETOOTH);
            SetupBluetoothStartSequence();
            MainState = BluetoothEnable_State;
        }
    }
    else // The Switch is released before the Long Press occurred.
    {
        GenOutCtrlBsp_SetInactive (GEN_OUT_CTRL_ID_POWER_LED);  // Turn off the LED
        MainState = Idle_State;
    }
}

//-------------------------------------------------------------------------
// State: Idle_State
//      Remain here until the User Switch goes active then we are going
//      enable driving, but first, we are doing a OON test.
//-------------------------------------------------------------------------
static void Idle_State (void)
{
    if (g_ExternalSwitchStatus & USER_SWITCH)
    {
        GenOutCtrlBsp_SetActive (GEN_OUT_CTRL_ID_POWER_LED);  // Turn off the LED
        beeperBeep (ANNOUNCE_POWER_ON);
        g_SwitchDelay = GetDelayTime() / MAIN_TASK_DELAY;
        if (g_SwitchDelay == 0)
        {
            // Delay Pot is fully CCW, go startup and do nothing else.
            MainState = Driving_Setup_State;
        }
        else
        {   // We're going to exit this state and look  for long 
            // USER PORT switch closure.
            MainState = ExitIdleState;
        }
    }
}

//-------------------------------------------------------------------------
// State: ExitIdleState
//  Enter this state from Idle_State when the User PORT switch
//  has been pressed. Remain here until the USER PORT switch has been
//  released or the Press Timeout (pot) has expired in which case, enable
//  bluetooth operation.
//-------------------------------------------------------------------------
static void ExitIdleState (void)
{
    if (g_ExternalSwitchStatus & USER_SWITCH)
    {
        if (g_SwitchDelay != 0)      // Sanity check.
        {
            --g_SwitchDelay;
        }
        // Did we wait long enough to switch to Bluetooth
        if (g_SwitchDelay == 0)
        {
            beeperBeep (ANNOUNCE_BLUETOOTH);
            SetupBluetoothStartSequence();
            MainState = BluetoothEnable_State;
        }
    }
    else // The Switch is released before the Long Press occurred.
    {
        MainState = Driving_Setup_State;
    }
}

//------------------------------------------------------------------------------
// BluetoothEnable_State sends the sequence to the bluetooth module
//  to tell it enable its operations.
//------------------------------------------------------------------------------

static void BluetoothEnable_State (void)
{
    g_BluetoothSequenceTimer += MAIN_TASK_DELAY; // Counting up
    if (g_BluetoothSequenceTimer >= g_BluetoothEnableSequence[g_BluetoothSetupStep][SEQUENCE_TIME])
    {
        g_BluetoothSequenceTimer = 0;   // Reset the "timer"/counter
        ++g_BluetoothSetupStep;         // execute next step in sequence
        if (g_BluetoothEnableSequence[g_BluetoothSetupStep][SEQUENCE_TIME] != 0)    // ensure we don't go past end of sequence indicator.
        {
            ControlAll_BT_Pads (g_BluetoothEnableSequence[g_BluetoothSetupStep][SEQUENCE_STATE]);
        }
        else // Time must be 0, we're done sending the startup sequence.
        {
            MainState = BluetoothSetup_State;
        }
    }
}

//-------------------------------------------------------------------------
// State: BluetoothSetup_State
//  This state execute the Bluetooth Enable Sequence.
//-------------------------------------------------------------------------

static void BluetoothSetup_State (void)
{
    // Check the USER PORT SWITCH to be inactive before we change states.
    if ((g_ExternalSwitchStatus & USER_SWITCH) == false)
    {
        GenOutCtrlBsp_SetActive (GEN_OUT_CTRL_ID_POWER_LED);
        // Allow the Power LED to follow the Bluetooth LED On/Off pattern.
        //GenOutCtrlBsp_SetInactive (GEN_OUT_CTRL_ID_BT_LED);
        MainState = DoBluetooth_State;        // Set to Blue tooth state
    }
}

//-------------------------------------------------------------------------
// State: DoBluetooth
//      Stay here and send active pad info to Bluetooth module.
//      If user port switch is active then
//          - Switch to check for Out-of-Neutral State
//-------------------------------------------------------------------------

static void DoBluetooth_State (void)
{

    // Check the user port for active.
    if (g_ExternalSwitchStatus & USER_SWITCH)
    {
//        GenOutCtrlApp_SetStateAll (GEN_OUT_BLUETOOTH_DISABLED);
//        GenOutCtrlBsp_SetInactive (GEN_OUT_CTRL_ID_BT_LED);
        beeperBeep (BEEPER_PATTERN_GOTO_IDLE);
        // Setup delay time.
        g_SwitchDelay = GetDelayTime() / MAIN_TASK_DELAY;
        SetupBluetoothStopSequence();
        MainState = BluetoothDisable_State;
        GenOutCtrlBsp_SetInactive (GEN_OUT_CTRL_ID_POWER_LED);
    }
    else    // Still doing Bluetooth stuff
    {
        if (BT_LED_IsActive())
            GenOutCtrlBsp_SetInactive (GEN_OUT_CTRL_ID_POWER_LED);
        else
            GenOutCtrlBsp_SetActive (GEN_OUT_CTRL_ID_POWER_LED);
        MirrorDigitalInputOnBluetoothOutput();
    }
}

//------------------------------------------------------------------------------
// BluetoothDisable_State sends the sequence to the Bluetooth module to
// tell the Bluetooth module to disable it's operations.
//------------------------------------------------------------------------------

static void BluetoothDisable_State (void)
{
    g_BluetoothSequenceTimer += MAIN_TASK_DELAY; // Counting up
    if (g_BluetoothSequenceTimer >= g_BluetoothDisableSequence[g_BluetoothSetupStep][SEQUENCE_TIME])
    {
        g_BluetoothSequenceTimer = 0;   // Reset the "timer"/counter
        ++g_BluetoothSetupStep;         // execute next step in sequence
        if (g_BluetoothDisableSequence[g_BluetoothSetupStep][SEQUENCE_TIME] != 0)    // ensure we don't go past end of sequence indicator.
        {
            ControlAll_BT_Pads (g_BluetoothDisableSequence[g_BluetoothSetupStep][SEQUENCE_STATE]);
        }
        else // Time must be 0, we're done sending the startup sequence.
        {
            MainState = Driving_UserSwitch_State; // OONAPU_Setup_State; 
        }
    }
}

//------------------------------------------------------------------------------
// This state is initiated when in the driving state and the MODE SWITCH is
// pressed. In response, we will first initiate a 100 millisecond pulse.
//------------------------------------------------------------------------------
static void Driving_ModeSwitch_State (void)
{
    if (g_PulseDelay > 0)
    {
        --g_PulseDelay;
        if (g_PulseDelay == 0)
        {
            GenOutCtrlBsp_SetInactive (GEN_OUT_CTRL_ID_RESET_OUT);  // Turn off the LED
            g_SwitchDelay = GetDelayTime() / MAIN_TASK_DELAY;
            if (g_SwitchDelay < (1000 / MAIN_TASK_DELAY))   // At least one second.
                g_SwitchDelay = (1000 / MAIN_TASK_DELAY);
            MainState = DriveMode_ModeSwitchDelay1;
        }
    }
    else // it should never get here, bit if does, we got this covered.
    {
        MainState = Driving_Setup_State;    // Go back to driving.
    }
}

//------------------------------------------------------------------------------
// This state waits for:
//  - Mode switch to go inactive... switch to driving mode
//  - Delay timer to expire... activate double pulse.
//------------------------------------------------------------------------------

static void DriveMode_ModeSwitchDelay1 (void)
{
    if ((g_ExternalSwitchStatus & MODE_SWITCH))
    {
        if (g_SwitchDelay > 0) // Has the timer expired?
        {   // No, timer is still active
            --g_SwitchDelay;
        }
        else // timer expired
        {
            beeperBeep (BEEPER_PATTERN_RESUME_DRIVING);
            g_PulseDelay = 100 / MAIN_TASK_DELAY;   // 100 milliseconds
            GenOutCtrlBsp_SetActive (GEN_OUT_CTRL_ID_RESET_OUT);
            MainState = DriveMode_ModeSwitchDelay2;
        }
    }
    else    // If the mode switch is released before the delay timer expires,
            // go to driving state
    {
        MainState = Driving_Setup_State;    // Go back to driving.
    }
}

//------------------------------------------------------------------------------
// This state waits for:
//  - Pulse timer to expire. Clear Reset Line and setup wait again.
//------------------------------------------------------------------------------

static void DriveMode_ModeSwitchDelay2 (void)
{
    if (g_PulseDelay > 0)
    {
        --g_PulseDelay;
        if (g_PulseDelay == 0)
        {
            GenOutCtrlBsp_SetInactive (GEN_OUT_CTRL_ID_RESET_OUT);
            g_PulseDelay = 200 / MAIN_TASK_DELAY;   // 200 millisecond delay
            MainState = DriveMode_ModeSwitchDelay3;
        }
    }
    else // it should never get here, bit if does, we got this covered.
    {
        MainState = Driving_Setup_State;    // Go back to driving.
    }
}

//------------------------------------------------------------------------------
// This state waits for:
//  - Pulse timer to expire. Set Reset Line and setup wait again.
//------------------------------------------------------------------------------

static void DriveMode_ModeSwitchDelay3 (void)
{
    if (g_PulseDelay > 0)
    {
        --g_PulseDelay;
        if (g_PulseDelay == 0)
        {
            beeperBeep (BEEPER_PATTERN_RESUME_DRIVING);
            GenOutCtrlBsp_SetActive (GEN_OUT_CTRL_ID_RESET_OUT);
            g_PulseDelay = 100 / MAIN_TASK_DELAY;   // 200 millisecond delay
            MainState = DriveMode_ModeSwitchDelay4;
        }
    }
    else // it should never get here, bit if does, we got this covered.
    {
        MainState = Driving_Setup_State;    // Go back to driving.
    }
}

//------------------------------------------------------------------------------
// This state waits for:
//  - Pulse timer to expire. Set Reset Line and setup wait again.
//------------------------------------------------------------------------------

static void DriveMode_ModeSwitchDelay4 (void)
{
    if (g_PulseDelay > 0)
    {
        --g_PulseDelay;
        if (g_PulseDelay == 0)
        {
            GenOutCtrlBsp_SetInactive (GEN_OUT_CTRL_ID_RESET_OUT);  // Turn off the LED
            g_SwitchDelay = GetDelayTime() / MAIN_TASK_DELAY;
            if (g_SwitchDelay < (1000 / MAIN_TASK_DELAY))
                g_SwitchDelay = (1000 / MAIN_TASK_DELAY);
            MainState = DriveMode_ModeSwitchDelay5;
        }
    }
    else // it should never get here, bit if does, we got this covered.
    {
        MainState = Driving_Setup_State;    // Go back to driving.
    }
}

//------------------------------------------------------------------------------
// This state waits for:
//  - Mode switch to go inactive... switch to driving mode
//  - Delay timer to expire... activate double pulse.
//------------------------------------------------------------------------------

static void DriveMode_ModeSwitchDelay5 (void)
{
    if ((g_ExternalSwitchStatus & MODE_SWITCH))
    {
        if (g_SwitchDelay > 0) // Has the timer expired?
        {   // No, timer is still active
            --g_SwitchDelay;
        }
        else // timer expired
        {
            beeperBeep (ANNOUNCE_BEEPER_RNET_SLEEP);
            g_PulseDelay = 3000 / MAIN_TASK_DELAY;   // 100 milliseconds
            GenOutCtrlBsp_SetActive (GEN_OUT_CTRL_ID_RESET_OUT);  // Turn off the LED
            MainState = DriveMode_ModeSwitchDelay6;
        }
    }
    else    // If the mode switch is released before the delay timer expires,
            // go to driving state
    {
        MainState = Driving_Setup_State;    // Go back to driving.
    }
}

//------------------------------------------------------------------------------

static void DriveMode_ModeSwitchDelay6 (void)
{
    if (g_PulseDelay > 0)
    {
        --g_PulseDelay;
        if (g_PulseDelay == 0)
        {
            GenOutCtrlBsp_SetInactive (GEN_OUT_CTRL_ID_RESET_OUT);  // Turn off the LED
            MainState = Driving_Setup_State;
        }
    }
    else // it should never get here, bit if does, we got this covered.
    {
        MainState = Driving_Setup_State;    // Go back to driving.
    }
}

//------------------------------------------------------------------------------
// This function affect all 3 pad signals to the Bluetooth module.
//------------------------------------------------------------------------------

static void ControlAll_BT_Pads (uint8_t active)
{
	for (int i = 0; i < (int) HEAD_ARRAY_SENSOR_EOL; i++)
	{
		bluetoothSimpleIfBspPadMirrorStateSet((HeadArraySensor_t)i, (active == true));
	}
}

//------------------------------------------------------------------------------
// This function sets up the information for stopping the Bluetooth operation
//------------------------------------------------------------------------------

static void SetupBluetoothStartSequence (void)
{
    g_BluetoothSetupStep = 0;
    g_BluetoothSequenceTimer = 0;
    ControlAll_BT_Pads (g_BluetoothEnableSequence[g_BluetoothSetupStep][SEQUENCE_STATE]);
}

//------------------------------------------------------------------------------
// This function sets up the information for stopping the Bluetooth operation
//------------------------------------------------------------------------------

static void SetupBluetoothStopSequence (void)
{
    //beeperBeep (ANNOUNCE_BLUETOOTH);
    g_BluetoothSetupStep = 0;
    g_BluetoothSequenceTimer = 0;
    ControlAll_BT_Pads (g_BluetoothDisableSequence[g_BluetoothSetupStep][SEQUENCE_STATE]);
}

//------------------------------------------------------------------------------
// Function: MirrorDigitalInputOnBluetoothOutput
//
// Description: Mirrors digital pad inputs on Bluetooth digital output lines. No mapping, just a one-to-one map.
//
//------------------------------------------------------------------------------

static void MirrorDigitalInputOnBluetoothOutput(void)
{
	for (HeadArraySensor_t i = 0; i < HEAD_ARRAY_SENSOR_EOL; i++)
	{
// This is removed to allow 4 pads to control the 4 Bluetooth (pad) control signals.
//        if (headArrayDigitalInputValue(HEAD_ARRAY_SENSOR_BACK)) // Is 4th back pad active?
//        {
//            if (Is_SW1_ON() == true)   // if SW1 is ON
//                continue;
//        }
        bluetoothSimpleIfBspPadMirrorStateSet(i, headArrayDigitalInputValue(i));
	}
}

//------------------------------------------------------------------------------
// Returns true if the Main State engine is in a condition that beeping
// is appropriate.
//------------------------------------------------------------------------------
bool Does_Main_Allow_Beeping (void)
{
    return (MainState != Idle_State);   // This is the only time to quiet the beeping
}

