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
#include "dac_bsp.h"
#include "general_output_ctrl.h"
#include "general_output_ctrl_app.h"
#include "app_common.h"

// from local
#include "head_array_bsp.h"
#include "head_array.h"

/* ******************************   Macros   ****************************** */

// The values below are based on empherical data gathered from a single rev3 board head array.
// The spec is: Vref/Neutral must be 5V <= 6V <= 7V
// 				The mix/max must be +/- 1.2V from Vref/Neutral values.
// NOTE: Vref refers to the voltage reference supplied to the DB9 connector that goes to the IN500
// NOTE: module. Also, the Vref and DAC output for neutral state must be VERY close to the same voltage.
// NOTE: The exact specs for "how close" is not known.
//OBSOLETED in EEPROM Version 2
//#define DAC_NEUTRAL_DAC_VAL 					((uint16_t)2009)    // Without IC1 which reduces the offset.
//#define DAC_NEUTRAL_DAC_VAL 					((uint16_t)2270)    // With IC1 which increases the offset.
//#define DAC_UPPER_AND_BOTTOM_RAIL_DIFFERENTIAL	((uint16_t)410)

// This is the portion of output control that proportional has when the output control for a pad
// is set to proportional.
//#define DAC_UPPER_AND_BOTTOM_RAIL_DIFFERENTIAL_80_PERCENT	((DAC_UPPER_AND_BOTTOM_RAIL_DIFFERENTIAL * (uint16_t)8) / (uint16_t)10)
//#define DAC_UPPER_AND_BOTTOM_RAIL_DIFFERENTIAL_20_PERCENT	((DAC_UPPER_AND_BOTTOM_RAIL_DIFFERENTIAL * (uint16_t)2) / (uint16_t)10)

//#define DAC_LOWER_RAIL							(DAC_NEUTRAL_DAC_VAL - DAC_UPPER_AND_BOTTOM_RAIL_DIFFERENTIAL)
//#define DAC_UPPER_RAIL							(DAC_NEUTRAL_DAC_VAL + DAC_UPPER_AND_BOTTOM_RAIL_DIFFERENTIAL)

//#define DAC_LEFT_MAX_DAC_VAL 					DAC_LOWER_RAIL
//#define DAC_RIGHT_MAX_DAC_VAL 				DAC_UPPER_RAIL
//#define DAC_FWD_MAX_DAC_VAL 					DAC_UPPER_RAIL
//#define DAC_REV_MAX_DAC_VAL 					DAC_LOWER_RAIL

// Reducing these items their equivalent value. It's either a +1 or -1.
// Left = LOWER_RAIL =  -1
// Right = UPPER RAIL = 1
// Forward = UPPER RAIL = 1
// Reverse = LOWER RAIL = -1
#define DAC_LEFT_DAC_VAL_MANIP_DIR (-1)     // ((DAC_LEFT_MAX_DAC_VAL > DAC_NEUTRAL_DAC_VAL) 	? ((int8_t)1) : ((int8_t)-1))
#define DAC_RIGHT_DAC_VAL_MANIP_DIR (1)     // ((DAC_RIGHT_MAX_DAC_VAL > DAC_NEUTRAL_DAC_VAL) 	? ((int8_t)1) : ((int8_t)-1))
#define DAC_FWD_DAC_VAL_MANIP_DIR (1)		//	((DAC_FWD_MAX_DAC_VAL > DAC_NEUTRAL_DAC_VAL) 	? ((int8_t)1) : ((int8_t)-1))
#define DAC_REV_DAC_VAL_MANIP_DIR (-1)		//	((DAC_REV_MAX_DAC_VAL > DAC_NEUTRAL_DAC_VAL) 	? ((int8_t)1) : ((int8_t)-1))

#define PROP_UNINITIALIZED_VAL					(0)
#define PROP_ERROR_STATE_VALUE                  (0x3ff) // This will be used to indicate a failure of this pad.

// Time to wait before claiming the system is in a neutral control state.
#define NEUTRAL_STATE_MIN_TIME_TO_CHECK_ms 		(750)   // 2 seconds is too long.

/* ***********************   File Scope Variables   *********************** */

// Type of input, or input disabled, function for each pad.
// Default values are set in eeprom_app.c
static volatile HeadArrayInputType_t head_arr_input_type[(int)HEAD_ARRAY_SENSOR_EOL];

// Each index refers to the input pad, and the value at an index refers to the output pad.
// Default values are set in eeprom_app.c
static volatile HeadArrayOutputFunction_t input_pad_to_output_pad_map[(int)HEAD_ARRAY_SENSOR_EOL];

static volatile FunctionalFeature_t curr_active_feature;

// Last recorded values for digital and proportional input states.
static volatile bool pad_dig_state[(int)HEAD_ARRAY_SENSOR_EOL];
static volatile uint16_t pad_prop_state[(int)HEAD_ARRAY_SENSOR_EOL];
static volatile uint16_t pad_raw_prop_state[(int)HEAD_ARRAY_SENSOR_EOL];
static volatile uint16_t pad_adc_min_thresh_val[(int)HEAD_ARRAY_SENSOR_EOL];
static volatile uint16_t pad_adc_max_thresh_val[(int)HEAD_ARRAY_SENSOR_EOL];

static volatile uint16_t pad_min_adc_val[(int)HEAD_ARRAY_SENSOR_EOL];
static volatile uint16_t pad_max_adc_val[(int)HEAD_ARRAY_SENSOR_EOL];
static volatile uint16_t pad_min_thresh_perc[(int)HEAD_ARRAY_SENSOR_EOL];
static volatile uint16_t pad_max_thresh_perc[(int)HEAD_ARRAY_SENSOR_EOL];
static volatile uint16_t pad_MinDriveSpeed[(int)HEAD_ARRAY_SENSOR_EOL];
static volatile uint16_t DAC_Proportional_percent[(int)HEAD_ARRAY_SENSOR_EOL];
static volatile uint16_t DAC_Minimum_percent[(int)HEAD_ARRAY_SENSOR_EOL];

static volatile uint16_t neutral_DAC_counts;        // This holds the DAC counts constant, used as the center of the DAC output.
static volatile uint16_t neutral_DAC_setting;       // The DAC counts used.
static volatile uint16_t neutral_DAC_range;         // This is the allowable range of the voltage swing in counts.
static volatile uint16_t DAC_lower_rail;
static volatile uint16_t DAC_upper_rail;
static volatile uint16_t DAC_max_forward_counts;
static volatile uint16_t DAC_max_left_counts;
static volatile uint16_t DAC_max_right_counts;
static volatile uint16_t DAC_max_reverse_counts;
//static volatile uint16_t DAC_Proportional_percent;
//static volatile uint16_t DAC_Minimum_percent;

// Must match exactly with the ordering in HeadArrayOutputFunction_t.
static const int8_t dac_output_manip_dir[(int)HEAD_ARRAY_OUT_FUNC_EOL] =
{
	DAC_LEFT_DAC_VAL_MANIP_DIR, DAC_RIGHT_DAC_VAL_MANIP_DIR, DAC_FWD_DAC_VAL_MANIP_DIR, DAC_REV_DAC_VAL_MANIP_DIR
};

static volatile bool wait_for_neutral = false;
static volatile bool neutral_test_fail = false;

static uint16_t g_HeartBeatTimeoutCounter;

FunctionalFeature_t g_CurrentFeature, g_NewFeature;
int g_PowerUpDelay = 0;

/* ***********************   Function Prototypes   ************************ */

static void HeadArrayInputControlTask(void);
static void CheckInputs(void);
static bool SetOutputs(void);

static bool SendStateRequestToLedControlModule(void);
static void MirrorUpdateDigitalInputValues(void);
static void MirrorUpdateProportionalInputValues(void);
static void MirrorDigitalInputOnBluetoothOutput(void);
static uint16_t ConvertPropInToOutValue(uint8_t sensor_id);
static void SetBT_DemandsToInactive(void);

static bool InNeutralState(void);
static void RefreshLimits(void);

#if defined(TEST_BASIC_DAC_CONTROL)
	static void TestBasicDacControl(void);
#endif

/* *******************   Public Function Definitions   ******************** */

//-------------------------------
// Function: headArrayinit
//
// Description: Initializes this module.
//
//-------------------------------
void headArrayinit(void)
{
	// Initialize other data
	for (int i = 0; i < (int)HEAD_ARRAY_SENSOR_EOL; i++)
	{
		pad_dig_state[i] = false;
		pad_prop_state[i] = PROP_UNINITIALIZED_VAL;
		pad_raw_prop_state[i] = PROP_UNINITIALIZED_VAL;
	}

    g_HeartBeatTimeoutCounter = 0;
    
	// Initialize all submodules controlled by this module.
	headArrayBspInit();
	bluetoothSimpleIfBspInit();
	
    (void)task_create(HeadArrayInputControlTask, NULL, HEAD_ARR_MGMT_TASK_PRIO, NULL, 0, 0);
}

//-------------------------------
// Function: headArrayOutputValue
//
// Description: Returns the value (digital or ADC value of proportional depending on setting) of a proportional sensor input from the last reading.
//		For digital, the value will either be 0 or whatever the max ADC value is, depending on the active/inactive state.
//
//-------------------------------
uint16_t headArrayOutputValue(HeadArrayOutputAxis_t axis_id)
{
	uint16_t out_val = neutral_DAC_setting;
    float attendantDemand;

    // If any pad is in error (not connected), return a Neutral DAC setting.
    // This catches a spurious signal on an opposite pad when one of the pads
    // is in error. Yes, it does happen and it does disengages the breaks but
    // does not supply enough demand to move the chair, but this prevents
    // any movement at all.
    if ((!headArrayPadIsConnected(HEAD_ARRAY_SENSOR_LEFT)) ||
        (!headArrayPadIsConnected(HEAD_ARRAY_SENSOR_RIGHT)) ||
        (!headArrayPadIsConnected(HEAD_ARRAY_SENSOR_CENTER)))
    {
        return out_val;
    }
    
	if (axis_id == HEAD_ARRAY_OUT_AXIS_LEFT_RIGHT)
	{        
        {
            for (uint8_t sensor_id = 0; sensor_id < (int)HEAD_ARRAY_SENSOR_EOL; sensor_id++)
            {
                // If pad is not connected, don't bother taking it into account.
                if (headArrayPadIsConnected((HeadArraySensor_t)sensor_id))
                {
                    if ((input_pad_to_output_pad_map[sensor_id] == HEAD_ARRAY_OUT_FUNC_LEFT) ||
                        (input_pad_to_output_pad_map[sensor_id] == HEAD_ARRAY_OUT_FUNC_RIGHT))
                    {
                        // Process RNet_SEATING feature here. If it's RNet_SEATING then
                        // .. force a digital implementation ONLY.
                        if (appCommonGetCurrentFeature() == FUNC_FEATURE_RNET_SEATING)
                        {
                            // Only care about an input sensor affecting output if it is active.
                            if (headArrayDigitalInputValue((HeadArraySensor_t)sensor_id))
                            {
                                out_val += (uint16_t)dac_output_manip_dir[(int)input_pad_to_output_pad_map[sensor_id]] * neutral_DAC_range;
                            }
                        }
                        else if (head_arr_input_type[sensor_id] == HEAD_ARR_INPUT_DIGITAL)
                        {
                            // Only care about an input sensor affecting output if it is active.
                            if (headArrayDigitalInputValue((HeadArraySensor_t)sensor_id))
                            {
                                out_val += (uint16_t)dac_output_manip_dir[(int)input_pad_to_output_pad_map[sensor_id]] * neutral_DAC_range;
                            }
                        }
                        else if (head_arr_input_type[sensor_id] == HEAD_ARR_INPUT_PROPORTIONAL)
                        {
                            out_val += (uint16_t)dac_output_manip_dir[(uint16_t)input_pad_to_output_pad_map[sensor_id]] *
                                           (uint16_t)ConvertPropInToOutValue(sensor_id);
                        }
                        else // Should never happen
                        {
                            // Input is of no care.
                            (void)0;
                        }
                    }
                    else
                    {
                        // It is either, none, forward, or backwards. Don't care.
                        (void)0;
                    }
                }
            }
        } // end else if not in attandent control
	}
	else // HEAD_ARRAY_OUT_AXIS_FWD_REV
	{
        {
            for (uint8_t sensor_id = 0; sensor_id < (int)HEAD_ARRAY_SENSOR_EOL; sensor_id++)
            {
                // If pad is not connected, don't bother taking it into account.
                if (headArrayPadIsConnected((HeadArraySensor_t)sensor_id))
                {
                    if ((input_pad_to_output_pad_map[sensor_id] == HEAD_ARRAY_OUT_FUNC_FWD) ||
                        (input_pad_to_output_pad_map[sensor_id] == HEAD_ARRAY_OUT_FUNC_REV))
                    {
                        // Process RNet_SEATING feature here. If it's RNet_SEATING then
                        // .. force the Forward/Reverse drive demand to be Neutral.
                        if (appCommonGetCurrentFeature() == FUNC_FEATURE_RNET_SEATING)
                        {
                            // out_val = neutral_DAC_setting;
                        }
                        else if (head_arr_input_type[sensor_id] == HEAD_ARR_INPUT_DIGITAL)
                        {
                            // Only care about an input sensor affecting output if it is active.
                            if (headArrayDigitalInputValue((HeadArraySensor_t)sensor_id))
                            {
                                out_val += (uint16_t)dac_output_manip_dir[(int)input_pad_to_output_pad_map[sensor_id]] * neutral_DAC_range;
                            }
                        }
                        else if (head_arr_input_type[sensor_id] == HEAD_ARR_INPUT_PROPORTIONAL)
                        {
                            out_val += (uint16_t)dac_output_manip_dir[(int)input_pad_to_output_pad_map[sensor_id]] *
                                           (uint16_t)ConvertPropInToOutValue(sensor_id);
                            // If the attendant active and assisting, then add the attendant demand
                        }
                        else // Should never happen
                        {
                            // Input is of no care.
                            (void)0;
                        }
                    }
                    else
                    {
                        // It is either, none, left, or right. Don't care.
                        (void)0;
                    }
                }
            }
        }
        // Check to see if any FWD/REV pad is active. If not, see if the
        // the Mode Reverse feature is active and if the mode switch is active.
        // We will issue a neutral demand if both are active.
#if 0
        if (eeprom8bitGet(EEPROM_STORED_ITEM_ENABLED_FEATURES_2) & FUNC_FEATURE2_MODE_REVERSE_BIT_MASK)
        {
            if (out_val == neutral_DAC_setting) // Is a forward or reverse pad active?
            {
                if (IsModeSwitchActive())       // Is the Mode switch active
                {
					out_val += (uint16_t)dac_output_manip_dir[HEAD_ARRAY_OUT_FUNC_REV] * neutral_DAC_range;
                }
            }
            else // We have a forward or reverse demand
            {
                if (IsModeSwitchActive())       // Is the Mode switch active?
                	out_val = neutral_DAC_setting; // Issue a neutral demand if multiple pads are active.
            }
        }
#endif
	}

	if (out_val < DAC_lower_rail)
	{
		out_val = DAC_lower_rail;
	}
	else if (out_val > DAC_upper_rail)
	{
		out_val = DAC_upper_rail;
	}
	else
	{
		// Nothing to do. The value is already in an acceptable range.
		(void)0;
	}
	
	return out_val;
}

//-------------------------------
// Function: headArrayDigitalInputValue
//
// Description: Returns the value of a digital sensor input from the last reading.
//
// NOTE: Do not use this for control or reporting related features that require taking into account
// NOTE: input->output mapping.
//
//-------------------------------
bool headArrayDigitalInputValue(HeadArraySensor_t sensor)
{
	return pad_dig_state[(int)sensor];
}

//-------------------------------
// Function: headArrayProportionalInputValueRaw
//
// Description: Returns the value of a raw proportional sensor input from the last reading.
//
// NOTE: Do not use this for control or reporting related features that require taking into account
// NOTE: input->output mapping.
//
//-------------------------------
uint16_t headArrayProportionalInputValueRaw(HeadArraySensor_t sensor)
{
	return pad_raw_prop_state[(int)sensor];
}

//-------------------------------
// Function: headArrayProportionalInputValue
//
// Description: Returns the value of a proportional sensor input from the last reading.
//
// NOTE: Do not use this for control or reporting related features that require taking into account
// NOTE: input->output mapping.
//
//-------------------------------
uint16_t headArrayProportionalInputValue(HeadArraySensor_t sensor)
{
	return pad_prop_state[(int)sensor];
}

//-------------------------------
// Function: headArrayPadIsConnected
//
// Description: Checks to see if a pad is connected.
//
//-------------------------------
bool headArrayPadIsConnected(HeadArraySensor_t sensor)
{
    // Let's use the max adc value to indicate a failure.
    if (PROP_ERROR_STATE_VALUE != pad_raw_prop_state[sensor])
        return true;
	return false;       // It's disconnected or failed
}

//-------------------------------
// Function: headArrayNeutralTestFail
//
// Description: Let's caller know whether or not the system is in a "neutral fail" state.
//
//-------------------------------
bool headArrayNeutralTestFail(void)
{
	return neutral_test_fail;
}

//-------------------------------
// Function: SetNeedForNeutralTest (void)
//
// Description: This sets a global flag that tells the Head Array Control task
//      to perform a neutral test.
//-------------------------------
void SetNeedForNeutralTest (void)
{
	wait_for_neutral = true;
}

/* ********************   Private Function Definitions   ****************** */

static void SetBT_DemandsToInactive(void)
{
    GenOutCtrlApp_SetStateAll(GEN_OUT_CTRL_BT_FORWARD_DEMAND_INACTIVE);
    GenOutCtrlApp_SetStateAll(GEN_OUT_CTRL_BT_LEFT_DEMAND_INACTIVE);
    GenOutCtrlApp_SetStateAll(GEN_OUT_CTRL_BT_RIGHT_DEMAND_INACTIVE);
}

//-------------------------------
// Function: HeadArrayInputControlTask
//
// Description: Does as the name suggests.
//
//-------------------------------

static void HeadArrayInputControlTask(void)
{

    task_open();
	
	bool outputs_are_off = false;
	StopWatch_t neutral_sw;

	// Start LED blinky sequence if needed.
	if (SendStateRequestToLedControlModule())
	{
		event_signal(genOutCtrlAppWakeEvent());
	}
	
    // Set outputs to neutral at power up.
    dacBspSet(DAC_SELECT_FORWARD_BACKWARD, neutral_DAC_setting);
    dacBspSet(DAC_SELECT_LEFT_RIGHT, neutral_DAC_setting);
    
    // Wait for head array to be in a neutral state on boot
//    if (appCommonIsPowerUpInIdleEnabled() == false)
//        wait_for_neutral = true;

    // Get the currently active feature
    g_CurrentFeature = appCommonGetCurrentFeature();
    // If BT, then make it different so we can send the BT Enable
    // Sequence to the BT Module.
    if (g_CurrentFeature == FUNC_FEATURE_OUT_CTRL_TO_BT_MODULE)
        ++g_CurrentFeature;

    GenOutCtrlApp_SetStateAll(GEN_OUT_CTRL_BT_FORWARD_DEMAND_INACTIVE);
    GenOutCtrlApp_SetStateAll(GEN_OUT_CTRL_BT_LEFT_DEMAND_INACTIVE);
    GenOutCtrlApp_SetStateAll(GEN_OUT_CTRL_BT_RIGHT_DEMAND_INACTIVE);
    
	while (1)
	{

        CheckInputs();

        g_NewFeature = appCommonGetCurrentFeature();

        // Check to see if we are entering or entering Bluetooth operation
        // If so, we need to enable or disable the Bluetooth module.
        // We need this short delay to allow the Control Task to initialize
        // the outputs before we send BT module a sequence.
        if (++g_PowerUpDelay > 20)
        {
            g_PowerUpDelay = 20;    // Prevent rollover
            if (g_NewFeature != g_CurrentFeature)
            {
                if (g_CurrentFeature == FUNC_FEATURE_OUT_CTRL_TO_BT_MODULE)
                {
                    // We need to send Disable sequence to BT Module
                    GenOutCtrlApp_SetStateAll(GEN_OUT_BLUETOOTH_DISABLED);
                }
                if (g_NewFeature == FUNC_FEATURE_OUT_CTRL_TO_BT_MODULE)
                {
                    // We need to send Enable Sequence to BT Module
                    GenOutCtrlApp_SetStateAll(GEN_OUT_BLUETOOTH_ENABLED);
                }
                g_CurrentFeature = g_NewFeature;

                // This forces this task to NOT execute the "SetOutputs" function
                // which allows the Enable Sequence to complete.
                wait_for_neutral = true;
            }
        }        
		if (wait_for_neutral)
		{
			if (!stopwatchIsActive(&neutral_sw))
			{
				stopwatchStart(&neutral_sw);
				
				bluetoothSimpleIfBspPadMirrorDisable();
			}
            // Shut down all outputs to system.
            dacBspSet(DAC_SELECT_FORWARD_BACKWARD, neutral_DAC_setting);
            dacBspSet(DAC_SELECT_LEFT_RIGHT, neutral_DAC_setting);

			// Need to wait for all pads to be inactive before controlling the wheelchair for safety reasons.
			if (InNeutralState())
			{
				if (stopwatchTimeElapsed(&neutral_sw, false) >= NEUTRAL_STATE_MIN_TIME_TO_CHECK_ms)
				{
					stopwatchStop(&neutral_sw);
					neutral_test_fail = false;
                    wait_for_neutral = false;
				}
			}
			else
			{
				stopwatchZero(&neutral_sw);
				neutral_test_fail = true;
			}
		}
		else
		{
			if (SetOutputs() && !outputs_are_off)
			{
				outputs_are_off = true;

				// Shut down all outputs.
				dacBspSet(DAC_SELECT_FORWARD_BACKWARD, neutral_DAC_setting);
				dacBspSet(DAC_SELECT_LEFT_RIGHT, neutral_DAC_setting);
				//bluetoothSimpleIfBspPadMirrorDisable();
			}
			else if (outputs_are_off)
			{
				// We're back in an active output control state
				outputs_are_off = false;
			}
		}

        task_wait(MILLISECONDS_TO_TICKS(20));
	}
    task_close();
}

//-------------------------------
// Function: CheckInputs
//
// Description: Checks the input values from the head array.
//
//-------------------------------
static void CheckInputs(void)
{
#if defined(TEST_BASIC_DAC_CONTROL)
	// Never returns.
	TestBasicDacControl();  
#endif

	MirrorUpdateProportionalInputValues();
	MirrorUpdateDigitalInputValues();
}

//-------------------------------
// Function: SetOutputs
//
// Description: Sets outputs to their appropriate values, based on system state and input values.
//
//-------------------------------

static bool SetOutputs(void)
{
	bool turn_outputs_off = true;
	int16_t out_val;
    float attendantDemand;

	if (1) // AppCommonDeviceActiveGet())
	{
		// "Power is on"
		if (1) // (!AppCommonCalibrationActiveGet())
		{
			// Not in calibration mode.
			switch ((FunctionalFeature_t)0) // eepromEnumGet(EEPROM_STORED_ITEM_CURRENT_ACTIVE_FEATURE))
			{
				case FUNC_FEATURE_POWER_ON_OFF:
				case FUNC_FEATURE_OUT_NEXT_FUNCTION:
				case FUNC_FEATURE_OUT_NEXT_PROFILE:
                case FUNC_FEATURE_RNET_SEATING:
                case FUNC_FEATURE_EOL:  // This occurs if no features are shown in the main screen
					// Direct control of the wheelchair from this device
					dacBspSet(DAC_SELECT_FORWARD_BACKWARD, headArrayOutputValue(HEAD_ARRAY_OUT_AXIS_FWD_REV));
					dacBspSet(DAC_SELECT_LEFT_RIGHT, headArrayOutputValue(HEAD_ARRAY_OUT_AXIS_LEFT_RIGHT));
					turn_outputs_off = false;
					break;
				
				case FUNC_FEATURE_OUT_CTRL_TO_BT_MODULE:
					// Control sent to a Bluetooth module.
                    // Check to see if the BT Enable Sequence is executing
                    // We don't want to interrupt it with Demands.
                    if (AppCommonGetBTSequenceActive () == false)
                    {
                        MirrorDigitalInputOnBluetoothOutput();
                    }
                    
                    dacBspSet(DAC_SELECT_FORWARD_BACKWARD, neutral_DAC_setting);
                    dacBspSet(DAC_SELECT_LEFT_RIGHT, neutral_DAC_setting);
					turn_outputs_off = false;
					break;
				
				default:
					// Nothing to do.
					break;
			}
		}
		else
		{
			// In calibration mode. Do not want to control the wheelchair right now!
			(void)0;
		}
	}
	else
	{
		// "Power is off". Better be in FUNC_FEATURE_POWER_ON_OFF state...
		// TODO: Put check here to ensure we're in FUNC_FEATURE_POWER_ON_OFF state
        // The following are required because the DAC output is sticky otherwise.
        dacBspSet(DAC_SELECT_FORWARD_BACKWARD, neutral_DAC_setting);
        dacBspSet(DAC_SELECT_LEFT_RIGHT, neutral_DAC_setting);
	}

	return turn_outputs_off;
}

//-------------------------------
// Function: SendStateRequestToLedControlModule
//
// Description: Determines what event needs to be sent to the LED state controller that reflects this
//		module's state and sends it on over.
//
//-------------------------------
static bool SendStateRequestToLedControlModule(void)
{
	GenOutState_t led_ctrl_state;

    // If the Head Array is allowed to issue drive commands, turn on Green LED.
	if (curr_active_feature == FUNC_FEATURE_OUT_CTRL_TO_BT_MODULE)
	{
		led_ctrl_state = GEN_OUT_CTRL_STATE_BLUETOOTH_OUTPUT;
	}
	else
	{
		led_ctrl_state = GEN_OUT_CTRL_STATE_HEAD_ARRAY_ACTIVE;
	}
    
    GenOutCtrlApp_SetStateAll(led_ctrl_state);

	return genOutCtrlAppNeedSendEvent();
}

//-------------------------------
// Function: MirrorUpdateDigitalInputValues
//
// Description: Update digital input values in RAM that communicate to the rest of the system what
//		the value of each input is.
//
//-------------------------------
static void MirrorUpdateDigitalInputValues(void)
{
	for (int sensor_id = 0; sensor_id < (int)HEAD_ARRAY_SENSOR_EOL; sensor_id++)
	{
		pad_dig_state[sensor_id] = headArrayBspDigitalState((HeadArraySensor_t)sensor_id);
	}
}

//-------------------------------
// Function: MirrorUpdateProportionalInputValues
//
// Description: Update proportional input values in RAM that communicate to the rest of the system what
//		the value of each input is.
//
// NOTE: ADC reads 0x02 when a pad is connected, and 0x00 when disconnected.
//		 It would be good to have a better distinction, but, well, there ya go.
//
//-------------------------------
static void MirrorUpdateProportionalInputValues(void)
{
	for (int sensor_id = 0; sensor_id < (int)HEAD_ARRAY_SENSOR_EOL; sensor_id++)
	{
        // Did we latch the errant condition?
        if (PROP_ERROR_STATE_VALUE != pad_raw_prop_state[sensor_id])
        {
    		pad_raw_prop_state[sensor_id] = headArrayBspAnalogState((HeadArraySensor_t)sensor_id);
        }

        if (pad_raw_prop_state[sensor_id] > 0x3f0)  // Are we at a level that indicates a failure?
        {
            pad_raw_prop_state[sensor_id] = PROP_ERROR_STATE_VALUE; // "latch" failure
			pad_prop_state[sensor_id] = 0;          // ensure no demand for this pad.
        }
        else
        {
            // For safety, make sure that min/max thresholds make sense before using them to control the output value.
            if ((pad_raw_prop_state[sensor_id] < pad_adc_min_thresh_val[sensor_id]) ||
                (pad_adc_max_thresh_val[sensor_id] <= pad_adc_min_thresh_val[sensor_id]))
            {
                pad_prop_state[sensor_id] = 0;
            }
            else if (pad_raw_prop_state[sensor_id] > pad_adc_max_thresh_val[sensor_id])
            {
                pad_prop_state[sensor_id] = 100;
            }
            else
            {
                uint16_t range = pad_adc_max_thresh_val[sensor_id] - pad_adc_min_thresh_val[sensor_id];

                // In between min and max limits
                pad_prop_state[sensor_id] = (uint16_t)(((uint32_t)100 * (uint32_t)(pad_raw_prop_state[sensor_id] - pad_adc_min_thresh_val[sensor_id])) / (uint32_t)range);
            }
        }
	}
}

//-------------------------------
// Function: MirrorDigitalInputOnBluetoothOutput
//
// Description: Mirrors digital pad inputs on Bluetooth digital output lines. No mapping, just a one-to-one map.
//
//-------------------------------

static void MirrorDigitalInputOnBluetoothOutput(void)
{
    // Process each pad for active or inactive
    if (pad_dig_state[HEAD_ARRAY_SENSOR_LEFT])
    {
        GenOutCtrlApp_SetStateAll(GEN_OUT_CTRL_BT_LEFT_DEMAND_ACTIVE);
    }
    else
    {
        GenOutCtrlApp_SetStateAll(GEN_OUT_CTRL_BT_LEFT_DEMAND_INACTIVE);
    }

    if (pad_dig_state[HEAD_ARRAY_SENSOR_RIGHT])
    {
        GenOutCtrlApp_SetStateAll(GEN_OUT_CTRL_BT_RIGHT_DEMAND_ACTIVE);
    }
    else
    {
        GenOutCtrlApp_SetStateAll(GEN_OUT_CTRL_BT_RIGHT_DEMAND_INACTIVE);
    }
    
    if (pad_dig_state[HEAD_ARRAY_SENSOR_CENTER])
    {
        GenOutCtrlApp_SetStateAll(GEN_OUT_CTRL_BT_FORWARD_DEMAND_ACTIVE);
    }
    else
    {
        GenOutCtrlApp_SetStateAll(GEN_OUT_CTRL_BT_FORWARD_DEMAND_INACTIVE);
    }
}

//-------------------------------
// Function: ConvertPropInToOutValue
//
// Description: Converts input proportional value to DAC output value.
//
//-------------------------------
static uint16_t ConvertPropInToOutValue(uint8_t sensor_id)
{
	uint16_t ret_val = 0;
    
    if ((pad_prop_state[sensor_id] != 0) || (headArrayDigitalInputValue((HeadArraySensor_t)sensor_id)))
    {
        ret_val = DAC_Minimum_percent[sensor_id];

        // The on-scale is 20%-100%. Where, the first 20% is always there if the digital input is active (and it
        // MUST be active in order for the proportional value to be considered) and the other 80% of control
        // comes for the proportional value.
        // As of Feb 1, 2020, the percentage is programmable is considered in RefreshLimits()
        ret_val += (pad_prop_state[sensor_id] * DAC_Proportional_percent[sensor_id]) / (uint16_t)100;
    }
	return ret_val;
}

//-------------------------------
// Function: InNeutralState
//
// Description: Checks to see if the wheelchair is in a neutral wheelchair control state.
//
// NOTE: Must be called from a task/ISR.
//
//-------------------------------
static bool InNeutralState(void)
{
	if ((headArrayOutputValue(HEAD_ARRAY_OUT_AXIS_LEFT_RIGHT) == neutral_DAC_setting) &&
		(headArrayOutputValue(HEAD_ARRAY_OUT_AXIS_FWD_REV) == neutral_DAC_setting))
	{
		return true;
	}
	else
	{
		return false;
	}
}

//-------------------------------
// Function: RefreshLimits
//
// Description: Set the DAC variables and constants
//
//-------------------------------
static void RefreshLimits(void)
{
    
	DAC_lower_rail = neutral_DAC_setting - neutral_DAC_range;
	DAC_upper_rail = neutral_DAC_setting + neutral_DAC_range;

	DAC_max_forward_counts = DAC_upper_rail;
	DAC_max_left_counts = DAC_lower_rail;
	DAC_max_right_counts = DAC_upper_rail;
	DAC_max_reverse_counts = DAC_lower_rail;
}

#if defined(TEST_BASIC_DAC_CONTROL)
//-------------------------------
// Function: TestBasicDacControl
//
// Description: Makes sure that we are able to properly set the DAC output values.
//
//-------------------------------
static void TestBasicDacControl(void)
{
	#define NUM_DAC_STEPS ((uint16_t)10)
	#define DAC_MAX_VALUE (((uint16_t)1 << 12) - 1)
	#define DAC_STEP_SIZE (DAC_MAX_VALUE / NUM_DAC_STEPS)

	while (1)
	{
#if 0
		// Use this case to make sure the output values set are what are expected.
		// This case makes it easy to see the transitions and settling levels.
		
		for (uint16_t i = 0; i < NUM_DAC_STEPS; i++)
		{
			dacBspSet(DAC_SELECT_FORWARD_BACKWARD, i * DAC_STEP_SIZE);
			dacBspSet(DAC_SELECT_LEFT_RIGHT, i * DAC_STEP_SIZE);
			bspDelayMs(75);
		}
		
		// Set to absolute max value.
		dacBspSet(DAC_SELECT_FORWARD_BACKWARD, DAC_MAX_VALUE);
		dacBspSet(DAC_SELECT_LEFT_RIGHT, DAC_MAX_VALUE);
		bspDelayMs(75);
#elif 0
		// See min/max/neutral are setting properly.
		dacBspSet(DAC_SELECT_FORWARD_BACKWARD, DAC_LOWER_RAIL);
		dacBspSet(DAC_SELECT_LEFT_RIGHT, DAC_LOWER_RAIL);
		bspDelayMs(1000);
		dacBspSet(DAC_SELECT_FORWARD_BACKWARD, neutral_DAC_setting);
		dacBspSet(DAC_SELECT_LEFT_RIGHT, neutral_DAC_setting);
		bspDelayMs(1000);
		dacBspSet(DAC_SELECT_FORWARD_BACKWARD, DAC_UPPER_RAIL);
		dacBspSet(DAC_SELECT_LEFT_RIGHT, DAC_UPPER_RAIL);
		bspDelayMs(1000);
#else
		// use this case to make sure that the output can be changed at the fastest rate this program can
		// without issues.

//		for (uint16_t i = 0; i < NUM_DAC_STEPS; i++)
//		{
//			dacBspSet(DAC_SELECT_FORWARD_BACKWARD, i * DAC_STEP_SIZE);
//			dacBspSet(DAC_SELECT_LEFT_RIGHT, i * DAC_STEP_SIZE);
//		}
		
		// Set to absolute max value.
		dacBspSet(DAC_SELECT_FORWARD_BACKWARD, DAC_MAX_VALUE);
		dacBspSet(DAC_SELECT_LEFT_RIGHT, DAC_MAX_VALUE);
        
        
		dacBspSet(DAC_SELECT_FORWARD_BACKWARD, 2048);
		dacBspSet(DAC_SELECT_FORWARD_BACKWARD, 2048);
#endif
	}
}
#endif

// end of file.
//-------------------------------------------------------------------------
