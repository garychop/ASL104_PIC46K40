//////////////////////////////////////////////////////////////////////////////
//
// Filename: beeper.c
//
// Description: Provides beeper control to the application.
//
// Author(s): Trevor Parsh (Embedded Wizardry, LLC)
//
// Modified for ASL on Date: 
//
//////////////////////////////////////////////////////////////////////////////


/* **************************   Header Files   *************************** */

// NOTE: This must ALWAYS be the first include in a file.
#include "device.h"

#include <stdint.h>
#include <stdbool.h>
#include "user_assert.h"

// from RTOS
#include "cocoos.h"

// from project
#include "rtos_task_priorities.h"
#include "common.h"
#include "stopwatch.h"
#include "app_common.h"

// from local
#include "beeper_bsp.h"
#include "beeper.h"

/* ******************************   Types   ******************************* */

// Defines the "beep" type
typedef struct
{
	// On time in milliseconds.
	TimerTick_t on_time_ms;
	
	// Off time in milliseconds.
	TimerTick_t off_time_ms;
} Beep_t;

/* ***********************   File Scope Variables   *********************** */

static uint8_t blocking_task_id;
static volatile BeepPattern_t session_pattern;
static volatile uint8_t curr_session_cycle;
static bool beeper_is_running = false;
static BeepPattern_t curr_session_pattern;

static StopWatch_t beep_stopwatch = {0, false};

static Beep_t beep_pattern_function[] =
{
	{50, 0}
};

// NOTE: Make sure that the values in the table below are multiples of the BeepPatternTask task execution rate.
static Beep_t beep_pattern_long_press[] =
{
	{25, 50},
	{25, 50},
	{50, 0}
};

// NOTE: Make sure that the values in the table below are multiples of the BeepPatternTask task execution rate.
static Beep_t beep_pattern_eeprom_not_init_on_boot[] =
{
	{25, 50},
	{50, 0}
};

static Beep_t g_AnnouncePowerOnOffFeature[] =
{
//    {25, 50},
//    {25, 50},
//    {50, 300},
    {150, 0}
};
static Beep_t g_AnnounceBluetoothFeature[] =
{
//    {25, 50},
//    {25, 50},
//    {50, 300},
    {100, 200},
    {100, 200}
};
static Beep_t g_AnnounceNextFunctionFeature[] =
{
//    {25, 50},
//    {25, 50},
//    {50, 300},
    {100, 200},
    {100, 200},
    {100, 200}
};
static Beep_t g_AnnounceNextProfileFeature[] =
{
//    {25, 50},
//    {25, 50},
//    {50, 300},
    {100, 200},
    {100, 200},
    {100, 200},
    {100, 200}
};
static Beep_t g_AnnounceRNetFeature[] =
{
    {100, 200},
    {100, 200},
    {100, 200},
    {100, 200},
    {100, 200}
};
static Beep_t g_AnnounceModeSwitchActive[] =
{
    {37, 50},
    {25, 0}
};

static Beep_t g_AnnounceRNet_Sleep_Feature[] =
{
    {50, 200},
    {50, 200},
    {50, 200},
    {100, 200},
    {100, 200},
    {100, 200}
};

// Priorities for beep patterns.  If a beep pattern of higher priority than one that's running
// is requested, then the higher priority one takes over.
//
// NOTE: Must match BeepPattern_t exactly
static const bool beep_pattern_prio[(int)BEEPER_PATTERN_EOL] =
{
    3,  // ANNOUNCE_POWER_ON
    4,  // ANNOUNCE_BLUETOOTH
    5,  // ANNOUNCE_NEXT_FUNCTION
    6,  // ANNOUNCE_NEXT_PROFILE
    8,  // BEEPER_PATTERN_RNET_MNEU_ACTIVE
	1,	// BEEPER_PATTERN_USER_BUTTON_SHORT_PRESS
	2,	// BEEPER_PATTERN_USER_BUTTON_LONG_PRESS
    7,  // BEEPER_PATTERN_MODE_ACTIVE
    9,  // ANNOUNC_RNET_SLEEP_FEATURE
	0	// BEEPER_PATTERN_EEPROM_NOT_INIT_ON_BOOT
};

static Evt_t os_event_start_beep_seq_id;
static Evt_t os_event_beep_seq_complete;
static uint8_t beeper_task_id;
static volatile bool signal_calling_task;

// Protects access to critical sections of code in this module
static volatile Sem_t data_lock_mutex;

/* ***********************   Function Prototypes   ************************ */

static void BeepPatternTask(void);
static Evt_t BeepPatternStart(BeepPattern_t pattern);
static Evt_t BeepPatternStop(void);
static bool CanRunBeepPattern(BeepPattern_t pattern);
static uint8_t BeepPatternNumCycles(BeepPattern_t pattern);
static Beep_t *BeepPatternPatternGet(BeepPattern_t pattern);
static void StartBeepPattern(void);

/* *******************   Public Function Definitions   ******************** */

//-------------------------------
// Function: beeperInit
//
// Description: Initializes this module.
//
//-------------------------------
void beeperInit(void)
{
	beeperBspInit();
	
	blocking_task_id = NO_TID;
	session_pattern = BEEPER_PATTERN_EOL;
	curr_session_pattern = BEEPER_PATTERN_EOL;
	curr_session_cycle = 0;
	beeper_is_running = false;
	signal_calling_task = false;

    data_lock_mutex = sem_bin_create(1); // Set up so the first task to try and take the semaphore succeeds

	os_event_start_beep_seq_id = event_create();
	os_event_beep_seq_complete = event_create();
    beeper_task_id = task_create(BeepPatternTask, NULL, BEEPER_MGMT_TASK_PRIO, NULL, 0, 0 );
}

//-------------------------------
// Function: beeperBeep
//
// Description: Beeps a pattern, in a non-blocking way.
//
// Note: If a beep session is running, new ones are ignored unless of higher priority
//
//-------------------------------
Evt_t beeperBeep(BeepPattern_t pattern)
{
	if (appCommonSoundEnabled())
	{
		return BeepPatternStart(pattern);
	}
	else
	{
		return NO_EVENT;
	}
}

//-------------------------------
// Function: beeperBeepBlocking
//
// Description: Beeps a pattern, in a blocking way.
//
// Usage:
// event_signal(BeeperBeepBlocking(PATTERN));
// event_wait(BeeperWaitUntilPatternCompletes());
//
//-------------------------------
Evt_t beeperBeepBlocking(BeepPattern_t pattern)
{
	if (appCommonSoundEnabled())
	{
		// TODO: Test this when we need it. It is written, but untested
		signal_calling_task = true;
		blocking_task_id = running_tid;

		return BeepPatternStart(pattern);
	}
	else
	{
		return NO_EVENT;
	}
}

//-------------------------------
// Function: BeeperWaitUntilPatternCompletes
//
// Description: See beeperBeepBlocking
//
//-------------------------------
Evt_t BeeperWaitUntilPatternCompletes(void)
{
	return os_event_beep_seq_complete;
}

/* ********************   Private Function Definitions   ****************** */


//-------------------------------
// Function: BeepPatternTick
//
// Description: Handles state control for a beep session.
//
//	Must be called from within a stable timing construct.
//
//-------------------------------
static void BeepPatternTask(void)
{
    task_open();

	uint32_t wait_time_ms = MILLISECONDS_TO_TICKS(25);

	// Someone desires a beep sequence on boot.  So, we shall go ahead and carry out the sequence.
	if (session_pattern != BEEPER_PATTERN_EOL)
	{
		StartBeepPattern();
	}

    while (1)
    {
		// Look for signals from other tasks or wait some amount of time if running a beep pattern.
		if (!beeper_is_running)
		{
			event_wait(os_event_start_beep_seq_id);
		}
		else
		{
			event_wait_timeout(os_event_start_beep_seq_id, wait_time_ms);
		}

		if (curr_session_pattern != session_pattern)
		{
			if (session_pattern == BEEPER_PATTERN_EOL)
			{
				// Stop the all the beeping!
				Evt_t notif_evt_id = BeepPatternStop();

				if (notif_evt_id != NO_EVENT)
				{
					event_signal(notif_evt_id);
					signal_calling_task = false;
				}
			}
			else if (CanRunBeepPattern(session_pattern))
			{
				if (beeper_is_running)
				{
					// Stop the current beep pattern
					Evt_t notif_evt_id = BeepPatternStop();

					if (notif_evt_id != NO_EVENT)
					{
						event_signal(notif_evt_id);
						signal_calling_task = false;
					}
				}

				StartBeepPattern();
			}
			
			session_pattern = curr_session_pattern;
		}

		if (beeper_is_running)
		{
			Beep_t *curr_beep_pattern = BeepPatternPatternGet(session_pattern);
			TimerTick_t session_time_elapsed_in_cycle_ms = stopwatchTimeElapsed(&beep_stopwatch, false);
			
			if (session_time_elapsed_in_cycle_ms < curr_beep_pattern[curr_session_cycle].on_time_ms)
			{
				wait_time_ms = MILLISECONDS_TO_TICKS(curr_beep_pattern[curr_session_cycle].on_time_ms - session_time_elapsed_in_cycle_ms);
			}
			else
			{
				// Turn beeper off if it is not already.
				if (beeperBspActiveGet())
				{
					beeperBspActiveSet(false);
				}
				
				uint16_t time_to_be_off = curr_beep_pattern[curr_session_cycle].on_time_ms +
										  curr_beep_pattern[curr_session_cycle].off_time_ms;

				if ((curr_beep_pattern[curr_session_cycle].off_time_ms == 0) ||
					(session_time_elapsed_in_cycle_ms >= time_to_be_off))
				{
					// See if there is a next cycle or if the beep pattern is finished.
					curr_session_cycle++;

					if (curr_session_cycle < BeepPatternNumCycles(session_pattern))
					{
						if (!beeperBspActiveGet())
						{
							stopwatchZero(&beep_stopwatch);
							beeperBspActiveSet(true);
							
							wait_time_ms = MILLISECONDS_TO_TICKS(curr_beep_pattern[curr_session_cycle].on_time_ms);
						}
					}
					else
					{
						// Stop the beep pattern
						Evt_t notif_evt_id = BeepPatternStop();

						if (notif_evt_id != NO_EVENT)
						{
							event_signal(notif_evt_id);
							signal_calling_task = false;
						}
					}
				}
				else
				{
					wait_time_ms = time_to_be_off - session_time_elapsed_in_cycle_ms;
				}
			}
		}
	}
    task_close();
}

//-------------------------------
// Function: BeepPatternStart
//
// Description: 
//
// NOTE: Could put sem locks between here and the task that carries out the action. But,
// NOTE: there should never be conflicts given the current definition of the system.
//
//-------------------------------
static Evt_t BeepPatternStart(BeepPattern_t pattern)
{
	session_pattern = pattern;

	if (os_running())
	{
		return os_event_start_beep_seq_id;
	}
	else
	{
		return NO_EVENT;
	}
}

//-------------------------------
// Function: BeepPatternStop
//
// Description: 
//
//-------------------------------
static Evt_t BeepPatternStop(void)
{
	session_pattern = BEEPER_PATTERN_EOL;
	curr_session_pattern = BEEPER_PATTERN_EOL;
	beeper_is_running = false;
	stopwatchStop(&beep_stopwatch);
	
	// Want to make sure that the beeper is off after releasing control of the beeper.
	if (beeperBspActiveGet())
	{
		beeperBspActiveSet(false);
	}

	if (signal_calling_task)
	{
		return os_event_beep_seq_complete;
	}
	else
	{
		return NO_EVENT;
	}
}

//-------------------------------
// Function: CanRunBeepPattern
//
// Description: 
//
//-------------------------------
static bool CanRunBeepPattern(BeepPattern_t pattern)
{
	if (!beeper_is_running || (beep_pattern_prio[(int)pattern] > beep_pattern_prio[(int)session_pattern]))
	{
		return true;
	}
	else
	{
		return false;
	}
}

//-------------------------------
// Function: 
//
// Description: 
//
// NOTE: Would rather use a pointer to Beep_t arrays, but CCS does not support that type of construct.
//
//-------------------------------
static uint8_t BeepPatternNumCycles(BeepPattern_t pattern)
{
	uint8_t ret_val;

	switch (pattern)
	{
		case BEEPER_PATTERN_USER_BUTTON_SHORT_PRESS:
			ret_val = NUM_ELEMENTS_IN_ARR(beep_pattern_function);
			break;
		
		case BEEPER_PATTERN_USER_BUTTON_LONG_PRESS:
			ret_val = NUM_ELEMENTS_IN_ARR(beep_pattern_long_press);
			break;

		case BEEPER_PATTERN_EEPROM_NOT_INIT_ON_BOOT:
			ret_val = NUM_ELEMENTS_IN_ARR(beep_pattern_eeprom_not_init_on_boot);
			break;

        case ANNOUNCE_POWER_ON:
            ret_val = NUM_ELEMENTS_IN_ARR(g_AnnouncePowerOnOffFeature);
            break;
            
        case ANNOUNCE_BLUETOOTH:
            ret_val = NUM_ELEMENTS_IN_ARR(g_AnnounceBluetoothFeature);
            break;
            
        case ANNOUNCE_NEXT_FUNCTION:
            ret_val = NUM_ELEMENTS_IN_ARR(g_AnnounceNextFunctionFeature);
            break;
            
        case ANNOUNCE_NEXT_PROFILE:
            ret_val = NUM_ELEMENTS_IN_ARR(g_AnnounceNextProfileFeature);
            break;
            
        case BEEPER_PATTERN_MODE_ACTIVE:
            ret_val = NUM_ELEMENTS_IN_ARR(g_AnnounceModeSwitchActive);
            break;

        case ANNOUNCE_RNET_SEATING_ACTIVE:
            ret_val = NUM_ELEMENTS_IN_ARR(g_AnnounceRNetFeature);
            break;
            
        case ANNOUNCE_BEEPER_RNET_SLEEP:
            ret_val = NUM_ELEMENTS_IN_ARR(g_AnnounceRNet_Sleep_Feature);
            break;
            
		default:
			ASSERT(pattern == BEEPER_PATTERN_EEPROM_NOT_INIT_ON_BOOT);
			ret_val = 0;
			break;
	}

	return ret_val;
}

//-------------------------------
// Function: 
//
// Description: 
//
// NOTE: Would rather use a pointer to Beep_t arrays, but CCS does not support that type of construct.
//
//-------------------------------
static Beep_t *BeepPatternPatternGet(BeepPattern_t pattern)
{
	Beep_t *ret_val;

	switch (pattern)
	{
		case BEEPER_PATTERN_USER_BUTTON_SHORT_PRESS:
			ret_val = (Beep_t *)beep_pattern_function;
			break;

		case BEEPER_PATTERN_USER_BUTTON_LONG_PRESS:
			ret_val = (Beep_t *)beep_pattern_long_press;
			break;

		case BEEPER_PATTERN_EEPROM_NOT_INIT_ON_BOOT:
			ret_val = (Beep_t *)beep_pattern_eeprom_not_init_on_boot;
			break;

        case ANNOUNCE_POWER_ON:
            ret_val = (Beep_t*) g_AnnouncePowerOnOffFeature;
            break;
            
        case ANNOUNCE_BLUETOOTH:
            ret_val = (Beep_t*) g_AnnounceBluetoothFeature;
            break;
            
        case ANNOUNCE_NEXT_FUNCTION:
            ret_val = (Beep_t*) g_AnnounceNextFunctionFeature;
            break;
            
        case ANNOUNCE_NEXT_PROFILE:
            ret_val = (Beep_t*) g_AnnounceNextProfileFeature;
            break;
			
        case BEEPER_PATTERN_MODE_ACTIVE:
            ret_val = (Beep_t*) g_AnnounceModeSwitchActive;
            break;

        case ANNOUNCE_RNET_SEATING_ACTIVE:
            ret_val = (Beep_t*) g_AnnounceNextProfileFeature; // It's the same
                        //.. they can't be active at the same time.
            break;

        case ANNOUNCE_BEEPER_RNET_SLEEP:
            ret_val = (Beep_t*) g_AnnounceRNet_Sleep_Feature;
            break;
            
		default:
			ASSERT(pattern == BEEPER_PATTERN_EEPROM_NOT_INIT_ON_BOOT);

			ret_val = (Beep_t *)beep_pattern_function;
			break;
	}

	return ret_val;
}

//-------------------------------
// Function: StartBeepPattern
//
// Description: Kicks off a beep sequence
//
//-------------------------------
static void StartBeepPattern(void)
{
	beeper_is_running = true;
	curr_session_pattern = session_pattern;
	curr_session_cycle = 0;
	stopwatchStart(&beep_stopwatch);
	
	beeperBspActiveSet(true);

	curr_session_pattern = session_pattern;
}


// end of file.
//-------------------------------------------------------------------------
