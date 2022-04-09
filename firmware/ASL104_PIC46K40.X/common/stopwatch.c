//////////////////////////////////////////////////////////////////////////////
//
// Filename: stopwatch.c
//
// Description: Generic stopwatch for general use.
//
// Author(s): Trevor Parsh (Embedded Wizardry, LLC)
//
// Modified for ASL on Date: 
//
// Use notes for simple timeout timer application:
// Initialization:
//      StopWatch_t sw;
//      stopwatchZero(&sw);
// 
// Superloop/Task:
//      ; Waiting for 2 seconds
//      if (stopwatchTimeUntilLimit(&sw, 2000)
//      {
//          ; Timed out
//      }
// 
// Timer:
//      stopwatchTick();
//
//////////////////////////////////////////////////////////////////////////////


/* **************************   Header Files   *************************** */

// NOTE: This must ALWAYS be the first include in a file.
#include "device.h"

// from stdlib
#include <stdbool.h>
#include "user_assert.h"

// from local
#include "stopwatch.h"

/* ***********************   File Scope Variables   *********************** */

// Current time value used to keep track of time for this module, in milliseconds
static volatile TimerTick_t curr_time_ms = 0;

/* *******************   Public Function Definitions   ******************** */

//-------------------------------
// Function: stopwatchStart
//
// Description: Starts a stop watch, zeroing it in the process.
//
//-------------------------------
void stopwatchStart(StopWatch_t *stop_watch)
{
	stopwatchZero(stop_watch);
	stop_watch->active = true;
}

//-------------------------------
// Function: stopwatchStop
//
// Description: Puts the stopwatch into an inactive state.
//
//-------------------------------
void stopwatchStop(StopWatch_t *stop_watch)
{
	stop_watch->active = false;
}

//-------------------------------
// Function: stopwatchZero
//
// Description: Zeros a stop watch.
//
//-------------------------------
void stopwatchZero(StopWatch_t *stop_watch)
{
    stop_watch->start_time_ms = curr_time_ms;
}

//-------------------------------
// Function: stopwatchIsActive
//
// Description: Checks to see if a stop watch is active.
//
//-------------------------------
bool stopwatchIsActive(StopWatch_t *stop_watch)
{
	return stop_watch->active;
}

//-------------------------------
// Function: stopwatchZero
//
// Description: Returns how much time has elapsed since the last time a stopwatch was zeroed.
//
// return: true - Zeroes the stopwatch before exit, false - let the stopwatch keep running
//
//-------------------------------
TimerTick_t stopwatchTimeElapsed(StopWatch_t *stop_watch, bool zero_after_check)
{
	ASSERT(stop_watch->active);

    TimerTick_t curr_time_elapsed_ms = curr_time_ms;

    // The unsigned math will work even on roll-over between start and current times.
    TimerTick_t ret_val = curr_time_elapsed_ms - stop_watch->start_time_ms;

    if (zero_after_check)
    {
        stop_watch->start_time_ms = curr_time_ms;
    }

    return ret_val;
}

//-------------------------------
// Function: stopwatchTimeUntilLimit
//
// Description: Safely figures out how much time there is between the current time elapsed for a stopwatch and
// 		a given time value.
//
//-------------------------------
TimerTick_t stopwatchTimeUntilLimit(StopWatch_t *stop_watch, TimerTick_t time_to_check_ms)
{
    TimerTick_t elapsed = stopwatchTimeElapsed(stop_watch, false);

    if (elapsed >= time_to_check_ms)
    {
        return 0;
    }
    else
    {
        return (time_to_check_ms - elapsed);
    }
}

//-------------------------------
// Function: stopwatchTick
//
// Description: increments the internal tick for this module.  Must be called at a frequency of 1 kHz (1 ms period)
//
//-------------------------------
void stopwatchTick(void)
{
	curr_time_ms++;
}

// end of file.
//-------------------------------------------------------------------------
