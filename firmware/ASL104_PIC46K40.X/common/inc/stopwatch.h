//////////////////////////////////////////////////////////////////////////////
//
// Filename: stopwatch.h
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

#ifndef STOPWATCH_H
#define STOPWATCH_H

/* ***************************    Includes     **************************** */

// from stdlib
#include <stdbool.h>

/* ******************************   Types   ******************************* */

// Enum for a stopwatch object. very simple, but the abstraction is nice and
// makes things simpler.
typedef struct
{
    TimerTick_t start_time_ms;
    bool active;
} StopWatch_t;

/* ***********************   Function Prototypes   ************************ */

void stopwatchStart(StopWatch_t *stop_watch);
void stopwatchStop(StopWatch_t *stop_watch);
void stopwatchZero(StopWatch_t *stop_watch);
bool stopwatchIsActive(StopWatch_t *stop_watch);
TimerTick_t stopwatchTimeElapsed(StopWatch_t *stop_watch, bool zero_after_check);
TimerTick_t stopwatchTimeUntilLimit(StopWatch_t *stop_watch, TimerTick_t time_to_check_ms);
void stopwatchTick(void);

#endif // STOPWATCH_H

// end of file.
//-------------------------------------------------------------------------
