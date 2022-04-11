//////////////////////////////////////////////////////////////////////////////
//
// Filename: rtos_task_priorities.h
//
// Description: Defines task priorities for all tasks in the system.
//
// Author(s): Trevor Parsh (Embedded Wizardry, LLC)
//
// Modified for ASL on Date: 
//
//////////////////////////////////////////////////////////////////////////////

#ifndef RTOS_TASK_PRIORITIES_H_
#define RTOS_TASK_PRIORITIES_H_

/* ******************************   Macros   ****************************** */

// 0 is the highest priority, 255 is lowest and also the highest allowable value
// NOTE: They must be unique
#define USER_BTN_MGMT_TASK_PRIO		(3)
#define EFIX_COMM_TASK_PRIO 		(1)
#define HEAD_ARR_MGMT_TASK_PRIO		(2)
#define GEN_OUT_CTRL_MGMT_TASK_PRIO	(4)
#define BEEPER_MGMT_TASK_PRIO		(5)
#define HA_HHP_IF_MGMT_TASK_PRIO	(2)
#define SYSTEM_SUPERVISOR_TASK_PRIO	(0)
#define MAIN_TASK_PRIO              (6)
#define NEW_TASK7                   (7)

// I'm including the task delays to ensure proper sequencing.
#define MAIN_TASK_DELAY (10)        // Number of milliseconds for the main task.
#define BEEPER_TASK_DELAY (15)      // Number of milliseconds for Beeper task.
// I believe that msg's must be pulled out of the queue. It behaves if the
// msg queue gets full even if use the "_async" calls.
#define HEAD_ARRAY_TASK_DELAY (20)
#define USER_BUTTON_TASK_DELAY (50)

#endif // End of RTOS_TASK_PRIORITIES_H_

// end of file.
//-------------------------------------------------------------------------
