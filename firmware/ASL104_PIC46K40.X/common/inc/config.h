//////////////////////////////////////////////////////////////////////////////
//
// Filename: config.h
//
// Description: Highest level configuration
//
// Author(s): Trevor Parsh (Embedded Wizardry, LLC)
//
// Modified for ASL on Date: 
//
//////////////////////////////////////////////////////////////////////////////

#ifndef CONFIG_H
#define CONFIG_H

/* ******************************   Macros   ****************************** */

// When defined, forces the program to set all values in EEPROM to default values
//#define SPECIAL_EEPROM_TO_DEFAULT_VALUES

// Define this IF you have the 12 Volt regulator on the Head Array board.
// If it's jumpered out, comment the following line.
#define USE_12VOLT_REGULATOR

/* ******************************   Tests   ******************************* */

// Tests. Generally, only one should be enabled. Unless it is known that >1 test can be run with
// another test(s)
// #define TEST_BASIC_DAC_CONTROL
// #define HA_HHP_COMMS_TEST_TIMING

// Checks timing of the critical pieces of the HA->HHP BSP
// #define HA_HHP_COMMS_BSP_TIMING_TEST

// Checks timing of us delay function
//#define HA_HHP_BSP_DELAY_TEST

// Tests basic connectivity between HA and HHP
//#define HA_HHP_COMMS_TEST_BASIC_HARDWARE

// Basic EEPROM comms test
//#define TEST_BASIC_EEPROM_CONTROL

#endif // CONFIG_H

// end of file.
//-------------------------------------------------------------------------
