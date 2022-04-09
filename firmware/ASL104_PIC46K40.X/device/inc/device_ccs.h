//////////////////////////////////////////////////////////////////////////////
//
// Filename: device_ccs.h
//
// Description: Defines device centric items when using the CCS for PIC build chain.
//
// Author(s): Trevor Parsh (Embedded Wizardry, LLC)
//
// Modified for ASL on Date: 
//
//////////////////////////////////////////////////////////////////////////////

#ifndef DECIVE_CCS_H
#define DECIVE_CCS_H

// from system
#include <18F4550.h>
#include <stdint.h>

/* ******************   Compiler Warning Suppressions   ******************* */

// 203: Suppress "expressions is always true".
//
// Yes, there are cases where this can bite ya. But, there are too many cases
// where it is desired (e.g. while (1)) that throw warnings that we're just going
// to disable the warning.
//
// 204: Suppress "expressions is always false".
//
// Yes, there are cases where this can bite ya. But, there are too many cases
// where it is desired (e.g. while (0), ASSERT(false);) that throw warnings that we're just going
// to disable the warning.
//
// 207: Code has no effect
//
// Want this disabled for lines such as (void)0;
#ignore_warnings 203, 204, 207

/* *************************   Device Config   *************************** */

// WARN: chip specific include MUST be above everything below this point.  This is because
// WARN: the chip selection pragma resets all system configuration, which is set below,
// WARN: during inclusion. I.e. if the configuration below comes before the chip specific 
// WARN: file inclusion, all configuration is reset to default values.

// All possible options for this chip:
//
// NOTE: the description below have been copy-pasted from CCS IDE View->Config Bits
// #fuses NOFCMEN        //Fail-safe clock monitor disabled
// #fuses PLL1           //No PLL PreScaler
// #fuses PLL2           //Divide By 2(8MHz oscillator input)
// #fuses PLL3           //Divide By 3(12MHz oscillator input)
// #fuses PLL4           //Divide By 4(16MHz oscillator input)
// #fuses PLL5           //Divide By 5(20MHz oscillator input)
// #fuses PLL6           //Divide By 6(24MHz oscillator input)
// #fuses PLL10          //Divide By 10(40MHz oscillator input)
// #fuses PLL12          //Divide By 12(48MHz oscillator input)
// #fuses CPUDIV1        //No System Clock Postscaler
// #fuses CPUDIV2        //System Clock by 2
// #fuses CPUDIV3        //System Clock by 3
// #fuses CPUDIV4        //System Clock by 4
// #fuses USBDIV         //USB clock source comes from PLL divide by 2
// #fuses NOUSBDIV       //USB clock source comes from primary oscillator
// #fuses XT             //Crystal osc <= 4mhz for PCM/PCH , 3mhz to 10 mhz for PCD
// #fuses XTPLL          //Crystal/Resonator with PLL enabled
// #fuses EC_IO          //External clock
// #fuses EC             //External clock with CLKOUT
// #fuses ECPLL_IO       //External Clock with PLL enabled, I/O on RA6
// #fuses ECPLL          //External Clock with PLL enabled and Fosc/4 on RA6
// #fuses INTRC_IO       //Internal RC Osc, no CLKOUT
// #fuses INTRC          //Internal RC Osc
// #fuses INTXT          //Internal Oscillator, XT used by USB
// #fuses INTHS          //Internal Oscillator, HS used by USB
// #fuses HS             //High speed Osc (> 4mhz for PCM/PCH) (>10mhz for PCD)
// #fuses HSPLL          //High Speed Crystal/Resonator with PLL enabled
// #fuses FCMEN          //Fail-safe clock monitor enabled
// #fuses IESO           //Internal External Switch Over mode enabled
// #fuses NOIESO         //Internal External Switch Over mode disabled
// #fuses PUT            //Power Up Timer
// #fuses NOPUT          //No Power Up Timer
// #fuses NOBROWNOUT     //No brownout reset
// #fuses BROWNOUT_SW    //Brownout controlled by configuration bit in special file register
// #fuses BROWNOUT_NOSL  //Brownout enabled during operation, disabled during SLEEP
// #fuses BROWNOUT       //Reset when brownout detected
// #fuses VREGEN         //USB voltage regulator enabled
// #fuses NOVREGEN       //USB voltage regulator disabled
// #fuses WDT            //Watch Dog Timer
// #fuses NOWDT          //No Watch Dog Timer
// #fuses WDT1           //Watch Dog Timer uses 1:1 Postscale
// #fuses WDT2           //Watch Dog Timer uses 1:2 Postscale
// #fuses WDT4           //Watch Dog Timer uses 1:4 Postscale
// #fuses WDT8           //Watch Dog Timer uses 1:8 Postscale
// #fuses WDT16          //Watch Dog Timer uses 1:16 Postscale
// #fuses WDT32          //Watch Dog Timer uses 1:32 Postscale
// #fuses WDT64          //Watch Dog Timer uses 1:64 Postscale
// #fuses WDT128         //Watch Dog Timer uses 1:128 Postscale
// #fuses WDT256         //Watch Dog Timer uses 1:256 Postscale
// #fuses WDT512         //Watch Dog Timer uses 1:512 Postscale
// #fuses WDT1024        //Watch Dog Timer uses 1:1024 Postscale
// #fuses WDT2048        //Watch Dog Timer uses 1:2048 Postscale
// #fuses WDT4096        //Watch Dog Timer uses 1:4096 Postscale
// #fuses WDT8192        //Watch Dog Timer uses 1:8192 Postscale
// #fuses WDT16384       //Watch Dog Timer uses 1:16384 Postscale
// #fuses WDT32768       //Watch Dog Timer uses 1:32768 Postscale
// #fuses CCP2B3         //CCP2 input/output multiplexed with RB3
// #fuses CCP2C1         //CCP2 input/output multiplexed with RC1
// #fuses PBADEN         //PORTB pins are configured as analog input channels on RESET
// #fuses NOPBADEN       //PORTB pins are configured as digital I/O on RESET
// #fuses LPT1OSC        //Timer1 configured for low-power operation
// #fuses NOLPT1OSC      //Timer1 configured for higher power operation
// #fuses MCLR           //Master Clear pin enabled
// #fuses NOMCLR         //Master Clear pin used for I/O
// #fuses STVREN         //Stack full/underflow will cause reset
// #fuses NOSTVREN       //Stack full/underflow will not cause reset
// #fuses LVP            //Low Voltage Programming on B3(PIC16) or B5(PIC18)
// #fuses NOLVP          //No low voltage prgming, B3(PIC16) or B5(PIC18) used for I/O
// #fuses ICSP1          //ICD uses PGC1/PGD1 pins
// #fuses ICSP2          //ICD uses PGC2/PGD2 pins
// #fuses XINST          //Extended set extension and Indexed Addressing mode enabled
// #fuses NOXINST        //Extended set extension and Indexed Addressing mode disabled (Legacy mode)
// #fuses DEBUG          //Debug mode for use with ICD
// #fuses NODEBUG        //No Debug mode for ICD
// #fuses PROTECT        //Code protected from reads
// #fuses NOPROTECT      //Code not protected from reading
// #fuses CPB            //Boot Block Code Protected
// #fuses NOCPB          //No Boot Block code protection
// #fuses CPD            //Data EEPROM Code Protected
// #fuses NOCPD          //No EE protection
// #fuses WRT            //Program Memory Write Protected
// #fuses NOWRT          //Program memory not write protected
// #fuses WRTC           //Configuration registers write protected
// #fuses NOWRTC         //Configuration registers not write protected
// #fuses WRTB           //Boot block write protected
// #fuses NOWRTB         //Boot block not write protected
// #fuses WRTD           //Data EEPROM write protected
// #fuses NOWRTD         //Data EEPROM not write protected
// #fuses EBTR           //Memory protected from table reads
// #fuses NOEBTR         //Memory not protected from table reads
// #fuses EBTRB          //Boot block protected from table reads
// #fuses NOEBTRB        //Boot block not protected from table reads

#fuses PLL1			// No PLL prescaler (div 1)
#fuses CPUDIV1		// System clock post-scale, div 2
#fuses HS           // High speed Osc (> 4mhz for PCM/PCH) (>10mhz for PCD) - 10 MHz for this application
#fuses FCMEN 		// Monitor Fail-Safe clock monitoring
#fuses IESO			// Allow system clock to switch over to internal clock if external clock fails.
#fuses PUT      	// Power Up Timer: A delay of ~65.5 ms after voltage to the chip reaches minimum operating voltage (ensure voltage is stable before starting core oscillators and functionality)

// TODO: Not sure what voltage triggers brownout. In MPLAB X it is settable through config bits. CCS does not. Should figure out how to control the trigger level.
#fuses BROWNOUT		// Reset when brownout detected
#fuses NOVREGEN 	// USB voltage regulator disabled

#if defined(USE_WDT)
	#fuses WDT		//Watch Dog Timer
	#fuses WDT512	//Watch Dog Timer uses 1:512 Postscale
#else
	#fuses NOWDT
#endif

#fuses CCP2C1		// CCP2 input/output multiplexed with RC1
#fuses PBADEN		// PORTB pins are configured as analog input channels on RESET
#fuses NOLPT1OSC	// Timer1 configured for higher power operation
#fuses MCLR			// Master Clear pin enabled
#fuses STVREN		// Stack full/underflow will cause reset
#fuses LVP			// Low Voltage Programming on B3(PIC16) or B5(PIC18)
#fuses NOXINST		// Extended set extension and Indexed Addressing mode disabled (Legacy mode)
#fuses NOPROTECT	// Code not protected from reading
#fuses NOCPB		// No Boot Block code protection
#fuses NOCPD		// No EE protection
#fuses NOWRT		// Program memory not write protected
#fuses NOWRTC		// Configuration registers not write protected
#fuses WRTB			// Boot block write protected
#fuses NOWRTD		// Data EEPROM not write protected
#fuses NOEBTR		// Memory not protected from table reads
#fuses NOEBTRB		// Boot block not protected from table reads

#if defined(LOAD_FROM_MPLABX_FOR_DEBUG)
    #fuses DEBUG		// Debug mode for use with ICD
#else
    #fuses NODEBUG      // No Debug mode for ICD
#endif

/* *********************   High Lvl Device Definition   ******************* */

// Generates code compatible with Microchips ICD debugging hardware.
#if defined(LOAD_FROM_MPLABX_FOR_DEBUG)
    #device ICD = TRUE
#endif

#device ADC = 8

/* ************************   MCU Register Access   *********************** */

// USB configuration register.
#byte UCFG = 0xF6F
#bit UTRDIS = UCFG.3 // On-Chip Transceiver Disable bit. 1: On-chip transceiver disabled; digital transceiver interface enabled, 0: On-chip transceiver active

/* ******************************   Types   ******************************* */

// NOTE: Must match the number of bits defined by "#use timer" in device.c
typedef uint32_t TimerTick_t;

// Measured to be 195 us on average. In small sample set, min seemed to be ~100 us min and ~250 us max for a tick.
// Keeping integer math for speed. If higher precision is required, more testing along with floating point math
// is required
#define TICKS_TO_MILLISECONDS(x) (x / 5)
#define MILLISECONDS_TO_TICKS(x) (x * 5)

#endif // DECIVE_CCS_H

// end of file.
//-------------------------------------------------------------------------
