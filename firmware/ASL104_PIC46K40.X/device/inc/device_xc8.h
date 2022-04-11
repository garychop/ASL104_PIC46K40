//////////////////////////////////////////////////////////////////////////////
//
// Filename: device_xc8.h
//
// Description: Defines device centric items when using the Microchip XC8 build chain.
//
// Author(s): Trevor Parsh (Embedded Wizardry, LLC)
//
// Modified for ASL on Date: 
//
//////////////////////////////////////////////////////////////////////////////

#ifndef DEVICE_XC8_H
#define DEVICE_XC8_H

/* **************************    Chip Config     ************************** */

#ifdef _18F46K40

	// CONFIG1L
	#pragma config FEXTOSC = HS     // External Oscillator mode Selection bits (HS (crystal oscillator) above 8 MHz; PFM set to high power)
	#pragma config RSTOSC = EXTOSC  // Power-up default value for COSC bits (EXTOSC operating per FEXTOSC bits (device manufacturing default))

	// CONFIG1H
	#pragma config CLKOUTEN = OFF   // Clock Out Enable bit (CLKOUT function is disabled)
	#pragma config CSWEN = ON       // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)

#ifdef DEBUG
	#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#else
	#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
//	#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor enabled)
#endif

	// CONFIG2L
	#pragma config MCLRE = EXTMCLR  // Master Clear Enable bit (If LVP = 0, MCLR pin is MCLR; If LVP = 1, RE3 pin function is MCLR )
#ifdef DEBUG
	#pragma config PWRTE = OFF      // Power-up Timer Enable bit (Power up timer disabled)
#else
	#pragma config PWRTE = ON       // Power-up Timer Enable bit (Power up timer enabled)
#endif

	#pragma config LPBOREN = OFF    // Low-power BOR enable bit (ULPBOR disabled)
	#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled , SBOREN bit is ignored)

	// CONFIG2H
	#pragma config BORV = VBOR_2P45 // Brown Out Reset Voltage selection bits (Brown-out Reset Voltage (VBOR) set to 2.45V)
	#pragma config ZCD = OFF        // ZCD Disable bit (ZCD disabled. ZCD can be enabled by setting the ZCDSEN bit of ZCDCON)
	#pragma config PPS1WAY = OFF    // PPSLOCK bit One-Way Set Enable bit (PPSLOCK bit can be set and cleared repeatedly (subject to the unlock sequence))
	#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)

#ifdef DEBUG
	#pragma config DEBUG = ON       // Debugger Enable bit (Background debugger enabled)
#else
	#pragma config DEBUG = OFF       // Debugger Enable bit (Background debugger disabled)
#endif

	#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Extended Instruction Set and Indexed Addressing Mode disabled)

	// CONFIG3L
	#pragma config WDTCPS = WDTCPS_31// WDT Period Select bits (Divider ratio 1:65536; software control of WDTPS)
	#pragma config WDTE = OFF       // WDT operating mode (WDT Disabled)

	// CONFIG3H
	#pragma config WDTCWS = WDTCWS_7// WDT Window Select bits (window always open (100%); software control; keyed access not required)
	#pragma config WDTCCS = SC      // WDT input clock selector (Software Control)

	// CONFIG4L
	#pragma config WRT0 = OFF       // Write Protection Block 0 (Block 0 (000800-003FFFh) not write-protected)
	#pragma config WRT1 = OFF       // Write Protection Block 1 (Block 1 (004000-007FFFh) not write-protected)
	#pragma config WRT2 = OFF       // Write Protection Block 2 (Block 2 (008000-00BFFFh) not write-protected)
	#pragma config WRT3 = OFF       // Write Protection Block 3 (Block 3 (00C000-00FFFFh) not write-protected)

	// CONFIG4H
	#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-30000Bh) not write-protected)
	#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
	#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)
	#pragma config SCANE = OFF      // Scanner Enable bit (Scanner module is NOT available for use, SCANMD bit is ignored)
	#pragma config LVP = OFF        // Low Voltage Programming Enable bit (HV on MCLR/VPP must be used for programming)

	// CONFIG5L
	#pragma config CP = OFF         // UserNVM Program Memory Code Protection bit (UserNVM code protection disabled)
	#pragma config CPD = OFF        // DataNVM Memory Code Protection bit (DataNVM code protection disabled)

	// CONFIG5H

	// CONFIG6L
	#pragma config EBTR0 = OFF      // Table Read Protection Block 0 (Block 0 (000800-003FFFh) not protected from table reads executed in other blocks)
	#pragma config EBTR1 = OFF      // Table Read Protection Block 1 (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)
	#pragma config EBTR2 = OFF      // Table Read Protection Block 2 (Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks)
	#pragma config EBTR3 = OFF      // Table Read Protection Block 3 (Block 3 (00C000-00FFFFh) not protected from table reads executed in other blocks)

	// CONFIG6H
	#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)

#else
	// CONFIG1L
	#pragma config PLLDIV	= 1       		// PLL Prescaler Selection bits (No prescale (4 MHz oscillator input drives PLL directly))
	#pragma config CPUDIV	= OSC1_PLL2		// System Clock Postscaler Selection bits ([Primary Oscillator Src: /1][96 MHz PLL Src: /2])
	#pragma config USBDIV	= 1       		// USB Clock Selection bit (used in Full-Speed USB mode only; UCFG:FSEN = 1) (USB clock source comes directly from the primary oscillator block with no postscale)

	// CONFIG1H
	#pragma config FOSC		= HS	// Oscillator Selection bits (HS oscillator (HS))
//GC Original	#pragma config FCMEN	= OFF	// Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
	#pragma config FCMEN	= OFF	// Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
	#pragma config IESO		= OFF	// Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)
//GC Original 	#pragma config IESO		= OFF	// Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

	// CONFIG2L
	#pragma config PWRT		= OFF	// Power-up Timer Enable bit (PWRT disabled)
	#pragma config BOR		= OFF	// Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
	#pragma config BORV		= 3		// Brown-out Reset Voltage bits (Minimum setting 2.05V)
//GC Original	#pragma config BORV		= 3		// Brown-out Reset Voltage bits (Minimum setting 2.05V)
	#pragma config VREGEN	= OFF	// USB Voltage Regulator Enable bit (USB voltage regulator disabled)

	// CONFIG2H
	#if defined(USE_WDT)
		#pragma config WDT	= ON	// Watchdog Timer Enable bit (WDT enabled)
	#else
		#pragma config WDT	= OFF	// Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
	#endif
	#pragma config WDTPS	= 32768	// Watchdog Timer Postscale Select bits (1:32768)

	// CONFIG3H
	#pragma config CCP2MX	= ON	// CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
//GC Original    #pragma config PBADEN	= ON	// PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
	#pragma config PBADEN	= OFF	// PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
    #pragma config LPT1OSC	= OFF	// Low-Power Timer 1 Oscillator Enable bit (Timer1 configured for higher power operation)
//GC Returned to OFF	#pragma config LPT1OSC	= ON	// Low-Power Timer 1 Oscillator Enable bit (Timer1 configured for higher power operation)
	#pragma config MCLRE	= ON	// MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

	// CONFIG4L
//GC Original	#pragma config STVREN 	= ON	// Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
	#pragma config STVREN 	= OFF	// Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
//GC Original	#pragma config LVP 		= ON	// Single-Supply ICSP Enable bit (Single-Supply ICSP enabled)
	#pragma config LVP 		= OFF	// Single-Supply ICSP Enable bit (Single-Supply ICSP enabled)
	#pragma config ICPRT 	= OFF	// Dedicated In-Circuit Debug/Programming Port (ICPORT) Enable bit (ICPORT disabled)
	#pragma config XINST 	= OFF	// Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

	// CONFIG5L
	#pragma config CP0		= OFF	// Code Protection bit (Block 0 (000800-001FFFh) is not code-protected)
	#pragma config CP1		= OFF	// Code Protection bit (Block 1 (002000-003FFFh) is not code-protected)
	#pragma config CP2		= OFF	// Code Protection bit (Block 2 (004000-005FFFh) is not code-protected)
	#pragma config CP3		= OFF	// Code Protection bit (Block 3 (006000-007FFFh) is not code-protected)

	// CONFIG5H
	#pragma config CPB		= OFF	// Boot Block Code Protection bit (Boot block (000000-0007FFh) is not code-protected)
	#pragma config CPD		= OFF	// Data EEPROM Code Protection bit (Data EEPROM is not code-protected)

	// CONFIG6L
	#pragma config WRT0		= OFF	// Write Protection bit (Block 0 (000800-001FFFh) is not write-protected)
	#pragma config WRT1		= OFF	// Write Protection bit (Block 1 (002000-003FFFh) is not write-protected)
	#pragma config WRT2		= OFF	// Write Protection bit (Block 2 (004000-005FFFh) is not write-protected)
	#pragma config WRT3		= OFF	// Write Protection bit (Block 3 (006000-007FFFh) is not write-protected)

	// CONFIG6H
	#pragma config WRTC		= OFF	// Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) are not write-protected)
	#pragma config WRTB		= OFF	// Boot Block Write Protection bit (Boot block (000000-0007FFh) is not write-protected)
	#pragma config WRTD		= OFF	// Data EEPROM Write Protection bit (Data EEPROM is not write-protected)

	// CONFIG7L
	#pragma config EBTR0	= OFF	// Table Read Protection bit (Block 0 (000800-001FFFh) is not protected from table reads executed in other blocks)
	#pragma config EBTR1	= OFF	// Table Read Protection bit (Block 1 (002000-003FFFh) is not protected from table reads executed in other blocks)
	#pragma config EBTR2	= OFF	// Table Read Protection bit (Block 2 (004000-005FFFh) is not protected from table reads executed in other blocks)
    #pragma config EBTR3	= OFF	// Table Read Protection bit (Block 3 (006000-007FFFh) is not protected from table reads executed in other blocks)

	// CONFIG7H
    #pragma config EBTRB	= OFF	// Boot Block Table Read Protection bit (Boot block (000000-0007FFh) is not protected from table reads executed in other blocks)

#endif

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

/* ***************************    Includes     **************************** */

#include <xc.h>

// from stdlib
#include <stdint.h>

/* ******************************   Macros   ****************************** */

// Measured to be 195 us on average. In small sample set, min seemed to be ~100 us min and ~250 us max for a tick.
// Keeping integer math for speed. If higher precision is required, more testing along with floating point math
// is required
#define TICKS_TO_MILLISECONDS(x) (x / 1)
#define MILLISECONDS_TO_TICKS(x) (x * 1)

/* ******************************   Types   ******************************* */

typedef uint16_t TimerTick_t;

#endif // DEVICE_XC8_H

// end of file.
//-------------------------------------------------------------------------
