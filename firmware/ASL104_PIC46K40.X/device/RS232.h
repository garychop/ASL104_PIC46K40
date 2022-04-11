/* Microchip Technology Inc. and its subsidiaries.  You may use this software 
 * and any derivatives exclusively with Microchip products. 
 * 
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER 
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED 
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A 
 * PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION 
 * WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS 
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE 
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS 
 * IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF 
 * ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE 
 * TERMS. 
 */

/* 
 * File:   RS232.h
 * Author: G. Chopcinski (Kg Solutions, LLC)
 * Comments: Supports the 18LF4550 RS-232 UART communication
 * Revision history: 
 *  Nov 25, 2020, GChop, Created
 */

#ifndef XC_HEADER_TEMPLATE_H
#define	XC_HEADER_TEMPLATE_H

#include <xc.h> // include processor files - each processor file is guarded.  

//------------------------------------------------------------------------------
// Function: RS232_Initialize
// Description: This function initializes the 18LF4550 UART communication hardware.
//      Note that the interrupts for receive and transmit are disabled.
// Returns: void
//------------------------------------------------------------------------------
void RS232_Initialize (void);

//------------------------------------------------------------------------------
// Function: RS232_TransmitReady()
// Description: Evaluates the CPU Regs to determine if it's OK to send
//      a character.
// Returns: true if transmit buffer is empty and ready
//          false if caller should wait just a little bit longer.
//------------------------------------------------------------------------------
bool RS232_TransmitReady (void);

//------------------------------------------------------------------------------
// Function: RS232_TransmitChar
// Description: Send a character out via the RS232 UART hardwawre. But only
//      if the transmit buffer is empty. If not, loop until it is.
// Returns: void
//------------------------------------------------------------------------------
void RS232_TransmitChar (unsigned char item);

//------------------------------------------------------------------------------
// Function: RS232_GetReceivedChar
// Description: Gets a character from the UART hardware.
// Returns: true if a character is was received
//          false if no character received.
//------------------------------------------------------------------------------
bool RS232_GetReceivedChar (unsigned char *item);

#endif	/* XC_HEADER_TEMPLATE_H */

