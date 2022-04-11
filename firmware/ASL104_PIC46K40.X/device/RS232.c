//////////////////////////////////////////////////////////////////////////////
//
// Filename: eFix_Communication.c
//
// Description: Implements application specific needs for the ASL110 related to
//		communicating with the eFix Model 35.
//  
//  Reference the following web page to create this driver:
//  https://openlabpro.com/guide/uart-interfacing-with-pic-microcontroller/
//
// Author(s): G. Chopcinski (Kg Solutions, LLC)
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

//------------------------------------------------------------------------------
// Function: RS232_Initialize
// Description: This function initializes the 18LF4550 UART communication hardware.
//      Note that the interrupts for receive and transmit are disabled.
// Returns: void
//------------------------------------------------------------------------------

void RS232_Initialize (void)
{
#ifdef READY_FOR_RS232
    // Set I/O Pin Directions
    TRISCbits.RC6 = 0;      // Port C pin 6 is Transmit.
    TRISCbits.RC7 = 1;      // Port C pin 7 is Receive.
    
    // Setup Transmitter
    TXSTAbits.CSRC = 0;
    TXSTAbits.BRGH = 1;     // Set for High Speed Synchronous operation
    TXSTAbits.SYNC = 0;     // "0" = Asynchronous operation
    TXSTAbits.TXEN = 1;     // This enables transmission.
    TXSTAbits.TX9 = 0;      // "0" = 8-bit operation. "1" = 9-bit
    // TXSTAbits.TX9D = 1;     // 9th bit data. (Stop bit?)
    
    // Set up Receiver
    RCSTAbits.CREN = 1;     // "1" allows continuous reception.
    RCSTAbits.RX9 = 0;      // "0" = 8-Bit operation.
    RCSTAbits.SPEN = 1;     // "1" enables both the Transmit and Received of this port.
    
    // Setup Baud rate and other communication options.
    BAUDCONbits.ABDOVF;     // This is a status bit, 0 = No BRG has occurred.
    BAUDCONbits.RCIDL;      // Status bit. 0 = Receive operation is active.
    BAUDCONbits.RXDTP = 0;  // "0" Indicates RX data is NOT inverted.
    BAUDCONbits.TXCKP = 0; // as found is "1";  // "0" Indicates TX data is NOT inverted.
    BAUDCONbits.BRG16 = 1;  // "1" Indicates 16-Bit Baud Rate Generation, SPBRGH and SPBRG are used.
    BAUDCONbits.WUE = 0;    // "0" Wake up Not Enabled.
    BAUDCONbits.ABDEN = 0;  // "0" = Auto Baud Rate detection is disabled.
    
    // Set for a baud rate of 115.2K
    SPBRG = 21;   // Framing error, 20, 19
    SPBRGH = 0;
    
    // Reference: Use TXREG to transmit data
    // Reference: Use RCREG to receive data
    
    PIE1bits.RCIE = 0;      // "1" enables Receive interrupt
    PIE1bits.TXIE = 0;      // "1" enables the Transmit complete interrupt.
#endif // #ifdef READY_FOR_RS232

}
//------------------------------------------------------------------------------
// Function: RS232_TransmitReady()
// Description: Evaluates the CPU Regs to determine if it's OK to send
//      a character.
// Returns: true if transmit buffer is empty and ready
//          false if caller should wait just a little bit longer.
//------------------------------------------------------------------------------

bool RS232_TransmitReady (void)
{
#ifdef READY_FOR_RS232
    return (PIR1bits.TXIF == 1); // "1" = transmit buffer is empty
#else
    return false;
#endif // #ifdef READY_FOR_RS232
}

//------------------------------------------------------------------------------
// Function: RS232_TransmitChar
// Description: Send a character out via the RS232 UART hardwawre. But only
//      if the transmit buffer is empty. If not, loop until it is.
// Returns: void
//------------------------------------------------------------------------------

void RS232_TransmitChar (unsigned char item)
{
#ifdef READY_FOR_RS232
    while (PIR1bits.TXIF == 0)  // "0" = Transmit buffer not empty.
        ;
    TXREG = item;
#endif // #ifdef READY_FOR_RS232
}

//------------------------------------------------------------------------------
// Function: RS232_GetReceivedChar
// Description: Gets a character from the UART hardware.
// Returns: true if a character is was received
//          false if no character received.
//------------------------------------------------------------------------------
bool RS232_GetReceivedChar (unsigned char *item)
{
#ifdef READY_FOR_RS232
    if (PIR1bits.RCIF != 0) // non0 = we got a character
    {
        *item = RCREG;
        return true;
    }
    *item = 0x00;
#endif // READY_FOR_RS232
    return false;
}

// END OF FILE


