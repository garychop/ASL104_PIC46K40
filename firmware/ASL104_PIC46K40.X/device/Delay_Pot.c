
//------------------------------------------------------------------------------
//
// Filename: Delay_Pot.c
//
// Description: BSP level definitions for reading Delay Pot
//
// Author(s): G. Chopcinski (Kg Solutions, LLC)
//
// Modified for ASL on Date: 12/12/2020
//
//------------------------------------------------------------------------------

#include <pic18lf4550.h>
#include "bsp.h"

//------------------------------------------------------------------------------
// Macros and defines

//------------------------------------------------------------------------------
// Local Variables

//------------------------------------------------------------------------------
// Forward Prototype Declarations
uint16_t ReadDelayPot(void);

//------------------------------------------------------------------------------

void DelayPot_INIT(void)
{
    ADCON1bits.VCFG0 = 0;
    ADCON1bits.VCFG1 = 0;
	//ADCON1bits.VCFG01 = 0; // VSS negative voltage reference
	//ADCON1bits.VCFG11 = 0; // VDD positive voltage reference
	ADCON1bits.PCFG = 0x0e; // Channels AN0 is enabled
    
    // NOTE: Time to capture is 6.4 us.  This should be fine for any operational environment as
	// NOTE: See Equation 21-3 of the PIC18F4550's datasheet.  Also, Table 21-1
	ADCON2bits.ADCS = 0x05; // FOSC / 16 = 625 kHz
	ADCON2bits.ACQT = 0x02; // 4 AD clock cycles per conversion.

	ADCON2bits.ADFM = 1; // Results right justified

	ADCON0bits.ADON = 1; // Enable ADC

}

//------------------------------------------------------------------------------
// Function: ReadDelayPot
//
// Description: Reads the analog input value of the back panel Delay Pot.
//
//------------------------------------------------------------------------------
uint16_t ReadDelayPot(void)
{
    ADCON0bits.CHS = 0;
    
	// Need to wait at least Tad * 3. Clock is FOSC/16, which gets us: 3/(625,000) = ~4.8 us.  Our delay resolution is not
	// great, so we just delay for the min time.
	bspDelayUs (US_DELAY_20_us);
	
	// Kick off the conversion
	ADCON0bits.GO_nDONE = 1;
	while (ADCON0bits.GO_nDONE == 1)
	{
		(void)0;
	}
    
	return ((uint16_t)ADRESL + ((uint16_t)(ADRESH & 0x3) << 8));
}

//------------------------------------------------------------------------------
// GetDelayTime: This function returns a time in milliseconds to act.
// Minimum is 0, max is 6000 (6 seconds)
// The ADC value is 0x09 to 0x3FB which is 9 to 1019
//------------------------------------------------------------------------------
uint16_t GetDelayTime (void)
{
    uint16_t myADC;
    uint16_t returnVal;
    
    myADC = ReadDelayPot();
    // I think we can simply multiply the ADC value by 6 to determine delay time.
    returnVal = myADC * 6;
    if (returnVal < 500)   // Check for minimum value and force to 0.
        returnVal = 0;
    
    if (returnVal > 6000)   // Force a max value.
        returnVal = 6000;      // 6 seconds in milliseconds.
    
    return (returnVal);
}

// End of file
//------------------------------------------------------------------------------

