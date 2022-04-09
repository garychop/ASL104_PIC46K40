//////////////////////////////////////////////////////////////////////////////
//
// Filename: head_array_bsp.c
//
// Description: Provides functionality for interpretting head array input.
//
// Author(s): Trevor Parsh (Embedded Wizardry, LLC)
//
// Modified for ASL on Date: 
//
//////////////////////////////////////////////////////////////////////////////


/* **************************   Header Files   *************************** */

// NOTE: This must ALWAYS be the first include in a file.
#include "device.h"

// from stdlib
#include <stdint.h>
#include <stdbool.h>
#include "user_assert.h"

// from project
#include "bsp.h"
#include "common.h"
#include "head_array_common.h"
#include "dac_bsp.h"

// from local
#include "head_array_bsp.h"

/* ******************************   Macros   ****************************** */

#define ADC_CH0_SEL 				0x00
#define ADC_CH1_SEL 				0x01
#define ADC_CH2_SEL 				0x02

#define DIG_PAD_ACTIVE_STATE		GPIO_LOW
#define DIG_PAD_INACTIVE_STATE		GPIO_HIGH

#ifdef _18F46K40
    #define DIG_LEFT_PAD_IS_ACTIVE()	(PORTBbits.RB4 == DIG_PAD_ACTIVE_STATE)
    #define DIG_LEFT_PAD_INIT()			INLINE_EXPR(TRISBbits.TRISB4 = GPIO_BIT_INPUT; ANSELBbits.ANSELB4 = 0)

    #define DIG_RIGHT_PAD_IS_ACTIVE()	(PORTBbits.RB2 == DIG_PAD_ACTIVE_STATE)
    #define DIG_RIGHT_PAD_INIT()		INLINE_EXPR(TRISBbits.TRISB2 = GPIO_BIT_INPUT; ANSELBbits.ANSELB2 = 0)

    #define DIG_CTR_PAD_IS_ACTIVE()		(PORTBbits.RB3 == DIG_PAD_ACTIVE_STATE)
    #define DIG_CTR_PAD_INIT()			INLINE_EXPR(TRISBbits.TRISB3 = GPIO_BIT_INPUT; ANSELBbits.ANSELB3 = 0)

    #define ANA_LEFT_PAD_INIT()			INLINE_EXPR(ANSELAbits.ANSELA0 = 1)
    #define ANA_RIGHT_PAD_INIT()		INLINE_EXPR(ANSELAbits.ANSELA1 = 1)
    #define ANA_CENTER_PAD_INIT()		INLINE_EXPR(ANSELAbits.ANSELA2 = 1)
#else
    #define DIG_LEFT_PAD_IS_ACTIVE()	(PORTBbits.RB4 == DIG_PAD_ACTIVE_STATE)
    #define DIG_LEFT_PAD_INIT()			INLINE_EXPR(TRISBbits.TRISB4 = GPIO_BIT_INPUT)

    #define DIG_RIGHT_PAD_IS_ACTIVE()	(PORTBbits.RB2 == DIG_PAD_ACTIVE_STATE)
    #define DIG_RIGHT_PAD_INIT()		INLINE_EXPR(TRISBbits.TRISB2 = GPIO_BIT_INPUT)

    #define DIG_CTR_PAD_IS_ACTIVE()		(PORTBbits.RB3 == DIG_PAD_ACTIVE_STATE)
    #define DIG_CTR_PAD_INIT()			INLINE_EXPR(TRISBbits.TRISB3 = GPIO_BIT_INPUT)
#endif

/* *******************   Public Function Definitions   ******************** */

//-------------------------------
// Function: headArrayBspInit
//
// Description: Initializes this module.
//
//-------------------------------
void headArrayBspInit(void)
{
	DIG_LEFT_PAD_INIT();
	DIG_RIGHT_PAD_INIT();
	DIG_CTR_PAD_INIT();

#ifdef _18F46K40
    ANA_LEFT_PAD_INIT();
    ANA_RIGHT_PAD_INIT();
    ANA_CENTER_PAD_INIT();

    ADCON0bits.ADCS = 0; // Fosc is the clock source for ADC clock
    ADCON0bits.ADFM = 1; // Results are right justified
    ADCON0bits.ADCONT = 0; // Not continuous conversion: one-shot
    
    // ADCON1bits is a don't care because we don't use pre-charge.
    
    ADCON2bits.ADMD = 0x00; // No filtering or averaging on ADC samples.
    
    // ADCON3bits are not required to be set.
    
    ADREFbits.ADPREF = 0x00; // VSS negative voltage reference
    ADREFbits.ADNREF = 0x00; // VDD positive voltage reference
    
    // Set clock to Fosc/(2*(ADCLKbits.ADCS+1)) = Fosc / 16 = 625 kHz
    ADCLKbits.ADCS = 7;
    
    ADPREbits.ADPRE = 0; // No pre-charge before taking an ADC sample.
    ADACQbits.ADACQ = 4; // 4 AD clock cycles per conversion.
    ADCAPbits.ADCAP = 0; // No external capacitance attached to the signal path.
    
    ADRPTbits.ADRPT = 0; // Repeat threshold: don't care since not filtering or averaging.
    ADCNTbits.ADCNT = 0; // Don't care since not filtering or averaging.
    ADFLTRHbits.ADFLTRH = 0; // Don't care since not filtering or averaging.
    ADFLTRLbits.ADFLTRL = 0; // Don't care since not filtering or averaging.
    
    ADCON0bits.ADON = 1; // Enable ADC
#else
	ADCON1bits.VCFG01 = 0; // VSS negative voltage reference
	ADCON1bits.VCFG11 = 0; // VDD positive voltage reference
	ADCON1bits.PCFG = 0x0C; // Channels AN0->2 are enabled

	// NOTE: Time to capture is 6.4 us.  This should be fine for any operational environment as
	// NOTE: See Equation 21-3 of the PIC18F4550's datasheet.  Also, Table 21-1
	ADCON2bits.ADCS = 0x05; // FOSC / 16 = 625 kHz
	ADCON2bits.ACQT = 0x02; // 4 AD clock cycles per conversion.

	ADCON2bits.ADFM = 1; // Results right justified

	ADCON0bits.ADON = 1; // Enable ADC
#endif
	
	dacBspInit();
}

//-------------------------------
// Function: headArrayBspDigitalState
//
// Description: Reads the digital input state of a single head array sensor.
//
//-------------------------------
bool headArrayBspDigitalState(HeadArraySensor_t sensor_id)
{
	switch (sensor_id)
	{
		case HEAD_ARRAY_SENSOR_LEFT:
			return DIG_LEFT_PAD_IS_ACTIVE();

		case HEAD_ARRAY_SENSOR_RIGHT:
			return DIG_RIGHT_PAD_IS_ACTIVE();
			
		case HEAD_ARRAY_SENSOR_CENTER:
			return DIG_CTR_PAD_IS_ACTIVE();
			
		case HEAD_ARRAY_SENSOR_EOL:
		default:
			ASSERT(sensor_id == HEAD_ARRAY_SENSOR_CENTER);
			return false; // Return something.
	}
}

//-------------------------------
// Function: headArrayBspAnalogState
//
// Description: Reads the analog input state of a single head array sensor.
//
//-------------------------------
uint16_t headArrayBspAnalogState(HeadArraySensor_t sensor_id)
{
    uint8_t selected_adc_ch;

	switch (sensor_id)
	{
		case HEAD_ARRAY_SENSOR_LEFT:
            selected_adc_ch = ADC_CH0_SEL; // Pin A0
			break;

		case HEAD_ARRAY_SENSOR_RIGHT:
            selected_adc_ch = ADC_CH1_SEL; // Pin A1
			break;
			
		case HEAD_ARRAY_SENSOR_CENTER:
            selected_adc_ch = ADC_CH2_SEL; // Pin A2
			break;
			
		case HEAD_ARRAY_SENSOR_EOL:
		default:
			ASSERT(sensor_id == HEAD_ARRAY_SENSOR_CENTER);
			break;
	}
    
#ifdef _18F46K40
    ADPCHbits.ADPCH = selected_adc_ch;
    
	// Need to wait at least Tad * 3. Clock is FOSC/16, which gets us: 3/(625,000) = ~4.8 us.  Our delay resolution is not
	// great, so we just delay for the min time.
	bspDelayUs(US_DELAY_20_us);
	
	// Kick off the conversion
	ADCON0bits.ADGO = 1;
	while (ADCON0bits.ADGO == 1)
	{
		(void)0;
	}
#else
    ADCON0bits.CHS = selected_adc_ch;
    
	// Need to wait at least Tad * 3. Clock is FOSC/16, which gets us: 3/(625,000) = ~4.8 us.  Our delay resolution is not
	// great, so we just delay for the min time.
	bspDelayUs(US_DELAY_20_us);
	
	// Kick off the conversion
	ADCON0bits.GO_nDONE = 1;
	while (ADCON0bits.GO_nDONE == 1)
	{
		(void)0;
	}
#endif
    
	return ((uint16_t)ADRESL + ((uint16_t)(ADRESH & 0x3) << 8));
}

//-------------------------------
// Function: headArrayBspProportionalMaxValue
//
// Description: Let's the caller know what the maximum value a proportional signal can be, in terms of # of ADC counts.
//
//-------------------------------
uint16_t headArrayBspProportionalMaxValue(HeadArraySensor_t sensor_id)
{
	switch (sensor_id)
	{
		case HEAD_ARRAY_SENSOR_LEFT:
			return ADC_LEFT_PAD_MAX_VAL;
			
		case HEAD_ARRAY_SENSOR_RIGHT:
			return ADC_RIGHT_PAD_MAX_VAL;

		case HEAD_ARRAY_SENSOR_CENTER:
			return ADC_CTR_PAD_MAX_VAL;

		default:
			ASSERT(sensor_id == HEAD_ARRAY_SENSOR_CENTER);
			return ADC_CTR_PAD_MAX_VAL;
			break;
	}
}

//-------------------------------
// Function: headArrayBspProportionalMinValue
//
// Description: Let's the caller know what the minimum value a proportional signal can be, in terms of # of ADC counts.
//
//-------------------------------
uint16_t headArrayBspProportionalMinValue(HeadArraySensor_t sensor_id)
{
	switch (sensor_id)
	{
		case HEAD_ARRAY_SENSOR_LEFT:
			return ADC_LEFT_PAD_MIN_VAL;
			
		case HEAD_ARRAY_SENSOR_RIGHT:
			return ADC_RIGHT_PAD_MIN_VAL;

		case HEAD_ARRAY_SENSOR_CENTER:
			return ADC_CTR_PAD_MIN_VAL;

		default:
			ASSERT(sensor_id == HEAD_ARRAY_SENSOR_CENTER);
			return ADC_CTR_PAD_MIN_VAL;
			break;
	}
}

// end of file.
//-------------------------------------------------------------------------
