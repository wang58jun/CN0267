/**
 ***************************************************************************** 
   @example  ADC_RTD.c
   @brief    ADC1 is used to measure an RTD input connected across AIN3 and AIN2.
   @version  V0.1
   @author   ADI
   @date     January 2016

   @par Revision History:
   - V0.1, January 2016: initial version.


All files for ADuCM360/361 provided by ADI, including this file, are
provided  as is without warranty of any kind, either expressed or implied.
The user assumes any and all risk from the use of this code.
It is the responsibility of the person integrating this code into an application
to ensure that the resulting application performs as required and is safe.

**/

#include <stdio.h>
#include <string.h>
#include <ADuCM360.h>
#include "ADC_RTD.h"

#include "common/AdcLib.h"

#define ADC1_FS_CODE  268435456 // Full scale code for voltage calculation
#define ADC1_VREF     1.2       // Internal Vref = 1.2V
float   fADC1VoltageRTD;        // ADC1 RTD result in Volts 
float   fADC1VoltageRef;        // ADC1 Ref result in Volts 

#define RREF          5620.0    // Reference resistor value
// RTD temperature calculated only as a linear approximation, 
// using average temperature coefficient 0.00388 /deg
#define RTD_COEF 2.575          // 1(deg) / 0.00388(/deg) / 100(ohm)
#define RTD_OFFS -100.0         // 0(deg) ~ 100(ohm)
float   fRTDResistance;         // RTD sensor result in ohms
float   fRTDTemperature;        // RTD sensor result in deg      


// Init. ADuCM360 to ADC1 for RTD measurement.
void ADC_RTD_INIT(void)
{
  AdcGo(pADI_ADC1, ADCMDE_ADCMD_IDLE);  // Place ADC1 in Idle mode

  AdcMski(pADI_ADC1, 0, 0);  // Disable ADC1 interrupt sources		
  AdcFlt(pADI_ADC1, 125, 1, FLT_NORMAL|ADCFLT_NOTCH2|ADCFLT_CHOP);  // ADC filter set for 8.335Hz update rate with chop on enabled
  AdcRng(pADI_ADC1, ADCCON_ADCREF_INTREF, ADCMDE_PGA_G1, ADCCON_ADCCODE_INT);  // Internal reference selected, Gain of 1, Signed integer output
  AdcBuf(pADI_ADC1, ADCCFG_EXTBUF_OFF, ADCCON_BUFBYPN|ADCCON_BUFBYPP|ADCCON_BUFPOWP|ADCCON_BUFPOWN);  // Turn off input buffers to ADC and external reference
  AdcPin(pADI_ADC1, ADCCON_ADCCN_AIN8, ADCCON_ADCCP_AIN8);  // Select AIN8(Grand) as postive and negative input

  pADI_ANA->REFCTRL = 0
    | REFCTRL_REFPD_DIS     // Int.Vref power down (DIS => Use the Vref :-)
  ;
  
  pADI_ANA->IEXCCON = 0
    | IEXCCON_PD_off        // Exc.current power down (PD_off => Use them :-)
    | IEXCCON_REFSEL_Int    // Exc.current ref.(Int = on-chip, Ext = IREF pin)
    | IEXCCON_IPSEL1_AIN6   // Exc.current 1 output pin
    | IEXCCON_IPSEL0_AIN4   // Exc.current 0 output pin
  ;
  
  pADI_ANA->IEXCDAT = 0
    | IEXCDAT_IDAT_100uA    // Exc.current value
    | IEXCDAT_IDAT0_DIS     // Extra 100uA current
  ;
}

float ADC_RTD_GetTemperature(void)
{
  volatile long lADC1Data = 0;
  uint16_t i;

  AdcGo(pADI_ADC1, ADCMDE_ADCMD_IDLE);  // Place ADC1 in Idle mode
  /* Step 1 - ADC1 next conversion should be on Rref */
  // Measure Rref (R9 5.62kohm 10ppm) voltage
  // Input(+) buffered, AIN7, via RC filter to R9 and connector J1 pin 1
  // Input(-) buffer bypassed, AIN8, AGND (and R9 and connector J1 pin 4)
  // Gain = 1
  // Reference internal 1.2V
  AdcRng(pADI_ADC1, ADCCON_ADCREF_INTREF, ADCMDE_PGA_G1, ADCCON_ADCCODE_INT);  // Internal reference selected, Gain of 1, Signed integer output
  AdcBuf(pADI_ADC1, ADCCFG_EXTBUF_OFF, ADCCON_BUFBYPN|ADCCON_BUFBYPP_DIS|ADCCON_BUFPOWP_DIS|ADCCON_BUFPOWN);  // Turn on input buffer for ADC1
  AdcPin(pADI_ADC1, ADCCON_ADCCN_AIN8, ADCCON_ADCCP_AIN7);  // Select AIN7 as postive input and AIN8 negative
  lADC1Data = AdcRd(pADI_ADC1);  // Clear ADC1RDY flag
  AdcGo(pADI_ADC1, ADCMDE_ADCMD_SINGLE);  // Start single conversion
  i = 0;
  while ((AdcSta(pADI_ADC1)&ADC1STA_RDY_MSK) != ADC1STA_RDY_EN)  // Waiting for conversion finished
  {
    if (i >= 50000)
    {
      break;
    }
    else
    {
      i++;
    }
  }
  lADC1Data = AdcRd(pADI_ADC1);  // Clear ADC1RDY flag
  AdcGo(pADI_ADC1, ADCMDE_ADCMD_IDLE);  // Place ADC1 in Idle mode
  if (i >= 50000)
  {
    return fRTDTemperature;  // Return last value
  }
  // Translate ADC data to Voltage
  fADC1VoltageRef = ADC1_VREF * (float)(lADC1Data) / (float)(ADC1_FS_CODE); // Volts

  /* Step 2 - ADC1 next conversion should be on RTD */
  // Measures RTD (PT100) voltage
  // Input(+) buffered, AIN3, via RC filter to RTD and connector J1 pin 3
  // Input(-) buffered, AIN2, via RC filter to RTD and connector J1 pin 2
  // Gain = 16
  // Reference internal 1.2V
  AdcRng(pADI_ADC1, ADCCON_ADCREF_INTREF, ADCMDE_PGA_G16, ADCCON_ADCCODE_INT);  // Internal reference selected, Gain of 1, Signed integer output
  AdcBuf(pADI_ADC1, ADCCFG_EXTBUF_OFF, ADCCON_BUFBYPN_DIS|ADCCON_BUFBYPP_DIS|ADCCON_BUFPOWP_DIS|ADCCON_BUFPOWN_DIS);  // Turn on input buffer for ADC1
  AdcPin(pADI_ADC1, ADCCON_ADCCN_AIN2, ADCCON_ADCCP_AIN3);  // Select AIN7 as postive input and AIN8 negative
  lADC1Data = AdcRd(pADI_ADC1);  // Clear ADC1RDY flag
  AdcGo(pADI_ADC1, ADCMDE_ADCMD_SINGLE);  // Start single conversion
  i = 0;
  while ((AdcSta(pADI_ADC1)&ADC1STA_RDY_MSK) != ADC1STA_RDY_EN)  // Waiting for conversion finished
  {
    if (i >= 50000)
    {
      break;
    }
    else
    {
      i++;
    }
  }
  lADC1Data = AdcRd(pADI_ADC1);  // Clear ADC1RDY flag
  AdcGo(pADI_ADC1, ADCMDE_ADCMD_IDLE);  // Place ADC1 in Idle mode
  if (i >= 50000)
  {
    return fRTDTemperature;  // Return last value
  }
  // Translate ADC data to Voltage
  fADC1VoltageRTD = ADC1_VREF * (float)(lADC1Data) / (float)(ADC1_FS_CODE); // Volts
  // Translate data to Resistance
  fRTDResistance = fADC1VoltageRTD / fADC1VoltageRef * RREF;
  
  /* Step 3 - Translate RTD Resistance to Temperature */
  // Linear approximation, good enough for demo:
  // Pt100, average 2.575'C / ohm, 100 ohm .. 0'C
  // +/-0.1'C   error in -10 to +50'C
  // +/-0.5'C   error in -40 to +85'C
  // +1.0'C     error at +105'C
  // +2.5'C     error at +150'C
  fRTDTemperature = RTD_COEF * (fRTDResistance + RTD_OFFS);

  return fRTDTemperature;
}