/**
 ***************************************************************************** 
   @example  AD5421.c
   @brief    This file contains functions written for the ADuCM360 that excercie the AD5421 via the SPI0 bus.
   @version  V0.3
   @author   ADI
   @date     January 2016

   @par Revision History:
   - V0.1, May 2012: initial version. 
   - V0.2, January 2013: added Doxygen comments
   - V0.3, January 2016: adapt to CN0267 board


All files for ADuCM360/361 provided by ADI, including this file, are
provided  as is without warranty of any kind, either expressed or implied.
The user assumes any and all risk from the use of this code.
It is the responsibility of the person integrating this code into an application
to ensure that the resulting application performs as required and is safe.

**/

#include <stdio.h>
#include <string.h>
#include <ADuCM360.h>
#include "AD5421.h"

#include "common/DioLib.h"
#include "common/SpiLib.h"
 
// SPI variables
unsigned char ucTxComplete = 0;                  // Flag used to indicate SPI transfer complete
unsigned long ulRxData = 0;                      // Used to read SPI0 values
unsigned long ulDacVal = 0;                      // Used to readback AD5421 DAC value through SPI0

// Init SPI0 Interface
inline static void SPI0INIT (void)
{
  pADI_GP1->GPCON &= ~(GPCON_CON7_MSK+GPCON_CON6_MSK+GPCON_CON5_MSK+GPCON_CON4_MSK);
  pADI_GP1->GPCON |= 0xAA00;                     // Configure P1[4:7] for SPI0
  SpiBaud(pADI_SPI0, 10, SPIDIV_BCRST_DIS);
  SpiCfg(pADI_SPI0, SPICON_MOD_TX3RX3, SPICON_MASEN_EN,
         SPICON_CON_EN|SPICON_RXOF_EN|SPICON_ZEN_EN|SPICON_TIM_TXWR|SPICON_CPHA_SAMPLETRAILING|SPICON_ENABLE_EN);
}

extern void delay (long int length);
// Init. ADuCM360 to AD5421 interface.
void AD5421INIT(void)
{
  DioPulPin(pADI_GP1, PIN1, 1);                  // Enable Pull-up on P1.1 to read Fault pin level
  DioOenPin(pADI_GP1, PIN1, 0);                  // Enable P1.1 as an input to read Fault pin level
  SPI0INIT();
  NVIC_EnableIRQ(SPI0_IRQn);
  
  AD5421_Reset();
  while (ucTxComplete == 0){}                    // Wait for SPI1 interrupt
  ucTxComplete = 0;
  delay(1500);                                   // AD5421 requires 50uS delay after reset command before issueing more commands
  
  ul5421FAULT = AD5421_ReadFault(0x0);
  while (ucTxComplete == 0){}
  ucTxComplete = 0;
  AD5421_WriteToCon(0xFC80);                     // Watchdog off, Int ref on, disable automatic readback of Fault register
  ucTxComplete = 0;	 
  ul5421CON = AD5421_ReadCon(0x0);               // Read AD5421 control register - debug only

  SpiFifoFlush(pADI_SPI1, SPICON_TFLUSH_DIS, SPICON_RFLUSH_EN);
  ul5421FAULT = AD5421_ReadFault(0x0);           // Read AD5421 fault register - debug only
  while (ucTxComplete == 0){}
  ucTxComplete = 0;	 
}

// Write to IDAC data register								
void AD5421_WriteToDAC(unsigned long ulDACValue)
{
  unsigned long ulCMD = 0;
  unsigned long ulValue = 0; 	

  SpiFifoFlush(pADI_SPI0, SPICON_TFLUSH_EN, SPICON_RFLUSH_DIS);
  ulCMD = ((WRITEDAC)<<16);
  ulValue = ulDACValue;
  ulValue = (ulValue & 0x0000FFFF);              // Take 16-bit Register value
  ulValue |= ulCMD;                              // Append Command to bits 23-16.
  SpiTx(pADI_SPI0,(unsigned char)(ulValue >> 16));
  SpiTx(pADI_SPI0,(unsigned char)(ulValue >> 8));
  SpiTx(pADI_SPI0,(unsigned char)(ulValue));
  ucTxComplete = 0;
  ulValue = 0;
  while (ucTxComplete == 0)
  {
    if (ulValue >= 1000)
    {
      break;
    }
    else
    {
      ulValue++;
    }
  }
  ucTxComplete = 0;
}

// Write to IDAC Control register		
void AD5421_WriteToCon(unsigned long ulConValue)
{
  unsigned long ulCMD = 0;
  unsigned long ulValue = 0; 	
  SpiFifoFlush(pADI_SPI0, SPICON_TFLUSH_EN, SPICON_RFLUSH_DIS);
  ulCMD = ((WRITECON)<<16);
  ulValue = ulConValue;
  ulValue = (ulValue & 0x0000FFFF);              // Take 16-bit Register value
  ulValue |= ulCMD;                              // Append Command to bits 23-16.
  SpiFifoFlush(pADI_SPI0, SPICON_TFLUSH_DIS, SPICON_RFLUSH_DIS);
  SpiTx(pADI_SPI0,(unsigned char)(ulValue >> 16));
  SpiTx(pADI_SPI0,(unsigned char)(ulValue >> 8));
  SpiTx(pADI_SPI0,(unsigned char)(ulValue));

  while (ucTxComplete == 0) {}
  ucTxComplete = 0;
}

// Write to IDAC Offset adjust register		
void AD5421_WriteToOffAdj(unsigned long ulOffAdjValue)
{

}

// Write to IDAC Gain adjust register
void AD5421_WriteToGnAdj(unsigned long ulDACValue)	
{

}

// Load the IDAC output
void AD5421_LoadDac(void)
{
 
}

// force Alarm condition on IDAC output								
void AD5421_ForceAlarm(void)
{
  unsigned long ulCMD = 0;

  ulCMD = ((FORCEALARM)<<16);
  SpiFifoFlush(pADI_SPI0, SPICON_TFLUSH_EN, SPICON_RFLUSH_DIS);
  SpiTx(pADI_SPI0, (unsigned char)(ulCMD >> 16));
  SpiTx(pADI_SPI0, (unsigned char)(ulCMD >> 8));
  SpiTx(pADI_SPI0, (unsigned char)(ulCMD));
  while (ucTxComplete == 0) {}
  ucTxComplete = 0;

  SpiFifoFlush(pADI_SPI0, SPICON_TFLUSH_EN, SPICON_RFLUSH_DIS);
  SpiTx(pADI_SPI0, (unsigned char)(ulCMD >> 16));
  SpiTx(pADI_SPI0, (unsigned char)(ulCMD >> 8));
  SpiTx(pADI_SPI0, (unsigned char)(ulCMD));
}

// Reset AD5421							
void AD5421_Reset(void)
{
  unsigned long ulCMD = 0;

  ulCMD = ((AD5421RESET)<<16);
  SpiFifoFlush(pADI_SPI0, SPICON_TFLUSH_EN, SPICON_RFLUSH_DIS);
  SpiTx(pADI_SPI0, (unsigned char)(ulCMD >> 16));
  SpiTx(pADI_SPI0, (unsigned char)(ulCMD >> 8));
  SpiTx(pADI_SPI0, (unsigned char)(ulCMD));
  while (ucTxComplete == 0) {}
  ucTxComplete = 0;

  SpiFifoFlush(pADI_SPI0, SPICON_TFLUSH_EN, SPICON_RFLUSH_DIS);
  SpiTx(pADI_SPI0, (unsigned char)(ulCMD >> 16));
  SpiTx(pADI_SPI0, (unsigned char)(ulCMD >> 8));
  SpiTx(pADI_SPI0, (unsigned char)(ulCMD));
}

// Measure Vloop or die temp via ADC								
void AD5421_InitADC(void)
{
 
}

// Read to IDAC data register								
unsigned long AD5421_ReadDAC(unsigned long ulDACValue)
{
  unsigned long ulCMD = 0;
  unsigned long ulValue = 0;
  unsigned long ulData = 0; 	
	
  SpiFifoFlush(pADI_SPI0, SPICON_TFLUSH_EN, SPICON_RFLUSH_EN);

  ulCMD = ((READDAC)<<16);
  ulValue = ulDACValue;
  ulValue = (ulValue & 0x0000FFFF);              // Take 16-bit Register value
  ulValue |= ulCMD;                              // Append Command to bits 23-16.
	
  SpiTx(pADI_SPI0, (unsigned char)(ulValue >> 16));
  SpiTx(pADI_SPI0, (unsigned char)(ulValue >> 8));
  SpiTx(pADI_SPI0, (unsigned char)(ulValue));
  while (ucTxComplete == 0) {}
  ucTxComplete = 0;

  ulData = ulRxData;
	
  return ulData;
}

// Read IDAC Control register		
unsigned long AD5421_ReadCon(unsigned long ulConValue)
{
  unsigned long ulCMD = 0;
  unsigned long ulValue = 0; 	
  unsigned long ulData = 0;
	
  SpiFifoFlush(pADI_SPI0, SPICON_TFLUSH_EN, SPICON_RFLUSH_EN);
  ulCMD = ((READCON)<<16);
  ulValue = ulConValue;
  ulValue = (ulValue & 0x0000FFFF);              // Take 16-bit Register value
  ulValue |= ulCMD;                              // Append Command to bits 23-16.

  SpiTx(pADI_SPI0, (unsigned char)(ulValue >> 16));
  SpiTx(pADI_SPI0, (unsigned char)(ulValue >> 8));
  SpiTx(pADI_SPI0, (unsigned char)(ulValue));
  while (ucTxComplete == 0) {}
  ucTxComplete = 0;

  ulData = ulRxData;

  return ulData;
}

// Read IDAC Offset adjust register	
void AD5421_ReadOffAdj(unsigned long ulOffAdjValue)
{
 
}	

// Read IDAC Gain adjust register
void AD5421_ReadGnAdj(unsigned long ulDACValue)
{
 
}

// Read Fault register		
unsigned long  AD5421_ReadFault(unsigned long ulDACValue)
{
  unsigned long ulCMD = 0;
  unsigned long ulValue = 0;
  unsigned long ulData = 0; 	
	
  SpiFifoFlush(pADI_SPI0, SPICON_TFLUSH_EN, SPICON_RFLUSH_DIS);
  SpiFifoFlush(pADI_SPI0, SPICON_TFLUSH_DIS, SPICON_RFLUSH_DIS);
  ulCMD = ((READFAULT)<<16);
  ulValue = ulDACValue;
  ulValue = (ulValue & 0x0000FFFF);              // Take 16-bit Register value
  ulValue |= ulCMD;                              // Append Command to bits 23-16.

  SpiTx(pADI_SPI0, (unsigned char)(ulValue >> 16));
  SpiTx(pADI_SPI0, (unsigned char)(ulValue >> 8));
  SpiTx(pADI_SPI0, (unsigned char)(ulValue));
  while (ucTxComplete == 0) {}
  ucTxComplete = 0;

  SpiFifoFlush(pADI_SPI0, SPICON_TFLUSH_DIS, SPICON_RFLUSH_EN);
  SpiFifoFlush(pADI_SPI0, SPICON_TFLUSH_DIS, SPICON_RFLUSH_DIS);
  SpiTx(pADI_SPI0, 0);
  SpiTx(pADI_SPI0, 0);
  SpiTx(pADI_SPI0, 0);
  while (ucTxComplete == 0) {}
  ulData = ulRxData;

  return ulData;
}		
 
void SPI0_Int_Handler ()
{
  unsigned char uiSPI0STA = 0;

  uiSPI0STA = SpiSta(pADI_SPI0);
  if ((uiSPI0STA & SPI0STA_RXOF) == SPI0STA_RXOF) // SPI0 Rx Overflow IRQ
  {

  }
  if ((uiSPI0STA & SPI0STA_RX) == SPI0STA_RX) // SPI0 Rx IRQ
  {

  }
  if ((uiSPI0STA & SPI0STA_TX) == SPI0STA_TX) // SPI0 Tx IRQ
  {
    ucTxComplete = 1;
    ulRxData = SpiRx(pADI_SPI0);
    ulRxData = (ulRxData << 8);
    ulRxData |= SpiRx(pADI_SPI0);
    ulRxData = (ulRxData << 8);
    ulRxData |= SpiRx(pADI_SPI0);
  }
  if ((uiSPI0STA & SPI0STA_TXUR) == SPI0STA_TXUR) // SPI0 Tx underflow IRQ
  {

  }
}

 