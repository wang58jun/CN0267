
#include <stdio.h>
#include <string.h>
#include <aducm360.h>

#include "common/WdtLib.h"
#include "common/ClkLib.h"
#include "common/DioLib.h"

#include "AD5421.h"
#include "ADC_RTD.h"
#include "HART_AD5700.h"

#if FLASH_ACCESS
#include "FlashEraseWrite.h"
#endif

/* 
 * Definitions & Variables
 */
#define DEV_REVISION         1
#define SF_REVISION          2
#define HW_REVISION          1
uint8_t DevTypeCode[2] = {0x04, 0x04};
uint8_t DevUniqueID[3] = {0x00, 0x00, 0x01};
uint8_t ManuID[2] = {0x00, 0x04};
uint8_t DeviceStatus = 0x00; // Device Status
uint8_t ColdStartFlg; // Cold Start Flag
uint8_t MoreStatusFlg; // More Status Flag
uint8_t DevSpecStatus[17] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // Command 48
uint8_t DSSUpdateFlg = 0; // Device Spec Status Update Flag

NVM_DATA_TypeDef NvmData = 
{
  0x00, // Command execute config change flag
  0x0000, // Configuration Change Counter
  0x00, // Polling Address
  0x01, // Loop Current Mode
  {0x82, 0x08, 0x20, 0x82, 0x08, 0x20,
   0x82, 0x08, 0x20, 0x82, 0x08, 0x20,
   0x82, 0x08, 0x20, 0x82, 0x08, 0x20,
   0x82, 0x08, 0x20, 0x82, 0x08, 0x20}, // Message (32 characters)
  {0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20,
   0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20,
   0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20,
   0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20}, // Long tag (Latin-1) 
  {0x82, 0x08, 0x20, 0x82, 0x08, 0x20}, // Tag (8 characters)
  {0x82, 0x08, 0x20, 0x82, 0x08, 0x20,
   0x82, 0x08, 0x20, 0x82, 0x08, 0x20}, // Descriptor (16 characters)
  {0x01, 0x01, 0x01}, // Date
  {0x00, 0x00, 0x00}, // Final assembly number
}; // NVM data

#if FLASH_ACCESS
// Fixed device ID address used for compatibility with other demo code examples
#define DEVICE_ID_ADDRESS    0xC864
#define NVM_DATA_ADDRESS     0x0001FC00
#define NVM_CHECK_VALUE      0x12345678
#endif

#define PV_USL               120.0f
#define PV_LSL               -40.0f
#define PV_URV               120.0f
#define PV_LRV               -40.0f
#define PV_UNIT              32 // DegC
float fPv = 25.0f; // Primary Value
float fPercent = 0.0f; // Primary %Range
float fmA = 4.0f; // Primary mA

uint32_t FreeTmrCnt = 0;

// Simple Delay routine
void delay (long int length)
{
  while (length > 0)
  {
    length--;
  }
}

/*
 * local functions
 */
// 4 ~ 20mA
inline static void SendResultToAD5421(void)
{
  unsigned int uiADC0RESULT = (unsigned int)(((fmA-4.0)/16.0f) * 0xFFFF);
  AD5421_WriteToDAC(uiADC0RESULT);
}

inline static void Float2Bytes(float fIn, uint8_t* pBytes)
{
  uint8_t* pdata = (uint8_t*)(&fIn);

  *(pBytes+3) = *(pdata++);
  *(pBytes+2) = *(pdata++);
  *(pBytes+1) = *(pdata++);
  *(pBytes) = *(pdata);
}

inline static void NvmUpdate(void)
{
  volatile unsigned char ucEraseSuccess = 0;
  unsigned long Array[80];
  unsigned int uiSize = sizeof(NvmData);
  
  /* Erase flash page */
  ucEraseSuccess = ErasePage(NVM_DATA_ADDRESS);
  /* Write flash */
  Array[0] = NVM_CHECK_VALUE;
  memcpy(((uint8_t*)Array)+4, &NvmData.CmdCfgChgFlg, uiSize);
  WriteToFlash(Array, NVM_DATA_ADDRESS, uiSize+4);
}

inline static void CfgChgInd(void)
{
  NvmData.CfgChgCnt ++; // Config Change Counter
  NvmData.CmdCfgChgFlg |= PM_SEL_BIT + SM_SEL_BIT; // Config Change Flags
#if FLASH_ACCESS
  NvmUpdate();
#endif
}

#if FLASH_ACCESS
inline static void LoadNvmData(void)
{
  unsigned long ulTemp;
  
  /* Load Device Unique ID */
  ulTemp = *(unsigned long*)DEVICE_ID_ADDRESS; // Read HART ID from Flash
  if (ulTemp != 0xFFFFFFFF)
  {
    DevUniqueID[0] = (ulTemp >> 16) & 0xff;
    DevUniqueID[1] = (ulTemp >> 8) & 0xff;
    DevUniqueID[2] = (ulTemp) & 0xff;
  }
  
  /* Load other Nvm data */
  ulTemp = *(unsigned long*)NVM_DATA_ADDRESS; // Read HART ID from Flash
  if (ulTemp == NVM_CHECK_VALUE)
  {
    uint8_t *pData = (uint8_t*)(NVM_DATA_ADDRESS+4);
    memcpy(&NvmData.CmdCfgChgFlg, pData, sizeof(NvmData));
  }
  else
  {
    NvmUpdate();
  }
}
#endif


/*
 * Main()
 */
int main()
{
  WdtCfg(T3CON_PRE_DIV1,T3CON_IRQ_EN,T3CON_PD_DIS);  // Disable Watchdog timer resets
  //Disable clock to unused peripherals: SPI1, I2C, PWM, T0, T1, DAC, DMA
  ClkDis(CLKDIS_DISSPI1CLK|CLKDIS_DISI2CCLK|CLKDIS_DISPWMCLK|CLKDIS_DIST0CLK|CLKDIS_DIST1CLK|CLKDIS_DISDACCLK|CLKDIS_DISDMACLK);
  ClkCfg(CLK_CD3,CLK_HF,CLKSYSDIV_DIV2EN,CLK_UCLKCG);  // Set CPU clock to 1MHz
  ClkSel(CLK_CD3,CLK_CD7,CLK_CD3,CLK_CD7);  // Select CD3 for SPI and UART System clock

  // Set P2.1 as an output to toggle the LED
  DioOenPin(pADI_GP2, PIN1, 1);
  // Set P2.2 as an input to change the device special status for HART UAL48 test case
  DioOenPin(pADI_GP2, PIN2, 0);
  DioPulPin(pADI_GP2, PIN2, 1);

  AD5421INIT();
  ADC_RTD_INIT();
#if FLASH_ACCESS
  LoadNvmData();
#endif
  HART_AD5700_Init();

  while (1)
  {
    DioTgl(pADI_GP2, 0x2); // Toggle P2.1

    /* Load PV value */
    fPv = ADC_RTD_GetTemperature();
    
    /* Update PV range% and mA */
    fPercent = (fPv - PV_LRV) / (PV_URV - PV_LRV) * 100.0f;
    if (fPercent < 0.0f)
    {
      fPercent = 0.0f;
    }
    if (fPercent > 100.0f)
    {
      fPercent = 100.0f;
    }
    if (NvmData.LoopCurrentMode)
    {
      fmA = fPercent/100.0f * 16 + 4.0f;
    }

    /* Drive 4~20mA out */
    SendResultToAD5421();

    /* Check key's status (for HART UAL048b test) */
    if (!(DioRd(pADI_GP2)&0x04))
    { /* Button pushed down */
      if (DSSUpdateFlg == 0)
      { /* Status need to be update */
        DevSpecStatus[0] = 0x01; // Set the LSB in command48 response data
        MoreStatusFlg = PM_SEL_BIT + SM_SEL_BIT; // Set More_Status bits
        DSSUpdateFlg = 1;
      }
    }
    else
    {
      DevSpecStatus[0] = MoreStatusFlg = DSSUpdateFlg = 0; // Reset flags
    }
    
    /* 1s period compensation */
    delay(0x20000);
    /* Free timer counter update (unit in 1/32ms) */
    FreeTmrCnt += 1000 * 32; // 1s updates
  }
}


/* HART Commands Execution Routines */
// Parameters:
//  *pIn: Request data, including byte_count and data
//  *pOut: Response data, including byte_count and data
#define CMD_REQ_BC           pIn[0]
#define pCMD_REQ_DATA        (pIn+1)
#define CMD_RSP_BC           pOut[0]
#define CMD_RSP_CODE         pOut[1]
#define pCMD_RSP_DATA        (pOut+3)
void Cmd0Routine(uint8_t *pIn, uint8_t *pOut)
{
  CMD_RSP_BC = 2 + 22;
  CMD_RSP_CODE = 0;

  pCMD_RSP_DATA[0] = 0xfe;
  pCMD_RSP_DATA[1] = DevTypeCode[0];
  pCMD_RSP_DATA[2] = DevTypeCode[1];
  pCMD_RSP_DATA[3] = 5;
  pCMD_RSP_DATA[4] = 7;
  pCMD_RSP_DATA[5] = DEV_REVISION;
  pCMD_RSP_DATA[6] = SF_REVISION;
  pCMD_RSP_DATA[7] = HW_REVISION << 3;
  pCMD_RSP_DATA[8] = 0;
  pCMD_RSP_DATA[9] = DevUniqueID[0];
  pCMD_RSP_DATA[10] = DevUniqueID[1];
  pCMD_RSP_DATA[11] = DevUniqueID[2];
  pCMD_RSP_DATA[12] = MIN_PRE_STM_NUM;
  pCMD_RSP_DATA[13] = 0; // Device Varialbe Max Code
  pCMD_RSP_DATA[14] = (NvmData.CfgChgCnt >> 8) & 0xff;
  pCMD_RSP_DATA[15] = NvmData.CfgChgCnt & 0xff;
  pCMD_RSP_DATA[16] = 0; // Extended Device Status
  pCMD_RSP_DATA[17] = ManuID[0];
  pCMD_RSP_DATA[18] = ManuID[1];
  pCMD_RSP_DATA[19] = ManuID[0];
  pCMD_RSP_DATA[20] = ManuID[1];
  pCMD_RSP_DATA[21] = 1;
}

void Cmd1Routine(uint8_t *pIn, uint8_t *pOut)
{
  CMD_RSP_BC = 2 + 5;
  CMD_RSP_CODE = 0;

  pCMD_RSP_DATA[0] = PV_UNIT;
  Float2Bytes(fPv, &pCMD_RSP_DATA[1]);
}

void Cmd2Routine(uint8_t *pIn, uint8_t *pOut)
{
  CMD_RSP_BC = 2 + 8;
  CMD_RSP_CODE = 0;

  Float2Bytes(fmA, &pCMD_RSP_DATA[0]);
  Float2Bytes(fPercent, &pCMD_RSP_DATA[4]);
}

void Cmd3Routine(uint8_t *pIn, uint8_t *pOut)
{
  CMD_RSP_BC = 2 + 9;
  CMD_RSP_CODE = 0;

  Float2Bytes(fmA, &pCMD_RSP_DATA[0]);

  pCMD_RSP_DATA[4] = PV_UNIT;
  Float2Bytes(fPv, &pCMD_RSP_DATA[5]);
}

void Cmd6Routine(uint8_t *pIn, uint8_t *pOut)
{
  if (CMD_REQ_BC < 1)
  {
    CMD_RSP_BC = 2;
    CMD_RSP_CODE = 5; // Too few bytes
    return;
  }

  if (pCMD_REQ_DATA[0] > 63)
  {
    CMD_RSP_BC = 2;
    CMD_RSP_CODE = 2; // Invalid selection
    return;
  }

  if (CMD_REQ_BC == 1)
  { /* HART 5 */
    if (pCMD_REQ_DATA[0] == 0)
    {
      NvmData.LoopCurrentMode = 1;
    }
    else
    {
      NvmData.LoopCurrentMode = 0;
    }
  }
  else
  { /* HART 6 & 7 */
    if (pCMD_REQ_DATA[1] > 1)
    { /* Check loop current mode */
      CMD_RSP_BC = 2;
      CMD_RSP_CODE = 2; // Invalid selection
      return;
    }
    else
    {
      NvmData.LoopCurrentMode = pCMD_REQ_DATA[1];
    }
  }

  if (NvmData.LoopCurrentMode == 0)
  {
    DeviceStatus |= CURRENT_FIXED_BIT;
  }
  else
  {
    DeviceStatus &= ~CURRENT_FIXED_BIT;
  }
  
  NvmData.PollingAddr = pCMD_REQ_DATA[0];
  if (NvmData.PollingAddr != 0)
  {
    fmA = 4.0f;
  }
  
  CfgChgInd();                       /* Config Changed            */
  
  CMD_RSP_BC = 2 + 2;
  CMD_RSP_CODE = 0;

  pCMD_RSP_DATA[0] = NvmData.PollingAddr;
  pCMD_RSP_DATA[1] = NvmData.LoopCurrentMode;
}

void Cmd7Routine(uint8_t *pIn, uint8_t *pOut)
{
  CMD_RSP_BC = 2 + 2;
  CMD_RSP_CODE = 0;

  pCMD_RSP_DATA[0] = NvmData.PollingAddr;
  pCMD_RSP_DATA[1] = NvmData.LoopCurrentMode;
}

void Cmd8Routine(uint8_t *pIn, uint8_t *pOut)
{
  CMD_RSP_BC = 2 + 4;
  CMD_RSP_CODE = 0;

  pCMD_RSP_DATA[0] = 64; // Classification: Temperature
  pCMD_RSP_DATA[1] = 250;
  pCMD_RSP_DATA[2] = 250;
  pCMD_RSP_DATA[3] = 250;
}

void Cmd9Routine(uint8_t *pIn, uint8_t *pOut)
{
  uint8_t i, DVCode;
  uint8_t bc = CMD_REQ_BC;
  
  if (bc < 1)
  {
    CMD_RSP_BC = 2;
    CMD_RSP_CODE = 5; // Too few bytes
    return;
  }
  
  if (bc > 8)
  {
    bc = 8;
  }
  
  CMD_RSP_BC = 2 + 1 + 8*bc + 4;
  CMD_RSP_CODE = 0;

  pCMD_RSP_DATA[0] = 0x00; // Extended Device Status
  
  for(i = 0; i < bc; i++)
  {
    DVCode = pCMD_RSP_DATA[1 + i * 8] = pCMD_REQ_DATA[i];
    if (DVCode == 0xff)
    {
      CMD_RSP_BC = 2;
      CMD_RSP_CODE = 2; // Invalid selection
      return;
    }
    else if ((DVCode == 0x00) || (DVCode == 246))
    { /* PV */
      pCMD_RSP_DATA[1 + i * 8 + 1] = 64; // Classification: Temperature
      pCMD_RSP_DATA[1 + i * 8 + 2] = PV_UNIT;
      Float2Bytes(fPv, &pCMD_RSP_DATA[1 + i * 8 + 3]);
      pCMD_RSP_DATA[1 + i * 8 + 7] = 0; // Device Variable Status: 0
    }
    else if (DVCode == 244)
    { /* %Range Percent */
      pCMD_RSP_DATA[1 + i * 8 + 1] = 0;
      pCMD_RSP_DATA[1 + i * 8 + 2] = 57;
      Float2Bytes(fPercent, &pCMD_RSP_DATA[1 + i * 8 + 3]);
      pCMD_RSP_DATA[1 + i * 8 + 7] = 0;
    }
    else if (DVCode == 245)
    { /* Loop Current */
      pCMD_RSP_DATA[1 + i * 8 + 1] = 0;
      pCMD_RSP_DATA[1 + i * 8 + 2] = 39;
      Float2Bytes(fmA, &pCMD_RSP_DATA[1 + i * 8 + 3]);
      pCMD_RSP_DATA[1 + i * 8 + 7] = 0;
    }
    else
    {
      pCMD_RSP_DATA[1 + i * 8 + 1] = 0;
      pCMD_RSP_DATA[1 + i * 8 + 2] = 250;
      pCMD_RSP_DATA[1 + i * 8 + 3] = 0x7F;
      pCMD_RSP_DATA[1 + i * 8 + 4] = 0xA0;
      pCMD_RSP_DATA[1 + i * 8 + 5] = 0;
      pCMD_RSP_DATA[1 + i * 8 + 6] = 0;
      pCMD_RSP_DATA[1 + i * 8 + 7] = 0x30;
    }
  }
  
  pCMD_RSP_DATA[1 + i * 8 + 0] = (FreeTmrCnt >> 24) & 0xff;
  pCMD_RSP_DATA[1 + i * 8 + 1] = (FreeTmrCnt >> 16) & 0xff;
  pCMD_RSP_DATA[1 + i * 8 + 2] = (FreeTmrCnt >> 8) & 0xff;
  pCMD_RSP_DATA[1 + i * 8 + 3] = FreeTmrCnt & 0xff;
}

void Cmd11Routine(uint8_t *pIn, uint8_t *pOut)
{
  CMD_RSP_BC = 0; // no response initial
  
  if (CMD_REQ_BC >= 6)
  {
    if ((pCMD_REQ_DATA[0] == NvmData.Tag[0]) &&
        (pCMD_REQ_DATA[1] == NvmData.Tag[1]) &&
        (pCMD_REQ_DATA[2] == NvmData.Tag[2]) &&
        (pCMD_REQ_DATA[3] == NvmData.Tag[3]) &&
        (pCMD_REQ_DATA[4] == NvmData.Tag[4]) &&
        (pCMD_REQ_DATA[5] == NvmData.Tag[5]))
    { /* Tag matched               */
      Cmd0Routine(pIn, pOut);
    }
  }
}

void Cmd12Routine(uint8_t *pIn, uint8_t *pOut)
{
  CMD_RSP_BC = 2 + 24;
  CMD_RSP_CODE = 0;

  memcpy(pCMD_RSP_DATA, NvmData.Message, 24);
}

void Cmd13Routine(uint8_t *pIn, uint8_t *pOut)
{
  CMD_RSP_BC = 2 + 6 + 12 + 3;
  CMD_RSP_CODE = 0;

  memcpy(pCMD_RSP_DATA, NvmData.Tag, 6);
  memcpy(pCMD_RSP_DATA+6, NvmData.Descriptor, 12);
  memcpy(pCMD_RSP_DATA+18, NvmData.Date, 3);
}

void Cmd14Routine(uint8_t *pIn, uint8_t *pOut)
{
  CMD_RSP_BC = 2 + 16;
  CMD_RSP_CODE = 0;

  pCMD_RSP_DATA[0] = pCMD_RSP_DATA[1] = pCMD_RSP_DATA[2] = 0x00; // Transducer Serial Number
  
  pCMD_RSP_DATA[3] = PV_UNIT;
  Float2Bytes(PV_USL, &pCMD_RSP_DATA[4]); // USL
  Float2Bytes(PV_LSL, &pCMD_RSP_DATA[8]); // LSL
  Float2Bytes(0.1f, &pCMD_RSP_DATA[12]); // Min Span
}

void Cmd15Routine(uint8_t *pIn, uint8_t *pOut)
{
  CMD_RSP_BC = 2 + 18;
  CMD_RSP_CODE = 0;

  pCMD_RSP_DATA[0] = 250; // Alarm Code: Not Used
  pCMD_RSP_DATA[1] = 0x00; // PV Transfer Function Code
  
  pCMD_RSP_DATA[2] = PV_UNIT;
  Float2Bytes(PV_URV, &pCMD_RSP_DATA[3]); // URV
  Float2Bytes(PV_LRV, &pCMD_RSP_DATA[7]); // LRV
  Float2Bytes(1.0f, &pCMD_RSP_DATA[11]); // Damp Value
  
  pCMD_RSP_DATA[15] = 251; // Write Protect: None
  pCMD_RSP_DATA[16] = 250;
  pCMD_RSP_DATA[17] = 0;
}

void Cmd16Routine(uint8_t *pIn, uint8_t *pOut)
{
  CMD_RSP_BC = 2 + 3;
  CMD_RSP_CODE = 0;

  memcpy(pCMD_RSP_DATA, NvmData.FinalAsmbNum, 3);
}

void Cmd17Routine(uint8_t *pIn, uint8_t *pOut)
{
  if (CMD_REQ_BC < 24)
  {
    CMD_RSP_BC = 2;
    CMD_RSP_CODE = 5;
    return;
  }

  CMD_RSP_BC = 2 + 24;
  CMD_RSP_CODE = 0;

  for (uint8_t i=0; i<24; i++)
  {
    pCMD_RSP_DATA[i] = NvmData.Message[i] = pCMD_REQ_DATA[i];
  }
  
  CfgChgInd();
}

void Cmd18Routine(uint8_t *pIn, uint8_t *pOut)
{
  uint8_t i;
  
  if (CMD_REQ_BC < 21)
  {
    CMD_RSP_BC = 2;
    CMD_RSP_CODE = 5;
    return;
  }

  CMD_RSP_BC = 2 + 6 + 12 + 3;
  CMD_RSP_CODE = 0;

  for (i=0; i<6; i++)
  {
    pCMD_RSP_DATA[i] = NvmData.Tag[i] = pCMD_REQ_DATA[i];
  }
  for (i=0; i<12; i++)
  {
    pCMD_RSP_DATA[i+6] = NvmData.Descriptor[i] = pCMD_REQ_DATA[i+6];
  }
  for (i=0; i<3; i++)
  {
    pCMD_RSP_DATA[i+18] = NvmData.Date[i] = pCMD_REQ_DATA[i+18];
  }
  
  CfgChgInd();
}

void Cmd19Routine(uint8_t *pIn, uint8_t *pOut)
{
  if (CMD_REQ_BC < 3)
  {
    CMD_RSP_BC = 2;
    CMD_RSP_CODE = 5;
    return;
  }

  CMD_RSP_BC = 2 + 3;
  CMD_RSP_CODE = 0;

  for (uint8_t i=0; i<3; i++)
  {
    pCMD_RSP_DATA[i] = NvmData.FinalAsmbNum[i] = pCMD_REQ_DATA[i];
  }
  
  CfgChgInd();
}

void Cmd20Routine(uint8_t *pIn, uint8_t *pOut)
{
  CMD_RSP_BC = 2 + 32;
  CMD_RSP_CODE = 0;

  memcpy(pCMD_RSP_DATA, NvmData.LongTag, 32);
}

void Cmd21Routine(uint8_t *pIn, uint8_t *pOut)
{
  CMD_RSP_BC = 0; // no response initial
  
  if (CMD_REQ_BC >= 32)
  {
    for (uint8_t i=0; i<32; i++)
    {
      if (NvmData.LongTag[i] != pCMD_REQ_DATA[i])
      {
        return;
      }
    }
    Cmd0Routine(pIn, pOut);
  }
}

void Cmd22Routine(uint8_t *pIn, uint8_t *pOut)
{
  if (CMD_REQ_BC < 32)
  {
    CMD_RSP_BC = 2;
    CMD_RSP_CODE = 5;
    return;
  }

  CMD_RSP_BC = 2 + 32;
  CMD_RSP_CODE = 0;

  for (uint8_t i=0; i<32; i++)
  {
    pCMD_RSP_DATA[i] = NvmData.LongTag[i] = pCMD_REQ_DATA[i];
  }
  
  CfgChgInd();
}

void Cmd38Routine(uint8_t *pIn, uint8_t *pOut)
{
  CMD_RSP_BC = 2 + 2;
  CMD_RSP_CODE = 0;

  if (CMD_REQ_BC == 0)
  {
    pCMD_RSP_DATA[0] = (NvmData.CfgChgCnt >> 8) & 0xff;
    pCMD_RSP_DATA[1] = NvmData.CfgChgCnt & 0xff;
  }
  else
  {
    if (CMD_REQ_BC < 2)
    {
      CMD_RSP_BC = 2;
      CMD_RSP_CODE = 5;
      return;
    }
    if (NvmData.CfgChgCnt != (pCMD_REQ_DATA[0] * 256 + pCMD_REQ_DATA[1]))
    {
      CMD_RSP_BC = 2;
      CMD_RSP_CODE = 9;
      return;
    }
    pCMD_RSP_DATA[0] = pCMD_REQ_DATA[0];
    pCMD_RSP_DATA[1] = pCMD_REQ_DATA[1];
  }

  if (GetMasterType() == PRIMARY_MASTER)
  {
    NvmData.CmdCfgChgFlg &= ~PM_SEL_BIT; // Clear Config Change Flag
  }
  else
  {
    NvmData.CmdCfgChgFlg &= ~SM_SEL_BIT;
  }
}

#define CMD48_DATA_BYTES 25
void Cmd48Routine(uint8_t *pIn, uint8_t *pOut)
{
  uint8_t i;

  CMD_RSP_BC = 2 + CMD48_DATA_BYTES;
  CMD_RSP_CODE = 0;

  if (CMD_REQ_BC != 0)
  { /* HART 7                    */
    if (CMD_REQ_BC < CMD48_DATA_BYTES)
    {
      CMD_RSP_BC = 2;
      CMD_RSP_CODE = 5;
      return;
    }
    
    for (i=0; i<6; i++)
    {
      if (DevSpecStatus[i] != pCMD_REQ_DATA[i]) // Check Device Spec Status 0~5
      {
        break;
      }
    }
    if (i >= 6)
    {
      for (; i<14; i++)
      {
        if (0 != pCMD_REQ_DATA[i]) // Check other bytes
        {
          break;
        }
      }
      if (i >= 14)
      {
        for (; i<CMD48_DATA_BYTES; i++)
        {
          if (DevSpecStatus[i - 14 + 6] != pCMD_REQ_DATA[i]) /* Check Spec Status 6~16 */
          {
            break;
          }
        }

        if (i >= CMD48_DATA_BYTES)
        {                                           
          if (GetMasterType() == PRIMARY_MASTER)
          {
            MoreStatusFlg &= ~PM_SEL_BIT; // Clear More Status Flag
          }
          else
          {
            MoreStatusFlg &= ~SM_SEL_BIT;
          }
        }
      }
    }
  }

  for (i=0; i<CMD48_DATA_BYTES; i++)
  {
    pCMD_RSP_DATA[i] = 0;
  }
  for (i=0; i<6; i++)
  {
    pCMD_RSP_DATA[i] = DevSpecStatus[i];
  }
  for (i=0; i<11; i++)
  {
    pCMD_RSP_DATA[14+i] = DevSpecStatus[i+6];
  }
}
