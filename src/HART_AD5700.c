/*
    Module       : HART_AD5700.h
    Description  : HART Implementations based on AD5700
    Date         : 01 Jan 2016
    Version      : v1.00
    Changelog    : v1.00 Initial
*/

#include <ADuCM360.h>
#include <stdio.h>
#include "HART_AD5700.h"

#include "common/DioLib.h"
#include "common/ClkLib.h"
#include "common/IntLib.h"
#include "common/UrtLib.h"

/* 
 * Definitions
 */
#define TRUE                 1
#define FALSE                0

// HART Communication Error Code
#define PHY_COMM_CORRECT     0x00u
#define PHY_OVERFLOW_ERROR   0x01u
#define PHY_BUFFER_OVERFLW   0x02u
#define PHY_CHECKSUM_ERROR   0x08u
#define PHY_FRAMING_ERROR    0x10u
#define PHY_OVERRUN_ERROR    0x20u
#define PHY_PARITY_ERROR     0x40u
#define PHY_COMM_ERROR       0x80u

typedef struct {
  uint8_t *pFrame;
  uint8_t PreambleNum;
  uint8_t FrmType;
  uint8_t CommErrCode;
} RCV_FRAME_TypeDef;

#define RCV_FRM_NONE         0
#define RCV_FRM_ACK          1
#define RCV_FRM_STX          2
#define RCV_FRM_BACK         3
#define RCV_FRM_COMM_ERR     4

// HART Address Type
#define ADDR_MODE_MASK       (0x01 << 7)
#define POLLING_ADDR_MODE    0x00u
#define UNIQUE_ADDR_MODE     0x80u

// HART Receiving Frame Type
#define FRAME_TYPE_MASK      (0x07 << 0)
#define FRAME_BACK           0x01u
#define FRAME_STX            0x02u
#define FRAME_ACK            0x06u

/* 
 * Variables
 */
static int DLLState;  // DLL state
#define DLL_STATE_WAIT       0
#define DLL_STATE_RCV        1
#define DLL_STATE_PRO        2
#define DLL_STATE_XMT        3

static int RcvState;  // DLL Receiving state
#define RCV_STATE_SOM        0
#define RCV_STATE_HEADER     1
#define RCV_STATE_DATA       2
#define RCV_STATE_WFE        3

#define RCV_BUFFER_LENGTH    255
static uint8_t RcvBuffer[RCV_BUFFER_LENGTH];
static uint8_t RcvBufferIdx = 0;
#define RCV_FRAME_SIZE       2
static RCV_FRAME_TypeDef RcvFrame[RCV_FRAME_SIZE];
static uint8_t RcvFrameIdx = 0; // Index for Receiving Frame Array
#define RCV_DELIMITER_BYTE   RcvFrame[RcvFrameIdx].pFrame[0]
#define RCV_ADDRESS_MODE     (RCV_DELIMITER_BYTE & ADDR_MODE_MASK)
#define RCV_ADDRESS_POINT    (RcvFrame[RcvFrameIdx].pFrame + 1)
#define RCV_MASTER_TYPE      (*RCV_ADDRESS_POINT & MASTER_TYPE_MASK)
static uint8_t *pRcvCmdAddr = NULL; // Address point for received command number byte
static uint8_t RxBrdcstAddr = FALSE;

#define XMT_BUFFER_LENGTH    127
static uint8_t XmtBuffer[XMT_BUFFER_LENGTH];
#define XMT_DELIMITER_BYTE   XmtBuffer[0]
#define XMT_ADDRESS_POINT    (XmtBuffer+1)
static uint8_t TxFrmLen; // Tx frame length (not including preambles and checkbyte)
static uint8_t TxByteCnt; // Byte-count for transmitting routine
static uint8_t TxCheckByte; // Tx Check Byte

/*
 * local functions
 */
/***** Physical Layer SAPs *****/
// RTS Control
inline static void PhyRtsClr(void)
{
  DioClr(pADI_GP0, 0x10);
}

inline static void PhyRtsSet(void)
{
  DioSet(pADI_GP0, 0x10);
}

// Uart Control
inline static void PhyUartEnable(void)
{
  pADI_UART->COMCON = COMCON_DISABLE_DIS;
}

inline static void PhyUartDisable(void)
{
  pADI_UART->COMCON = COMCON_DISABLE_EN;
  UrtIntCfg(pADI_UART, 0);  // Diable all UART IRQs
  NVIC_DisableIRQ(UART_IRQn);
}

inline static void PhyUartRxEnable(void)
{
  PhyUartEnable();
  UrtIntCfg(pADI_UART, COMIEN_ERBFI);  // Enable UART Rx IRQ
  NVIC_EnableIRQ(UART_IRQn);
}

inline static void PhyUartTxEnable(void)
{
  PhyUartEnable();
  UrtIntCfg(pADI_UART, COMIEN_ETBEI);  // Enable UART Tx IRQ
  NVIC_EnableIRQ(UART_IRQn);
}

inline static void PhyUartTxByte(uint8_t u)
{
  UrtTx(pADI_UART, u);
}

/***** DLL Layer SAPs *****/
static void DllRcvThread(uint8_t ComErrCode, uint8_t RxByte);
static void DllProcessThread(void);
static uint8_t VerifyAddress(void);
static void TxAckFrame(uint8_t *pFrmPayLoad);
static void DllXmtThread(void);

/***** APP Layer SAPs *****/
static uint8_t CmdExeThread(uint8_t MasterType, uint8_t *pCmd);

/*
 * Prototypes
 */
void HART_AD5700_Init(void)
{
  /*** Physical Layer Initial ***/
  // Setup Dios
  DioOenPin(pADI_GP0, PIN3, 1);  // P0.3: out for CLK_CFG
  DioOenPin(pADI_GP0, PIN4, 1);  // P0.4: out for HART_RTS
  DioOenPin(pADI_GP0, PIN5, 0);  // P0.5: in for HART_CD
  DioOenPin(pADI_GP0, PIN6, 0);  // P0.6: in for HART_RXD
  DioOenPin(pADI_GP0, PIN7, 1);  // P0.7: out for HART_TXD
  DioOenPin(pADI_GP1, PIN0, 0);  // P1.0: in for HART_CLK
  
  DioClr(pADI_GP0, 0x08);  // P0.3 CLK_CFG clear, no clockout from AD5700
  PhyRtsSet();  // Initial in demodulate mode

  // Setup Uart
  pADI_GP0->GPCON &= ~(GPCON_CON7_MSK+GPCON_CON6_MSK);
  pADI_GP0->GPCON |= 0x9000;  // Configure P0.6/P0.7 for UART
  UrtCfg(pADI_UART, B1200, COMLCR_WLS_8BITS, 0|COMLCR_PEN_EN);  // setup baud rate for 1200, 8-bits, odd-parity
  // --------------------------------------------------------------------------
  // Baud rate (for more accurate calculating)
  // --------------------------------------------------------------------------
  // Optimized for minimum clock frequency => minimum power consumption
  // NOTE: UART clock set in CLKCON1 must be >= uC core clock set in CLKCON0 !!
  // --------------------------------------------------------------------------
  #define SYSCLK 16000000 // Should be taken from / moved to somewhere global?
  unsigned long ulTmp1;
  unsigned long ulTmp2;

  // Calculate Ratio of UART clock frequency to Baud Rate
  ulTmp1 = SYSCLK << 7;       // Multiply by 128, headroom for further rounding
  ulTmp1 >>= pADI_CLKCTL->CLKSYSDIV;                           // System Clock
  ulTmp1 >>= (pADI_CLKCTL->CLKCON1 & CLKCON1_UARTCD_MSK) >> 9; // UART Clock
  ulTmp1 /= B1200;            // Temp1 = 128 * (Ratio = UART Clock / Baud Rate)

  // Set UART COMDIV as round(Ratio / 128)
  // This maximizes UART fractional divider value for best Baud accuracy 
  ulTmp2 = ulTmp1 >>13;       // divided by 128, divided by 64
  ulTmp2++;                   // Correction for rounding
  ulTmp2 >>= 1;               // Final division, round(Ratio / 128)
  pADI_UART->COMDIV = ulTmp2; // Write to register
  
  // Set UART COMFBR as round(64 * Ratio / ComDiv)
  ulTmp1 /= ulTmp2;           // 128 * Ratio / ComDiv
  ulTmp1++;                   // Correction for rounding
  ulTmp1 >>= 1;               // Final division, round(64 * Ratio / ComDiv)
  ulTmp1 |=COMFBR_ENABLE;     // Enable fractional divider
  pADI_UART->COMFBR = ulTmp1; // Write to register
  // --------------------------------------------------------------------------
  // --------------------------------------------------------------------------
  PhyUartDisable();  // Disable UART at initial
  
  // Setup external interrupt for HART_CD
  EiCfg(EXTINT1, INT_EN, INT_RISE);
  NVIC_EnableIRQ(EINT1_IRQn);

  /*** DLL Layer Initial ***/
  DLLState = DLL_STATE_WAIT;
  RcvState = RCV_STATE_SOM;
    
  /*** APP Layer Initial ***/
  DeviceStatus = MoreStatusFlg = 0x00;
  ColdStartFlg = PM_SEL_BIT + SM_SEL_BIT; // Set Cold start flag for both masters
}

// Interrupt service points
void Ext_Int1_Handler(void)
{
  EiClr(EXTINT1); 

  if ((pADI_INTERRUPT->EI0CFG&EI0CFG_IRQ1MDE_MSK) == EI0CFG_IRQ1MDE_RISE)
  { // CD rising, Phy_enable_ind
    if (DLLState == DLL_STATE_WAIT)
    {
      // Change state to receiving
      DLLState = DLL_STATE_RCV;
      // Initial variables for receiving thread
      RcvState = RCV_STATE_SOM;
      RcvBufferIdx = 0;
      RcvFrame[0].pFrame = RcvFrame[1].pFrame = NULL;
      RcvFrame[0].PreambleNum = RcvFrame[1].PreambleNum = 0;
      RcvFrame[0].FrmType = RcvFrame[1].FrmType = RCV_FRM_NONE;
      RcvFrame[0].CommErrCode = RcvFrame[1].CommErrCode = 0;
      RcvFrameIdx = 0;
      // Enable Uart Rx
      PhyUartRxEnable();
      // Config to trigger CD fall next time
      EiCfg(EXTINT1, INT_EN, INT_FALL);
    }
  }
  else if ((pADI_INTERRUPT->EI0CFG&EI0CFG_IRQ1MDE_MSK) == EI0CFG_IRQ1MDE_FALL)
  { // CD falling, Phy_disable_ind
    if (DLLState == DLL_STATE_RCV)
    {
      // Disable Uart
      PhyUartDisable();
      // Config to trigger CD rise next time
      EiCfg(EXTINT1, INT_EN, INT_RISE);
      // Change state to receiving
      DLLState = DLL_STATE_PRO;
      // Go to process thread
      DllProcessThread();
    }
  }
}

void UART_Int_Handler(void)
{
  volatile uint8_t ucCOMIIR = UrtIntSta(pADI_UART)&0x07; // Read UART Interrupt ID - must be read
  uint8_t TempByte = UrtLinSta(pADI_UART); // Read Line Status - can be read only once

  if (((pADI_UART->COMIEN&COMIEN_ERBFI_MSK) == COMIEN_ERBFI_EN) &&
      ((ucCOMIIR==COMIIR_STA_RXLINESTATUS) || (ucCOMIIR==COMIIR_STA_RXBUFFULL)))
  { // Receive byte
    uint8_t UartCommErr = PHY_COMM_CORRECT;
    if ((TempByte&COMLSR_OE_MSK) == COMLSR_OE_SET)
    {
      UartCommErr |= PHY_OVERRUN_ERROR;
    }
    if ((TempByte&COMLSR_PE_MSK) == COMLSR_PE_SET)
    {
      UartCommErr |= PHY_PARITY_ERROR;
    }
    if ((TempByte&COMLSR_FE_MSK) == COMLSR_FE_SET)
    {
      UartCommErr |= PHY_FRAMING_ERROR;
    }
    if ((TempByte&COMLSR_DR_MSK) == COMLSR_DR_SET)
    {
      TempByte = (uint8_t)UrtRx(pADI_UART);
    }
    DllRcvThread(UartCommErr, TempByte);
  }
  else if (((pADI_UART->COMIEN&COMIEN_ETBEI_MSK) == COMIEN_ETBEI_EN) && (ucCOMIIR == COMIIR_STA_TXBUFEMPTY))
  { // Transmit buffer empty
    DllXmtThread();
  }
}

/***** DLL Layer SAPs *****/
#define MIN_PRE_MTS_NUM      2
static void DllRcvThread(uint8_t ComErrCode, uint8_t RxByte)
{
  static uint8_t RcvDataCnt = 0;
  static uint8_t CheckByte = 0; // Check Byte
    
  if ((DLLState == DLL_STATE_RCV) && (RcvFrameIdx < RCV_FRAME_SIZE) && (RcvBufferIdx < RCV_BUFFER_LENGTH))
  {
    /* for debug */
    //RcvBuffer[RcvBufferIdx++] = RxByte;
    //RcvBuffer[RcvBufferIdx++] = ComErrCode;
    //return;
    switch(RcvState)
    {
    case RCV_STATE_SOM:
      if (ComErrCode == PHY_COMM_CORRECT)
      {
        if (RxByte == 0xff)
        {
          if (RcvFrame[RcvFrameIdx].PreambleNum <= 50)
          {
            RcvFrame[RcvFrameIdx].PreambleNum ++;
          }
        }
        else
        { /* Not 0xff */
          if (RcvFrame[RcvFrameIdx].PreambleNum >= MIN_PRE_MTS_NUM)
          {
            uint8_t TempByte = RxByte & FRAME_TYPE_MASK;
            if ((TempByte == FRAME_BACK) || (TempByte == FRAME_STX) || (TempByte == FRAME_ACK))
            { /* Valid SOM values */
              RcvDataCnt = (RxByte & ADDR_MODE_MASK) ? 7:3;
              RcvFrame[RcvFrameIdx].CommErrCode = PHY_COMM_CORRECT;
              CheckByte = RcvBuffer[RcvBufferIdx] = RxByte;
              RcvFrame[RcvFrameIdx].pFrame = &RcvBuffer[RcvBufferIdx];
              if (RxByte & 0x60)
              { /* Expanded byte num > 0 */
                RcvFrame[RcvFrameIdx].FrmType = RCV_FRM_NONE;
                RcvState = RCV_STATE_WFE; // DLL003
              }
              else
              {
                RcvBufferIdx ++;
                RcvState = RCV_STATE_HEADER; // Change to Rcv Header state
              }
            }
            else
            { /* Invalid SOM values */
                RcvFrame[RcvFrameIdx].PreambleNum = 0;
            }
          }
          else
          { /* preamble numbers too short */
            RcvFrame[RcvFrameIdx].PreambleNum = 0;
          }
        }
      }
      else
      { /* Physical layer communication error */
        RcvFrame[RcvFrameIdx].PreambleNum = 0;
      }
      break;
    
    case RCV_STATE_HEADER:
      if (ComErrCode == PHY_COMM_CORRECT)
      {
        RcvBuffer[RcvBufferIdx++] = RxByte;
        CheckByte ^= RxByte; // Update Check Byte
        RcvDataCnt --;
        if (RcvDataCnt == 0)
        { /* Header receiving end */
          RcvDataCnt = RxByte + 1; // Plus one more check byte
          RcvState = RCV_STATE_DATA; // Change to Rcv Data state
        }
      }
      else
      { /* Physical layer communication error */
        RcvFrame[RcvFrameIdx].FrmType = RCV_FRM_NONE;
        RcvFrame[RcvFrameIdx].PreambleNum = 0;
        RcvState = RCV_STATE_SOM; // Back to waiting for SOM
      }
      break;
    
    case RCV_STATE_DATA:
      if (ComErrCode == PHY_COMM_CORRECT)
      {
        RcvDataCnt --;
        if (RcvDataCnt != 0)
        { /* Data receiving */
          RcvBuffer[RcvBufferIdx++] = RxByte;
          CheckByte ^= RxByte; // Update Check Byte
          if (RcvBufferIdx >= RCV_BUFFER_LENGTH)
          { /* Buffer over flow error */
            RcvFrame[RcvFrameIdx].CommErrCode |= PHY_COMM_ERROR + PHY_BUFFER_OVERFLW;
            RcvFrame[RcvFrameIdx].FrmType = RCV_FRM_COMM_ERR;
            // Back to waiting for SOM
            RcvState = RCV_STATE_SOM; 
            RcvFrameIdx++; // Index to next receiving frame buffer
            if (RcvFrameIdx < RCV_FRAME_SIZE)
            {
              RcvFrame[RcvFrameIdx].PreambleNum = 0;
            }
          }
        }
        else
        { /* Check byte */
          if (RxByte != CheckByte)
          { /* Check byte error */
            RcvFrame[RcvFrameIdx].CommErrCode |= PHY_COMM_ERROR + PHY_CHECKSUM_ERROR;
            RcvFrame[RcvFrameIdx].FrmType = RCV_FRM_COMM_ERR;
          }
          else
          { /* Check byte ok */
            switch (RCV_DELIMITER_BYTE & FRAME_TYPE_MASK)
            { /* Setup Rcv Frame Type */
            case FRAME_BACK:
              RcvFrame[RcvFrameIdx].FrmType = RCV_FRM_BACK;
              break;
            case FRAME_STX:
              RcvFrame[RcvFrameIdx].FrmType = RCV_FRM_STX;
              break;
            case FRAME_ACK:
              RcvFrame[RcvFrameIdx].FrmType = RCV_FRM_ACK;
              break;
            default:
              RcvFrame[RcvFrameIdx].FrmType = RCV_FRM_NONE;
              break;
            }
          }
          // Back to waiting for SOM
          RcvState = RCV_STATE_SOM; 
          RcvFrameIdx++; // Index to next receiving frame buffer
          if (RcvFrameIdx < RCV_FRAME_SIZE)
          {
            RcvFrame[RcvFrameIdx].PreambleNum = 0;
          }
        }
      }
      else
      { /* Physical layer communication error */
        RcvFrame[RcvFrameIdx].CommErrCode |= PHY_COMM_ERROR + ComErrCode;
        RcvFrame[RcvFrameIdx].FrmType = RCV_FRM_COMM_ERR;
        // Back to waiting for SOM
        RcvState = RCV_STATE_SOM; 
        RcvFrameIdx++; // Index to next receiving frame buffer
        if (RcvFrameIdx < RCV_FRAME_SIZE)
        {
          RcvFrame[RcvFrameIdx].PreambleNum = 0;
        }
      }
      break;
      
    case RCV_STATE_WFE:
    default:
      break;
    }
  }
}

static uint8_t VerifyAddress(void)
{
  uint8_t rtn = FALSE;
  
  RxBrdcstAddr = FALSE;
  
  if (RCV_ADDRESS_MODE == POLLING_ADDR_MODE)
  {
    pRcvCmdAddr = RCV_ADDRESS_POINT+1;
    if ((*RCV_ADDRESS_POINT&0x3f) == (NvmData.PollingAddr&0x3f))
    {
      if (*pRcvCmdAddr == 0)
      { /* Command 0 */
        rtn = TRUE;
      }
      else
      {
        rtn = FALSE;
      }
    }
    else
    {
      rtn = FALSE;
    }
  }
  else
  { /* Unique Address */
    pRcvCmdAddr = RCV_ADDRESS_POINT+5;
    if (((*RCV_ADDRESS_POINT&0x3f) == (DevTypeCode[0]&0x3f))
         && (*(RCV_ADDRESS_POINT+1) == DevTypeCode[1])
         && (*(RCV_ADDRESS_POINT+2) == DevUniqueID[0])
         && (*(RCV_ADDRESS_POINT+3) == DevUniqueID[1])
         && (*(RCV_ADDRESS_POINT+4) == DevUniqueID[2]))
    {
      rtn = TRUE;
    }
    else
    {
      if ((*pRcvCmdAddr == 11) || (*pRcvCmdAddr == 21))
      {
        if (((*RCV_ADDRESS_POINT&0x3f) != 0) 
            || (*(RCV_ADDRESS_POINT+1) != 0)
            || (*(RCV_ADDRESS_POINT+2) != 0)
            || (*(RCV_ADDRESS_POINT+3) != 0)
            || (*(RCV_ADDRESS_POINT+4) != 0))
        {
          rtn = FALSE;
        }
        else
        {
          rtn = TRUE;
          RxBrdcstAddr = TRUE;
        }
      }
      else
      { /* Address Error */
        rtn = FALSE;
      }
    }
  }
  
  return rtn;
}

uint8_t GetMasterType(void)
{
  return RCV_MASTER_TYPE;
}

static void DllProcessThread(void)
{
  if (DLLState == DLL_STATE_PRO)
  {
    uint8_t flag = FALSE;
    RcvFrameIdx = RCV_FRAME_SIZE - 1;
    while (1)
    { /* Search for last valid frame */
      if ((RcvFrame[RcvFrameIdx].FrmType == RCV_FRM_STX) ||
          (RcvFrame[RcvFrameIdx].FrmType == RCV_FRM_ACK) ||
          (RcvFrame[RcvFrameIdx].FrmType == RCV_FRM_BACK))
      {
        flag = TRUE;
        break;
      }
      if (RcvFrameIdx == 0)
      {
        break;
      }
      else
      {
        RcvFrameIdx--;
      }
    }
    
    if ((RcvFrameIdx == 0) && (flag == FALSE))
    { /* No valid frames found */
      RcvFrameIdx = RCV_FRAME_SIZE - 1;
      while (1)
      { /* Search for last communication error frame */
        if (RcvFrame[RcvFrameIdx].FrmType == RCV_FRM_COMM_ERR)
        { /* Found the last one */
          if (VerifyAddress() == TRUE)
          { /* Valid address */
            uint8_t  TxCommErrFrm[3];
            TxCommErrFrm[0] = 0x02;
            TxCommErrFrm[1] = RcvFrame[RcvFrameIdx].CommErrCode; // 1st byte is Communication Code
            TxCommErrFrm[2] = 0x00; // 2nd is 0
            TxAckFrame(TxCommErrFrm); // start Tx
            return;
          }
        }
        if (RcvFrameIdx == 0)
        { /* No valid frams found */
          DLLState = DLL_STATE_WAIT;
          return;
        }
        else
        {
          RcvFrameIdx--;
        }
      }
    }
    else if (flag == TRUE)
    { /* Valid frame was found */
      if ((RcvFrame[RcvFrameIdx].FrmType == RCV_FRM_STX) && (VerifyAddress() == TRUE))
      {
        if (0 == CmdExeThread(RCV_MASTER_TYPE, pRcvCmdAddr))
        { /* Command execution not correct */
          DLLState = DLL_STATE_WAIT;
        }
        return;
      }
      else
      {
        DLLState = DLL_STATE_WAIT;
        return;
      }
    }
  }
}

static void TxAckFrame(uint8_t *pFrmPayLoad)
{
  uint8_t i, BC;

  /* 1-byte delimiter */
  XMT_DELIMITER_BYTE = 0x06; // ACK frame
  XMT_DELIMITER_BYTE |= RCV_ADDRESS_MODE; // Address Mode: Polling or Unique
  TxFrmLen = 1;

  /* 1 or 5 bytes address */
  if (RCV_ADDRESS_MODE == POLLING_ADDR_MODE)
  { /* polling address mode      */
    *XMT_ADDRESS_POINT = *RCV_ADDRESS_POINT & 0xbf; // Burst bit disable
    TxFrmLen++;
  }
  else
  { /* unique address mode       */ 
    if (RxBrdcstAddr == TRUE)
    { /* Broadcast mode */
      *XMT_ADDRESS_POINT = *(XMT_ADDRESS_POINT+1) = *(XMT_ADDRESS_POINT+2) = *(XMT_ADDRESS_POINT+3) = *(XMT_ADDRESS_POINT+4) = 0x00;
      if (RCV_MASTER_TYPE == PRIMARY_MASTER)
      {
        *XMT_ADDRESS_POINT |= PRIMARY_MASTER;
      }
    }
    else
    {
      *XMT_ADDRESS_POINT = *RCV_ADDRESS_POINT & 0xbf; // Burst bit disable
      *(XMT_ADDRESS_POINT+1) = *(RCV_ADDRESS_POINT+1);
      *(XMT_ADDRESS_POINT+2) = *(RCV_ADDRESS_POINT+2);
      *(XMT_ADDRESS_POINT+3) = *(RCV_ADDRESS_POINT+3);
      *(XMT_ADDRESS_POINT+4) = *(RCV_ADDRESS_POINT+4);
    }
    TxFrmLen += 5;
  }

  /* 1-byte command number */
  XmtBuffer[TxFrmLen++] = *pRcvCmdAddr;
  
  /* Byte count and data */
  BC = XmtBuffer[TxFrmLen++] = *pFrmPayLoad; // Byte Count byte
  if (BC >= XMT_BUFFER_LENGTH - 10)
  { // Error byte count
    DLLState = DLL_STATE_WAIT;
    return;
  }
  for (i = 1; i <= BC; i++)
  {
    XmtBuffer[TxFrmLen++] = *(pFrmPayLoad + i);
  }
  
  /* Start transmitting */
  TxByteCnt = TxCheckByte = 0;
  PhyUartTxEnable(); // Enable Uart Tx
//  PhyRtsClr(); // Enable RTS
  PhyUartTxByte(0xff); // Send out the first 0xff byte

  DLLState = DLL_STATE_XMT; // Enter into XMT state
  return;
}

static void DllXmtThread(void)
{
  uint8_t TempByte;
  if (DLLState == DLL_STATE_XMT)
  {
    TxByteCnt++;
    if (TxByteCnt == 1)
    { /* Tx start */
      PhyRtsClr(); // Enable RTS
    }
    if (TxByteCnt <= MIN_PRE_STM_NUM)
    { /* Preamble */
      PhyUartTxByte(0xff);
    }
    else if (TxByteCnt <= (MIN_PRE_STM_NUM+TxFrmLen))
    { /* Tx buffer data: from delimiter to the end of data */
      TempByte = XmtBuffer[TxByteCnt-MIN_PRE_STM_NUM-1];
      TxCheckByte ^= TempByte;
      PhyUartTxByte(TempByte);
    }
    else if (TxByteCnt == (MIN_PRE_STM_NUM+TxFrmLen+1))
    { /* Check byte */
      PhyUartTxByte(TxCheckByte);
    }
    else if (TxByteCnt == (MIN_PRE_STM_NUM+TxFrmLen+2))
    { /* One more byte */
      PhyUartTxByte(0xaa);
    }
    else
    { /* Tx end */
      PhyUartDisable();
      PhyRtsSet();
      DLLState = DLL_STATE_WAIT; // Back to wait state
    }
  }
  else
  { // Error event
    PhyUartDisable();
    PhyRtsSet();
    DLLState = DLL_STATE_WAIT;
  }
}

/***** APP Layer SAPs *****/
static uint8_t CmdRspData[100];
static uint8_t CmdExeThread(uint8_t MasterType, uint8_t *pCmd)
{
  switch(*pCmd)
  { /* Commands table */
  case 0: Cmd0Routine(pCmd+1, CmdRspData); break;
  case 1: Cmd1Routine(pCmd+1, CmdRspData); break;
  case 2: Cmd2Routine(pCmd+1, CmdRspData); break;
  case 3: Cmd3Routine(pCmd+1, CmdRspData); break;
  case 6: Cmd6Routine(pCmd+1, CmdRspData); break;
  case 7: Cmd7Routine(pCmd+1, CmdRspData); break;
  case 8: Cmd8Routine(pCmd+1, CmdRspData); break;
  case 9: Cmd9Routine(pCmd+1, CmdRspData); break;
  case 11: Cmd11Routine(pCmd+1, CmdRspData); break;
  case 12: Cmd12Routine(pCmd+1, CmdRspData); break;
  case 13: Cmd13Routine(pCmd+1, CmdRspData); break;
  case 14: Cmd14Routine(pCmd+1, CmdRspData); break;
  case 15: Cmd15Routine(pCmd+1, CmdRspData); break;
  case 16: Cmd16Routine(pCmd+1, CmdRspData); break;
  case 17: Cmd17Routine(pCmd+1, CmdRspData); break;
  case 18: Cmd18Routine(pCmd+1, CmdRspData); break;
  case 19: Cmd19Routine(pCmd+1, CmdRspData); break;
  case 20: Cmd20Routine(pCmd+1, CmdRspData); break;
  case 21: Cmd21Routine(pCmd+1, CmdRspData); break;
  case 22: Cmd22Routine(pCmd+1, CmdRspData); break;
  case 38: Cmd38Routine(pCmd+1, CmdRspData); break;
  case 48: Cmd48Routine(pCmd+1, CmdRspData); break;
  default:
    CmdRspData[0] = 2;
    CmdRspData[1] = 64; // Command not implemented
    break;
  }

  if (CmdRspData[0] > 0)
  { /* Update Device Status */
    CmdRspData[2] = DeviceStatus;
    if (MasterType == PRIMARY_MASTER)
    {
      if (NvmData.CmdCfgChgFlg & PM_SEL_BIT)
      {
        CmdRspData[2] |= CONFIG_CHANGED_BIT;
      }
      else
      {
        CmdRspData[2] &= ~CONFIG_CHANGED_BIT;
      }

      if (ColdStartFlg & PM_SEL_BIT)
      {
        CmdRspData[2] |= COLD_START_BIT;
        ColdStartFlg &= ~PM_SEL_BIT; // Clear Cold_Start_Bits for primary master
      }
      else
      {
        CmdRspData[2] &= ~COLD_START_BIT;
      }
      
      if (MoreStatusFlg & PM_SEL_BIT)
      {
        CmdRspData[2] |= MORE_STATUS_BIT;
      }
      else
      {
        CmdRspData[2] &= ~MORE_STATUS_BIT;
      }
    }
    else if (MasterType == SECONDARY_MASTER)
    {
      if (NvmData.CmdCfgChgFlg & SM_SEL_BIT)
      {
        CmdRspData[2] |= CONFIG_CHANGED_BIT;
      }
      else
      {
        CmdRspData[2] &= ~CONFIG_CHANGED_BIT;
      }
      
      if (ColdStartFlg & SM_SEL_BIT)
      {
        CmdRspData[2] |= COLD_START_BIT;
        ColdStartFlg &= ~SM_SEL_BIT;
      }
      else
      {
        CmdRspData[2] &= ~COLD_START_BIT;
      }
      
      if (MoreStatusFlg & SM_SEL_BIT)
      {
        CmdRspData[2] |= MORE_STATUS_BIT;
      }
      else
      {
        CmdRspData[2] &= ~MORE_STATUS_BIT;
      }
    }
    TxAckFrame(CmdRspData);
    return 1;
  }
  else
  {
    return 0;
  }
}