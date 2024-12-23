#ifndef _HART_AD5700
#define _HART_AD5700

extern uint8_t DevTypeCode[2];
extern uint8_t DevUniqueID[3];
extern uint8_t ManuID[2];

extern uint8_t DeviceStatus; // Device Status
#define MAL_FUNCTION_BIT          0x80
#define CONFIG_CHANGED_BIT        0x40
#define COLD_START_BIT            0x20
#define MORE_STATUS_BIT           0x10
#define CURRENT_FIXED_BIT         0x08
#define CURRENT_SATUR_BIT         0x04
#define NONPV_OUT_OF_LIMIT_BIT    0x02
#define PV_OUT_OF_LIMIT_BIT       0x01

#define PM_SEL_BIT           0x01     
#define SM_SEL_BIT           0x02     
extern uint8_t ColdStartFlg; // Cold Start Flag
extern uint8_t MoreStatusFlg; // More Status Flag
extern uint8_t DevSpecStatus[17]; // Command 48

#define MIN_PRE_STM_NUM      5 // Minimum preamble numbers from slave to master

typedef  struct
{
  uint8_t     CmdCfgChgFlg;                      /* Command execute config change flag */
  uint16_t    CfgChgCnt;                         /* Configuration Change Counter */
  uint8_t     PollingAddr;                       /* Polling Address           */
  uint8_t     LoopCurrentMode;                   /* Loop Current Mode         */
  uint8_t     Message[24];                       /* Message (32 characters)   */
  uint8_t     LongTag[32];                       /* Long tag (Latin-1)        */
  uint8_t     Tag[6];                            /* Tag (8 characters)        */
  uint8_t     Descriptor[12];                    /* Descriptor (16 characters) */
  uint8_t     Date[3];                           /* Date                      */
  uint8_t     FinalAsmbNum[3];                   /* Final assembly number     */
} NVM_DATA_TypeDef;
extern NVM_DATA_TypeDef NvmData;

// Prototypes
extern void HART_AD5700_Init(void);
extern uint8_t GetMasterType(void);
// HART Master Type
#define MASTER_TYPE_MASK     (0x01 << 7)
#define PRIMARY_MASTER       0x80u
#define SECONDARY_MASTER     0x00u

// Commands table
// Parameters:
//  *pIn: Request data, including byte_count and data
//  *pOut: Response data, including byte_count and data
extern void Cmd0Routine(uint8_t *pIn, uint8_t *pOut);
extern void Cmd1Routine(uint8_t *pIn, uint8_t *pOut);
extern void Cmd2Routine(uint8_t *pIn, uint8_t *pOut);
extern void Cmd3Routine(uint8_t *pIn, uint8_t *pOut);
extern void Cmd6Routine(uint8_t *pIn, uint8_t *pOut);
extern void Cmd7Routine(uint8_t *pIn, uint8_t *pOut);
extern void Cmd8Routine(uint8_t *pIn, uint8_t *pOut);
extern void Cmd9Routine(uint8_t *pIn, uint8_t *pOut);
extern void Cmd11Routine(uint8_t *pIn, uint8_t *pOut);
extern void Cmd12Routine(uint8_t *pIn, uint8_t *pOut);
extern void Cmd13Routine(uint8_t *pIn, uint8_t *pOut);
extern void Cmd14Routine(uint8_t *pIn, uint8_t *pOut);
extern void Cmd15Routine(uint8_t *pIn, uint8_t *pOut);
extern void Cmd16Routine(uint8_t *pIn, uint8_t *pOut);
extern void Cmd17Routine(uint8_t *pIn, uint8_t *pOut);
extern void Cmd18Routine(uint8_t *pIn, uint8_t *pOut);
extern void Cmd19Routine(uint8_t *pIn, uint8_t *pOut);
extern void Cmd20Routine(uint8_t *pIn, uint8_t *pOut);
extern void Cmd21Routine(uint8_t *pIn, uint8_t *pOut);
extern void Cmd22Routine(uint8_t *pIn, uint8_t *pOut);
extern void Cmd38Routine(uint8_t *pIn, uint8_t *pOut);
extern void Cmd48Routine(uint8_t *pIn, uint8_t *pOut);

#endif /* _HART_AD5700 */
