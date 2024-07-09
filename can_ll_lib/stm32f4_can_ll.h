/*
 * stm32f4_can_LL.h
 *
 *  Created on: Jul 3, 2024
 *      Author: dell
 */

#ifndef STM32F4_CAN_LL_H_
#define STM32F4_CAN_LL_H_
/*Header LIB*/

#include <stdint.h>
#include <stdio.h>
#include "stm32f4xx.h"
/*---------------------------------*/

/*Controller Area Network TxMailBox*/
typedef struct
{
  volatile uint32_t TIR;  /*!< CAN TX mailbox identifier register */
  volatile uint32_t TDTR; /*!< CAN mailbox data length control and time stamp register */
  volatile uint32_t TDLR; /*!< CAN mailbox data low register */
  volatile uint32_t TDHR; /*!< CAN mailbox data high register */
} LL_CAN_TxMailBox_TypeDef_t;

/*CAN1 control and status registers*/
typedef struct
{
  volatile uint32_t CAN_MCR;                // CAN master control register
  volatile uint32_t CAN_MSR;                // CAN master status register
  volatile uint32_t CAN_TSR;                // CAN transmit status register
  volatile uint32_t CAN_RF0R;               // CAN receive FIFO 0 register
  volatile uint32_t CAN_RF1R;               // CAN receive FIFO 1 register
  volatile uint32_t CAN_IER;                // CAN interrupt enable register (CAN_IER)
  volatile uint32_t CAN_ESR;                // CAN error status register
  volatile uint32_t CAN_BTR;                // CAN bit timing register
  uint32_t CAN_RESERVED0[88];               // Reserved, 0x020 - 0x17F
  LL_CAN_TxMailBox_TypeDef_t sTxMailBox[3]; //< CAN Tx MailBox, Address offset: 0x180 - 0x1AC */
} CAN_TypeDef_t;

// Data mask position
#define CAN_TDLR_DATA0_Pos 0
#define CAN_TDLR_DATA1_Pos 8
#define CAN_TDLR_DATA2_Pos 16
#define CAN_TDLR_DATA3_Pos 24
#define CAN_TDHR_DATA4_Pos 0
#define CAN_TDHR_DATA5_Pos 8
#define CAN_TDHR_DATA6_Pos 16
#define CAN_TDHR_DATA7_Pos 24

/* CAN base address*/

#define _CAN1_REG_BASE ((CAN_TypeDef_t *)0x40006400UL)
#define _CAN2_REG_BASE ((CAN_TypeDef_t *)0x40006800UL)

/*CAN1 mailbox offset*/

#define CAN_TI0R (0x180) // CAN TX0 mailbox identifier offset
#define CAN_TI1R (0x190) // CAN TX1 mailbox identifier offset
#define CAN_TI2R (0x1A0) // CAN TX2 mailbox identifier offset

#define CAN_TDT0R (0x184) // CAN mailbox data length control and time stamp offset 0
#define CAN_TDT1R (0x194) // CAN mailbox data length control and time stamp offset 1
#define CAN_TDT2R (0x1A4) // CAN mailbox data length control and time stamp offset 2

#define CAN_TDL0R (0x188) // CAN mailbox data low offset 0
#define CAN_TDL1R (0x198) // CAN mailbox data low offset 1
#define CAN_TDL2R (0x1A8) // CAN mailbox data low offset 2

#define CAN_TDH0R (0x18C) // CAN mailbox data high offset 0
#define CAN_TDH1R (0x19C) // CAN mailbox data high offset 1
#define CAN_TDH2R (0x1AC) // CAN mailbox data high offset 2

#define CAN_RI0R (0x1B0) // CAN receive FIFO mailbox identifier offset 0
#define CAN_RI1R (0x1C0) // CAN receive FIFO mailbox identifier offset 1

#define CAN_RDT0R (0x1B4) // CAN receive FIFO mailbox data length control and time stamp offset 0
#define CAN_RDT1R (0x1C4) // CAN receive FIFO mailbox data length control and time stamp offset 1

#define CAN_RDL0R (0x1B8) // CAN receive FIFO mailbox data low offset 0
#define CAN_RDL1R (0x1C8) // CAN receive FIFO mailbox data low offset 1

#define CAN_RDH0R (0x1BC) // CAN receive FIFO mailbox data high offset 0
#define CAN_RDH1R (0x1CC) // CAN receive FIFO mailbox data high offset 1

/*CAN1 filter offsets*/

#define CAN_FMR (0x200)   // CAN filter master offset
#define CAN_FM1R (0x204)  // CAN filter mode offset
#define CAN_FS1R (0x20C)  // CAN filter scale offset
#define CAN_FFA1R (0x214) // CAN filter FIFO assignment offset
#define CAN_FA1R (0x21C)  // CAN filter activation offset

// Filter banks
#define CAN_F0R1 (0x240)
#define CAN_F0R2 (0x244)
#define CAN_F1R1 (0x248)
#define CAN_F1R2 (0x24C)
#define CAN_F2R1 (0x250)
#define CAN_F2R2 (0x254)
#define CAN_F3R1 (0x258)
#define CAN_F3R2 (0x25C)
#define CAN_F4R1 (0x260)
#define CAN_F4R2 (0x264)
#define CAN_F5R1 (0x268)
#define CAN_F5R2 (0x26C)
#define CAN_F6R1 (0x270)
#define CAN_F6R2 (0x274)
#define CAN_F7R1 (0x278)
#define CAN_F7R2 (0x27C)
#define CAN_F8R1 (0x280)
#define CAN_F8R2 (0x284)
#define CAN_F9R1 (0x288)
#define CAN_F9R2 (0x28C)
#define CAN_F10R1 (0x290)
#define CAN_F10R2 (0x294)
#define CAN_F11R1 (0x298)
#define CAN_F11R2 (0x29C)
#define CAN_F12R1 (0x2A0)
#define CAN_F12R2 (0x2A4)
#define CAN_F13R1 (0x2A8)
#define CAN_F13R2 (0x2AC)
#define CAN_F14R1 (0x2B0)
#define CAN_F14R2 (0x2B4)
#define CAN_F15R1 (0x2B8)
#define CAN_F15R2 (0x2BC)
#define CAN_F16R1 (0x2C0)
#define CAN_F16R2 (0x2C4)
#define CAN_F17R1 (0x2C8)
#define CAN_F17R2 (0x2CC)
#define CAN_F18R1 (0x2D0)
#define CAN_F18R2 (0x2D4)
#define CAN_F19R1 (0x2D8)
#define CAN_F19R2 (0x2DC)
#define CAN_F20R1 (0x2E0)
#define CAN_F20R2 (0x2E4)
#define CAN_F21R1 (0x2E8)
#define CAN_F21R2 (0x2EC)
#define CAN_F22R1 (0x2F0)
#define CAN_F22R2 (0x2F4)
#define CAN_F23R1 (0x2F8)
#define CAN_F23R2 (0x2FC)
#define CAN_F24R1 (0x300)
#define CAN_F24R2 (0x304)
#define CAN_F25R1 (0x308)
#define CAN_F25R2 (0x30C)
#define CAN_F26R1 (0x310)
#define CAN_F26R2 (0x314)
#define CAN_F27R1 (0x318)
#define CAN_F27R2 (0x31C)

#define DISABLE 0
#define ENABLE 1

typedef struct
{
  uint8_t TimeTriggeredMode : 1; /*!< Enable or disable the time triggered communication mode.
                                         This parameter can be set to ENABLE or DISABLE. */

  uint8_t AutoBusOff : 1; /*!< Enable or disable the automatic bus-off management.
                                  This parameter can be set to ENABLE or DISABLE. */

  uint8_t AutoWakeUp : 1; /*!< Enable or disable the automatic wake-up mode.
                                  This parameter can be set to ENABLE or DISABLE. */

  uint8_t AutoRetransmission : 1; /*!< Enable or disable the non-automatic retransmission mode.
                                          This parameter can be set to ENABLE or DISABLE. */

  uint8_t ReceiveFifoLocked : 1; /*!< Enable or disable the Receive FIFO Locked mode.
                                         This parameter can be set to ENABLE or DISABLE. */

  uint8_t TransmitFifoPriority : 1; /*!< Enable or disable the transmit FIFO priority.
                                            This parameter can be set to ENABLE or DISABLE. */
} __attribute__((packed)) FunctionalState_t;

typedef enum
{
  _BAUDRATE_500,
  _BAUDRATE_800,
  _BAUDRATE_1000,

} LL_BaudrateTypeDef;

typedef enum
{
  _NORMAL_MODE,
  _LOOPBACK_MODE,
  _SILENT_MODE,
  _SILENT_LOOPBACK_MODE

} LL_CAN_MODE_t;

/**
 * @brief  LL CAN init structure definition
 */
typedef struct
{
  LL_BaudrateTypeDef Baudrate; /* Specifies baudrate: 500, 800 or 1000*/
  LL_CAN_MODE_t Mode;          /* Specifies can mode: normal, loopback, silent or silent loopback*/
  FunctionalState_t status;    /* status of function for can */

} LL_CAN_InitTypeDef;

/**
 * @brief  LL CAN init structure definition
 */
typedef enum
{
  _CAN1,
  _CAN2
} LL_CAN_TypeDef;

/**
 * @brief  LL CAN filter configuration structure definition
 */
typedef struct
{

  uint32_t FilterID; /*!< Specifies the filter identification number.
                                This parameter must be a number between
                                Min_Data = 0x0000 and Max_Data = 0xFFFF	*/

  uint32_t FilterMask; /*!< Specifies the filter identification number.
                                This parameter must be a number between
                                Min_Data = 0x0000 and Max_Data = 0xFFFF	*/

  uint32_t FilterFIFOAssignment; /*!< Specifies the FIFO (0 or 1U) which will be assigned to the filter.
                                      This parameter can be a value of @ref CAN_filter_FIFO */

  uint32_t FilterBank; /*!< Specifies the filter bank which will be initialized.
                            For single CAN instance(14 dedicated filter banks),
                            this parameter must be a number between Min_Data = 0 and Max_Data = 13.
                            For dual CAN instances(28 filter banks shared),
                            this parameter must be a number between Min_Data = 0 and Max_Data = 27. */

  uint32_t FilterMode; /*!< Specifies the filter mode to be initialized.
                            This parameter can be a value of @ref CAN_filter_mode */

  uint32_t FilterScale; /*!< Specifies the filter scale.
                             This parameter can be a value of @ref CAN_filter_scale */

  uint32_t FilterActivation; /*!< Enable or disable the filter.
                                  This parameter can be a value of @ref CAN_filter_activation */

  uint32_t SlaveStartFilterBank; /*!< Select the start filter bank for the slave CAN instance.
                                      For single CAN instances, this parameter is meaningless.
                                      For dual CAN instances, all filter banks with lower index are assigned to master
                                      CAN instance, whereas all filter banks with greater index are assigned to slave
                                      CAN instance.
                                      This parameter must be a number between Min_Data = 0 and Max_Data = 27. */

} LL_CAN_FilterTypeDef;

/**
 * @brief  LL CAN Tx message header structure definition
 */
typedef struct
{
  uint32_t StdId; /*!< Specifies the standard identifier.
                       This parameter must be a number between Min_Data = 0 and Max_Data = 0x7FF. */

  uint32_t ExtId; /*!< Specifies the extended identifier.
                       This parameter must be a number between Min_Data = 0 and Max_Data = 0x1FFFFFFF. */

  uint32_t _IDE; /*!< Specifies the type of identifier for the message that will be transmitted.
                     This parameter can be a value of @ref CAN_identifier_type */

  uint32_t _RTR; /*!< Specifies the type of frame for the message that will be transmitted.
                     This parameter can be a value of @ref CAN_remote_transmission_request */

  uint32_t _DLC; /*!< Specifies the length of the frame that will be transmitted.
                     This parameter must be a number between Min_Data = 0 and Max_Data = 8. */

  FunctionalState TransmitGlobalTime; /*!< Specifies whether the timestamp counter value captured on start
                          of frame transmission, is sent in DATA6 and DATA7 replacing pData[6] and pData[7].
                          @note: Time Triggered Communication Mode must be enabled.
                          @note: DLC must be programmed as 8 bytes, in order these 2 bytes are sent.
                          This parameter can be set to ENABLE or DISABLE. */

} LL_CAN_TxHeaderTypeDef;

/**
 * @brief  LL CAN Rx message header structure definition
 */
typedef struct
{
  uint32_t StdId; /*!< Specifies the standard identifier.
                       This parameter must be a number between Min_Data = 0 and Max_Data = 0x7FF. */

  uint32_t ExtId; /*!< Specifies the extended identifier.
                       This parameter must be a number between Min_Data = 0 and Max_Data = 0x1FFFFFFF. */

  uint32_t _IDE; /*!< Specifies the type of identifier for the message that will be transmitted.
                     This parameter can be a value of @ref CAN_identifier_type */

  uint32_t _RTR; /*!< Specifies the type of frame for the message that will be transmitted.
                     This parameter can be a value of @ref CAN_remote_transmission_request */

  uint32_t _DLC; /*!< Specifies the length of the frame that will be transmitted.
                     This parameter must be a number between Min_Data = 0 and Max_Data = 8. */

  uint32_t Timestamp; /*!< Specifies the timestamp counter value captured on start of frame reception.
                          @note: Time Triggered Communication Mode must be enabled.
                          This parameter must be a number between Min_Data = 0 and Max_Data = 0xFFFF. */

  uint32_t FilterMatchIndex; /*!< Specifies the index of matching acceptance filter element.
                          This parameter must be a number between Min_Data = 0 and Max_Data = 0xFF. */

} LL_CAN_RxHeaderTypeDef;

/* Define register bit
 * ------------------------------------*/

// CAN_BTR register
#define SILM 31
#define LBKM 30
#define SJW 24
#define TS2 20
#define TS1 16
#define BRP 0

// CAN_MCR register
#define DBF 16
#define RESET 15
#define TTCM 7
#define ABOM 6
#define AWUM 5
#define NART 4
#define RFLM 3
#define TXFP 2
#define SLEEP 1
#define INRQ 0

// CAN_MSR register
#define INAK 0
#define SLAK 1
#define ERRI 2
#define WKUI 3
#define SLAKI 4
#define TXM 8
#define RXM 9
#define SAMP 10
#define RX 11
// CAN_FMR register
#define CAN2SB 8
#define FINIT 0

// CAN_TSR register
#define LOW2 31
#define LOW1 30
#define LOW0 29
#define TME2 28
#define TME1 27
#define TME0 26
#define CODE 24
#define ABRQ2 23
#define TERR2 19
#define ALST2 18
#define TXOK2 17
#define RQCP2 16
#define ABRQ1 15
#define TERR1 11
#define ALST1 10
#define TXOK1 9
#define RQCP1 8
#define ABRQ0 7
#define TERR0 3
#define ALST0 2
#define TXOK0 1
#define RQCP0 0

// CAN_TIxR register
#define STID 21
#define EXID 3
#define IDE 2
#define RTR 1
#define TXRQ 0

// CAN_TDTxR register
#define TIME 16
#define TGT 8
#define DLC 0

/** @defgroup CAN_operating_mode CAN Operating Mode*/

#define _CAN_MODE_NORMAL ((0 << SILM) | (0 << LBKM))          /*!< Normal mode   */
#define _CAN_MODE_LOOPBACK ((0 << SILM) | (1 << LBKM))        /*!< Loopback mode */
#define _CAN_MODE_SILENT ((1 << SILM) | (0 << LBKM))          /*!< Silent mode   */
#define _CAN_MODE_SILENT_LOOPBACK ((1 << SILM) | (1 << LBKM)) /*!< Loopback combined with silent mode   */

/** @defgroup CAN_identifier_type CAN Identifier Type
 * @{
 */
#define _CAN_ID_STD (0UL) /*!< Standard Id */
#define _CAN_ID_EXT (1UL) /*!< Extended Id */
/**
 * @}
 */

/** @defgroup CAN_remote_transmission_request CAN Remote Transmission Request
 * @{
 */
#define _CAN_RTR_DATA (0UL)   /*!< Data frame   */
#define _CAN_RTR_REMOTE (1UL) /*!< Remote frame */

/*Prototype functions*/
ErrorStatus LL_CAN_GPIO_Init(uint8_t can_type);
ErrorStatus LL_CAN_Init(LL_CAN_TypeDef can_type, LL_CAN_InitTypeDef *hcan);
ErrorStatus LL_CAN_ConfigFilter(LL_CAN_TypeDef can_type, LL_CAN_FilterTypeDef *hfilter);
ErrorStatus LL_CAN_Transmit(LL_CAN_TypeDef can_type,  const uint8_t data[], LL_CAN_TxHeaderTypeDef *htxheader);
ErrorStatus LL_CAN_Start(LL_CAN_TypeDef can_type);

#endif /* STM32F4_CAN_LL_H_ */
