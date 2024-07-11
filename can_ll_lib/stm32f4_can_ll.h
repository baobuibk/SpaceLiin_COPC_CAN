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

/* Controller Area Network FilterRegister*/
typedef struct
{
  volatile uint32_t FR1; /*!< CAN Filter bank register 1 */
  volatile uint32_t FR2; /*!< CAN Filter bank register 1 */
} CAN_FilterRegister_TypeDef_t;

/*CAN control registers*/
typedef struct
{
  volatile uint32_t CAN_MCR;                            // CAN master control register              Address offset: 0x00
  volatile uint32_t CAN_MSR;                            // CAN master status register               Address offset: 0x04
  volatile uint32_t CAN_TSR;                            // CAN transmit status register             Address offset: 0x08
  volatile uint32_t CAN_RF0R;                           // CAN receive FIFO 0 register              Address offset: 0x0C
  volatile uint32_t CAN_RF1R;                           // CAN receive FIFO 1 register              Address offset: 0x10
  volatile uint32_t CAN_IER;                            // CAN interrupt enable register (CAN_IER)  Address offset: 0x14
  volatile uint32_t CAN_ESR;                            // CAN error status register                Address offset: 0x18
  volatile uint32_t CAN_BTR;                            // CAN bit timing register                  Address offset: 0x1C
  uint32_t CAN_RESERVED0[88];                           // Reserved, 0x020 - 0x17F
  LL_CAN_TxMailBox_TypeDef_t sTxMailBox[3];             // CAN Tx MailBox,                          Address offset: 0x180 - 0x1AC
  CAN_FIFOMailBox_TypeDef sFIFOMailBox[2];              // CAN FIFO MailBox,                        Address offset: 0x1B0 - 0x1CC
  uint32_t CAN_RESERVED1[12];                           // Reserved, 0x1D0 - 0x1FF
  volatile uint32_t CAN_FMR;                            // CAN filter master register,              Address offset: 0x200
  volatile uint32_t CAN_FM1R;                           // CAN filter mode register,                Address offset: 0x204
  uint32_t CAN_RESERVED2;                               // Reserved, 0x208
  volatile uint32_t CAN_FS1R;                           // CAN filter scale register,               Address offset: 0x20C
  uint32_t CAN_RESERVED3;                               // Reserved, 0x210
  volatile uint32_t CAN_FFA1R;                          // CAN filter FIFO assignment register,     Address offset: 0x214
  uint32_t CAN_RESERVED4;                               // Reserved, 0x218
  volatile uint32_t CAN_FA1R;                           // CAN filter activation register,          Address offset: 0x21C
  uint32_t CAN_RESERVED5[8];                            // Reserved, 0x220-0x23F
  CAN_FilterRegister_TypeDef_t CAN_sFilterRegister[28]; // CAN Filter Register,                     Address offset: 0x240-0x31C
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

#define DISABLE 0
#define ENABLE 1

/*CAN function state structure definition*/
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

/*CAN MODE structure definition*/
typedef enum
{
  _NORMAL_MODE,
  _LOOPBACK_MODE,
  _SILENT_MODE,
  _SILENT_LOOPBACK_MODE

} LL_CAN_MODE_t;

/*CAN init structure definition*/
typedef struct
{
  uint32_t Prescaler; /*!< Specifies the length of a time quantum.
                         This parameter must be a number between Min_Data = 1 and Max_Data = 1024. */

  uint32_t SyncJumpWidth; /*!< Specifies the maximum number of time quanta the CAN hardware
                             is allowed to lengthen or shorten a bit to perform resynchronization.
                             This parameter can be a value of @ref _CAN_synchronisation_jump_width */

  uint32_t TimeSeg1; /*!< Specifies the number of time quanta in Bit Segment 1.
                          This parameter can be a value of @ref _CAN_time_quantum_in_bit_segment_1 */

  uint32_t TimeSeg2; /*!< Specifies the number of time quanta in Bit Segment 2.
                          This parameter can be a value of @ref _CAN_time_quantum_in_bit_segment_2 */

  LL_CAN_MODE_t Mode; /* Specifies can mode: normal, loopback, silent or silent loopback*/

  FunctionalState_t status; /* status of function for can */

} LL_CAN_InitTypeDef_t;

/*CAN init structure definition*/
typedef enum
{
  _CAN1,
  _CAN2
} LL_CAN_TypeDef_t;

/*CAN filter configuration structure definition*/
typedef struct
{

  uint32_t FilterIdHigh; /*!< Specifies the filter identification number (MSBs for a 32-bit
                          configuration, first one for a 16-bit configuration).
                          This parameter must be a number between
                          Min_Data = 0x0000 and Max_Data = 0xFFFF. */

  uint32_t FilterIdLow; /*!< Specifies the filter identification number (LSBs for a 32-bit
                             configuration, second one for a 16-bit configuration).
                             This parameter must be a number between
                             Min_Data = 0x0000 and Max_Data = 0xFFFF. */

  uint32_t FilterMaskIdHigh; /*!< Specifies the filter mask number or identification number,
                                  according to the mode (MSBs for a 32-bit configuration,
                                  first one for a 16-bit configuration).
                                  This parameter must be a number between
                                  Min_Data = 0x0000 and Max_Data = 0xFFFF. */

  uint32_t FilterMaskIdLow; /*!< Specifies the filter mask number or identification number,
                                 according to the mode (LSBs for a 32-bit configuration,
                                 second one for a 16-bit configuration).
                                 This parameter must be a number between
                                 Min_Data = 0x0000 and Max_Data = 0xFFFF. */

  uint32_t FilterFIFOAssignment; /*!< Specifies the FIFO (0 or 1U) which will be assigned to the filter.
                                      This parameter can be a value of @ref _CAN_filter_FIFO */

  uint32_t FilterBank; /*!< Specifies the filter bank which will be initialized.
                            For single CAN instance(14 dedicated filter banks),
                            this parameter must be a number between Min_Data = 0 and Max_Data = 13.
                            For dual CAN instances(28 filter banks shared),
                            this parameter must be a number between Min_Data = 0 and Max_Data = 27. */

  uint32_t FilterMode; /*!< Specifies the filter mode to be initialized.
                            This parameter can be a value of @ref _CAN_filter_mode */

  uint32_t FilterScale; /*!< Specifies the filter scale.
                             This parameter can be a value of @ref _CAN_filter_scale */

  uint32_t FilterActivation; /*!< Enable or disable the filter.
                                  This parameter can be a value of @ref _CAN_filter_activation */

  uint32_t SlaveStartFilterBank; /*!< Select the start filter bank for the slave CAN instance.
                                      For single CAN instances, this parameter is meaningless.
                                      For dual CAN instances, all filter banks with lower index are assigned to master
                                      CAN instance, whereas all filter banks with greater index are assigned to slave
                                      CAN instance.
                                      This parameter must be a number between Min_Data = 0 and Max_Data = 27. */

} LL_CAN_FilterTypeDef_t;

/*CAN Tx message header structure definition*/
typedef struct
{
  uint32_t StdId; /*!< Specifies the standard identifier.
                       This parameter must be a number between Min_Data = 0 and Max_Data = 0x7FF. */

  uint32_t ExtId; /*!< Specifies the extended identifier.
                       This parameter must be a number between Min_Data = 0 and Max_Data = 0x1FFFFFFF. */

  uint32_t _IDE; /*!< Specifies the type of identifier for the message that will be transmitted.
                     This parameter can be a value of @ref _CAN_identifier_type */

  uint32_t _RTR; /*!< Specifies the type of frame for the message that will be transmitted.
                     This parameter can be a value of @ref _CAN_remote_transmission_request */

  uint32_t _DLC; /*!< Specifies the length of the frame that will be transmitted.
                     This parameter must be a number between Min_Data = 0 and Max_Data = 8. */

  FunctionalState TransmitGlobalTime; /*!< Specifies whether the timestamp counter value captured on start
                          of frame transmission, is sent in DATA6 and DATA7 replacing pData[6] and pData[7].
                          @note: Time Triggered Communication Mode must be enabled.
                          @note: DLC must be programmed as 8 bytes, in order these 2 bytes are sent.
                          This parameter can be set to ENABLE or DISABLE. */

} LL_CAN_TxHeaderTypeDef_t;

/*CAN Rx message header structure definition*/
typedef struct
{
  uint32_t StdId; /*!< Specifies the standard identifier.
                       This parameter must be a number between Min_Data = 0 and Max_Data = 0x7FF. */

  uint32_t ExtId; /*!< Specifies the extended identifier.
                       This parameter must be a number between Min_Data = 0 and Max_Data = 0x1FFFFFFF. */

  uint32_t _IDE; /*!< Specifies the type of identifier for the message that will be transmitted.
                     This parameter can be a value of @ref _CAN_identifier_type */

  uint32_t _RTR; /*!< Specifies the type of frame for the message that will be transmitted.
                     This parameter can be a value of @ref _CAN_remote_transmission_request */

  uint32_t _DLC; /*!< Specifies the length of the frame that will be transmitted.
                     This parameter must be a number between Min_Data = 0 and Max_Data = 8. */

  uint32_t Timestamp; /*!< Specifies the timestamp counter value captured on start of frame reception.
                          @note: Time Triggered Communication Mode must be enabled.
                          This parameter must be a number between Min_Data = 0 and Max_Data = 0xFFFF. */

  uint32_t FilterMatchIndex; /*!< Specifies the index of matching acceptance filter element.
                          This parameter must be a number between Min_Data = 0 and Max_Data = 0xFF. */

} LL_CAN_RxHeaderTypeDef_t;

/**---------------------------------Define bit position of can register-------------------------------------------------**/
//

// Bit position of CAN_BTR register
#define SILM (31U)
#define LBKM (30U)
#define SJW (24U)
#define TS2 (20U)
#define TS1 (16U)
#define BRP (0U)

// Bit position of CAN_MCR register
#define DBF (16U)
#define RESET (15U)
#define TTCM (7U)
#define ABOM (6U)
#define AWUM (5U)
#define NART (4U)
#define RFLM (3U)
#define TXFP (2U)
#define SLEEP (1U)
#define INRQ (0U)

// Bit position of CAN_MSR register
#define INAK (0U)
#define SLAK (1U)
#define ERRI (2U)
#define WKUI (3U)
#define SLAKI (4U)
#define TXM (8U)
#define RXM (9U)
#define SAMP (10)
#define RX (11U)

// Bit position of CAN_FMR register
#define CAN2SB (8U)
#define FINIT (0U)

// Bit position of CAN_TSR register
#define LOW2 (31U)
#define LOW1 (30U)
#define LOW0 (29U)
#define TME2 (28U)
#define TME1 (9U)
#define TME0 (26U)
#define CODE (24U)
#define ABRQ2 (23U)
#define TERR2 (19U)
#define ALST2 (18U)
#define TXOK2 (17U)
#define RQCP2 (16U)
#define ABRQ1 (15U)
#define TERR1 (11U)
#define ALST1 (10U)
#define TXOK1 (9U)
#define RQCP1 (8U)
#define ABRQ0 (7U)
#define TERR0 (3U)
#define ALST0 (2U)
#define TXOK0 (1U)
#define RQCP0 (0U)

// Bit position of CAN_TIR register
#define STID (21U)
#define EXID (3U)
#define IDE (2U)
#define RTR (1U)
#define TXRQ (0U)

// Bit position of CAN_TDTxR register
#define TIME (16U)
#define TGT (8U)
#define DLC (0U)

// Bit position of CAN_IER register
#define TMEIE (0U)
#define FMPIE0 (1U)
#define FFIE0 (2U)
#define FOVIE0 (3U)
#define FMPIE1 (4U)
#define FFIE1 (5U)
#define FOVIE1 (6U)
#define EWGIE (8U)
#define EPVIE (9U)
#define BOFIE (10U)
#define LECIE (11U)
#define ERRIE (15U)
#define WKUIE (16U)
#define SLKIE (17U)

/**---------------------------------@defgroup-------------------------------------------------**/
/*@defgroup _CAN_operating_mode */
#define _CAN_MODE_NORMAL ((0 << SILM) | (0 << LBKM))          /*!< Normal mode   */
#define _CAN_MODE_LOOPBACK ((0 << SILM) | (1 << LBKM))        /*!< Loopback mode */
#define _CAN_MODE_SILENT ((1 << SILM) | (0 << LBKM))          /*!< Silent mode   */
#define _CAN_MODE_SILENT_LOOPBACK ((1 << SILM) | (1 << LBKM)) /*!< Loopback combined with silent mode   */

/*@defgroup _CAN_identifier_type */
#define _CAN_ID_STD (0UL) /*!< Standard Id */
#define _CAN_ID_EXT (1UL) /*!< Extended Id */

/*@defgroup _CAN_remote_transmission_request*/
#define _CAN_RTR_DATA (0UL)   /*!< Data frame   */
#define _CAN_RTR_REMOTE (1UL) /*!< Remote frame */

/*@defgroup _CAN_filter_FIFO*/
#define _CAN_FILTER_FIFO0 (0U) /*!< Filter FIFO 0 assignment for filter x */
#define _CAN_FILTER_FIFO1 (1U) /*!< Filter FIFO 1 assignment for filter x */

/*@defgroup _CAN_filter_mode*/
#define _CAN_FILTERMODE_IDMASK (0U) /*!< Identifier mask mode */
#define _CAN_FILTERMODE_IDLIST (1U) /*!< Identifier list mode */

/*@defgroup _CAN_filter_scale */
#define _CAN_FILTERSCALE_16BIT (0U) /*!< Two 16-bit filters */
#define _CAN_FILTERSCALE_32BIT (1U) /*!< One 32-bit filter  */

/** @defgroup _CAN_filter_activation CAN Filter Activation*/
#define _CAN_FILTER_DISABLE (0U) /*!< Disable filter */
#define _CAN_FILTER_ENABLE (1U)  /*!< Enable filter  */

/** @defgroup _CAN_synchronisation_jump_width*/
#define _CAN_SJW_1TQ (0U) /*!< 1 time quantum */
#define _CAN_SJW_2TQ (1U) /*!< 2 time quantum */
#define _CAN_SJW_3TQ (2U) /*!< 3 time quantum */
#define _CAN_SJW_4TQ (3U) /*!< 4 time quantum */

/** @defgroup _CAN_time_quantum_in_bit_segment_1*/
#define _CAN_BS1_1TQ (0U)   /*!< 1 time quantum  */
#define _CAN_BS1_2TQ (1U)   /*!< 2 time quantum  */
#define _CAN_BS1_3TQ (2U)   /*!< 3 time quantum  */
#define _CAN_BS1_4TQ (3U)   /*!< 4 time quantum  */
#define _CAN_BS1_5TQ (4U)   /*!< 5 time quantum  */
#define _CAN_BS1_6TQ (5U)   /*!< 6 time quantum  */
#define _CAN_BS1_7TQ (6U)   /*!< 7 time quantum  */
#define _CAN_BS1_8TQ (7U)   /*!< 8 time quantum  */
#define _CAN_BS1_9TQ (8U)   /*!< 9 time quantum  */
#define _CAN_BS1_10TQ (9U)  /*!< 10 time quantum */
#define _CAN_BS1_11TQ (10U) /*!< 11 time quantum */
#define _CAN_BS1_12TQ (11U) /*!< 12 time quantum */
#define _CAN_BS1_13TQ (12U) /*!< 13 time quantum */
#define _CAN_BS1_14TQ (13U) /*!< 14 time quantum */
#define _CAN_BS1_15TQ (14U) /*!< 15 time quantum */
#define _CAN_BS1_16TQ (15U) /*!< 16 time quantum */

/** @defgroup _CAN_time_quantum_in_bit_segment_2*/
#define _CAN_BS2_1TQ (0U) /*!< 1 time quantum */
#define _CAN_BS2_2TQ (1U) /*!< 2 time quantum */
#define _CAN_BS2_3TQ (2U) /*!< 3 time quantum */
#define _CAN_BS2_4TQ (3U) /*!< 4 time quantum */
#define _CAN_BS2_5TQ (4U) /*!< 5 time quantum */
#define _CAN_BS2_6TQ (5U) /*!< 6 time quantum */
#define _CAN_BS2_7TQ (6U) /*!< 7 time quantum */
#define _CAN_BS2_8TQ (7U) /*!< 8 time quantum */
/**---------------------------------Prototype functions-------------------------------------------------**/

ErrorStatus LL_CAN_GPIO_Init(uint8_t can_type);
ErrorStatus LL_CAN_Init(LL_CAN_TypeDef_t can_type, LL_CAN_InitTypeDef_t *hcan);
ErrorStatus LL_CAN_ConfigFilter(LL_CAN_TypeDef_t can_type, LL_CAN_FilterTypeDef_t *hfilter);
ErrorStatus LL_CAN_Transmit(LL_CAN_TypeDef_t can_type, const uint8_t data[], LL_CAN_TxHeaderTypeDef_t *htxheader);
ErrorStatus LL_CAN_Start(LL_CAN_TypeDef_t can_type);

#endif /* STM32F4_CAN_LL_H_ */
