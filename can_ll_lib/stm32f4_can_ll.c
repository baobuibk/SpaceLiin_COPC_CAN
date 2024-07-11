/*
 * stm32f4_can_LL.c
 *
 *  Created on: Jul 3, 2024
 *      Author: dell
 */

#include "stm32f4_can_ll.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_bus.h"
#include <stdlib.h>
#include "main.h"

// Global variables
CAN_TypeDef_t *canbase;

// Static prototypes
static ErrorStatus Trans(CAN_TypeDef_t *canbase, const uint8_t data[], LL_CAN_TxHeaderTypeDef_t *htxheader);

/**
 * The function `LL_CAN_GPIO_Init` initializes GPIO pins for CAN communication based on the specified
 * CAN type.
 *
 * @param can_type The `can_type` parameter in the `LL_CAN_GPIO_Init` function is used to determine
 * which CAN bus (CAN1 or CAN2) configuration to initialize based on the value passed to it. The
 * function configures the GPIO pins for CAN communication based on the specified CAN type.
 */
ErrorStatus LL_CAN_GPIO_Init(uint8_t can_type)
{

	ErrorStatus status = ERROR;
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* Peripheral clock enable */
	if (can_type == _CAN1)
		RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;
	else if (can_type == _CAN2)
		RCC->APB1ENR |= RCC_APB1ENR_CAN2EN;
	else
		status = ERROR;

	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

	if (can_type == _CAN1)
	{
		/**CAN1 GPIO Configuration
		PB8   ------> CAN1_RX
		PB9   ------> CAN1_TX
		*/
		GPIO_InitStruct.Pin = LL_GPIO_PIN_8 | LL_GPIO_PIN_9;
		GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
		GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
		GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
		GPIO_InitStruct.Alternate = LL_GPIO_AF_9;
		LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		status = SUCCESS;
	}
	else if (can_type == _CAN2)
	{
		/**CAN1 GPIO Configuration
		PB12   ------> CAN2_RX
		PB13   ------> CAN2_TX
		*/
		GPIO_InitStruct.Pin = LL_GPIO_PIN_12 | LL_GPIO_PIN_13;
		GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
		GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
		GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
		GPIO_InitStruct.Alternate = LL_GPIO_AF_9;
		LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		status = SUCCESS;
	}
	else
		status = ERROR;
	return status;
}

/**
 * The function `LL_CAN_Init` initializes a CAN module based on the provided configuration parameters.
 *
 * @param can_type The `can_type` parameter in the `LL_CAN_Init` function is used to specify which CAN
 * instance to initialize. It is of type `LL_CAN_TypeDef`, which likely represents the CAN hardware
 * instance (e.g., CAN1 or CAN2).
 * @param hcan The `hcan` parameter is a pointer to a structure of type `LL_CAN_InitTypeDef` which
 * contains configuration settings for initializing a CAN (Controller Area Network) module. The
 * structure likely includes fields such as `Baudrate`, `Mode`, and `status` which in turn contains
 * flags for
 *
 * @return The function `LL_CAN_Init` is returning an `ErrorStatus` enum value, which is either `ERROR`
 * or `SUCCESS`.
 */
ErrorStatus LL_CAN_Init(LL_CAN_TypeDef_t can_type, LL_CAN_InitTypeDef_t *hcan)
{

	ErrorStatus status = ERROR;
	uint8_t time_out = 50;
	uint8_t time_start = 0;
	assert_param(hcan->Baudrate);
	assert_param(hcan->Mode);
	assert_param(hcan->status.AutoBusOff);
	assert_param(hcan->status.AutoRetransmission);
	assert_param(hcan->status.AutoWakeUp);
	assert_param(hcan->status.ReceiveFifoLocked);
	assert_param(hcan->status.TimeTriggeredMode);
	assert_param(hcan->status.TransmitFifoPriority);

	// Check CAN instance is CAN1 or CAN2
	if (can_type == _CAN1)
	{
		canbase = _CAN1_REG_BASE;
		status = SUCCESS;
	}
	else if (can_type == _CAN2)
	{
		canbase = _CAN2_REG_BASE;
		status = SUCCESS;
	}

	// Enter Initialization mode
	(canbase->CAN_MCR) |= (1U << INRQ);

	// Wait until INAK bit has been set by hardware.
	while (!((canbase->CAN_MSR) & (1U << INAK)) && (time_start < time_out))
	{
		++time_start;
	}
	if (time_start > time_out)
		return ERROR;
	// Reset time start
	time_start = 0;

	// Exit sleep mode
	(canbase->CAN_MCR) &= ~(1U << SLEEP);

	// Wait until bit SLAK=0 of MSR register
	while (!((canbase->CAN_MSR) & (1U << SLAK)) && (time_start < time_out))
	{
		++time_start;
	}
	if (time_start > time_out)
		return ERROR;

	// Configure CAN mode
	switch (hcan->Mode)
	{
	case _NORMAL_MODE:
		(canbase->CAN_BTR) |= _CAN_MODE_NORMAL;
		break;
	case _LOOPBACK_MODE:
		(canbase->CAN_BTR) |= _CAN_MODE_LOOPBACK;
		break;
	case _SILENT_MODE:
		(canbase->CAN_BTR) |= _CAN_MODE_SILENT;
		break;
	case _SILENT_LOOPBACK_MODE:
		(canbase->CAN_BTR) |= _CAN_MODE_SILENT_LOOPBACK;
		break;
	}

	// Clear bits of SJW | BRP | TS1 | TS2
	(canbase->CAN_BTR) &= 0xF0000000;

	// Can baudrate calculation bases on F_APB1=24Mhz, SJW | BRP | TS1 | TS2
	(canbase->CAN_BTR) |= (((hcan->SyncJumpWidth) << SJW) | (((hcan->Prescaler) - 1) << BRP) | ((hcan->TimeSeg1) << TS1) | ((hcan->TimeSeg2) << TS2));

	// Can Autowakeup, default value of bit AWUM=0
	if (hcan->status.AutoWakeUp == ENABLE)
		(canbase->CAN_MCR) |= (1U << AWUM);
	else
		(canbase->CAN_MCR) &= ~(1U << AWUM);

	// Can No Autorestranmission, default value of bit NART=0
	if (hcan->status.AutoRetransmission == DISABLE)
		(canbase->CAN_MCR) |= (1U << NART);
	else
		(canbase->CAN_MCR) &= (1U << NART);

	// Can ReceiveFifoLocked, default value of bit RFLM=0
	if (hcan->status.ReceiveFifoLocked == ENABLE)
		(canbase->CAN_MCR) |= (1U << RFLM);
	else
		(canbase->CAN_MCR) &= ~(1U << RFLM);

	// Can Time trigger mode, default value of bit TTCM=0
	if (hcan->status.TimeTriggeredMode == ENABLE)
		(canbase->CAN_MCR) |= (1U << TTCM);
	else
		(canbase->CAN_MCR) &= ~(1U << TTCM);

	// Can TransmitFifoPriority, default value of bit TXFP=0
	if (hcan->status.TransmitFifoPriority == ENABLE)
		(canbase->CAN_MCR) |= (1U << TXFP);
	else
		(canbase->CAN_MCR) &= ~(1U << TXFP);

	// Can AutoBusOff, default value of bit ABOM=0
	if (hcan->status.AutoBusOff == ENABLE)
		canbase->CAN_MCR |= (1U << ABOM);
	else
		(canbase->CAN_MCR) &= ~(1U << ABOM);

	(canbase->CAN_MCR) |= (1U << DBF);
	return status;
}

/**
 * The function LL_CAN_ConfigFilter configures CAN filter settings based on the provided parameters.
 *
 * @param can_type The `can_type` parameter in the `LL_CAN_ConfigFilter` function is of type
 * `LL_CAN_TypeDef`, which is used to specify the CAN instance (CAN1 or CAN2) for which the filter
 * configuration is intended.
 * @param hfilter The `hfilter` parameter is a pointer to a structure of type `LL_CAN_FilterTypeDef`,
 * which contains the configuration settings for a CAN filter. This structure likely includes members
 * such as `FilterMode`, `FilterBank`, `FilterScale`, `FilterFIFOAssignment`, `FilterActivation`, `
 *
 * @return The function `ErrorStatus LL_CAN_ConfigFilter(LL_CAN_TypeDef can_type, LL_CAN_FilterTypeDef
 * *hfilter)` is returning the status of the operation, which is either `SUCCESS` or `ERROR`.
 */
ErrorStatus LL_CAN_ConfigFilter(LL_CAN_TypeDef_t can_type, LL_CAN_FilterTypeDef_t *hfilter)
{
	ErrorStatus status = ERROR;
	uint32_t filter_bank_pos;
	// Check CAN`s instance is CAN1 or CAN2
	status = SUCCESS;
	if (can_type == _CAN1)
	{
		canbase = _CAN1_REG_BASE;
		status = SUCCESS;
	}
	else if (can_type == _CAN2)
	{
		canbase = _CAN2_REG_BASE;
		status = SUCCESS;
	}

	// Check the parameters
	assert_param(hfilter->FilterActivation);
	assert_param(hfilter->FilterBank);
	assert_param(hfilter->FilterFIFOAssignment);
	assert_param(hfilter->FilterID);
	assert_param(hfilter->FilterMask);
	assert_param(hfilter->FilterMode);
	assert_param(hfilter->FilterScale);
	assert_param(hfilter->SlaveStartFilterBank);

	// Don`t have SlaveStartFilterBank configuration

	// Init filter mode
	(canbase->CAN_FMR) |= (1U << FINIT);

	// If CAN2 is on, we can use Slave start filter bank
	if (can_type == _CAN2)
	{
		(canbase->CAN_FMR) &= ~(1U << CAN2SB);
		(canbase->CAN_FMR) |= ((hfilter->SlaveStartFilterBank) << CAN2SB);
	}

	// Convert filter bank number into bit position
	filter_bank_pos = (1U << ((hfilter->FilterBank) & 0x1FU));

	// Config filter mode
	if (hfilter->FilterMode == _CAN_FILTERMODE_IDMASK)
	{
		(canbase->CAN_FM1R) &= ~(filter_bank_pos);
	}
	else
	{
		(canbase->CAN_FM1R) |= (filter_bank_pos);
	}

	// Config filter scale
	if (hfilter->FilterScale == _CAN_FILTERSCALE_16BIT)
	{
		(canbase->CAN_FS1R) &= ~(filter_bank_pos);

		// First 16 bits is Identifier, second 16 bits is Mask
		(canbase->CAN_sFilterRegister[hfilter->FilterBank].FR1) =
			((0x0000FFFFU & (uint32_t)hfilter->FilterMaskIdLow) << 16U) |
			(0x0000FFFFU & (uint32_t)hfilter->FilterIdLow);

		// First 16 bits is Identifier, second 16 bits is Mask
		(canbase->CAN_sFilterRegister[hfilter->FilterBank].FR2) =
			((0x0000FFFFU & (uint32_t)hfilter->FilterMaskIdHigh) << 16U) |
			(0x0000FFFFU & (uint32_t)hfilter->FilterIdHigh);
	}
	else if (hfilter->FilterScale == _CAN_FILTERSCALE_32BIT)
	{
		(canbase->CAN_FS1R) |= (filter_bank_pos);

		// 32-bit identifier or First 32-bit identifier
		(canbase->CAN_sFilterRegister[hfilter->FilterBank].FR1) =
			((0x0000FFFFU & (uint32_t)hfilter->FilterIdHigh) << 16U) |
			(0x0000FFFFU & (uint32_t)hfilter->FilterIdLow);

		// 32-bit mask or Second 32-bit identifier
		(canbase->CAN_sFilterRegister[hfilter->FilterBank].FR2) =
			((0x0000FFFFU & (uint32_t)hfilter->FilterMaskIdHigh) << 16U) |
			(0x0000FFFFU & (uint32_t)hfilter->FilterMaskIdLow);
	}

	// Config filter FIFO assignment (FIFO0 or FIFO1)
	if (hfilter->FilterFIFOAssignment == _CAN_FILTER_FIFO0)
	{
		(canbase->CAN_FFA1R) &= ~(filter_bank_pos);
	}
	else
	{
		(canbase->CAN_FFA1R) |= (filter_bank_pos);
	}

	// Active or Deactive filter
	if (hfilter->FilterActivation == _CAN_FILTER_ENABLE)
	{
		(canbase->CAN_FA1R) |= (filter_bank_pos);
	}
	else
	{
		(canbase->CAN_FA1R) &= ~(filter_bank_pos);
	}

	return status;
}

/**
 * The function LL_CAN_Start attempts to start a CAN bus and returns the status of the operation.
 *
 * @param can_type The `can_type` parameter is of type `LL_CAN_TypeDef`, which is likely a structure or
 * enumeration representing a Controller Area Network (CAN) type. It is used as an input parameter to
 * the `LL_CAN_Start` function for starting the CAN communication.
 *
 * @return The function LL_CAN_Start is returning an ErrorStatus enum value, which can be either ERROR
 * or SUCCESS.
 */
ErrorStatus LL_CAN_Start(LL_CAN_TypeDef_t can_type)
{
	ErrorStatus status = ERROR;
	uint8_t time_out = 50;
	uint8_t time_start = 0;
	// Leave Initialization mode
	(canbase->CAN_MCR) &= ~(1U << INRQ);
	// Wait until INAK bit has been cleared by hardware.
	while (((canbase->CAN_MSR) & (1U << INAK)) && (time_start < time_out))
	{
		++time_start;
	}
	if (time_start > time_out)
		status = ERROR;
	else
		status = SUCCESS;
	return status;
}

/**
 * The function LL_CAN_Transmit transmits data over a CAN bus using the specified CAN type and
 * transmission header.
 *
 * @param can_type The `can_type` parameter is of type `LL_CAN_TypeDef` and is used to specify which
 * CAN instance to transmit data on. It can be either `_CAN1` or `_CAN2`.
 * @param data The `data` parameter is a pointer to the data that you want to transmit over the CAN
 * bus. It is of type `uint8_t *`, which means it is a pointer to an array of unsigned 8-bit integers
 * (bytes).
 * @param htxheader The `htxheader` parameter is a pointer to a structure of type
 * `LL_CAN_TxHeaderTypeDef`. This structure likely contains information about the CAN message to be
 * transmitted, such as the DLC (Data Length Code), IDE (Identifier Extension), and RTR (Remote
 * Transmission Request) flags.
 *
 * @return The function `ErrorStatus LL_CAN_Transmit(LL_CAN_TypeDef can_type, uint8_t *data,
 * LL_CAN_TxHeaderTypeDef *htxheader)` returns an `ErrorStatus` value, which is either `ERROR` or a
 * different value based on the success of the transmission operation.
 */
ErrorStatus LL_CAN_Transmit(LL_CAN_TypeDef_t can_type, const uint8_t data[], LL_CAN_TxHeaderTypeDef_t *htxheader)
{
	ErrorStatus status = ERROR;

	// Check the parameters
	assert_param(data);
	assert_param(htxheader->_DLC);
	assert_param(htxheader->_IDE);
	assert_param(htxheader->_RTR);
	assert_param(htxheader->TransmitGlobalTime);

	if (htxheader->_IDE == _CAN_ID_STD)
	{
		assert_param(htxheader->StdId);
	}
	else
	{
		assert_param(htxheader->ExtId);
	}
	// Check CAN`s instance is CAN1 or CAN2
	if (can_type == _CAN1)
	{
		canbase = _CAN1_REG_BASE;
	}
	else
	{
		canbase = _CAN2_REG_BASE;
	}

	// Check if any mailbox is empty
	if (((canbase->CAN_TSR) & (1U << TME0)) || ((canbase->CAN_TSR) & (1U << TME1)) || ((canbase->CAN_TSR) & (1U << TME2)))
	{
		status = Trans(canbase, data, htxheader);
	}

	return status;
}

/**
 * The function `Trans0` is responsible for configuring and initiating a CAN message transmission based
 * on the provided parameters.
 *
 * @param canbase The `canbase` parameter is a pointer to a structure of type `CAN_TypeDef_t`, which
 * likely represents a CAN (Controller Area Network) peripheral in a microcontroller or embedded
 * system. This structure would contain registers and configurations related to the CAN module.
 * @param data The `data` parameter in the provided function `ErrorStatus Trans0` is a pointer to an
 * array of uint8_t type, which represents the data to be transmitted over the CAN bus. The function
 * uses this data to write the message payload into the CAN registers for transmission.
 * @param htxheader The `htxheader` parameter in the `Trans0` function is a pointer to a structure of
 * type `LL_CAN_TxHeaderTypeDef`. This structure likely contains information related to the CAN message
 * transmission, such as the message ID, data length code (DLC), remote transmission request (R
 *
 * @return The function `ErrorStatus Trans0(CAN_TypeDef_t *canbase, uint8_t *data,
 * LL_CAN_TxHeaderTypeDef *htxheader)` returns an `ErrorStatus` value, which is either `ERROR` or
 * `SUCCESS` based on the conditions checked within the function.
 */
static ErrorStatus Trans(CAN_TypeDef_t *canbase, const uint8_t data[], LL_CAN_TxHeaderTypeDef_t *htxheader)
{
	ErrorStatus status = ERROR;
	uint32_t transmitmailbox;

	// Select an empty transmit mailbox
	transmitmailbox = (((canbase->CAN_TSR) >> CODE) & 0x03);

	// Set up the ID
	if (htxheader->_IDE == _CAN_ID_STD)
	{
		(canbase->sTxMailBox[transmitmailbox].TIR) = ((htxheader->StdId) << STID) | ((htxheader->_RTR) << RTR);
	}
	else
	{
		(canbase->sTxMailBox[transmitmailbox].TIR) = (((htxheader->ExtId) << EXID) | ((htxheader->_RTR) << RTR) | ((htxheader->_IDE) << IDE));
	}

	// Set up the DLC
	(canbase->sTxMailBox[transmitmailbox].TDTR) = ((htxheader->_DLC) << DLC);

	// Set up data
	(canbase->sTxMailBox[0].TDHR) = (((uint32_t)data[7] << CAN_TDHR_DATA7_Pos) | ((uint32_t)data[6] << CAN_TDHR_DATA6_Pos) | ((uint32_t)data[5] << CAN_TDHR_DATA5_Pos) | ((uint32_t)data[4] << CAN_TDHR_DATA4_Pos));
	(canbase->sTxMailBox[0].TDLR) = ((uint32_t)(data[3] << CAN_TDLR_DATA3_Pos) | ((uint32_t)data[2] << CAN_TDLR_DATA2_Pos) | ((uint32_t)data[1] << CAN_TDLR_DATA1_Pos) | ((uint32_t)data[0] << CAN_TDLR_DATA0_Pos));

	// Set up the Transmit Global Time mode
	if (htxheader->TransmitGlobalTime == ENABLE)
	{
		canbase->sTxMailBox[transmitmailbox].TDTR |= ((htxheader->TransmitGlobalTime) << TGT);
	}

	// Request transmission by enable bit TXRQ
	(canbase->sTxMailBox[transmitmailbox].TIR) |= (1UL << TXRQ);

	// Check pending transmission request on the selected Tx Mailboxes
	while (1)
	{
		if (!((canbase->CAN_TSR) & ((1U << transmitmailbox) << TME0)))
		{
			status = SUCCESS;
			break;
		}
	}
	return status;
}

/**
 * The HAL_CAN_IRQHandler function handles various interrupts and error conditions in a CAN bus
 * communication system.
 *
 * @param hcan The `hcan` parameter in the `HAL_CAN_IRQHandler` function is a pointer to a
 * `CAN_HandleTypeDef` structure, which typically contains all the necessary information and
 * configurations for handling a CAN peripheral in an STM32 microcontroller. This structure is used to
 * access the CAN hardware registers and manage the CAN
 */
//void LL_CAN_IRQHandler(LL_CAN_TypeDef_t can_type)
//{
//
//	// Check CAN instance is CAN1 or CAN2
//	if (can_type == _CAN1)
//	{
//		canbase = _CAN1_REG_BASE;
//	}
//	else if (can_type == _CAN2)
//	{
//		canbase = _CAN2_REG_BASE;
//	}
//	uint32_t errorcode = (0x00000000U);
//	uint32_t interrupts = (canbase->CAN_IER);
//	uint32_t msrflags = (canbase->CAN_MSR);
//	uint32_t tsrflags = (canbase->CAN_TSR);
//	uint32_t rf0rflags = (canbase->CAN_RF0R);
//	uint32_t rf1rflags = (canbase->CAN_RF1R);
//	uint32_t esrflags = (canbase->CAN_ESR);
//
//	/* Transmit Mailbox empty interrupt management *****************************/
//	if ((interrupts & (1U << TMEIE)) != 0U)
//	{
//		/* Transmit Mailbox 0 management *****************************************/
//		if ((tsrflags & (1U << RQCP0)) != 0U)
//		{
//			/* Clear the Transmission Complete flag (and TXOK0,ALST0,TERR0 bits) */
//			__HAL_CAN_CLEAR_FLAG(hcan, CAN_FLAG_RQCP0);
//
//			if ((tsrflags & CAN_TSR_TXOK0) != 0U)
//			{
//				/* Transmission Mailbox 0 complete callback */
//#if USE_HAL_CAN_REGISTER_CALLBACKS == 1
//				/* Call registered callback*/
//				hcan->TxMailbox0CompleteCallback(hcan);
//#else
//				/* Call weak (surcharged) callback */
//				HAL_CAN_TxMailbox0CompleteCallback(hcan);
//#endif /* USE_HAL_CAN_REGISTER_CALLBACKS */
//			}
//			else
//			{
//				if ((tsrflags & CAN_TSR_ALST0) != 0U)
//				{
//					/* Update error code */
//					errorcode |= HAL_CAN_ERROR_TX_ALST0;
//				}
//				else if ((tsrflags & CAN_TSR_TERR0) != 0U)
//				{
//					/* Update error code */
//					errorcode |= HAL_CAN_ERROR_TX_TERR0;
//				}
//				else
//				{
//					/* Transmission Mailbox 0 abort callback */
//#if USE_HAL_CAN_REGISTER_CALLBACKS == 1
//					/* Call registered callback*/
//					hcan->TxMailbox0AbortCallback(hcan);
//#else
//					/* Call weak (surcharged) callback */
//					HAL_CAN_TxMailbox0AbortCallback(hcan);
//#endif /* USE_HAL_CAN_REGISTER_CALLBACKS */
//				}
//			}
//		}
//
//		/* Transmit Mailbox 1 management *****************************************/
//		if ((tsrflags & CAN_TSR_RQCP1) != 0U)
//		{
//			/* Clear the Transmission Complete flag (and TXOK1,ALST1,TERR1 bits) */
//			__HAL_CAN_CLEAR_FLAG(hcan, CAN_FLAG_RQCP1);
//
//			if ((tsrflags & CAN_TSR_TXOK1) != 0U)
//			{
//				/* Transmission Mailbox 1 complete callback */
//#if USE_HAL_CAN_REGISTER_CALLBACKS == 1
//				/* Call registered callback*/
//				hcan->TxMailbox1CompleteCallback(hcan);
//#else
//				/* Call weak (surcharged) callback */
//				HAL_CAN_TxMailbox1CompleteCallback(hcan);
//#endif /* USE_HAL_CAN_REGISTER_CALLBACKS */
//			}
//			else
//			{
//				if ((tsrflags & CAN_TSR_ALST1) != 0U)
//				{
//					/* Update error code */
//					errorcode |= HAL_CAN_ERROR_TX_ALST1;
//				}
//				else if ((tsrflags & CAN_TSR_TERR1) != 0U)
//				{
//					/* Update error code */
//					errorcode |= HAL_CAN_ERROR_TX_TERR1;
//				}
//				else
//				{
//					/* Transmission Mailbox 1 abort callback */
//#if USE_HAL_CAN_REGISTER_CALLBACKS == 1
//					/* Call registered callback*/
//					hcan->TxMailbox1AbortCallback(hcan);
//#else
//					/* Call weak (surcharged) callback */
//					HAL_CAN_TxMailbox1AbortCallback(hcan);
//#endif /* USE_HAL_CAN_REGISTER_CALLBACKS */
//				}
//			}
//		}
//
//		/* Transmit Mailbox 2 management *****************************************/
//		if ((tsrflags & CAN_TSR_RQCP2) != 0U)
//		{
//			/* Clear the Transmission Complete flag (and TXOK2,ALST2,TERR2 bits) */
//			__HAL_CAN_CLEAR_FLAG(hcan, CAN_FLAG_RQCP2);
//
//			if ((tsrflags & CAN_TSR_TXOK2) != 0U)
//			{
//				/* Transmission Mailbox 2 complete callback */
//#if USE_HAL_CAN_REGISTER_CALLBACKS == 1
//				/* Call registered callback*/
//				hcan->TxMailbox2CompleteCallback(hcan);
//#else
//				/* Call weak (surcharged) callback */
//				HAL_CAN_TxMailbox2CompleteCallback(hcan);
//#endif /* USE_HAL_CAN_REGISTER_CALLBACKS */
//			}
//			else
//			{
//				if ((tsrflags & CAN_TSR_ALST2) != 0U)
//				{
//					/* Update error code */
//					errorcode |= HAL_CAN_ERROR_TX_ALST2;
//				}
//				else if ((tsrflags & CAN_TSR_TERR2) != 0U)
//				{
//					/* Update error code */
//					errorcode |= HAL_CAN_ERROR_TX_TERR2;
//				}
//				else
//				{
//					/* Transmission Mailbox 2 abort callback */
//#if USE_HAL_CAN_REGISTER_CALLBACKS == 1
//					/* Call registered callback*/
//					hcan->TxMailbox2AbortCallback(hcan);
//#else
//					/* Call weak (surcharged) callback */
//					HAL_CAN_TxMailbox2AbortCallback(hcan);
//#endif /* USE_HAL_CAN_REGISTER_CALLBACKS */
//				}
//			}
//		}
//	}
//
//	/* Receive FIFO 0 overrun interrupt management *****************************/
//	if ((interrupts & CAN_IT_RX_FIFO0_OVERRUN) != 0U)
//	{
//		if ((rf0rflags & CAN_RF0R_FOVR0) != 0U)
//		{
//			/* Set CAN error code to Rx Fifo 0 overrun error */
//			errorcode |= HAL_CAN_ERROR_RX_FOV0;
//
//			/* Clear FIFO0 Overrun Flag */
//			__HAL_CAN_CLEAR_FLAG(hcan, CAN_FLAG_FOV0);
//		}
//	}
//
//	/* Receive FIFO 0 full interrupt management ********************************/
//	if ((interrupts & CAN_IT_RX_FIFO0_FULL) != 0U)
//	{
//		if ((rf0rflags & CAN_RF0R_FULL0) != 0U)
//		{
//			/* Clear FIFO 0 full Flag */
//			__HAL_CAN_CLEAR_FLAG(hcan, CAN_FLAG_FF0);
//
//			/* Receive FIFO 0 full Callback */
//#if USE_HAL_CAN_REGISTER_CALLBACKS == 1
//			/* Call registered callback*/
//			hcan->RxFifo0FullCallback(hcan);
//#else
//			/* Call weak (surcharged) callback */
//			HAL_CAN_RxFifo0FullCallback(hcan);
//#endif /* USE_HAL_CAN_REGISTER_CALLBACKS */
//		}
//	}
//
//	/* Receive FIFO 0 message pending interrupt management *********************/
//	if ((interrupts & CAN_IT_RX_FIFO0_MSG_PENDING) != 0U)
//	{
//		/* Check if message is still pending */
//		if ((hcan->Instance->RF0R & CAN_RF0R_FMP0) != 0U)
//		{
//			/* Receive FIFO 0 message pending Callback */
//#if USE_HAL_CAN_REGISTER_CALLBACKS == 1
//			/* Call registered callback*/
//			hcan->RxFifo0MsgPendingCallback(hcan);
//#else
//			/* Call weak (surcharged) callback */
//			HAL_CAN_RxFifo0MsgPendingCallback(hcan);
//#endif /* USE_HAL_CAN_REGISTER_CALLBACKS */
//		}
//	}
//
//	/* Receive FIFO 1 overrun interrupt management *****************************/
//	if ((interrupts & CAN_IT_RX_FIFO1_OVERRUN) != 0U)
//	{
//		if ((rf1rflags & CAN_RF1R_FOVR1) != 0U)
//		{
//			/* Set CAN error code to Rx Fifo 1 overrun error */
//			errorcode |= HAL_CAN_ERROR_RX_FOV1;
//
//			/* Clear FIFO1 Overrun Flag */
//			__HAL_CAN_CLEAR_FLAG(hcan, CAN_FLAG_FOV1);
//		}
//	}
//
//	/* Receive FIFO 1 full interrupt management ********************************/
//	if ((interrupts & CAN_IT_RX_FIFO1_FULL) != 0U)
//	{
//		if ((rf1rflags & CAN_RF1R_FULL1) != 0U)
//		{
//			/* Clear FIFO 1 full Flag */
//			__HAL_CAN_CLEAR_FLAG(hcan, CAN_FLAG_FF1);
//
//			/* Receive FIFO 1 full Callback */
//#if USE_HAL_CAN_REGISTER_CALLBACKS == 1
//			/* Call registered callback*/
//			hcan->RxFifo1FullCallback(hcan);
//#else
//			/* Call weak (surcharged) callback */
//			HAL_CAN_RxFifo1FullCallback(hcan);
//#endif /* USE_HAL_CAN_REGISTER_CALLBACKS */
//		}
//	}
//
//	/* Receive FIFO 1 message pending interrupt management *********************/
//	if ((interrupts & CAN_IT_RX_FIFO1_MSG_PENDING) != 0U)
//	{
//		/* Check if message is still pending */
//		if ((hcan->Instance->RF1R & CAN_RF1R_FMP1) != 0U)
//		{
//			/* Receive FIFO 1 message pending Callback */
//#if USE_HAL_CAN_REGISTER_CALLBACKS == 1
//			/* Call registered callback*/
//			hcan->RxFifo1MsgPendingCallback(hcan);
//#else
//			/* Call weak (surcharged) callback */
//			HAL_CAN_RxFifo1MsgPendingCallback(hcan);
//#endif /* USE_HAL_CAN_REGISTER_CALLBACKS */
//		}
//	}
//
//	/* Sleep interrupt management *********************************************/
//	if ((interrupts & CAN_IT_SLEEP_ACK) != 0U)
//	{
//		if ((msrflags & CAN_MSR_SLAKI) != 0U)
//		{
//			/* Clear Sleep interrupt Flag */
//			__HAL_CAN_CLEAR_FLAG(hcan, CAN_FLAG_SLAKI);
//
//			/* Sleep Callback */
//#if USE_HAL_CAN_REGISTER_CALLBACKS == 1
//			/* Call registered callback*/
//			hcan->SleepCallback(hcan);
//#else
//			/* Call weak (surcharged) callback */
//			HAL_CAN_SleepCallback(hcan);
//#endif /* USE_HAL_CAN_REGISTER_CALLBACKS */
//		}
//	}
//
//	/* WakeUp interrupt management *********************************************/
//	if ((interrupts & CAN_IT_WAKEUP) != 0U)
//	{
//		if ((msrflags & CAN_MSR_WKUI) != 0U)
//		{
//			/* Clear WakeUp Flag */
//			__HAL_CAN_CLEAR_FLAG(hcan, CAN_FLAG_WKU);
//
//			/* WakeUp Callback */
//#if USE_HAL_CAN_REGISTER_CALLBACKS == 1
//			/* Call registered callback*/
//			hcan->WakeUpFromRxMsgCallback(hcan);
//#else
//			/* Call weak (surcharged) callback */
//			HAL_CAN_WakeUpFromRxMsgCallback(hcan);
//#endif /* USE_HAL_CAN_REGISTER_CALLBACKS */
//		}
//	}
//
//	/* Error interrupts management *********************************************/
//	if ((interrupts & CAN_IT_ERROR) != 0U)
//	{
//		if ((msrflags & CAN_MSR_ERRI) != 0U)
//		{
//			/* Check Error Warning Flag */
//			if (((interrupts & CAN_IT_ERROR_WARNING) != 0U) &&
//				((esrflags & CAN_ESR_EWGF) != 0U))
//			{
//				/* Set CAN error code to Error Warning */
//				errorcode |= HAL_CAN_ERROR_EWG;
//
//				/* No need for clear of Error Warning Flag as read-only */
//			}
//
//			/* Check Error Passive Flag */
//			if (((interrupts & CAN_IT_ERROR_PASSIVE) != 0U) &&
//				((esrflags & CAN_ESR_EPVF) != 0U))
//			{
//				/* Set CAN error code to Error Passive */
//				errorcode |= HAL_CAN_ERROR_EPV;
//
//				/* No need for clear of Error Passive Flag as read-only */
//			}
//
//			/* Check Bus-off Flag */
//			if (((interrupts & CAN_IT_BUSOFF) != 0U) &&
//				((esrflags & CAN_ESR_BOFF) != 0U))
//			{
//				/* Set CAN error code to Bus-Off */
//				errorcode |= HAL_CAN_ERROR_BOF;
//
//				/* No need for clear of Error Bus-Off as read-only */
//			}
//
//			/* Check Last Error Code Flag */
//			if (((interrupts & CAN_IT_LAST_ERROR_CODE) != 0U) &&
//				((esrflags & CAN_ESR_LEC) != 0U))
//			{
//				switch (esrflags & CAN_ESR_LEC)
//				{
//				case (CAN_ESR_LEC_0):
//					/* Set CAN error code to Stuff error */
//					errorcode |= HAL_CAN_ERROR_STF;
//					break;
//				case (CAN_ESR_LEC_1):
//					/* Set CAN error code to Form error */
//					errorcode |= HAL_CAN_ERROR_FOR;
//					break;
//				case (CAN_ESR_LEC_1 | CAN_ESR_LEC_0):
//					/* Set CAN error code to Acknowledgement error */
//					errorcode |= HAL_CAN_ERROR_ACK;
//					break;
//				case (CAN_ESR_LEC_2):
//					/* Set CAN error code to Bit recessive error */
//					errorcode |= HAL_CAN_ERROR_BR;
//					break;
//				case (CAN_ESR_LEC_2 | CAN_ESR_LEC_0):
//					/* Set CAN error code to Bit Dominant error */
//					errorcode |= HAL_CAN_ERROR_BD;
//					break;
//				case (CAN_ESR_LEC_2 | CAN_ESR_LEC_1):
//					/* Set CAN error code to CRC error */
//					errorcode |= HAL_CAN_ERROR_CRC;
//					break;
//				default:
//					break;
//				}
//
//				/* Clear Last error code Flag */
//				CLEAR_BIT(hcan->Instance->ESR, CAN_ESR_LEC);
//			}
//		}
//
//		/* Clear ERRI Flag */
//		__HAL_CAN_CLEAR_FLAG(hcan, CAN_FLAG_ERRI);
//	}
//
//	/* Call the Error call Back in case of Errors */
//	if (errorcode != HAL_CAN_ERROR_NONE)
//	{
//		/* Update error code in handle */
//		hcan->ErrorCode |= errorcode;
//
//		/* Call Error callback function */
//#if USE_HAL_CAN_REGISTER_CALLBACKS == 1
//		/* Call registered callback*/
//		hcan->ErrorCallback(hcan);
//#else
//		/* Call weak (surcharged) callback */
//		HAL_CAN_ErrorCallback(hcan);
//#endif /* USE_HAL_CAN_REGISTER_CALLBACKS */
//	}
//}
/*
  ==============================================================================
						  ##### Callback functions #####
  ==============================================================================
	[..]
	This subsection provides the following callback functions:
	  (+) HAL_CAN_TxMailbox0CompleteCallback
	  (+) HAL_CAN_TxMailbox1CompleteCallback
	  (+) HAL_CAN_TxMailbox2CompleteCallback
	  (+) HAL_CAN_TxMailbox0AbortCallback
	  (+) HAL_CAN_TxMailbox1AbortCallback
	  (+) HAL_CAN_TxMailbox2AbortCallback
	  (+) HAL_CAN_RxFifo0MsgPendingCallback
	  (+) HAL_CAN_RxFifo0FullCallback
	  (+) HAL_CAN_RxFifo1MsgPendingCallback
	  (+) HAL_CAN_RxFifo1FullCallback
	  (+) HAL_CAN_SleepCallback
	  (+) HAL_CAN_WakeUpFromRxMsgCallback
	  (+) HAL_CAN_ErrorCallback
*/

/**
 * @brief  Transmission Mailbox 0 complete callback.
 * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @retval None
 */
__weak void LL_CAN_TxMailbox0CompleteCallback(LL_CAN_TypeDef_t can_type)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(can_type);

	/* NOTE : This function Should not be modified, when the callback is needed,
			  the LL_CAN_TxMailbox0CompleteCallback could be implemented in the
			  user file
	 */
}

/**
 * @brief  Transmission Mailbox 1 complete callback.
 * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @retval None
 */
__weak void LL_CAN_TxMailbox1CompleteCallback(LL_CAN_TypeDef_t can_type)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(can_type);

	/* NOTE : This function Should not be modified, when the callback is needed,
			  the LL_CAN_TxMailbox1CompleteCallback could be implemented in the
			  user file
	 */
}

/**
 * @brief  Transmission Mailbox 2 complete callback.
 * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @retval None
 */
__weak void LL_CAN_TxMailbox2CompleteCallback(LL_CAN_TypeDef_t can_type)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(can_type);

	/* NOTE : This function Should not be modified, when the callback is needed,
			  the LL_CAN_TxMailbox2CompleteCallback could be implemented in the
			  user file
	 */
}

/**
 * @brief  Transmission Mailbox 0 Cancellation callback.
 * @param  hcan pointer to an CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @retval None
 */
__weak void LL_CAN_TxMailbox0AbortCallback(LL_CAN_TypeDef_t can_type)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(can_type);

	/* NOTE : This function Should not be modified, when the callback is needed,
			  the LL_CAN_TxMailbox0AbortCallback could be implemented in the
			  user file
	 */
}

/**
 * @brief  Transmission Mailbox 1 Cancellation callback.
 * @param  hcan pointer to an CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @retval None
 */
__weak void LL_CAN_TxMailbox1AbortCallback(LL_CAN_TypeDef_t can_type)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(can_type);

	/* NOTE : This function Should not be modified, when the callback is needed,
			  the LL_CAN_TxMailbox1AbortCallback could be implemented in the
			  user file
	 */
}

/**
 * @brief  Transmission Mailbox 2 Cancellation callback.
 * @param  hcan pointer to an CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @retval None
 */
__weak void LL_CAN_TxMailbox2AbortCallback(LL_CAN_TypeDef_t can_type)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(can_type);

	/* NOTE : This function Should not be modified, when the callback is needed,
			  the LL_CAN_TxMailbox2AbortCallback could be implemented in the
			  user file
	 */
}

/**
 * @brief  Rx FIFO 0 message pending callback.
 * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @retval None
 */
__weak void HAL_CAN_RxFifo0MsgPendingCallback(LL_CAN_TypeDef_t can_type)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(can_type);

	/* NOTE : This function Should not be modified, when the callback is needed,
			  the LL_CAN_RxFifo0MsgPendingCallback could be implemented in the
			  user file
	 */
}

/**
 * @brief  Rx FIFO 0 full callback.
 * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @retval None
 */
__weak void HAL_CAN_RxFifo0FullCallback(LL_CAN_TypeDef_t can_type)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(can_type);

	/* NOTE : This function Should not be modified, when the callback is needed,
			  the LL_CAN_RxFifo0FullCallback could be implemented in the user
			  file
	 */
}

/**
 * @brief  Rx FIFO 1 message pending callback.
 * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @retval None
 */
__weak void LL_CAN_RxFifo1MsgPendingCallback(LL_CAN_TypeDef_t can_type)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(can_type);

	/* NOTE : This function Should not be modified, when the callback is needed,
			  the LL_CAN_RxFifo1MsgPendingCallback could be implemented in the
			  user file
	 */
}

/**
 * @brief  Rx FIFO 1 full callback.
 * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @retval None
 */
__weak void LL_CAN_RxFifo1FullCallback(LL_CAN_TypeDef_t can_type)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(can_type);

	/* NOTE : This function Should not be modified, when the callback is needed,
			  the LL_CAN_RxFifo1FullCallback could be implemented in the user
			  file
	 */
}

/**
 * @brief  Sleep callback.
 * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @retval None
 */
__weak void LL_CAN_SleepCallback(LL_CAN_TypeDef_t can_type)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(can_type);

	/* NOTE : This function Should not be modified, when the callback is needed,
			  the LL_CAN_SleepCallback could be implemented in the user file
	 */
}

/**
 * @brief  WakeUp from Rx message callback.
 * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @retval None
 */
__weak void LL_CAN_WakeUpFromRxMsgCallback(LL_CAN_TypeDef_t can_type)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(can_type);

	/* NOTE : This function Should not be modified, when the callback is needed,
			  the LL_CAN_WakeUpFromRxMsgCallback could be implemented in the
			  user file
	 */
}

/**
 * @brief  Error CAN callback.
 * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @retval None
 */
__weak void LL_CAN_ErrorCallback(LL_CAN_TypeDef_t can_type)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(can_type);

	/* NOTE : This function Should not be modified, when the callback is needed,
			  the LL_CAN_ErrorCallback could be implemented in the user file
	 */
}
