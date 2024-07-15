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
static void Trans(CAN_TypeDef_t *canbase, const uint8_t data[], LL_CAN_TxHeaderTypeDef_t *htxheader, uint32_t *TxMailBox);

/**
 * The function `LL_CAN_GPIO_Init` initializes GPIO pins for CAN communication based on the specified
 * CAN type.
 *
 * @param can_type The `can_type` parameter in the `LL_CAN_GPIO_Init` function is used to determine
 * which CAN bus (CAN1 or CAN2) configuration to initialize based on the value passed to it. The
 * function configures the GPIO pins for CAN communication based on the specified CAN type.
 */
ErrorStatus LL_CAN_GPIO_Init(LL_CAN_Handler_t *hcan)
{

	ErrorStatus status = ERROR;
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* Peripheral clock enable */
	if (hcan->Instance == _CAN1)
		RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;
	else
		RCC->APB1ENR |= RCC_APB1ENR_CAN2EN;

	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

	if (hcan->Instance == _CAN1)
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
	else if (hcan->Instance == _CAN2)
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
ErrorStatus LL_CAN_Init(LL_CAN_Handler_t *hcan)
{

	ErrorStatus status = ERROR;
	uint8_t time_out = 50;
	uint8_t time_start = 0;
	assert_param(hcan->Init.Prescaler);
	assert_param(hcan->Init.SyncJumpWidth);
	assert_param(hcan->Init.TimeSeg1);
	assert_param(hcan->Init.TimeSeg2);
	assert_param(hcan->Init.Mode);
	assert_param(hcan->Init.status.AutoBusOff);
	assert_param(hcan->Init.status.AutoRetransmission);
	assert_param(hcan->Init.status.AutoWakeUp);
	assert_param(hcan->Init.status.ReceiveFifoLocked);
	assert_param(hcan->Init.status.TimeTriggeredMode);
	assert_param(hcan->Init.status.TransmitFifoPriority);

	// Check CAN instance is CAN1 or CAN2
	if (hcan->Instance == _CAN1)
	{
		canbase = _CAN1_REG_BASE;
		status = SUCCESS;
	}
	else if (hcan->Instance == _CAN2)
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
	switch (hcan->Init.Mode)
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
	(canbase->CAN_BTR) |= (((hcan->Init.SyncJumpWidth) << SJW) | (((hcan->Init.Prescaler) - 1) << BRP) | ((hcan->Init.TimeSeg1) << TS1) | ((hcan->Init.TimeSeg2) << TS2));

	// Can Autowakeup, default value of bit AWUM=0
	if (hcan->Init.status.AutoWakeUp == ENABLE)
		(canbase->CAN_MCR) |= (1U << AWUM);
	else
		(canbase->CAN_MCR) &= ~(1U << AWUM);

	// Can No Autorestranmission, default value of bit NART=0
	if (hcan->Init.status.AutoRetransmission == DISABLE)
		(canbase->CAN_MCR) |= (1U << NART);
	else
		(canbase->CAN_MCR) &= ~(1U << NART);

	// Can ReceiveFifoLocked, default value of bit RFLM=0
	if (hcan->Init.status.ReceiveFifoLocked == ENABLE)
		(canbase->CAN_MCR) |= (1U << RFLM);
	else
		(canbase->CAN_MCR) &= ~(1U << RFLM);

	// Can Time trigger mode, default value of bit TTCM=0
	if (hcan->Init.status.TimeTriggeredMode == ENABLE)
		(canbase->CAN_MCR) |= (1U << TTCM);
	else
		(canbase->CAN_MCR) &= ~(1U << TTCM);

	// Can TransmitFifoPriority, default value of bit TXFP=0
	if (hcan->Init.status.TransmitFifoPriority == ENABLE)
		(canbase->CAN_MCR) |= (1U << TXFP);
	else
		(canbase->CAN_MCR) &= ~(1U << TXFP);

	// Can AutoBusOff, default value of bit ABOM=0
	if (hcan->Init.status.AutoBusOff == ENABLE)
		canbase->CAN_MCR |= (1U << ABOM);
	else
		(canbase->CAN_MCR) &= ~(1U << ABOM);

	return status;
}

/**
 * The function LL_CAN_ConfigFilter configures CAN filter settings based on the provided parameters.
 *
 * @param hcan The `hcan` parameter is a pointer to a structure `LL_CAN_Handler_t` which contains
 * information about the CAN handler, including the CAN instance (`Instance`) which can be either
 * `_CAN1` or `_CAN2`.
 * @param hfilter - FilterActivation: Indicates whether the filter is enabled or disabled.
 *
 * @return The function `ErrorStatus LL_CAN_ConfigFilter(LL_CAN_Handler_t *hcan, LL_CAN_FilterTypeDef_t
 * *hfilter)` is returning the status of the configuration process, which is either `SUCCESS` or
 * `ERROR`.
 */
ErrorStatus LL_CAN_ConfigFilter(LL_CAN_Handler_t *hcan, LL_CAN_FilterTypeDef_t *hfilter)
{
	ErrorStatus status = ERROR;
	uint32_t filter_bank_pos;
	// Check CAN`s instance is CAN1 or CAN2
	status = SUCCESS;
	if (hcan->Instance == _CAN1)
	{
		canbase = _CAN1_REG_BASE;
		status = SUCCESS;
	}
	else
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

	// Select the start filter number of CAN2 slave instance
	if (hcan->Instance == _CAN1)
	{
		(canbase->CAN_FMR) &= ~(1U << CAN2SB);
		(canbase->CAN_FMR) |= ((hfilter->SlaveStartFilterBank) << CAN2SB);
	}

	// Convert filter bank number into bit position
	filter_bank_pos = (1U << ((hfilter->FilterBank) & 0x1FU));

	// Filter deactivation
	(canbase->CAN_FA1R) &= ~(filter_bank_pos);

	// Config filter scale
	if (hfilter->FilterScale == _CAN_FILTERSCALE_16BIT)
	{
		(canbase->CAN_FS1R) &= ~(filter_bank_pos);

		// First 16-bit identifier and First 16-bit mask
		// Or First 16-bit identifier and Second 16-bit identifier
		(canbase->CAN_sFilterRegister[hfilter->FilterBank].FR1) =
			((0x0000FFFFU & (uint32_t)hfilter->FilterMaskIdLow) << 16U) |
			(0x0000FFFFU & (uint32_t)hfilter->FilterIdLow);

		// Second 16-bit identifier and Second 16-bit mask */
		//  Or Third 16-bit identifier and Fourth 16-bit identifier */
		(canbase->CAN_sFilterRegister[hfilter->FilterBank].FR2) =
			((0x0000FFFFU & (uint32_t)hfilter->FilterMaskIdHigh) << 16U) |
			(0x0000FFFFU & (uint32_t)hfilter->FilterIdHigh);
	}
	if (hfilter->FilterScale == _CAN_FILTERSCALE_32BIT)
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

	// Config filter mode
	if (hfilter->FilterMode == _CAN_FILTERMODE_IDMASK)
	{
		(canbase->CAN_FM1R) &= ~(filter_bank_pos);
	}
	else
	{
		(canbase->CAN_FM1R) |= (filter_bank_pos);
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

	// Filter activation
	if (hfilter->FilterActivation == _CAN_FILTER_ENABLE)
	{
		(canbase->CAN_FA1R) |= (filter_bank_pos);
	}
	else
	{
		(canbase->CAN_FA1R) &= ~(filter_bank_pos);
	}

	// Leave the initialisation mode for the filter
	(canbase->CAN_FMR) &= ~(1U << FINIT);
	return status;
}

/**
 * The function LL_CAN_Start initializes and starts a CAN bus communication interface.
 *
 * @param hcan The `hcan` parameter is a pointer to a structure of type `LL_CAN_Handler_t`, which
 * likely contains information about the CAN (Controller Area Network) handler or controller being used
 * in the code. The function `LL_CAN_Start` is responsible for starting the CAN communication and
 * configuring the controller.
 *
 * @return The function LL_CAN_Start is returning an ErrorStatus enum value, which can be either
 * SUCCESS or ERROR.
 */
ErrorStatus LL_CAN_Start(LL_CAN_Handler_t *hcan)
{
	ErrorStatus status = ERROR;
	uint8_t time_out = 50;
	uint8_t time_start = 0;
	if (hcan->Instance == _CAN1)
	{
		canbase = _CAN1_REG_BASE;
		status = SUCCESS;
	}
	else if (hcan->Instance == _CAN2)
	{
		canbase = _CAN2_REG_BASE;
		status = SUCCESS;
	}

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
 * The function `LL_CAN_AddTxMessage` adds a transmit message to a CAN bus and checks if any mailbox is
 * empty for transmission.
 *
 * @param hcan `hcan` is a pointer to a structure `LL_CAN_Handler_t` which likely contains information
 * about the CAN controller and its configuration.
 * @param data An array of uint8_t data to be transmitted via CAN bus.
 * @param htxheader - _DLC: Data Length Code
 * @param TxMailBox The `TxMailBox` parameter in the `LL_CAN_AddTxMessage` function is a pointer to a
 * `uint32_t` variable that will be used to store the mailbox number where the transmitted message is
 * placed. This variable will be updated by the function `Trans` which is called if there
 *
 * @return The function `ErrorStatus LL_CAN_AddTxMessage` is returning the status of the operation,
 * which can be either `SUCCESS` or `ERROR`.
 */
ErrorStatus LL_CAN_AddTxMessage(LL_CAN_Handler_t *hcan, const uint8_t data[], LL_CAN_TxHeaderTypeDef_t *htxheader, uint32_t *TxMailBox)
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
	if (hcan->Instance == _CAN1)
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
		Trans(canbase, data, htxheader, TxMailBox);
		status = SUCCESS;
	}

	return status;
}

/**
 * The function `Trans` sets up and transmits a CAN message using the provided data and header
 * information.
 *
 * @param canbase The `canbase` parameter in the `Trans` function is a pointer to a structure of type
 * `CAN_TypeDef_t`, which likely represents a CAN (Controller Area Network) peripheral in a
 * microcontroller or embedded system. This structure would contain registers and configurations
 * related to the CAN module, allowing you to
 * @param data The `data` parameter in the `Trans` function is an array of uint8_t type, which
 * represents the data to be transmitted over the CAN bus. The array contains 8 elements, with index 0
 * to 7, where each element holds a byte of data to be sent.
 * @param htxheader The `htxheader` parameter is a pointer to a structure of type
 * `LL_CAN_TxHeaderTypeDef_t`. This structure likely contains information related to the CAN message
 * transmission, such as the message ID, data length, RTR bit, IDE bit, Transmit Global Time flag, etc.
 * @param TxMailBox The `TxMailBox` parameter is a pointer to a `uint32_t` variable where the function
 * will store the selected transmit mailbox for sending the CAN message.
 */
static void Trans(CAN_TypeDef_t *canbase, const uint8_t data[], LL_CAN_TxHeaderTypeDef_t *htxheader, uint32_t *TxMailBox)
{

	uint32_t transmitmailbox;

	// Select an empty transmit mailbox
	transmitmailbox = (((canbase->CAN_TSR) >> CODE) & 0x03);
	*TxMailBox = transmitmailbox;
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
	// Set up the Transmit Global Time mode
	if (htxheader->TransmitGlobalTime == ENABLE)
	{
		canbase->sTxMailBox[transmitmailbox].TDTR |= ((htxheader->TransmitGlobalTime) << TGT);
	}

	// Set up data
	(canbase->sTxMailBox[transmitmailbox].TDHR) = (((uint32_t)data[7] << CAN_TDHR_DATA7_Pos) | ((uint32_t)data[6] << CAN_TDHR_DATA6_Pos) | ((uint32_t)data[5] << CAN_TDHR_DATA5_Pos) | ((uint32_t)data[4] << CAN_TDHR_DATA4_Pos));
	(canbase->sTxMailBox[transmitmailbox].TDLR) = ((uint32_t)(data[3] << CAN_TDLR_DATA3_Pos) | ((uint32_t)data[2] << CAN_TDLR_DATA2_Pos) | ((uint32_t)data[1] << CAN_TDLR_DATA1_Pos) | ((uint32_t)data[0] << CAN_TDLR_DATA0_Pos));

	// Request transmission by enable bit TXRQ
	(canbase->sTxMailBox[transmitmailbox].TIR) |= (1UL << TXRQ);
}

/**
 * The function checks if there is a pending transmission request on a specific Tx mailbox of a CAN
 * controller.
 *
 * @param hcan The `hcan` parameter is a pointer to a structure `LL_CAN_Handler_t` which likely
 * contains information about the CAN controller being used, such as the instance (CAN1 or CAN2).
 * @param TxMailBox The `TxMailBox` parameter in the `ErrorStatus LL_CAN_IsTxMessagePending` function
 * represents the mailbox number of the CAN controller where the transmission message is pending. It is
 * used to check if there is a pending transmission request on the selected Tx mailbox specified by the
 * user.
 *
 * @return an ErrorStatus value, which can be either ERROR or SUCCESS.
 */
ErrorStatus LL_CAN_IsTxMessagePending(LL_CAN_Handler_t *hcan, uint32_t *TxMailBox)
{
	ErrorStatus status = ERROR;
	// Check CAN`s instance is CAN1 or CAN2
	if (hcan->Instance == _CAN1)
	{
		canbase = _CAN1_REG_BASE;
	}
	else
	{
		canbase = _CAN2_REG_BASE;
	}
	// Check pending transmission request on the selected Tx Mailboxes
	if (!((canbase->CAN_TSR) & ((1U << *TxMailBox) << TME0)))
	{
		status = SUCCESS;
	}
	return status;
}

/**
 * The function `LL_CAN_GetRxFifoFillLevel` retrieves the fill level of a specified receive FIFO in a
 * CAN peripheral.
 *
 * @param hcan The `hcan` parameter is a pointer to a structure `LL_CAN_Handler_t` which likely
 * contains information about the CAN handler, such as the CAN instance (CAN1 or CAN2).
 * @param RxFifo RxFifo is the receive FIFO (First In First Out) buffer in a CAN (Controller Area
 * Network) controller. In this function, it is used to specify which receive FIFO buffer to check for
 * fill level - either FIFO0 or FIFO1.
 *
 * @return The function `LL_CAN_GetRxFifoFillLevel` returns the fill level of the specified receive
 * FIFO (RxFifo) for the given CAN handler (hcan).
 */
uint32_t LL_CAN_GetRxFifoFillLevel(LL_CAN_Handler_t *hcan, uint32_t RxFifo)
{
	uint32_t filllevel = 0U;

	// Check CAN`s instance is CAN1 or CAN2
	if (hcan->Instance == _CAN1)
	{
		canbase = _CAN1_REG_BASE;
	}
	else
	{
		canbase = _CAN2_REG_BASE;
	}

	if (RxFifo == _CAN_RX_FIFO0)
	{
		filllevel = (canbase->CAN_RF0R & (3U << FMP0));
	}
	else /* RxFifo == _CAN_RX_FIFO1 */
	{
		filllevel = (canbase->CAN_RF1R & (3U << FMP1));
	}
	return filllevel;
}

/**
 * The function `LL_CAN_GetRxMessage` retrieves a received CAN message from the specified CAN hardware
 * instance and populates the header and data into the provided structures.
 *
 * @param hcan The `hcan` parameter is a pointer to a structure `LL_CAN_Handler_t` which contains
 * information about the CAN controller instance being used.
 * @param hrxheader The `hrxheader` parameter is a pointer to a structure of type
 * `LL_CAN_RxHeaderTypeDef_t`. This structure likely contains fields to store information about the
 * received CAN message, such as the ID, DLC, RTR, Filter Match Index, Timestamp, etc. The function
 * `Error
 * @param rxdata The `rxdata` parameter is an array where the received CAN message data will be stored.
 * It is expected to be a valid memory location where the data can be written to by the function.
 * @param RxFifo The `RxFifo` parameter in the provided function `ErrorStatus LL_CAN_GetRxMessage` is
 * used to specify which Receive FIFO (0 or 1) to read the message from. It is an input parameter that
 * determines from which FIFO the message should be retrieved.
 *
 * @return either SUCCESS or ERROR based on the conditions checked within the function. If the Rx FIFO
 * is empty or if there is a parameter error, the function will return ERROR. Otherwise, it will return
 * SUCCESS after successfully retrieving the CAN message data.
 */
ErrorStatus LL_CAN_GetRxMessage(LL_CAN_Handler_t *hcan, LL_CAN_RxHeaderTypeDef_t *hrxheader, uint8_t rxdata[], uint32_t RxFifo)
{
	// Check the parameters
	assert_param(rxdata);
	assert_param(RxFifo);

	if (hcan->Instance == _CAN1)
	{
		canbase = _CAN1_REG_BASE;
	}
	else
	{
		canbase = _CAN2_REG_BASE;
	}

	/* Check the Rx FIFO */
	if (RxFifo == _CAN_RX_FIFO0) /* Rx element is assigned to Rx FIFO 0 */
	{
		/* Check that the Rx FIFO 0 is not empty */
		if (((canbase->CAN_RF0R) & (1 << FMP0)) == 0U)
		{
			/* Update error code */
			hcan->ErrorCode |= LL_CAN_ERROR_PARAM;

			return ERROR;
		}
	}
	else /* Rx element is assigned to Rx FIFO 1 */
	{
		/* Check that the Rx FIFO 1 is not empty */
		if (((canbase->CAN_RF1R) & (1 << FMP1)) == 0U)
		{
			/* Update error code */
			hcan->ErrorCode |= LL_CAN_ERROR_PARAM;

			return ERROR;
		}
	}

	// Get ID
	hrxheader->_IDE = (((1U << IDE_rx) & (canbase->sFIFOMailBox[RxFifo].RIR)) >> IDE_rx);
	if (hrxheader->_IDE == _CAN_ID_STD)
	{
		hrxheader->StdId = (((0x7FF << STID_rx) & (canbase->sFIFOMailBox[RxFifo].RIR)) >> STID_rx);
	}
	else
	{
		hrxheader->ExtId = (((0x1FFFFFFF << EXID_rx) & (canbase->sFIFOMailBox[RxFifo].RIR)) >> EXID);
	}

	// Get Remote transmission request
	hrxheader->_RTR = (((1U << RTR_rx) & (canbase->sFIFOMailBox[RxFifo].RIR)) >> RTR_rx);
	if ((((0xF << DLC_rx) & (canbase->sFIFOMailBox[RxFifo].RDTR)) >> DLC_rx) >= 8U)
	{
		/* Truncate DLC to 8 if received field is over range */
		hrxheader->_DLC = 8U;
	}
	else
	{
		hrxheader->_DLC = (((0xF << DLC_rx) & (canbase->sFIFOMailBox[RxFifo].RDTR)) >> DLC_rx);
	}

	hrxheader->FilterMatchIndex = (((0xFF << FMI_rx) & (canbase->sFIFOMailBox[RxFifo].RDTR)) >> FMI_rx);
	hrxheader->Timestamp = (((0xFFFF << TIME_rx) & (canbase->sFIFOMailBox[RxFifo].RDTR)) >> TIME_rx);

	/* Get the data */
	rxdata[0] = (uint8_t)(((0XFF << CAN_RDLR_DATA0_Pos) & (canbase->sFIFOMailBox[RxFifo].RDLR)) >> CAN_RDLR_DATA0_Pos);
	rxdata[1] = (uint8_t)(((0XFF << CAN_RDLR_DATA1_Pos) & (canbase->sFIFOMailBox[RxFifo].RDLR)) >> CAN_RDLR_DATA1_Pos);
	rxdata[2] = (uint8_t)(((0XFF << CAN_RDLR_DATA2_Pos) & (canbase->sFIFOMailBox[RxFifo].RDLR)) >> CAN_RDLR_DATA2_Pos);
	rxdata[3] = (uint8_t)(((0XFF << CAN_RDLR_DATA3_Pos) & (canbase->sFIFOMailBox[RxFifo].RDLR)) >> CAN_RDLR_DATA3_Pos);
	rxdata[4] = (uint8_t)(((0XFF << CAN_RDHR_DATA4_Pos) & (canbase->sFIFOMailBox[RxFifo].RDHR)) >> CAN_RDHR_DATA4_Pos);
	rxdata[5] = (uint8_t)(((0XFF << CAN_RDHR_DATA5_Pos) & (canbase->sFIFOMailBox[RxFifo].RDHR)) >> CAN_RDHR_DATA5_Pos);
	rxdata[6] = (uint8_t)(((0XFF << CAN_RDHR_DATA6_Pos) & (canbase->sFIFOMailBox[RxFifo].RDHR)) >> CAN_RDHR_DATA6_Pos);
	rxdata[7] = (uint8_t)(((0XFF << CAN_RDHR_DATA7_Pos) & (canbase->sFIFOMailBox[RxFifo].RDHR)) >> CAN_RDHR_DATA7_Pos);

	/* Release the FIFO */
	if (RxFifo == _CAN_RX_FIFO0) /* Rx element is assigned to Rx FIFO 0 */
	{
		/* Release RX FIFO 0 */
		(canbase->CAN_RF0R) |= (1U << RFOM0);
	}
	else /* Rx element is assigned to Rx FIFO 1 */
	{
		/* Release RX FIFO 1 */
		(canbase->CAN_RF1R) |= (1U << RFOM1);
	}

	return SUCCESS;
}
/**
 * The function LL_CAN_ActivateInterrupt activates selected interrupts for a CAN handler based on the
 * specified ActiveITs.
 *
 * @param hcan The `hcan` parameter is a pointer to a structure of type `LL_CAN_Handler_t`, which
 * likely contains information and configurations related to a CAN (Controller Area Network) peripheral
 * in a microcontroller.
 * @param ActiveITs The `ActiveITs` parameter in the `LL_CAN_ActivateInterrupt` function is used to
 * specify which interrupts should be enabled. It is a bitmask where each bit corresponds to a specific
 * interrupt source. By ORing the `ActiveITs` bitmask with the `CAN_IER` register,
 */

void LL_CAN_ActivateInterrupt(LL_CAN_Handler_t *hcan, uint32_t ActiveITs)
{
	if (hcan->Instance == _CAN1)
	{
		canbase = _CAN1_REG_BASE;
	}
	else
	{
		canbase = _CAN2_REG_BASE;
	}
	/* Enable the selected interrupts */
	(canbase->CAN_IER) |= (ActiveITs);
}

/*
  ==============================================================================
						  ##### Callback functions #####
  ==============================================================================
	[..]
	This subsection provides the following callback functions:
	  (+) LL_CAN_TxMailbox0CompleteCallback
	  (+) LL_CAN_TxMailbox1CompleteCallback
	  (+) LL_CAN_TxMailbox2CompleteCallback
	  (+) LL_CAN_TxMailbox0AbortCallback
	  (+) LL_CAN_TxMailbox1AbortCallback
	  (+) LL_CAN_TxMailbox2AbortCallback
	  (+) LL_CAN_RxFifo0MsgPendingCallback
	  (+) LL_CAN_RxFifo0FullCallback
	  (+) LL_CAN_RxFifo1MsgPendingCallback
	  (+) LL_CAN_RxFifo1FullCallback
	  (+) LL_CAN_SleepCallback
	  (+) LL_CAN_WakeUpFromRxMsgCallback
	  (+) LL_CAN_ErrorCallback
*/

/**
 * @brief  Transmission Mailbox 0 complete callback.
 * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @retval None
 */
__weak void LL_CAN_TxMailbox0CompleteCallback(LL_CAN_Handler_t *hcan)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(&hcan);

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
__weak void LL_CAN_TxMailbox1CompleteCallback(LL_CAN_Handler_t *hcan)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(&hcan);

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
__weak void LL_CAN_TxMailbox2CompleteCallback(LL_CAN_Handler_t *hcan)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(&hcan);

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
__weak void LL_CAN_TxMailbox0AbortCallback(LL_CAN_Handler_t *hcan)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(&hcan);

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
__weak void LL_CAN_TxMailbox1AbortCallback(LL_CAN_Handler_t *hcan)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(&hcan);

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
__weak void LL_CAN_TxMailbox2AbortCallback(LL_CAN_Handler_t *hcan)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(&hcan);

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
__weak void LL_CAN_RxFifo0MsgPendingCallback(LL_CAN_Handler_t *hcan)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(&hcan);

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
__weak void LL_CAN_RxFifo0FullCallback(LL_CAN_Handler_t *hcan)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(&hcan);

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
__weak void LL_CAN_RxFifo1MsgPendingCallback(LL_CAN_Handler_t *hcan)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(&hcan);

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
__weak void LL_CAN_RxFifo1FullCallback(LL_CAN_Handler_t *hcan)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(&hcan);

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
__weak void LL_CAN_SleepCallback(LL_CAN_Handler_t *hcan)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(&hcan);

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
__weak void LL_CAN_WakeUpFromRxMsgCallback(LL_CAN_Handler_t *hcan)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(&hcan);

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
__weak void LL_CAN_ErrorCallback(LL_CAN_Handler_t *hcan)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(&hcan);

	/* NOTE : This function Should not be modified, when the callback is needed,
			  the LL_CAN_ErrorCallback could be implemented in the user file
	 */
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
void LL_CAN_IRQHandler(LL_CAN_Handler_t *hcan)
{

	// Check CAN instance is CAN1 or CAN2
	if (hcan->Instance == _CAN1)
	{
		canbase = _CAN1_REG_BASE;
	}
	else
	{
		canbase = _CAN2_REG_BASE;
	}
	uint32_t errorcode = LL_CAN_ERROR_NONE;
	uint32_t interrupts = (canbase->CAN_IER);
	uint32_t msrflags = (canbase->CAN_MSR);
	uint32_t tsrflags = (canbase->CAN_TSR);
	uint32_t rf0rflags = (canbase->CAN_RF0R);
	uint32_t rf1rflags = (canbase->CAN_RF1R);
	uint32_t esrflags = (canbase->CAN_ESR);

	/* Transmit Mailbox empty interrupt management *****************************/
	if ((interrupts & (1U << TMEIE)) != 0U)
	{
		/* Transmit Mailbox 0 management *****************************************/
		if ((tsrflags & (1U << RQCP0)) != 0U)
		{
			/* Clear the Transmission Complete flag (and TXOK0,ALST0,TERR0 bits) */
			(canbase->CAN_TSR) |= (1U << RQCP0);

			if ((tsrflags & (1U << TXOK0)) != 0U)
			{
				/* Transmission Mailbox 0 complete callback */
				LL_CAN_TxMailbox0CompleteCallback(hcan);
			}
			else
			{
				if ((tsrflags & (1U << ALST0)) != 0U)
				{
					/* Update error code */
					errorcode |= LL_CAN_ERROR_TX_ALST0;
				}
				else if ((tsrflags & (1U << TERR0)) != 0U)
				{
					/* Update error code */
					errorcode |= LL_CAN_ERROR_TX_TERR0;
				}
				else
				{
					/* Transmission Mailbox 0 abort callback */
					LL_CAN_TxMailbox0AbortCallback(hcan);
				}
			}
		}

		/* Transmit Mailbox 1 management *****************************************/
		if ((tsrflags & (1U << RQCP1)) != 0U)
		{
			/* Clear the Transmission Complete flag (and TXOK1,ALST1,TERR1 bits) */
			tsrflags |= (1U << RQCP1);

			if ((tsrflags & (1U << TXOK1)) != 0U)
			{
				/* Transmission Mailbox 1 complete callback */
				LL_CAN_TxMailbox1CompleteCallback(hcan);
			}
			else
			{
				if ((tsrflags & (1U << ALST1)) != 0U)
				{
					/* Update error code */
					errorcode |= LL_CAN_ERROR_TX_ALST1;
				}
				else if ((tsrflags & CAN_TSR_TERR1) != 0U)
				{
					/* Update error code */
					errorcode |= LL_CAN_ERROR_TX_TERR1;
				}
				else
				{
					/* Transmission Mailbox 1 abort callback */
					LL_CAN_TxMailbox1AbortCallback(hcan);
				}
			}
		}

		/* Transmit Mailbox 2 management *****************************************/
		if ((tsrflags & (1U << RQCP2)) != 0U)
		{
			/* Clear the Transmission Complete flag (and TXOK2,ALST2,TERR2 bits) */
			tsrflags |= (1U << RQCP2);

			if ((tsrflags & (1U << TXOK2)) != 0U)
			{
				/* Transmission Mailbox 2 complete callback */
				LL_CAN_TxMailbox2CompleteCallback(hcan);
			}
			else
			{
				if ((tsrflags & (1U << ALST2)) != 0U)
				{
					/* Update error code */
					errorcode |= LL_CAN_ERROR_TX_ALST2;
				}
				else if ((tsrflags & (1U << TERR2)) != 0U)
				{
					/* Update error code */
					errorcode |= LL_CAN_ERROR_TX_TERR2;
				}
				else
				{
					/* Transmission Mailbox 2 abort callback */
					LL_CAN_TxMailbox2AbortCallback(hcan);
				}
			}
		}
	}

	/* Receive FIFO 0 overrun interrupt management *****************************/
	if ((interrupts & (1U << FOVIE0)) != 0U)
	{
		if ((rf0rflags & (1U << FOVR0)) != 0U)
		{
			/* Set CAN error code to Rx Fifo 0 overrun error */
			errorcode |= LL_CAN_ERROR_RX_FOV0;

			/* Clear FIFO0 Overrun Flag */
			(canbase->CAN_RF0R) &= ~(1U << FOVR0);
		}
	}

	/* Receive FIFO 0 full interrupt management ********************************/
	if ((interrupts & (1U << FFIE0)) != 0U)
	{
		if ((rf0rflags & (1U << FULL0)) != 0U)
		{
			/* Clear FIFO 0 full Flag */
			rf0rflags &= ~(1U << FULL0);

			/* Receive FIFO 0 full Callback */
			LL_CAN_RxFifo0FullCallback(hcan);
		}
	}

	/* Receive FIFO 0 message pending interrupt management *********************/
	if ((interrupts & (1U << FMPIE0)) != 0U)
	{
		/* Check if message is still pending */
		if ((rf0rflags & (3U << FMP0)) != 0U)
		{
			/* Receive FIFO 0 message pending Callback */
			LL_CAN_RxFifo0MsgPendingCallback(hcan);
		}
	}

	/* Receive FIFO 1 overrun interrupt management *****************************/
	if ((interrupts & (1U << FOVIE1)) != 0U)
	{
		if ((rf1rflags & (1U << FOVR1)) != 0U)
		{
			/* Set CAN error code to Rx Fifo 1 overrun error */
			errorcode |= LL_CAN_ERROR_RX_FOV1;

			/* Clear FIFO1 Overrun Flag */
			rf1rflags &= ~(1U << FOVR1);
		}
	}

	/* Receive FIFO 1 full interrupt management ********************************/
	if ((interrupts & (1U << FFIE1)) != 0U)
	{
		if ((rf1rflags & (1U << FULL1)) != 0U)
		{
			/* Clear FIFO 1 full Flag */
			rf1rflags &= ~(1U << FULL1);
			/* Receive FIFO 1 full Callback */
			LL_CAN_RxFifo1FullCallback(hcan);
		}
	}

	/* Receive FIFO 1 message pending interrupt management *********************/
	if ((interrupts & (1U << FMPIE1)) != 0U)
	{
		/* Check if message is still pending */
		if ((rf1rflags & (3U << FMP1)) != 0U)
		{
			/* Receive FIFO 1 message pending Callback */
			LL_CAN_RxFifo1MsgPendingCallback(hcan);
		}
	}

	/* Sleep interrupt management *********************************************/
	if ((interrupts & (1U << SLKIE)) != 0U)
	{
		if ((msrflags & (1U << SLAKI)) != 0U)
		{
			/* Clear Sleep interrupt Flag */
			msrflags &= ~(1U << SLAKI);

			/* Sleep Callback */
			LL_CAN_SleepCallback(hcan);
		}
	}

	/* WakeUp interrupt management *********************************************/
	if ((interrupts & (1U << WKUIE)) != 0U)
	{
		if ((msrflags & (WKUI)) != 0U)
		{
			/* Clear WakeUp Flag */
			msrflags &= ~(1U << WKUI);

			/* WakeUp Callback */
			LL_CAN_WakeUpFromRxMsgCallback(hcan);
		}
	}

	/* Error interrupts management *********************************************/
	if ((interrupts & (1U << ERRIE)) != 0U)
	{
		if ((msrflags & (1U << ERRI)) != 0U)
		{
			/* Check Error Warning Flag */
			if (((interrupts & (1U << EWGIE)) != 0U) &&
				((esrflags & (1U << EWGF)) != 0U))
			{
				/* Set CAN error code to Error Warning */
				errorcode |= LL_CAN_ERROR_EWG;

				/* No need for clear of Error Warning Flag as read-only */
			}

			/* Check Error Passive Flag */
			if (((interrupts & (1U << EPVIE)) != 0U) &&
				((esrflags & (1U << EPVF)) != 0U))
			{
				/* Set CAN error code to Error Passive */
				errorcode |= LL_CAN_ERROR_EPV;

				/* No need for clear of Error Passive Flag as read-only */
			}

			/* Check Bus-off Flag */
			if (((interrupts & (1U << BOFIE)) != 0U) &&
				((esrflags & (1U << BOFF)) != 0U))
			{
				/* Set CAN error code to Bus-Off */
				errorcode |= LL_CAN_ERROR_BOF;

				/* No need for clear of Error Bus-Off as read-only */
			}

			/* Check Last Error Code Flag */
			if (((interrupts & (1U << LECIE)) != 0U) &&
				((esrflags & (1U << LEC)) != 0U))
			{
				switch (esrflags & (7U << LEC))
				{
				case (1U << LEC):
					/* Set CAN error code to Stuff error */
					errorcode |= LL_CAN_ERROR_STF;
					break;
				case (2U << LEC):
					/* Set CAN error code to Form error */
					errorcode |= LL_CAN_ERROR_FOR;
					break;
				case ((2U << LEC) | (1u << LEC)):
					/* Set CAN error code to Acknowledgement error */
					errorcode |= LL_CAN_ERROR_ACK;
					break;
				case (4U << LEC):
					/* Set CAN error code to Bit recessive error */
					errorcode |= LL_CAN_ERROR_BR;
					break;
				case ((4U << LEC) | (1U << LEC)):
					/* Set CAN error code to Bit Dominant error */
					errorcode |= LL_CAN_ERROR_BD;
					break;
				case ((4U << LEC) | (2U << LEC)):
					/* Set CAN error code to CRC error */
					errorcode |= LL_CAN_ERROR_CRC;
					break;
				default:
					break;
				}

				/* Clear Last error code Flag */
				esrflags &= ~(7U << LEC);
			}
		}

		/* Clear ERRI Flag */
		msrflags &= ~(1U << ERRI);
	}

	/* Call the Error call Back in case of Errors */
	if (errorcode != LL_CAN_ERROR_NONE)
	{
		/* Update error code in handle */
		hcan->ErrorCode |= errorcode;

		LL_CAN_ErrorCallback(hcan);
	}
}
