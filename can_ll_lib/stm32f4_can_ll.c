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
static ErrorStatus Trans(CAN_TypeDef_t *canbase, const uint8_t data[], LL_CAN_TxHeaderTypeDef *htxheader);

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
ErrorStatus LL_CAN_Init(LL_CAN_TypeDef can_type, LL_CAN_InitTypeDef *hcan)
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

	// Check if can instance is can1 or can2
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
	(canbase->CAN_MCR) |= (0x01 << INRQ);

	// Wait until INAK bit has been set by hardware.
	while (!((canbase->CAN_MSR) & (1UL << INAK)) && (time_start < time_out))
	{
		++time_start;
	}
	if (time_start > time_out)
		return ERROR;
	// Reset time start
	time_start = 0;

	// Exit sleep mode
	(canbase->CAN_MCR) &= ~(0x01 << SLEEP);

	// Wait until bit SLAK=0 of MSR register
	while (!((canbase->CAN_MSR) & (1UL << SLAK)) && (time_start < time_out))
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
	switch (hcan->Baudrate)
	{
	case _BAUDRATE_500:
		(canbase->CAN_BTR) |= ((0x01 << SJW) | (0x02 << BRP) | (0x0C << TS1) | (0x01 << TS2));
		break;
	case _BAUDRATE_800:
		(canbase->CAN_BTR) |= ((0x01 << SJW) | (0x01 << BRP) | (0x0B << TS1) | (0x01 << TS2));
		break;
	case _BAUDRATE_1000:
		(canbase->CAN_BTR) |= ((0x01 << SJW) | (0x01 << BRP) | (0x09 << TS1) | (0x00 << TS2));
		break;
	}

	// Can Autowakeup, default value of bit AWUM=0
	if (hcan->status.AutoWakeUp == ENABLE)
		(canbase->CAN_MCR) |= (1UL << AWUM);
	else
		(canbase->CAN_MCR) &= ~(1UL << AWUM);

	// Can No Autorestranmission, default value of bit NART=0
	if (hcan->status.AutoRetransmission == DISABLE)
		(canbase->CAN_MCR) |= (1UL << NART);
	else
		(canbase->CAN_MCR) &= (1UL << NART);

	// Can ReceiveFifoLocked, default value of bit RFLM=0
	if (hcan->status.ReceiveFifoLocked == ENABLE)
		(canbase->CAN_MCR) |= (1UL << RFLM);
	else
		(canbase->CAN_MCR) &= ~(1UL << RFLM);

	// Can Time trigger mode, default value of bit TTCM=0
	if (hcan->status.TimeTriggeredMode == ENABLE)
		(canbase->CAN_MCR) |= (1UL << TTCM);
	else
		(canbase->CAN_MCR) &= ~(1UL << TTCM);

	// Can TransmitFifoPriority, default value of bit TXFP=0
	if (hcan->status.TransmitFifoPriority == ENABLE)
		(canbase->CAN_MCR) |= (1UL << TXFP);
	else
		(canbase->CAN_MCR) &= ~(1UL << TXFP);

	// Can AutoBusOff, default value of bit ABOM=0
	if (hcan->status.AutoBusOff == ENABLE)
		canbase->CAN_MCR |= (1UL << ABOM);
	else
		(canbase->CAN_MCR) &= ~(1UL << ABOM);

	(canbase->CAN_MCR) |= (0x01 << DBF);
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
ErrorStatus LL_CAN_ConfigFilter(LL_CAN_TypeDef can_type, LL_CAN_FilterTypeDef *hfilter)
{
	ErrorStatus status = ERROR;
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

	// Check parameters
	assert_param(hfilter->FilterActivation);
	assert_param(hfilter->FilterBank);
	assert_param(hfilter->FilterFIFOAssignment);
	assert_param(hfilter->FilterID);
	assert_param(hfilter->FilterMask);
	assert_param(hfilter->FilterMode);
	assert_param(hfilter->FilterScale);
	assert_param(hfilter->SlaveStartFilterBank);

	// Don`t have SlaveStartFilterBank configuration

	// Init filter
	*((uint32_t *)(canbase) + CAN_FMR) |= (0x01 << FINIT);

	// Config filter mode
	*((uint32_t *)(canbase) + CAN_FM1R) |= (hfilter->FilterMode << hfilter->FilterBank);

	// Config filter scale
	(*((uint32_t *)(canbase) + CAN_FS1R) |= (hfilter->FilterScale << hfilter->FilterBank));

	// Config filter FIFO assignment
	(*((uint32_t *)(canbase) + CAN_FFA1R) |= (hfilter->FilterFIFOAssignment << hfilter->FilterBank));

	// Active or deactive filter
	(*((uint32_t *)(canbase) + CAN_FA1R) |= (hfilter->FilterActivation << hfilter->FilterBank));

	// Config indentifier for filter
	(*((uint32_t *)(canbase) + CAN_F0R1 + (0x08 * hfilter->FilterBank)) |= (hfilter->FilterID));

	// Condfig mask for filter
	(*((uint32_t *)(canbase) + CAN_F0R2 + (0x08 * hfilter->FilterBank)) |= (hfilter->FilterMask));

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
ErrorStatus LL_CAN_Start(LL_CAN_TypeDef can_type)
{
	ErrorStatus status = ERROR;
	uint8_t time_out = 50;
	uint8_t time_start = 0;
	// Leave Initialization mode
	(canbase->CAN_MCR) &= ~(0x01 << INRQ);
	// Wait until INAK bit has been cleared by hardware.
	while (((canbase->CAN_MSR) & (0x01 << INAK)) && (time_start < time_out))
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
ErrorStatus LL_CAN_Transmit(LL_CAN_TypeDef can_type, const uint8_t data[], LL_CAN_TxHeaderTypeDef *htxheader)
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
	if (((canbase->CAN_TSR) & (0x01 << TME0)) || ((canbase->CAN_TSR) & (0x01 << TME1)) || ((canbase->CAN_TSR) & (0x01 << TME2)))
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
static ErrorStatus Trans(CAN_TypeDef_t *canbase, const uint8_t data[], LL_CAN_TxHeaderTypeDef *htxheader)
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

	//	// Check transmission error of mailbox and arbitration lost for mailbox
	//	switch (transmitmailbox)
	//	{
	//	case 0:
	//		if ((canbase->CAN_TSR & (1UL << TERR0)) || (canbase->CAN_TSR & (1UL << ALST0)))
	//		{
	//			return ERROR;
	//		}
	//		break;
	//	case 1:
	//		if ((canbase->CAN_TSR & (1UL << TERR1)) || (canbase->CAN_TSR & (1UL << ALST1)))
	//		{
	//			return ERROR;
	//		}
	//		break;
	//	case 2:
	//		if ((canbase->CAN_TSR & (1UL << TERR2)) || (canbase->CAN_TSR & (1UL << ALST2)))
	//		{
	//			return ERROR;
	//		}
	//		break;
	//	}

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
