# CAN LL Library-STM32F4

## Define can handler, filter and header

```
    LL_CAN_Handler_t hcan;
    LL_CAN_FilterTypeDef_t hfilter;
    LL_CAN_TxHeaderTypeDef_t Txheader;
    LL_CAN_RxHeaderTypeDef_t Rxheader;
```
> [!IMPORTANT]
> Define can instance first

```
    hcan.Instance = _CAN1 
```

or 

```
    hcan.Instance = _CAN2
```

## Can initialization

***1. GPIO initialization***

```
    LL_CAN_GPIO_Init(&hcan);
```

**2. Enable IRQ**

Example:

```
    NVIC_SetPriority(CAN1_TX_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 15, 0));
    NVIC_EnableIRQ(CAN1_TX_IRQn);
```

***3. Enable interruption flag***

Example:

```
    LL_CAN_ActivateInterrupt(&hcan, _CAN_IT_RX_FIFO0_MSG_PENDING_Pos | _CAN_IT_TX_MAILBOX_EMPTY_Pos);
```

***4. Can parameters initialization (baudrate,mode, status)***

```
    hcan.Init.Prescaler = 2;
    hcan.Init.SyncJumpWidth = _CAN_SJW_1TQ;
    hcan.Init.TimeSeg1 = _CAN_BS1_10TQ;
    hcan.Init.TimeSeg2 = _CAN_BS2_1TQ;
    hcan.Init.Mode = _LOOPBACK_MODE;
    hcan.Init.status.AutoBusOff = DISABLE;
    hcan.Init.status.AutoRetransmission = ENABLE;
    hcan.Init.status.AutoWakeUp = DISABLE;
    hcan.Init.status.ReceiveFifoLocked = DISABLE;
    hcan.Init.status.TimeTriggeredMode = DISABLE;
    hcan.Init.status.TransmitFifoPriority = DISABLE;

    LL_CAN_Init(&hcan);
```
***5. Can filter configuration***

```
    hfilter1.FilterActivation = _CAN_FILTER_ENABLE;
    hfilter1.FilterBank = 0;
    hfilter1.FilterFIFOAssignment = _CAN_FILTER_FIFO0;
    hfilter1.FilterIdHigh = 0;
    hfilter1.FilterIdLow = 0;
    hfilter1.FilterMaskIdHigh = 0;
    hfilter1.FilterMaskIdLow = 0;
    hfilter1.FilterMode = _CAN_FILTERMODE_IDMASK;
    hfilter1.FilterScale = _CAN_FILTERSCALE_32BIT;

    LL_CAN_ConfigFilter(&hcan1, &hfilter1);
```

> [!IMPORTANT]
> Enable filter or you can`t use reception mode.

## Can start

```
    LL_CAN_Start(&hcan);
```

## Can transmit and receive functions

***1. Transmission***

```
    LL_CAN_AddTxMessage(LL_CAN_Handler_t *hcan, const uint8_t data[], LL_CAN_TxHeaderTypeDef_t *htxheader, uint32_t *TxMailBox);

    LL_CAN_IsTxMessagePending(LL_CAN_Handler_t *hcan, uint32_t *TxMailBox);
```
> [!NOTE]   
> If you use intterupt, just invoke the second line.

***2. Reception***

```
    LL_CAN_GetRxFifoFillLevel(LL_CAN_Handler_t *hcan, uint32_t RxFifo);

    LL_CAN_GetRxMessage(LL_CAN_Handler_t *hcan, LL_CAN_RxHeaderTypeDef_t *hrxheader, uint8_t rxdata[], uint32_t RxFifo);
```

> [!NOTE]  
> If you use intterupt, just invoke the second line.