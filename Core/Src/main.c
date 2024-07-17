/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f4_can_ll.h"
#include <string.h>
// #include "CAN_GPIO.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
LL_CAN_Handler_t hcan1;
LL_CAN_FilterTypeDef_t hfilter1;
LL_CAN_TxHeaderTypeDef_t Txheader;
LL_CAN_RxHeaderTypeDef_t Rxheader;
uint8_t data[8] = {0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x80};
uint8_t data1[8] = {11, 12, 13, 14, 15, 11, 12, 13};
uint8_t rxdata[8];
char msg[70];

uint32_t TxMailBox;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void Anti_WDG();
void Can_Rx();
void Can_Tx();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  hcan1.Instance = _CAN1;

  LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_5);

  if (LL_CAN_GPIO_Init(&hcan1) == ERROR)
  {
    sprintf(msg, "GPIO initialization fail\n");
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, 50, 1000);
  }
  else
  {
    sprintf(msg, "GPIO initialization successfully\n");
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, 50, 1000);
  }

  //  // Config NVIC
  NVIC_SetPriority(CAN1_TX_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 15, 0));
  NVIC_SetPriority(CAN1_RX0_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 15, 0));
  NVIC_EnableIRQ(CAN1_RX0_IRQn);
  NVIC_EnableIRQ(CAN1_TX_IRQn);

  // Enable interrupt
  LL_CAN_ActivateInterrupt(&hcan1, _CAN_IT_RX_FIFO0_MSG_PENDING_Pos | _CAN_IT_TX_MAILBOX_EMPTY_Pos);

  // Set flag to
  hcan1.Init.Prescaler = 2;
  hcan1.Init.SyncJumpWidth = _CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = _CAN_BS1_10TQ;
  hcan1.Init.TimeSeg2 = _CAN_BS2_1TQ;
  hcan1.Init.Mode = _NORMAL_MODE;
  hcan1.Init.status.AutoBusOff = DISABLE;
  hcan1.Init.status.AutoRetransmission = ENABLE;
  hcan1.Init.status.AutoWakeUp = DISABLE;
  hcan1.Init.status.ReceiveFifoLocked = DISABLE;
  hcan1.Init.status.TimeTriggeredMode = DISABLE;
  hcan1.Init.status.TransmitFifoPriority = DISABLE;

  if (LL_CAN_Init(&hcan1) == ERROR)
  {
    sprintf(msg, "Can initialization fail\n");
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 1000);
  }
  else
  {
    sprintf(msg, "CAN initialization successfully\n");
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 1000);
  }

  // Set up config filter
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

  // Enable interrupt
  LL_CAN_ActivateInterrupt(&hcan1, _CAN_IT_RX_FIFO0_MSG_PENDING_Pos | _CAN_IT_TX_MAILBOX_EMPTY_Pos);

  // Start Can
  if (LL_CAN_Start(&hcan1) == ERROR)
  {
    sprintf(msg, "Can start fail\n");
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 1000);
  }
  else
  {
    sprintf(msg, "CAN start successfully\n");
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 1000);
  }

  Txheader.StdId = 135;
  Txheader._DLC = 6;
  Txheader._RTR = _CAN_RTR_DATA;
  Txheader._IDE = _CAN_ID_STD;
  Txheader.TransmitGlobalTime = DISABLE;

  /* USER CODE END 2 */

  Can_Tx();
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */



//    Anti_WDG();
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
  while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_1)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_RCC_HSE_Enable();

  /* Wait till HSE is ready */
  while (LL_RCC_HSE_IsReady() != 1)
  {
  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_4, 192, LL_RCC_PLLP_DIV_4);
  LL_RCC_PLL_Enable();

  /* Wait till PLL is ready */
  while (LL_RCC_PLL_IsReady() != 1)
  {
  }
  while (LL_PWR_IsActiveFlag_VOS() == 0)
  {
  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_2);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

  /* Wait till System clock is ready */
  while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  }
  LL_SetSystemCoreClock(48000000);

  /* Update the time base */
  if (HAL_InitTick(TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOH);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);

  /**/
  LL_GPIO_ResetOutputPin(Anti_WDG_GPIO_Port, Anti_WDG_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED_GPIO_Port, LED_Pin);

  /**/
  GPIO_InitStruct.Pin = Anti_WDG_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(Anti_WDG_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void Anti_WDG()
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 1);
  HAL_Delay(200);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0);
  HAL_Delay(1000);
}

void Can_Rx()
{
  if (LL_CAN_GetRxFifoFillLevel(&hcan1, _CAN_RX_FIFO0) != 0)
  {
    if (LL_CAN_GetRxMessage(&hcan1, &Rxheader, rxdata, _CAN_RX_FIFO0) == ERROR)
    {
      sprintf(msg, "Receive Fail\n");
      HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 1000);
    }
  }
}
void Can_Tx()
{
  if (LL_CAN_AddTxMessage(&hcan1, data1, &Txheader, &TxMailBox) == ERROR)
  {
    //          if (LL_CAN_IsTxMessagePending(&hcan1, &TxMailBox) == SUCCESS)
    //          {
    //            LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_5);
    //          }
    sprintf(msg, "Transmit Fail\n");
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 1000);
  }
}

void LL_CAN_TxMailbox0CompleteCallback(LL_CAN_Handler_t *hcan)
{
  sprintf(msg, "Transmit Successfully M0\n");
  HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 1000);
}
void LL_CAN_TxMailbox1CompleteCallback(LL_CAN_Handler_t *hcan)
{
  sprintf(msg, "Transmit Successfully M1\n");
  HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 1000);
}
void LL_CAN_TxMailbox2CompleteCallback(LL_CAN_Handler_t *hcan)
{
  sprintf(msg, "Transmit Successfully M2\n");
  HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 1000);
}
void LL_CAN_RxFifo0MsgPendingCallback(LL_CAN_Handler_t *hcan)
{
  if (LL_CAN_GetRxMessage(&hcan1, &Rxheader, rxdata, _CAN_RX_FIFO0) == ERROR)
  {
    sprintf(msg, "Receive Fail\n");
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 1000);
  }
  else
  {
	  if (LL_CAN_AddTxMessage(&hcan1, rxdata, &Txheader, &TxMailBox) == ERROR)
	  {
	    sprintf(msg, "Receive and Transmit Successfully\n");
	    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 1000);
	  }
    sprintf(msg, "string\n");
//    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 1000);
    for (int i = 0; i < Rxheader._DLC; i++)
    {
    	HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 1000);
//    	HAL_UART_Transmit(&huart2, (uint8_t *)rxdata, 8, 1000);
    }
    sprintf(msg, "Receive Successfully\n");
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 1000);
    LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_5);
  }
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
