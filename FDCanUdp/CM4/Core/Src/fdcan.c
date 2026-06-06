/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fdcan.c
  * @brief   This file provides code for the configuration
  *          of the FDCAN instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "fdcan.h"

/* USER CODE BEGIN 0 */
#include "mab_can.h"

/* TX one ping frame, wait up to 2 ms for a response in FIFO0. */
static errorMab_E fdcan1_pingTxRx(uint32_t    driverId,
                                   const void* txBuf,
                                   size_t      txSize,
                                   void*       rxBuf,
                                   size_t      rxSize)
{
    (void)txSize;
    (void)rxSize;

    FDCAN_TxHeaderTypeDef txHeader;
    txHeader.Identifier          = driverId;
    txHeader.IdType              = FDCAN_STANDARD_ID;
    txHeader.TxFrameType         = FDCAN_DATA_FRAME;
    txHeader.DataLength          = FDCAN_DLC_BYTES_6;
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHeader.BitRateSwitch       = FDCAN_BRS_ON;
    txHeader.FDFormat            = FDCAN_FD_CAN;
    txHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    txHeader.MessageMarker       = 0;

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, (uint8_t*)txBuf) != HAL_OK)
        return ERROR_OPERATION_FAILED;

    uint32_t tickstart = HAL_GetTick();
    while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) == 0)
    {
        if ((HAL_GetTick() - tickstart) >= 2U)
            return ERROR_OPERATION_FAILED;
    }

    FDCAN_RxHeaderTypeDef rxHeader;
    if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rxHeader, (uint8_t*)rxBuf) != HAL_OK)
        return ERROR_OPERATION_FAILED;

    return OK;
}
/* USER CODE END 0 */

FDCAN_HandleTypeDef hfdcan1;

/* FDCAN1 init function */
void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 1;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 39;
  hfdcan1.Init.NominalTimeSeg2 = 10;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 2;
  hfdcan1.Init.DataTimeSeg1 = 7;
  hfdcan1.Init.DataTimeSeg2 = 2;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 8;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.RxFifo0ElmtsNbr = 16;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_64;
  hfdcan1.Init.RxFifo1ElmtsNbr = 0;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 0;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 0;
  hfdcan1.Init.TxBuffersNbr = 0;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 16;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_64;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */
  /* Accept all standard frames into FIFO0 (mask=0 → any ID matches) */
  FDCAN_FilterTypeDef sFilterConfig;
  sFilterConfig.IdType       = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex  = 0;
  sFilterConfig.FilterType   = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1    = 0x000U;
  sFilterConfig.FilterID2    = 0x000U;
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
    Error_Handler();

  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    Error_Handler();
  /* USER CODE END FDCAN1_Init 2 */

}

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspInit 0 */

  /* USER CODE END FDCAN1_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* FDCAN1 clock enable */
    __HAL_RCC_FDCAN_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**FDCAN1 GPIO Configuration
    PB8     ------> FDCAN1_RX
    PB9     ------> FDCAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* FDCAN1 interrupt Init */
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
  /* USER CODE BEGIN FDCAN1_MspInit 1 */

  /* USER CODE END FDCAN1_MspInit 1 */
  }
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspDeInit 0 */

  /* USER CODE END FDCAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_FDCAN_CLK_DISABLE();

    /**FDCAN1 GPIO Configuration
    PB8     ------> FDCAN1_RX
    PB9     ------> FDCAN1_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);

    /* FDCAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
  /* USER CODE BEGIN FDCAN1_MspDeInit 1 */

  /* USER CODE END FDCAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
static const uint32_t sLeftLegIds[] = {10U, 20U, 30U, 40U, 50U, 60U};

uint8_t FDCAN1_PingMotors(void)
{
    /* Flush any stale messages that arrived before the scan */
    FDCAN_RxHeaderTypeDef dummyRxHeader;
    uint8_t               dummyData[8];
    while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0)
        HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &dummyRxHeader, dummyData);

    uint8_t txBuf[PING_FRAME_SIZE];
    uint8_t rxBuf[PING_FRAME_SIZE];
    uint8_t foundCount = 0U;

    if (pingPrepareDataFrame(txBuf, sizeof(txBuf)) != OK)
        return 0U;

    for (size_t i = 0; i < sizeof(sLeftLegIds) / sizeof(sLeftLegIds[0]); i++)
    {
        if (fdcan1_pingTxRx(sLeftLegIds[i], txBuf, sizeof(txBuf), rxBuf, sizeof(rxBuf)) == OK)
        {
            if (pingParseResponse(rxBuf, sizeof(rxBuf)) == OK)
                foundCount++;
        }
    }

    return foundCount;
}
/* USER CODE END 1 */

