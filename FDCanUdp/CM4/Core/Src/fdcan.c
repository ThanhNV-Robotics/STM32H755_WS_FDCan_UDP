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

/*
 * fdcan1_pingTxRx — low-level CAN send + receive for one motor probe.
 *
 * Sends 'txBuf' (a 6-byte MAB ping request) to the motor at 'driverId',
 * then waits up to 2 ms for the motor to reply into 'rxBuf'.
 * Returns OK if a reply arrived, ERROR_OPERATION_FAILED otherwise.
 *
 * This function matches the pingTxRxCb_F callback signature in mab_can.h.
 */
static errorMab_E fdcan1_pingTxRx(uint32_t    driverId,
                                   const void* txBuf,
                                   size_t      txSize,
                                   void*       rxBuf,
                                   size_t      rxSize)
{
    /* txSize / rxSize are always PING_FRAME_SIZE (6); suppress unused warnings. */
    (void)txSize;
    (void)rxSize;

    /* ------------------------------------------------------------------
     * Fill the transmit frame header.
     * Every FDCAN frame needs this descriptor on top of the data bytes.
     * ------------------------------------------------------------------ */
    FDCAN_TxHeaderTypeDef txHeader;

    /* The motor's CAN address (11-bit number, e.g. 10, 20 … 60). */
    txHeader.Identifier = driverId;

    /* FDCAN_STANDARD_ID = 11-bit address scheme (classic CAN style).
     * The alternative is FDCAN_EXTENDED_ID = 29-bit; MD80 uses 11-bit. */
    txHeader.IdType = FDCAN_STANDARD_ID;

    /* DATA_FRAME means the frame carries payload bytes.
     * REMOTE_FRAME is a request-without-data, not used here. */
    txHeader.TxFrameType = FDCAN_DATA_FRAME;

    /* Number of payload bytes to send.
     * FDCAN_DLC_BYTES_6 = 6 bytes, matching PING_FRAME_SIZE. */
    txHeader.DataLength = FDCAN_DLC_BYTES_6;

    /* ESI_ACTIVE tells the receiver this node is error-active (healthy).
     * The motor will ignore frames from a node in error-passive state. */
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;

    /* BRS_ON = Bit Rate Switch ON.
     * The arbitration header is sent at 1 Mbps (nominal), then the
     * payload bytes switch to 5 Mbps (data rate) — that is the "FD" advantage. */
    txHeader.BitRateSwitch = FDCAN_BRS_ON;

    /* FDCAN_FD_CAN = use the CAN-FD frame format, which supports BRS
     * and payloads up to 64 bytes.  Classic CAN would be FDCAN_CLASSIC_CAN. */
    txHeader.FDFormat = FDCAN_FD_CAN;

    /* Do not record this transmission in the TX Event FIFO.
     * TX events are a logging mechanism; we don't need them for a simple ping. */
    txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;

    /* An 8-bit user tag copied into the TX event log entry.
     * Unused because TX events are disabled above. */
    txHeader.MessageMarker = 0;

    /* ------------------------------------------------------------------
     * Transmit: hand the frame to the FDCAN hardware TX FIFO.
     * The peripheral sends it automatically once the CAN bus is free.
     * ------------------------------------------------------------------ */
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, (uint8_t*)txBuf) != HAL_OK)
        return ERROR_OPERATION_FAILED;

    /* ------------------------------------------------------------------
     * Wait for the motor's reply to appear in RX FIFO0 (timeout = 2 ms).
     *
     * HAL_GetTick() returns the SysTick counter in milliseconds (1 ms resolution).
     * The FDCAN hardware stores every accepted incoming frame in FIFO0
     * automatically — no interrupt or DMA needed here.
     *
     * 2 ms is generous:  arbitration header ≈ 0.05 ms,  data phase ≈ 0.01 ms,
     * motor processing + reply ≈ 0.1 ms.  If nothing arrives within 2 ms,
     * the motor is absent or the bus cable is unplugged.
     * ------------------------------------------------------------------ */
    uint32_t tickstart = HAL_GetTick();
    while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) == 0)
    {
        /* GetRxFifoFillLevel returns how many frames are waiting in FIFO0.
         * We loop until at least one frame arrives, or the 2 ms window expires. */
        if ((HAL_GetTick() - tickstart) >= 2U)
            return ERROR_OPERATION_FAILED; /* timeout — motor did not answer */
    }

    /* ------------------------------------------------------------------
     * Pop one frame out of FIFO0.
     * rxHeader will hold metadata (sender ID, DLC, timestamp …).
     * rxBuf will hold the 6 data bytes of the motor's response.
     * ------------------------------------------------------------------ */
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

  /* ------------------------------------------------------------------
   * Configure an RX acceptance filter so the hardware knows which
   * incoming frames to keep (and routes them to FIFO0 for us to read).
   *
   * Without at least one filter, the FDCAN peripheral rejects everything.
   * ------------------------------------------------------------------ */
  FDCAN_FilterTypeDef sFilterConfig;

  /* This filter applies to standard (11-bit) CAN IDs. */
  sFilterConfig.IdType = FDCAN_STANDARD_ID;

  /* Filter slot index.  We have 8 slots (StdFiltersNbr = 8); use slot 0. */
  sFilterConfig.FilterIndex = 0;

  /* MASK mode: FilterID1 is the pattern to match,
   *            FilterID2 is the bitmask (1 = bit must match, 0 = don't care).
   * With ID2 = 0x000, every bit is "don't care" → accept ANY standard ID. */
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;

  /* Frames that match this filter are stored in RX FIFO0. */
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;

  /* Pattern = 0, Mask = 0  →  any_id & 0x000 == 0x000  →  always true.
   * This is an "accept all" filter.  We rely on pingParseResponse() to
   * validate the payload content instead of filtering by ID here. */
  sFilterConfig.FilterID1 = 0x000U;
  sFilterConfig.FilterID2 = 0x000U;

  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
    Error_Handler();

  /* Start the FDCAN peripheral — after this call the hardware is live on
   * the bus and can transmit/receive frames. */
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

/* CAN IDs of the six left-leg MD80 motor drivers. */
static const uint32_t sLeftLegIds[] = {10U, 20U, 30U, 40U, 50U, 60U};

/*
 * FDCAN1_PingMotors — probe the left-leg motors and return how many replied.
 *
 * Called once at startup (from main.c).  If the return value is 0 it means
 * no motor answered → the caller turns on LED_Red.
 */
uint8_t FDCAN1_PingMotors(void)
{
    /* ------------------------------------------------------------------
     * Flush any frames that may have accumulated in FIFO0 since boot
     * (e.g. bus noise, leftover init frames).  If we don't clear them,
     * the first fdcan1_pingTxRx() call would read a stale frame and
     * falsely count a motor as present.
     * ------------------------------------------------------------------ */
    FDCAN_RxHeaderTypeDef dummyRxHeader;
    uint8_t               dummyData[8];
    while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0)
        HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &dummyRxHeader, dummyData);

    /* PING_FRAME_SIZE = 6 bytes (defined in mab_can.h). */
    uint8_t txBuf[PING_FRAME_SIZE]; /* request frame to send to each motor */
    uint8_t rxBuf[PING_FRAME_SIZE]; /* buffer for the motor's reply        */
    uint8_t foundCount = 0U;

    /* ------------------------------------------------------------------
     * Build the ping request once — it is identical for every motor ID.
     *
     * pingPrepareDataFrame() fills txBuf with a "read QuickStatus register"
     * command (MAB CAN protocol, frame type 0x43, register 0x805, 6 bytes).
     * ------------------------------------------------------------------ */
    if (pingPrepareDataFrame(txBuf, sizeof(txBuf)) != OK)
        return 0U;

    /* ------------------------------------------------------------------
     * Iterate over the 6 left-leg motor IDs.
     *
     * sizeof(sLeftLegIds) / sizeof(sLeftLegIds[0]) is the standard C idiom
     * for "number of elements in an array" — here it evaluates to 6.
     * ------------------------------------------------------------------ */
    for (size_t i = 0; i < sizeof(sLeftLegIds) / sizeof(sLeftLegIds[0]); i++)
    {
        /* Send the ping and wait up to 2 ms for a CAN reply. */
        if (fdcan1_pingTxRx(sLeftLegIds[i], txBuf, sizeof(txBuf), rxBuf, sizeof(rxBuf)) == OK)
        {
            /* A frame arrived — verify it is a valid MAB CAN response
             * (checks that rxBuf[0] == 0x43, the expected frame-type byte). */
            if (pingParseResponse(rxBuf, sizeof(rxBuf)) == OK)
                foundCount++; /* this motor is alive and on the bus */
        }
        /* If fdcan1_pingTxRx returned an error (timeout), we simply move on
         * to the next ID without incrementing foundCount. */
    }

    return foundCount; /* 0 = no motors found, 1–6 = number of live motors */
}
/* USER CODE END 1 */

