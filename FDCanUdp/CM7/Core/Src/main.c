/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "lwip.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../../../Common/Inc/ipc_log.h"
#include "cm7_log.h"
#include "udp_server.h"
#include "../../../Common/Inc/wbc_protocol.h"
#include "lwip/netif.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* DUAL_CORE_BOOT_SYNC_SEQUENCE: Define for dual core boot synchronization    */
/*                             demonstration code based on hardware semaphore */
/* This define is present in both CM7/CM4 projects                            */
/* To comment when developping/debugging on a single core                     */
#define DUAL_CORE_BOOT_SYNC_SEQUENCE

#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern struct netif gnetif;   /* lwip.c */

/* Main-loop timing (DWT cycles, 400 MHz): worst pass and worst MX_LWIP_Process() since the
 * last [NET] line. Shows CPU stalls that let the 4 RX descriptors overflow (RBU). */
static uint32_t loopMaxCyc, lwipMaxCyc;

/* LED_UDP (LD2, PE1): network status
 *   off             Ethernet link down (no cable / PC port off)
 *   slow blink 1 Hz link up, no valid command packets
 *   fast blink 10Hz valid WBC command packets in the last 200 ms */
static void udpLedUpdate(void)
{
  static uint32_t lastToggleMs;
  uint32_t now = HAL_GetTick();
  if (!netif_is_link_up(&gnetif))
  {
    HAL_GPIO_WritePin(LED_UDP_GPIO_Port, LED_UDP_Pin, GPIO_PIN_RESET);
    return;
  }
  bool commands = udpStats.rxValid > 0U && (now - udpStats.lastRxMs) < 200U;
  uint32_t halfPeriod = commands ? 50U : 500U;
  if (now - lastToggleMs >= halfPeriod)
  {
    lastToggleMs = now;
    HAL_GPIO_TogglePin(LED_UDP_GPIO_Port, LED_UDP_Pin);
  }
}

/* Network status: immediately on link up/down, otherwise every 5 s. Shows where packets stop:
 * MAC receiver on (re), RX DMA running (rxDma), RX DMA flags (RBU = no free receive buffer,
 * RPS = receive process stopped, FBE = fatal bus error), and the MAC packet counters. */
static void netStatusUpdate(void)
{
  extern ETH_HandleTypeDef heth;   /* ethernetif.c */
  extern volatile uint32_t ethRxAllocFail, ethRxBadBuff;
  extern uint32_t ETH_RxPoolFree(void);
  static uint32_t lastPrintMs;
  static int lastLink = -1;
  uint32_t now = HAL_GetTick();
  int link = netif_is_link_up(&gnetif) ? 1 : 0;
  if (link == lastLink && now - lastPrintMs < 5000U) return;
  lastLink = link;
  lastPrintMs = now;
  uint32_t csr = ETH->DMACSR;
  extern volatile uint32_t lwipDiagTxMax, lwipDiagRxMax, lwipDiagTmrMax, lwipRxDmaKicks;
  CM7_LOG("[NET] link %s  rxUni %lu txGood %lu crcErr %lu  RBU %lu kicks %lu  pool %lu/12 allocFail %lu badBuff %lu  loop max %lu us (rx %lu, tmr %lu, tx %lu)  heth err 0x%lx",
          link ? "UP" : "DOWN", ETH->MMCRUPGR, ETH->MMCTPCGR, ETH->MMCRCRCEPR,
          (csr & ETH_DMACSR_RBU) ? 1UL : 0UL, lwipRxDmaKicks,
          ETH_RxPoolFree(), ethRxAllocFail, ethRxBadBuff,
          loopMaxCyc / 400U, lwipDiagRxMax / 400U, lwipDiagTmrMax / 400U, lwipDiagTxMax / 400U,
          heth.ErrorCode);
  loopMaxCyc = lwipMaxCyc = 0;
  lwipDiagTxMax = lwipDiagRxMax = lwipDiagTmrMax = 0;
}

/* UDP statistics every 5 s, only while commands are arriving or right after they stop */
static void udpStatsUpdate(void)
{
  static uint32_t lastPrintMs, lastRx;
  uint32_t now = HAL_GetTick();
  if (now - lastPrintMs < 5000U) return;
  lastPrintMs = now;
  if (udpStats.rxValid == lastRx && udpStats.rxBadSize == 0U && udpStats.rxBadMagic == 0U) return;
  lastRx = udpStats.rxValid;
  UDP_Server_PrintStats();
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  int32_t timeout;
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_0 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  /* Wait until CPU2 boots and enters in stop mode or timeout */
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  /* Gracefully continue on timeout — CM4 may be in standalone development mode */
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* Instruction cache on: code is never written by DMA, so it needs no maintenance.
   * The D-cache stays OFF (ETH buffers, SRAM4 mailboxes would need MPU/cache upkeep). */
  SCB_EnableICache();
  /* Clear the CM4 log buffer before CM4 is released */
  IPC_LogInit();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
HSEM notification */
/*HW semaphore Clock enable*/
__HAL_RCC_HSEM_CLK_ENABLE();
/*Take HSEM */
HAL_HSEM_FastTake(HSEM_ID_0);
/*Release HSEM in order to notify the CPU2(CM4)*/
HAL_HSEM_Release(HSEM_ID_0,0);
/* wait until CPU2 wakes up from stop mode */
timeout = 0xFFFF;
while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
if ( timeout < 0 )
{
Error_Handler();
}
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */
  /* Allocate D2 SRAM3 (0x30040000: Ethernet DMA descriptors, RX pool, LwIP heap) to CM7,
   * as ST's H7 LwIP examples do. */
  __HAL_RCC_D2SRAM3_CLK_ENABLE();

  /* Initialise all of SRAM3 with 32-bit writes. Every H7 SRAM has ECC and holds random
   * data + ECC after power-up; an 8/16-bit write does read-modify-write and faults
   * (imprecise bus error) on a word that was never written in full. The Ethernet
   * section is NOLOAD, so the startup code does not clear it, and lwIP writes 16-bit
   * pbuf fields into each newly used RX buffer. Must run before MX_LWIP_Init(). */
  for (volatile uint32_t* w = (volatile uint32_t*)0x30040000UL; w < (volatile uint32_t*)0x30048000UL; w++)
    *w = 0U;
  __DSB();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_LWIP_Init();
  /* USER CODE BEGIN 2 */
  CM7_LOG("");
  CM7_LOG("=== CM7 up: SYSCLK %lu Hz, forwarding CM4 log ===", HAL_RCC_GetSysClockFreq());
  if (UDP_Server_Init())
    CM7_LOG("[UDP] listening on 192.168.10.10:%u (command %u B, state %u B)",
            WBC_UDP_PORT, (unsigned)sizeof(WbcCommandPacket), (unsigned)sizeof(WbcStatePacket));
  else
    CM7_LOG("[UDP] ERROR: could not open port %u", WBC_UDP_PORT);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t t0 = DWT->CYCCNT;
    MX_LWIP_Process();   /* Ethernet RX (UDP callbacks), LwIP timers, link state */
    uint32_t t1 = DWT->CYCCNT;
    udpLedUpdate();
    udpStatsUpdate();
    netStatusUpdate();
    CM7_LogPump();       /* one log byte per pass, never blocks */
    uint32_t t2 = DWT->CYCCNT;
    if (t1 - t0 > lwipMaxCyc) lwipMaxCyc = t1 - t0;
    if (t2 - t0 > loopMaxCyc) loopMaxCyc = t2 - t0;
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 200;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
  PeriphClkInitStruct.PLL2.PLL2M = 2;
  PeriphClkInitStruct.PLL2.PLL2N = 200;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 10;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_2;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  static const char msg[] = "\r\n[CM7] Error_Handler\r\n";
  if (huart3.gState != HAL_UART_STATE_RESET)
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, sizeof(msg) - 1U, 100U);
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
