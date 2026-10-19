/* USER CODE BEGIN Header */
/**
 ******************************************************************************
  * File Name          : LWIP.c
  * Description        : This file provides initialization code for LWIP
  *                      middleWare.
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
#include "lwip.h"
#include "lwip/init.h"
#include "lwip/netif.h"
#if defined ( __CC_ARM )  /* MDK ARM Compiler */
#include "lwip/sio.h"
#endif /* MDK ARM Compiler */
#include "ethernetif.h"

/* USER CODE BEGIN 0 */
/* TX bookkeeping fix for the CubeMX NO_SYS port.
 * low_level_output() sends with the blocking HAL_ETH_Transmit(). That function never sets
 * TxDescList.CurrentPacketAddress (only HAL_ETH_Transmit_IT() does), so PacketAddress[] stays
 * NULL, HAL_ETH_ReleaseTxPacket() can never release anything, and ETH_Prepare_Tx_Descriptors()
 * adds +1 to TxDescList.BuffersInUse for every frame. Calling ReleaseTxPacket() then loops
 * BuffersInUse times per call (measured: one send grew from 20 us to several ms, then stalls of
 * seconds). The blocking send has finished with its descriptor when it returns, so reset the
 * counter instead. HAL_ETH_TxFreeCallback (ethernetif.c) stays a no-op: lwIP frees the pbuf. */
extern ETH_HandleTypeDef heth;   /* ethernetif.c */
static netif_linkoutput_fn sLowLevelOutput;

/* DIAG: worst-case cycle counts (400 MHz), printed and reset by CM7 main.c */
volatile uint32_t lwipDiagTxMax, lwipDiagRxMax, lwipDiagTmrMax;
volatile uint32_t lwipRxDmaKicks;   /* RX DMA restarts after "receive buffer unavailable" */
static uint32_t sDiagT0;

static err_t linkOutputAndRelease(struct netif *netif, struct pbuf *p)
{
  uint32_t t = DWT->CYCCNT;
  err_t err = sLowLevelOutput(netif, p);   /* blocking: returns when the DMA has sent it */
  heth.TxDescList.BuffersInUse = 0U;
  t = DWT->CYCCNT - t;
  if (t > lwipDiagTxMax) lwipDiagTxMax = t;
  return err;
}
/* USER CODE END 0 */
/* Private function prototypes -----------------------------------------------*/
static void ethernet_link_status_updated(struct netif *netif);
static void Ethernet_Link_Periodic_Handle(struct netif *netif);
/* ETH Variables initialization ----------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
uint32_t EthernetLinkTimer;

/* Variables Initialization */
struct netif gnetif;
ip4_addr_t ipaddr;
ip4_addr_t netmask;
ip4_addr_t gw;
uint8_t IP_ADDRESS[4];
uint8_t NETMASK_ADDRESS[4];
uint8_t GATEWAY_ADDRESS[4];

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */

/**
  * LwIP initialization function
  */
void MX_LWIP_Init(void)
{
  /* IP addresses initialization */
  IP_ADDRESS[0] = 192;
  IP_ADDRESS[1] = 168;
  IP_ADDRESS[2] = 10;
  IP_ADDRESS[3] = 10;
  NETMASK_ADDRESS[0] = 255;
  NETMASK_ADDRESS[1] = 255;
  NETMASK_ADDRESS[2] = 255;
  NETMASK_ADDRESS[3] = 0;
  GATEWAY_ADDRESS[0] = 192;
  GATEWAY_ADDRESS[1] = 168;
  GATEWAY_ADDRESS[2] = 10;
  GATEWAY_ADDRESS[3] = 1;

/* USER CODE BEGIN IP_ADDRESSES */
  /* Direct link to the PC (see NOTES.md, UDP design): set here so CubeMX input-mask
   * typos cannot change it. Nucleo 192.168.10.10/24, PC (gateway) 192.168.10.1. */
  IP_ADDRESS[0] = 192;      IP_ADDRESS[1] = 168;      IP_ADDRESS[2] = 10;      IP_ADDRESS[3] = 10;
  NETMASK_ADDRESS[0] = 255; NETMASK_ADDRESS[1] = 255; NETMASK_ADDRESS[2] = 255; NETMASK_ADDRESS[3] = 0;
  GATEWAY_ADDRESS[0] = 192; GATEWAY_ADDRESS[1] = 168; GATEWAY_ADDRESS[2] = 10;  GATEWAY_ADDRESS[3] = 1;
/* USER CODE END IP_ADDRESSES */

  /* Initialize the LwIP stack without RTOS */
  lwip_init();

  /* IP addresses initialization without DHCP (IPv4) */
  IP4_ADDR(&ipaddr, IP_ADDRESS[0], IP_ADDRESS[1], IP_ADDRESS[2], IP_ADDRESS[3]);
  IP4_ADDR(&netmask, NETMASK_ADDRESS[0], NETMASK_ADDRESS[1] , NETMASK_ADDRESS[2], NETMASK_ADDRESS[3]);
  IP4_ADDR(&gw, GATEWAY_ADDRESS[0], GATEWAY_ADDRESS[1], GATEWAY_ADDRESS[2], GATEWAY_ADDRESS[3]);

  /* add the network interface (IPv4/IPv6) without RTOS */
  netif_add(&gnetif, &ipaddr, &netmask, &gw, NULL, &ethernetif_init, &ethernet_input);

  /* Registers the default network interface */
  netif_set_default(&gnetif);

  /* We must always bring the network interface up connection or not... */
  netif_set_up(&gnetif);

  /* Set the link callback function, this function is called on change of link status*/
  netif_set_link_callback(&gnetif, ethernet_link_status_updated);

/* USER CODE BEGIN 3 */
  /* see USER CODE 0: keep the HAL TX bookkeeping from growing without bound */
  sLowLevelOutput = gnetif.linkoutput;
  gnetif.linkoutput = linkOutputAndRelease;
/* USER CODE END 3 */
}

#ifdef USE_OBSOLETE_USER_CODE_SECTION_4
/* Kept to help code migration. (See new 4_1, 4_2... sections) */
/* Avoid to use this user section which will become obsolete. */
/* USER CODE BEGIN 4 */
/* USER CODE END 4 */
#endif

/**
  * @brief  Ethernet Link periodic check
  * @param  netif
  * @retval None
  */
static void Ethernet_Link_Periodic_Handle(struct netif *netif)
{
/* USER CODE BEGIN 4_4_1 */
/* USER CODE END 4_4_1 */

  /* Ethernet Link every 100ms */
  if (HAL_GetTick() - EthernetLinkTimer >= 100)
  {
    EthernetLinkTimer = HAL_GetTick();
    ethernet_link_check_state(netif);
  }
/* USER CODE BEGIN 4_4 */
/* USER CODE END 4_4 */
}

/**
 * ----------------------------------------------------------------------
 * Function given to help user to continue LwIP Initialization
 * Up to user to complete or change this function ...
 * Up to user to call this function in main.c in while (1) of main(void)
 *-----------------------------------------------------------------------
 * Read a received packet from the Ethernet buffers
 * Send it to the lwIP stack for handling
 * Handle timeouts if LWIP_TIMERS is set and without RTOS
 * Handle the llink status if LWIP_NETIF_LINK_CALLBACK is set and without RTOS
 */
void MX_LWIP_Process(void)
{
/* USER CODE BEGIN 4_1 */
  sDiagT0 = DWT->CYCCNT;
/* USER CODE END 4_1 */
  ethernetif_input(&gnetif);

/* USER CODE BEGIN 4_2 */
  { uint32_t t = DWT->CYCCNT; if (t - sDiagT0 > lwipDiagRxMax) lwipDiagRxMax = t - sDiagT0; sDiagT0 = t; }
  /* RX DMA self-heal: after "receive buffer unavailable" (all RX descriptors were full, e.g.
   * after a stall) the DMA stays suspended until the tail pointer is written again. The HAL
   * only writes it when it rebuilds descriptors, so kick it here once they are refilled. */
  if ((ETH->DMACSR & ETH_DMACSR_RBU) != 0U)
  {
    ETH->DMACSR = ETH_DMACSR_RBU;            /* write 1 to clear */
    ETH->DMACRDTPR = ETH->DMACRDTPR;         /* re-write the tail pointer: resume RX DMA */
    lwipRxDmaKicks++;
  }
/* USER CODE END 4_2 */
  /* Handle timeouts */
  sys_check_timeouts();

  Ethernet_Link_Periodic_Handle(&gnetif);

/* USER CODE BEGIN 4_3 */
  { uint32_t t = DWT->CYCCNT - sDiagT0; if (t > lwipDiagTmrMax) lwipDiagTmrMax = t; }
/* USER CODE END 4_3 */
}

/**
  * @brief  Notify the User about the network interface config status
  * @param  netif: the network interface
  * @retval None
  */
static void ethernet_link_status_updated(struct netif *netif)
{
  if (netif_is_up(netif))
  {
/* USER CODE BEGIN 5 */
/* USER CODE END 5 */
  }
  else /* netif is down */
  {
/* USER CODE BEGIN 6 */
/* USER CODE END 6 */
  }
}

#if defined ( __CC_ARM )  /* MDK ARM Compiler */
/**
 * Opens a serial device for communication.
 *
 * @param devnum device number
 * @return handle to serial device if successful, NULL otherwise
 */
sio_fd_t sio_open(u8_t devnum)
{
  sio_fd_t sd;

/* USER CODE BEGIN 7 */
  sd = 0; // dummy code
/* USER CODE END 7 */

  return sd;
}

/**
 * Sends a single character to the serial device.
 *
 * @param c character to send
 * @param fd serial device handle
 *
 * @note This function will block until the character can be sent.
 */
void sio_send(u8_t c, sio_fd_t fd)
{
/* USER CODE BEGIN 8 */
/* USER CODE END 8 */
}

/**
 * Reads from the serial device.
 *
 * @param fd serial device handle
 * @param data pointer to data buffer for receiving
 * @param len maximum length (in bytes) of data to receive
 * @return number of bytes actually received - may be 0 if aborted by sio_read_abort
 *
 * @note This function will block until data can be received. The blocking
 * can be cancelled by calling sio_read_abort().
 */
u32_t sio_read(sio_fd_t fd, u8_t *data, u32_t len)
{
  u32_t recved_bytes;

/* USER CODE BEGIN 9 */
  recved_bytes = 0; // dummy code
/* USER CODE END 9 */
  return recved_bytes;
}

/**
 * Tries to read from the serial device. Same as sio_read but returns
 * immediately if no data is available and never blocks.
 *
 * @param fd serial device handle
 * @param data pointer to data buffer for receiving
 * @param len maximum length (in bytes) of data to receive
 * @return number of bytes actually received
 */
u32_t sio_tryread(sio_fd_t fd, u8_t *data, u32_t len)
{
  u32_t recved_bytes;

/* USER CODE BEGIN 10 */
  recved_bytes = 0; // dummy code
/* USER CODE END 10 */
  return recved_bytes;
}
#endif /* MDK ARM Compiler */

