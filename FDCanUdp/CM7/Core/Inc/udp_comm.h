/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    udp_comm.h
  * @brief   UDP send / receive interface for CM7 <-> host PC communication.
  *
  *  STM32 static IP : 192.168.1.100  (set in lwip.c)
  *  Host PC IP      : 192.168.1.10   (set by HOST_IP_x defines below)
  *  Local UDP port  : 5005
  *  Remote UDP port : 5005
  *
  *  Usage:
  *    1. Call UDP_Init() once after MX_LWIP_Init().
  *    2. Call MX_LWIP_Process() every iteration of the main loop.
  *    3. Call UDP_Send(data, len) whenever you want to transmit to the host.
  *    4. Received packets are delivered to udp_rx_callback() automatically.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef UDP_COMM_H
#define UDP_COMM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ---- Network addresses (edit to match your setup) ------------------------ */

/* IP address of the host PC — must be in the same /24 subnet as the STM32. */
#define HOST_IP_0  192
#define HOST_IP_1  168
#define HOST_IP_2  1
#define HOST_IP_3  10    /* change this to your PC's last octet */

/* UDP port used for both TX and RX. */
#define UDP_LOCAL_PORT   5005U
#define UDP_HOST_PORT    5005U

/* -------------------------------------------------------------------------- */

/**
 * @brief  Initialise the UDP socket: create PCB, bind to LOCAL_PORT,
 *         register the receive callback.
 *         Call once after MX_LWIP_Init().
 */
void UDP_Init(void);

/**
 * @brief  Send a UDP datagram to the host PC.
 * @param  data  Pointer to the bytes to send.
 * @param  len   Number of bytes (max ~1472 for a standard Ethernet frame).
 */
void UDP_Send(const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* UDP_COMM_H */
