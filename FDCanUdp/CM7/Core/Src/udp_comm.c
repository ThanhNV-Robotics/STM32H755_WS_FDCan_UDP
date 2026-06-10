/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    udp_comm.c
  * @brief   UDP send / receive implementation using the LwIP raw API.
  *
  *  We use the "raw" LwIP API (not sockets, not netconn) because
  *  lwipopts.h sets NO_SYS=1 (bare-metal, no RTOS).
  *
  *  Flow overview:
  *    TX:  your code calls UDP_Send()
  *             -> pbuf_alloc allocates a packet buffer
  *             -> udp_sendto hands it to LwIP
  *             -> ethernetif_output sends it over Ethernet
  *             -> pbuf_free releases the buffer
  *
  *    RX:  a packet arrives on the wire
  *             -> ETH DMA writes it to RAM, interrupt fires
  *             -> MX_LWIP_Process() -> ethernetif_input() reads from DMA
  *             -> LwIP demultiplexes: UDP port 5005 -> udp_rx_callback()
  *             -> your code handles it inside the callback
  *             -> pbuf_free releases the buffer
  ******************************************************************************
  */
/* USER CODE END Header */

#include "udp_comm.h"

/* LwIP raw UDP API */
#include "lwip/udp.h"   /* udp_new, udp_bind, udp_recv, udp_sendto        */
#include "lwip/pbuf.h"  /* pbuf_alloc, pbuf_free                           */
#include "lwip/ip_addr.h" /* ip4_addr_t, IP4_ADDR                          */

#include <string.h>     /* memcpy */

/* --------------------------------------------------------------------------
 * Module-private state
 * -------------------------------------------------------------------------- */

/* UDP Protocol Control Block — LwIP's handle for one UDP "socket". */
static struct udp_pcb *sUdpPcb = NULL;

/* Pre-built destination address for UDP_Send(). */
static ip4_addr_t sHostAddr;

/* --------------------------------------------------------------------------
 * Receive callback
 *
 * LwIP calls this function automatically (from inside MX_LWIP_Process)
 * whenever a UDP datagram arrives on our bound port.
 *
 * Parameters supplied by LwIP:
 *   arg   — user data pointer passed to udp_recv() (NULL here)
 *   pcb   — the UDP PCB that received the packet
 *   p     — pbuf chain holding the payload bytes
 *   addr  — sender's IP address
 *   port  — sender's UDP port
 * -------------------------------------------------------------------------- */
static void udp_rx_callback(void       *arg,
                             struct udp_pcb *pcb,
                             struct pbuf    *p,
                             const ip_addr_t *addr,
                             u16_t           port)
{
    (void)arg;   /* not used */
    (void)pcb;   /* not used — we only have one PCB */
    (void)addr;  /* sender IP  — use if you want to reply dynamically */
    (void)port;  /* sender port — use if you want to reply dynamically */

    if (p == NULL)
        return;

    /* ----------------------------------------------------------------
     * p->payload  points to the first byte of UDP data.
     * p->len      is the number of bytes in this pbuf segment.
     * p->tot_len  is the total length across all chained pbufs
     *             (for simple datagrams, tot_len == len).
     *
     * Example: echo the received bytes back to the sender
     *   UDP_Send((uint8_t *)p->payload, p->len);
     *
     * TODO: replace this with your application logic, e.g.:
     *   memcpy(gRxBuffer, p->payload, p->len);
     *   gRxNewData = 1;
     * ---------------------------------------------------------------- */

    /* Always free the pbuf when you are done — LwIP will not do it for you. */
    pbuf_free(p);
}

/* --------------------------------------------------------------------------
 * UDP_Init
 * -------------------------------------------------------------------------- */
void UDP_Init(void)
{
    /* Build the host PC destination address from the four octets defined
     * in udp_comm.h so the address is easy to change in one place. */
    IP4_ADDR(&sHostAddr, HOST_IP_0, HOST_IP_1, HOST_IP_2, HOST_IP_3);

    /* Create a new UDP Protocol Control Block.
     * This is LwIP's internal structure for one UDP endpoint. */
    sUdpPcb = udp_new();
    if (sUdpPcb == NULL)
        return; /* out of memory — increase MEM_SIZE in lwipopts.h */

    /* Bind to all local interfaces (IP_ADDR_ANY) on our chosen port.
     * From this point LwIP will route incoming UDP:5005 to our callback. */
    udp_bind(sUdpPcb, IP_ADDR_ANY, UDP_LOCAL_PORT);

    /* Register the receive callback.
     * LwIP will call udp_rx_callback() whenever a matching packet arrives. */
    udp_recv(sUdpPcb, udp_rx_callback, NULL);
}

/* --------------------------------------------------------------------------
 * UDP_Send
 * -------------------------------------------------------------------------- */
void UDP_Send(const uint8_t *data, uint16_t len)
{
    if (sUdpPcb == NULL || data == NULL || len == 0)
        return;

    /* Allocate a pbuf (packet buffer) in the TRANSPORT layer.
     * PBUF_RAM means the payload memory is allocated from the LwIP heap.
     * LwIP will prepend UDP/IP/Ethernet headers automatically. */
    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);
    if (p == NULL)
        return; /* out of memory — LwIP heap too small, increase MEM_SIZE */

    /* Copy your data into the pbuf payload area. */
    memcpy(p->payload, data, len);

    /* Hand the packet to LwIP.
     * udp_sendto adds UDP + IP + Ethernet headers and passes it to the
     * ETH DMA for transmission. */
    udp_sendto(sUdpPcb, p, &sHostAddr, UDP_HOST_PORT);

    /* Release the pbuf — after udp_sendto returns, we no longer own it. */
    pbuf_free(p);
}
