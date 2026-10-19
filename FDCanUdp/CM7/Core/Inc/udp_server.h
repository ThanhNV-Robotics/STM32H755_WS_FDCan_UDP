/**
 * udp_server.h - WBC UDP server on CM7 (LwIP raw API, NO_SYS)
 *
 * Receives WbcCommandPacket on port 9999 and answers each valid one with a
 * WbcStatePacket to the sender. Packet layout: Common/Inc/wbc_protocol.h.
 * Step 2 (UDP only): the states are placeholders (WBC_ST_NO_CM4_DATA); the
 * CM4 mailboxes come in step 3.
 */
#ifndef UDP_SERVER_H
#define UDP_SERVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint32_t rxValid;      /* command packets accepted */
    uint32_t rxBadSize;    /* wrong length */
    uint32_t rxBadMagic;   /* wrong magic or version */
    uint32_t txOk;         /* state packets sent */
    uint32_t txFail;       /* pbuf allocation or udp_sendto failed */
    uint32_t seqGaps;      /* commands missing between two received seq numbers */
    uint32_t lastSeq;
    uint32_t lastRxMs;     /* HAL_GetTick() of the last valid command */
} UdpServerStats_t;

extern UdpServerStats_t udpStats;   /* also visible in the debugger */

bool UDP_Server_Init(void);          /* call once after MX_LWIP_Init() */
void UDP_Server_PrintStats(void);    /* one log line, call e.g. every few seconds */

#ifdef __cplusplus
}
#endif

#endif /* UDP_SERVER_H */
