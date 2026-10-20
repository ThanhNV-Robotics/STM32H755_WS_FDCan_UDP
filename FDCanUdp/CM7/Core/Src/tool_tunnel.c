/**
 * tool_tunnel.c - nucleotool UDP tunnel on CM7 (LwIP raw API, NO_SYS)
 *
 * One request in flight per bus: the receive callback (inside MX_LWIP_Process) copies
 * the CANdle bytes into that bus's SRAM4 mailbox slot, CM4 executes them, and
 * ToolTunnel_Poll() sends the reply. Requests are refused at once (no CM4 involvement)
 * when CM4 is not in MAINTENANCE mode or the bus is still busy with a previous request.
 */
#include "tool_tunnel.h"
#include "cm7_log.h"
#include "main.h"
#include "lwip/udp.h"
#include "lwip/pbuf.h"
#include "../../../Common/Inc/tool_shm.h"
#include <string.h>

#define SESSION_IDLE_MS 3000U   /* gap that ends a tool session in the log */

typedef struct
{
    bool      inFlight;     /* request handed to CM4, reply not sent yet */
    ip_addr_t clientAddr;
    u16_t     clientPort;
    uint16_t  clientSeq;
} TunnelBus_t;

static struct udp_pcb* sPcb;
static TunnelBus_t sBus[TOOL_NUM_BUSES];
static uint8_t     sBuf[TOOL_TUNNEL_HDR + TOOL_MAX_LEN];

/* Session statistics, printed when the tool goes quiet */
static bool     sSession;
static uint32_t sLastRxMs, sRequests, sRefused;

static void sendReply(const ip_addr_t* addr, u16_t port, uint8_t status, uint16_t seq,
                      const volatile uint8_t* data, uint32_t len)
{
    struct pbuf* q = pbuf_alloc(PBUF_TRANSPORT, (u16_t)(TOOL_TUNNEL_HDR + len), PBUF_RAM);
    if (q == NULL) return;
    uint8_t* d = (uint8_t*)q->payload;
    d[0] = TOOL_TUNNEL_MAGIC;
    d[1] = status;
    d[2] = (uint8_t)seq;
    d[3] = (uint8_t)(seq >> 8);
    for (uint32_t i = 0; i < len; i++) d[TOOL_TUNNEL_HDR + i] = data[i];
    udp_sendto(sPcb, q, addr, port);
    pbuf_free(q);
}

static void onReceive(void* arg, struct udp_pcb* pcb, struct pbuf* p,
                      const ip_addr_t* addr, u16_t port)
{
    (void)arg;
    (void)pcb;
    uint32_t n = p->tot_len;
    if (n <= TOOL_TUNNEL_HDR || n > sizeof(sBuf))
    {
        pbuf_free(p);
        return;
    }
    pbuf_copy_partial(p, sBuf, (u16_t)n, 0);
    pbuf_free(p);
    if (sBuf[0] != TOOL_TUNNEL_MAGIC) return;

    uint16_t seq = (uint16_t)(sBuf[2] | (sBuf[3] << 8));
    uint8_t  bus = sBuf[1];

    if (!sSession)
    {
        sSession = true;
        sRequests = sRefused = 0;
        CM7_LOG("[TOOL] session from %s", ipaddr_ntoa(addr));
    }
    sLastRxMs = HAL_GetTick();
    sRequests++;

    ToolShm_t* shm = TOOL_SHM;
    uint8_t status = TOOL_ST_OK;
    if (bus >= TOOL_NUM_BUSES)
        status = TOOL_ST_BAD_REQUEST;
    else if (shm->magic != TOOL_SHM_MAGIC || shm->cm4Mode != TOOL_MODE_MAINTENANCE)
        status = TOOL_ST_NOT_MAINTENANCE;
    else if (sBus[bus].inFlight || shm->slot[bus].reqSeq != shm->slot[bus].repSeq)
        status = TOOL_ST_BUSY;
    if (status != TOOL_ST_OK)
    {
        sRefused++;
        sendReply(addr, port, status, seq, NULL, 0);
        return;
    }

    ToolSlot_t*  slot = &shm->slot[bus];
    TunnelBus_t* tb   = &sBus[bus];
    uint32_t len = n - TOOL_TUNNEL_HDR;
    for (uint32_t i = 0; i < len; i++) slot->req[i] = sBuf[TOOL_TUNNEL_HDR + i];
    slot->reqLen = len;
    ip_addr_copy(tb->clientAddr, *addr);
    tb->clientPort = port;
    tb->clientSeq  = seq;
    tb->inFlight   = true;
    __DMB();   /* request complete before CM4 can see the new seq */
    slot->reqSeq = slot->reqSeq + 1U;
}

bool ToolTunnel_Init(void)
{
    sPcb = udp_new();
    if (sPcb == NULL)
        return false;
    if (udp_bind(sPcb, IP_ADDR_ANY, TOOL_UDP_PORT) != ERR_OK)
    {
        udp_remove(sPcb);
        sPcb = NULL;
        return false;
    }
    udp_recv(sPcb, onReceive, NULL);
    return true;
}

void ToolTunnel_Poll(void)
{
    ToolShm_t* shm = TOOL_SHM;
    bool anyInFlight = false;
    for (uint32_t b = 0; b < TOOL_NUM_BUSES; b++)
    {
        TunnelBus_t* tb   = &sBus[b];
        ToolSlot_t*  slot = &shm->slot[b];
        if (tb->inFlight && slot->repSeq == slot->reqSeq)
        {
            __DMB();   /* read the reply only after seeing CM4's seq */
            uint32_t len = slot->repLen;
            if (len > TOOL_MAX_LEN) len = 0;
            if (slot->repStatus != TOOL_ST_OK) sRefused++;
            sendReply(&tb->clientAddr, tb->clientPort, (uint8_t)slot->repStatus, tb->clientSeq,
                      slot->rep, len);
            tb->inFlight = false;
        }
        anyInFlight |= tb->inFlight;
    }

    if (sSession && !anyInFlight && (HAL_GetTick() - sLastRxMs) >= SESSION_IDLE_MS)
    {
        sSession = false;
        CM7_LOG("[TOOL] session end: %lu requests, %lu refused/failed", sRequests, sRefused);
    }
}
