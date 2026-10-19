/**
 * udp_server.c - WBC UDP server on CM7 (LwIP raw API, NO_SYS)
 *
 * The receive callback runs inside MX_LWIP_Process() (main loop), so no locking
 * is needed. Every valid command is answered at once with one state packet.
 */
#include "udp_server.h"
#include "cm7_log.h"
#include "main.h"
#include "lwip/udp.h"
#include "lwip/pbuf.h"
#include "../../../Common/Inc/wbc_protocol.h"
#include <stdio.h>
#include <string.h>

UdpServerStats_t udpStats;

static struct udp_pcb* sPcb;

/* ─── Microsecond timebase from the CM7 DWT cycle counter ─── */
static uint32_t sLastCycles, sCycleRemainder, sMicros, sCyclesPerUs;

static void time_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    sCyclesPerUs = SystemCoreClock / 1000000U;
    sLastCycles = DWT->CYCCNT;
}

/* Accumulates elapsed cycles so the result wraps at 2^32 us; call at least every ~10 s */
static uint32_t micros(void)
{
    uint32_t now = DWT->CYCCNT;
    sCycleRemainder += now - sLastCycles;
    sLastCycles = now;
    sMicros += sCycleRemainder / sCyclesPerUs;
    sCycleRemainder %= sCyclesPerUs;
    return sMicros;
}

/* ─── State packet (placeholder until the CM4 mailboxes exist) ─── */
static void buildState(WbcStatePacket* st, const WbcCommandPacket* cmd)
{
    memset(st, 0, sizeof(*st));
    st->magic     = WBC_STATE_MAGIC;
    st->version   = WBC_VERSION;
    st->status    = WBC_ST_NO_CM4_DATA;
    st->seqEcho   = cmd->seq;
    st->stmTimeUs = micros();
}

static void onReceive(void* arg, struct udp_pcb* pcb, struct pbuf* p,
                      const ip_addr_t* addr, u16_t port)
{
    (void)arg;
    WbcCommandPacket cmd;

    if (p->tot_len != sizeof(cmd))
    {
        udpStats.rxBadSize++;
        pbuf_free(p);
        return;
    }
    pbuf_copy_partial(p, &cmd, sizeof(cmd), 0);
    pbuf_free(p);

    if (cmd.magic != WBC_CMD_MAGIC || cmd.version != WBC_VERSION)
    {
        udpStats.rxBadMagic++;
        return;
    }

    if (udpStats.rxValid > 0U && cmd.seq > udpStats.lastSeq + 1U)
        udpStats.seqGaps += cmd.seq - udpStats.lastSeq - 1U;
    udpStats.rxValid++;
    udpStats.lastSeq  = cmd.seq;
    udpStats.lastRxMs = HAL_GetTick();

    struct pbuf* q = pbuf_alloc(PBUF_TRANSPORT, sizeof(WbcStatePacket), PBUF_RAM);
    if (q == NULL)
    {
        udpStats.txFail++;
        return;
    }
    buildState((WbcStatePacket*)q->payload, &cmd);
    if (udp_sendto(pcb, q, addr, port) == ERR_OK) udpStats.txOk++;
    else                                          udpStats.txFail++;
    pbuf_free(q);
}

bool UDP_Server_Init(void)
{
    time_init();
    sPcb = udp_new();
    if (sPcb == NULL)
        return false;
    if (udp_bind(sPcb, IP_ADDR_ANY, WBC_UDP_PORT) != ERR_OK)
    {
        udp_remove(sPcb);
        sPcb = NULL;
        return false;
    }
    udp_recv(sPcb, onReceive, NULL);
    return true;
}

void UDP_Server_PrintStats(void)
{
    CM7_LOG("[UDP] rx %lu  badSize %lu  badMagic %lu  tx %lu  txFail %lu  seqGaps %lu  lastSeq %lu",
            udpStats.rxValid, udpStats.rxBadSize, udpStats.rxBadMagic,
            udpStats.txOk, udpStats.txFail, udpStats.seqGaps, udpStats.lastSeq);
}
