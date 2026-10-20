/**
 * tool_shm.h - CM7 <-> CM4 mailbox for the maintenance tool (nucleotool), SRAM4
 *
 * nucleotool (CANdle-SDK candletool with a UDP bus) sends raw CANdle commands to CM7
 * on UDP port 9998. CM7 copies them into this mailbox; CM4 runs them on FDCAN1/2
 * (candle_emu.c) and writes the replies back. CM7 only forwards requests while CM4
 * reports MAINTENANCE mode, so the tool never touches the bus while the control loop
 * runs, and the control loop never looks at this mailbox.
 *
 * One slot per bus, one request in flight per slot, so both legs can be served at the
 * same time. Handshake per slot: CM7 writes req/reqLen, then reqSeq + 1. CM4 sees
 * reqSeq != repSeq, writes rep/repLen/repStatus, then repSeq = reqSeq.
 *
 * Location: SRAM4 0x38004000 (after the 8 KB CM4 log at 0x38000000). The CM7 D-cache
 * is off and CM4 has none, so plain volatile access plus barriers is enough.
 */
#ifndef TOOL_SHM_H
#define TOOL_SHM_H

#include <stdint.h>

#define TOOL_SHM_ADDR   0x38004000UL
#define TOOL_SHM_MAGIC  0x4C4F4F54UL   /* "TOOL" */
#define TOOL_UDP_PORT   9998U
#define TOOL_MAX_LEN    512U           /* largest CANdle message: packed frame, 497 B */
#define TOOL_NUM_BUSES  2U             /* slot 0 = L / FDCAN1, slot 1 = R / FDCAN2 */

/* UDP tunnel header in front of the CANdle bytes:
 *   request [0xCD][bus 0=L/FDCAN1, 1=R/FDCAN2][seq u16 LE]
 *   reply   [0xCD][status                    ][seq u16 LE] */
#define TOOL_TUNNEL_MAGIC 0xCDU
#define TOOL_TUNNEL_HDR   4U

/* CM4 mode */
#define TOOL_MODE_BOOT        0U   /* CM4 not ready yet */
#define TOOL_MODE_MAINTENANCE 1U   /* tool may use the buses, nothing is commanded */
#define TOOL_MODE_CONTROL     2U   /* control loop (bench for now) owns the buses */

/* Tunnel status (must match UDP::TunnelStatus_E in CANdle-SDK-udp) */
#define TOOL_ST_OK              0U
#define TOOL_ST_NOT_MAINTENANCE 1U
#define TOOL_ST_BUSY            2U
#define TOOL_ST_BAD_REQUEST     3U
#define TOOL_ST_UNSUPPORTED     4U
#define TOOL_ST_DATARATE        5U

typedef struct
{
    volatile uint32_t reqSeq;      /* CM7: +1 after a request is complete */
    volatile uint32_t repSeq;      /* CM4: = reqSeq after the reply is complete */
    volatile uint32_t reqLen;
    volatile uint32_t repLen;
    volatile uint32_t repStatus;
    volatile uint8_t  req[TOOL_MAX_LEN];
    volatile uint8_t  rep[TOOL_MAX_LEN];
} ToolSlot_t;

typedef struct
{
    volatile uint32_t magic;
    volatile uint32_t cm4Mode;     /* written by CM4 */
    ToolSlot_t        slot[TOOL_NUM_BUSES];
} ToolShm_t;

#define TOOL_SHM ((ToolShm_t*)TOOL_SHM_ADDR)

/* CM7, before releasing CM4. Whole-word writes: SRAM ECC must be initialised before
 * any byte access (see IPC_LogInit). */
static inline void ToolShm_Init(void)
{
    volatile uint32_t* p = (volatile uint32_t*)TOOL_SHM_ADDR;
    for (uint32_t i = 0; i < sizeof(ToolShm_t) / 4U; i++)
        p[i] = 0U;
    __asm volatile("dmb" ::: "memory");
    TOOL_SHM->magic = TOOL_SHM_MAGIC;
}

#endif /* TOOL_SHM_H */
