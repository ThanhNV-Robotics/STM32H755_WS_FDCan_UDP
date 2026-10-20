/**
 * candle_emu.c - CANdle emulation on CM4 for nucleotool
 *
 * CANdle USB protocol, as used by CANdle-SDK candlelib (candle.hpp / candle.cpp):
 *   RESET     [9][0]                               -> [9][1]
 *   DATARATE  [2][rate 1/2/5/8][classic CAN]       -> [2][1][tag][rev][minor][major]
 *   CAN FRAME [4][len][timeout ms][id LE16][data]  -> [4][1 = got reply][64 data bytes]
 *   PACKED    [15][ack][count][count x DTO][CRC32] -> same layout, one reply per DTO
 *     DTO (70 B): [id LE16][timeout LE16, 100 us][len][seq][64 data], CRC32 MPEG-2
 *     (poly 0x04C11DB7, init 0xFFFFFFFF, no final XOR) over everything before it.
 *     A slot with id 0 / len 0 means "no reply".
 * The buses stay at 1 Mbps / 8 Mbps BRS: other datarates and classic CAN are refused
 * with TOOL_ST_DATARATE so nucleotool tells the user instead of failing silently.
 *
 * Each bus has its own mailbox slot and its own state machine, so both legs are served
 * at the same time: CandleEmu_Poll() never waits, it sends a frame and checks for the
 * reply (or the timeout) on the next calls.
 */
#include "candle_emu.h"
#include "mab_md.h"
#include "../../../Common/Inc/ipc_log.h"
#include "../../../Common/Inc/tool_shm.h"
#include <stdbool.h>
#include <string.h>

#define CANDLE_CMD_DATARATE  2U
#define CANDLE_CMD_CAN_FRAME 4U
#define CANDLE_CMD_RESET     9U
#define CANDLE_CMD_PACKED    15U

#define CAN_FRAME_REPLY_LEN 66U
#define DTO_SIZE      70U
#define DTO_MAX_COUNT 7U
#define BUS_DATARATE  8U   /* CANdleDatarate_E of the leg buses (8M) */

/* Reported CANdle firmware: >= 2.4 so the SDK uses its asynchronous (packed) API */
#define EMU_VER_MAJOR 2U
#define EMU_VER_MINOR 4U
#define EMU_VER_REV   0U
#define EMU_VER_TAG   'N'

#define DTO_TIMEOUT_MAX_US 100000U   /* clamp per packed frame: 7 x 100 ms worst case */

typedef struct
{
    bool     busy;        /* request taken from the slot, reply not written yet */
    bool     waiting;     /* frame sent, waiting for a reply or the timeout */
    uint32_t seq;
    uint32_t reqLen;
    uint32_t repLen;
    uint32_t status;
    uint32_t dtoIdx;      /* packed: DTO being processed */
    uint32_t startUs, timeoutUs;
    uint8_t  req[TOOL_MAX_LEN];
    uint8_t  rep[TOOL_MAX_LEN];
} EmuBus_t;

static EmuBus_t sEmu[TOOL_NUM_BUSES];
static uint32_t sBusOffCount[TOOL_NUM_BUSES];

/* Bus-off (TEC > 255) puts the controller in INIT, where it stays until software
 * clears INIT; the controller then rejoins after 129 x 11 recessive bits (~1.4 ms). */
static void recoverBusOff(uint32_t b)
{
    FDCAN_GlobalTypeDef* fd = mdLegs[b].bus->Instance;
    if ((fd->CCCR & FDCAN_CCCR_INIT) == 0U) return;
    sBusOffCount[b]++;
    LOG("[TOOL] leg %s: bus-off (TEC %lu, PSR 0x%08lX), restarting the controller (%lu)",
        mdLegs[b].name, (fd->ECR & FDCAN_ECR_TEC) >> FDCAN_ECR_TEC_Pos, fd->PSR, sBusOffCount[b]);
    fd->CCCR &= ~FDCAN_CCCR_INIT;
}

static uint32_t crc32Mpeg2(const uint8_t* d, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFUL;
    for (uint32_t i = 0; i < len; i++)
    {
        crc ^= (uint32_t)d[i] << 24;
        for (uint32_t b = 0; b < 8U; b++)
            crc = (crc & 0x80000000UL) ? (crc << 1) ^ 0x04C11DB7UL : crc << 1;
    }
    return crc;
}

static void finish(uint32_t b)
{
    EmuBus_t*   e    = &sEmu[b];
    ToolSlot_t* slot = &TOOL_SHM->slot[b];
    for (uint32_t i = 0; i < e->repLen; i++) slot->rep[i] = e->rep[i];
    slot->repLen    = e->repLen;
    slot->repStatus = e->status;
    __DMB();   /* reply complete before CM7 can see the seq */
    slot->repSeq = e->seq;
    e->busy    = false;
    e->waiting = false;
}

/* Queue one CAN FD frame; the reply is picked up by CandleEmu_Poll(). */
static bool startFrame(uint32_t b, uint16_t id, const uint8_t* data, uint32_t len, uint32_t timeoutUs)
{
    FDCAN_HandleTypeDef* bus = mdLegs[b].bus;
    uint8_t tx[64] = {0};
    memcpy(tx, data, len);
    MD_FlushRx(bus);
    if (!MD_SendFrame(bus, id, tx, len))
        return false;
    sEmu[b].startUs   = MD_Micros();
    sEmu[b].timeoutUs = timeoutUs;
    sEmu[b].waiting   = true;
    return true;
}

/* Packed: start the next valid DTO, or write the CRC and finish when none is left. */
static void nextDto(uint32_t b)
{
    EmuBus_t* e = &sEmu[b];
    uint32_t count = e->req[2];
    for (; e->dtoIdx < count; e->dtoIdx++)
    {
        const uint8_t* q = &e->req[3U + e->dtoIdx * DTO_SIZE];
        uint16_t id  = (uint16_t)(q[0] | (q[1] << 8));
        uint32_t tmo = (uint32_t)(q[2] | (q[3] << 8)) * 100U;
        uint32_t len = q[4];
        if (id == 0U || len == 0U || len > 64U) continue;   /* invalid slot stays zero */
        if (tmo == 0U) tmo = 1000U;
        if (tmo > DTO_TIMEOUT_MAX_US) tmo = DTO_TIMEOUT_MAX_US;
        if (startFrame(b, id, &q[6], len, tmo)) return;
        /* could not queue: leave the reply slot empty ("no reply") */
    }
    uint32_t n = e->reqLen;
    uint32_t crc = crc32Mpeg2(e->rep, n - 4U);
    e->rep[n - 4U] = (uint8_t)crc;
    e->rep[n - 3U] = (uint8_t)(crc >> 8);
    e->rep[n - 2U] = (uint8_t)(crc >> 16);
    e->rep[n - 1U] = (uint8_t)(crc >> 24);
    finish(b);
}

/* A reply arrived (rxLen > 0) or the wait timed out (rxLen == 0). */
static void onReply(uint32_t b, const uint8_t* rx, uint32_t rxLen)
{
    EmuBus_t* e = &sEmu[b];
    e->waiting = false;
    if (e->req[0] == CANDLE_CMD_CAN_FRAME)
    {
        e->rep[1] = rxLen ? 1U : 0U;
        if (rxLen) memcpy(&e->rep[2], rx, rxLen);
        finish(b);
        return;
    }
    /* packed */
    if (rxLen)
    {
        const uint8_t* q = &e->req[3U + e->dtoIdx * DTO_SIZE];
        uint8_t*       r = &e->rep[3U + e->dtoIdx * DTO_SIZE];
        memcpy(r, q, 4);              /* id, timeout */
        r[4] = (uint8_t)rxLen;
        r[5] = q[5];                  /* sequence number */
        memcpy(&r[6], rx, rxLen);
    }
    e->dtoIdx++;
    nextDto(b);
}

/* Take a new request from the slot: answer it at once or start its first frame. */
static void startRequest(uint32_t b)
{
    EmuBus_t*   e    = &sEmu[b];
    ToolSlot_t* slot = &TOOL_SHM->slot[b];
    e->busy   = true;
    e->status = TOOL_ST_OK;
    e->repLen = 0;
    e->reqLen = slot->reqLen;
    if (e->reqLen == 0U || e->reqLen > TOOL_MAX_LEN)
    {
        e->status = TOOL_ST_BAD_REQUEST;
        finish(b);
        return;
    }
    for (uint32_t i = 0; i < e->reqLen; i++) e->req[i] = slot->req[i];
    const uint8_t* q = e->req;
    uint32_t n = e->reqLen;

    switch (q[0])
    {
        case CANDLE_CMD_RESET:
            e->rep[0] = CANDLE_CMD_RESET;
            e->rep[1] = 1U;
            e->repLen = 2;
            break;

        case CANDLE_CMD_DATARATE:
            if (n < 3U || q[1] != BUS_DATARATE || q[2] != 0U)
            {
                e->status = TOOL_ST_DATARATE;
                break;
            }
            e->rep[0] = CANDLE_CMD_DATARATE;
            e->rep[1] = 1U;
            e->rep[2] = EMU_VER_TAG;
            e->rep[3] = EMU_VER_REV;
            e->rep[4] = EMU_VER_MINOR;
            e->rep[5] = EMU_VER_MAJOR;
            e->repLen = 6;
            break;

        case CANDLE_CMD_CAN_FRAME:
        {
            uint32_t len = q[1];
            if (n < 5U || len == 0U || len > 64U || n < 5U + len)
            {
                e->status = TOOL_ST_BAD_REQUEST;
                break;
            }
            memset(e->rep, 0, CAN_FRAME_REPLY_LEN);
            e->rep[0] = CANDLE_CMD_CAN_FRAME;
            e->repLen = CAN_FRAME_REPLY_LEN;
            uint16_t id = (uint16_t)(q[3] | (q[4] << 8));
            uint32_t timeoutUs = (q[2] ? q[2] : 1U) * 1000U;
            if (startFrame(b, id, &q[5], len, timeoutUs)) return;
            break;   /* could not queue: reply "no answer" */
        }

        case CANDLE_CMD_PACKED:
        {
            uint32_t count = q[2];
            if (n < 7U || count > DTO_MAX_COUNT || n != 3U + count * DTO_SIZE + 4U)
            {
                e->status = TOOL_ST_BAD_REQUEST;
                break;
            }
            const uint8_t* c = &q[n - 4U];
            uint32_t crc = (uint32_t)c[0] | ((uint32_t)c[1] << 8) | ((uint32_t)c[2] << 16) | ((uint32_t)c[3] << 24);
            if (crc != crc32Mpeg2(q, n - 4U))
            {
                e->status = TOOL_ST_BAD_REQUEST;
                break;
            }
            memset(e->rep, 0, n);
            e->rep[0] = CANDLE_CMD_PACKED;
            e->rep[1] = 1U;   /* ACK */
            e->rep[2] = (uint8_t)count;
            e->repLen = n;
            e->dtoIdx = 0;
            nextDto(b);
            return;
        }

        default:
            e->status = TOOL_ST_UNSUPPORTED;
            break;
    }
    finish(b);
}

void CandleEmu_SetMode(unsigned mode)
{
    TOOL_SHM->cm4Mode = mode;
}

void CandleEmu_Poll(void)
{
    ToolShm_t* shm = TOOL_SHM;
    if (shm->magic != TOOL_SHM_MAGIC) return;

    for (uint32_t b = 0; b < TOOL_NUM_BUSES; b++)
    {
        EmuBus_t* e = &sEmu[b];
        recoverBusOff(b);
        if (!e->busy)
        {
            uint32_t seq = shm->slot[b].reqSeq;
            if (seq == shm->slot[b].repSeq) continue;
            __DMB();   /* read the request only after seeing CM7's seq */
            e->seq = seq;
            startRequest(b);
            continue;
        }
        if (!e->waiting) continue;

        /* Accept the first frame that comes back after the send */
        FDCAN_HandleTypeDef* bus = mdLegs[b].bus;
        uint8_t rx[64];
        FDCAN_RxHeaderTypeDef h;
        if (HAL_FDCAN_GetRxFifoFillLevel(bus, FDCAN_RX_FIFO0) != 0U &&
            HAL_FDCAN_GetRxMessage(bus, FDCAN_RX_FIFO0, &h, rx) == HAL_OK)
        {
            onReply(b, rx, MD_DlcToBytes(h.DataLength));
        }
        else if ((uint32_t)(MD_Micros() - e->startUs) >= e->timeoutUs)
        {
            MD_AbortPendingTx(bus);   /* e.g. nobody ACKed: stop the automatic retransmission */
            onReply(b, NULL, 0);
        }
    }
}
