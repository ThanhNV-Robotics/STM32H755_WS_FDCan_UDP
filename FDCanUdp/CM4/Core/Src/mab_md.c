/**
 * mab_md.c - MAB MD drivers over FDCAN1 (CM4 core)
 *
 * Port of Portenta_H7/fdcan_udp/fdcan_bench/fdcan_bench.ino (protocol part).
 */
#include "mab_md.h"
#include "fdcan.h"
#include "../../../Common/Inc/ipc_log.h"
#include <string.h>

#define MD_REPLY_TIMEOUT_US 3000U  /* blocking config transactions */
#define MD_INTER_FRAME_US   300U   /* MD drops frames arriving right after a reply */

MD_Motor_t mdMotors[MD_NUM_MOTORS] = {
    {.canId = 10}, {.canId = 20}, {.canId = 30},
    {.canId = 40}, {.canId = 50}, {.canId = 60},
};
FDCAN_ProtocolStatusTypeDef mdProtocolStatus;
FDCAN_ErrorCountersTypeDef  mdErrorCounters;

/* FDCAN controller the motors are on (MD_SetBus) */
static FDCAN_HandleTypeDef* sBus = &hfdcan1;

void MD_SetBus(FDCAN_HandleTypeDef* bus)
{
    sBus = bus;
}

FDCAN_HandleTypeDef* MD_Bus(void)
{
    return sBus;
}

/* ============================================================================
 * Timebase
 * ========================================================================== */
static uint32_t sLastCycles;
static uint32_t sCycleRemainder;
static uint32_t sMicros;
static uint32_t sCyclesPerUs;

void MD_TimeInit(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    /* HAL_Init() set SystemCoreClock; it is right only if CM7 configured the
     * clocks first (DUAL_CORE_BOOT_SYNC_SEQUENCE). */
    sCyclesPerUs = SystemCoreClock / 1000000U;
    sLastCycles = DWT->CYCCNT;
}

/* Accumulates elapsed cycles so the result wraps at 2^32 us like Arduino micros(). */
uint32_t MD_Micros(void)
{
    uint32_t now = DWT->CYCCNT;
    sCycleRemainder += now - sLastCycles;
    sLastCycles = now;
    sMicros += sCycleRemainder / sCyclesPerUs;
    sCycleRemainder %= sCyclesPerUs;
    return sMicros;
}

void MD_DelayUs(uint32_t us)
{
    uint32_t start = MD_Micros();
    while ((uint32_t)(MD_Micros() - start) < us) {}
}

/* ============================================================================
 * FDCAN helpers
 * ========================================================================== */
uint32_t MD_DlcToBytes(uint32_t dlc)
{
    switch (dlc)
    {
        case FDCAN_DLC_BYTES_0:  return 0;
        case FDCAN_DLC_BYTES_1:  return 1;
        case FDCAN_DLC_BYTES_2:  return 2;
        case FDCAN_DLC_BYTES_3:  return 3;
        case FDCAN_DLC_BYTES_4:  return 4;
        case FDCAN_DLC_BYTES_5:  return 5;
        case FDCAN_DLC_BYTES_6:  return 6;
        case FDCAN_DLC_BYTES_7:  return 7;
        case FDCAN_DLC_BYTES_8:  return 8;
        case FDCAN_DLC_BYTES_12: return 12;
        case FDCAN_DLC_BYTES_16: return 16;
        case FDCAN_DLC_BYTES_20: return 20;
        case FDCAN_DLC_BYTES_24: return 24;
        case FDCAN_DLC_BYTES_32: return 32;
        case FDCAN_DLC_BYTES_48: return 48;
        default:                 return 64;
    }
}

/* Smallest DLC that holds len bytes (frame is zero padded, MD stops at reg 0x0000) */
static uint32_t md_bytesToDlc(uint32_t len)
{
    if (len <= 8)  return FDCAN_DLC_BYTES_8;
    if (len <= 12) return FDCAN_DLC_BYTES_12;
    if (len <= 16) return FDCAN_DLC_BYTES_16;
    if (len <= 20) return FDCAN_DLC_BYTES_20;
    if (len <= 24) return FDCAN_DLC_BYTES_24;
    if (len <= 32) return FDCAN_DLC_BYTES_32;
    if (len <= 48) return FDCAN_DLC_BYTES_48;
    return FDCAN_DLC_BYTES_64;
}

uint32_t MD_PutReg(uint8_t* buf, uint32_t idx, uint16_t reg, const void* val, uint32_t size)
{
    buf[idx++] = reg & 0xFFU;
    buf[idx++] = reg >> 8;
    memcpy(&buf[idx], val, size);
    return idx + size;
}

void MD_FlushRx(void)
{
    FDCAN_RxHeaderTypeDef h;
    uint8_t d[64];
    while (HAL_FDCAN_GetRxFifoFillLevel(sBus, FDCAN_RX_FIFO0) > 0U)
        HAL_FDCAN_GetRxMessage(sBus, FDCAN_RX_FIFO0, &h, d);
}

/* Cancel every TX FIFO request still pending (e.g. retrying because nobody ACKs) */
void MD_AbortPendingTx(void)
{
    HAL_FDCAN_AbortTxRequest(sBus, (1UL << 16) - 1U);   /* all 16 TX elements */
}

/* tx must hold at least the padded DLC size (callers use 8- or 64-byte zeroed buffers) */
bool MD_SendFrame(uint16_t canId, const uint8_t* tx, uint32_t len)
{
    FDCAN_TxHeaderTypeDef txHeader = {
        .Identifier          = canId,
        .IdType              = FDCAN_STANDARD_ID,
        .TxFrameType         = FDCAN_DATA_FRAME,
        .DataLength          = md_bytesToDlc(len),
        .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
        .BitRateSwitch       = FDCAN_BRS_ON,
        .FDFormat            = FDCAN_FD_CAN,
        .TxEventFifoControl  = FDCAN_NO_TX_EVENTS,
        .MessageMarker       = 0,
    };
    return HAL_FDCAN_AddMessageToTxFifoQ(sBus, &txHeader, (uint8_t*)tx) == HAL_OK;
}

typedef enum
{
    MD_XFER_OK,
    MD_XFER_TX_FAIL,   /* frame could not be queued */
    MD_XFER_TIMEOUT,   /* no reply from canId */
} MD_Xfer_E;

/* Blocking MD transaction: send frame to canId, wait for its reply.
 * rx must hold 64 bytes. Frames from other IDs are printed and skipped. */
static MD_Xfer_E md_transact(uint16_t canId, const uint8_t* tx, uint32_t txLen,
                             uint8_t* rx, uint32_t* rxLen, uint32_t* rttUs)
{
    MD_FlushRx();
    if (!MD_SendFrame(canId, tx, txLen))
        return MD_XFER_TX_FAIL;

    uint32_t start = MD_Micros();
    FDCAN_RxHeaderTypeDef h;
    while ((uint32_t)(MD_Micros() - start) < MD_REPLY_TIMEOUT_US)
    {
        if (HAL_FDCAN_GetRxFifoFillLevel(sBus, FDCAN_RX_FIFO0) == 0U) continue;
        if (HAL_FDCAN_GetRxMessage(sBus, FDCAN_RX_FIFO0, &h, rx) != HAL_OK) continue;
        if (h.Identifier == canId)
        {
            *rttUs = MD_Micros() - start;
            *rxLen = MD_DlcToBytes(h.DataLength);
            return MD_XFER_OK;
        }
        LOG("  (foreign frame id=%lu len=%lu: %02X %02X %02X %02X)",
            h.Identifier, MD_DlcToBytes(h.DataLength), rx[0], rx[1], rx[2], rx[3]);
    }
    MD_AbortPendingTx();
    return MD_XFER_TIMEOUT;
}

/* ============================================================================
 * Diagnostics
 * ========================================================================== */
void MD_PrintConfig(void)
{
    uint32_t clk  = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN);
    uint32_t nTq  = 1U + sBus->Init.NominalTimeSeg1 + sBus->Init.NominalTimeSeg2;
    uint32_t dTq  = 1U + sBus->Init.DataTimeSeg1 + sBus->Init.DataTimeSeg2;
    LOG("[CLK] CM4 core %lu Hz, FDCAN kernel %lu Hz", SystemCoreClock, clk);
    LOG("[CAN] motors on %s", sBus->Instance == FDCAN1 ? "FDCAN1 (PD0 RX / PD1 TX)"
                            : sBus->Instance == FDCAN2 ? "FDCAN2 (PB5 RX / PB6 TX)" : "?");
    LOG("[CAN] nominal %lu bps (pre %lu, %lu tq), data %lu bps (pre %lu, %lu tq), started=%s",
        clk / (sBus->Init.NominalPrescaler * nTq), sBus->Init.NominalPrescaler, nTq,
        clk / (sBus->Init.DataPrescaler * dTq), sBus->Init.DataPrescaler, dTq,
        sBus->State == HAL_FDCAN_STATE_BUSY ? "yes" : "NO");
}

void MD_UpdateBusDiag(void)
{
    HAL_FDCAN_GetProtocolStatus(sBus, &mdProtocolStatus);
    HAL_FDCAN_GetErrorCounters(sBus, &mdErrorCounters);
}

void MD_PrintBusDiag(void)
{
    /* LEC: 0 none, 1 stuff, 2 form, 3 ACK, 4 bit1, 5 bit0, 6 CRC, 7 no change */
    LOG("[DIAG] busOff=%lu errPassive=%lu warn=%lu TEC=%lu REC=%lu LEC=%lu DLEC=%lu",
        mdProtocolStatus.BusOff, mdProtocolStatus.ErrorPassive, mdProtocolStatus.Warning,
        mdErrorCounters.TxErrorCnt, mdErrorCounters.RxErrorCnt,
        mdProtocolStatus.LastErrorCode, mdProtocolStatus.DataLastErrorCode);
    if (mdProtocolStatus.LastErrorCode == FDCAN_PROTOCOL_ERROR_ACK)
        LOG("[DIAG] LEC=ACK error: nobody acknowledged (wiring/transceiver/termination/motor power?)");
    if (mdProtocolStatus.BusOff)
        LOG("[DIAG] bus-off: controller stopped sending (bit timing or bus short?)");
}

/* ============================================================================
 * Ping = read firmwareVersion, like MD::init() in the CANdle SDK
 * ========================================================================== */
bool MD_Ping(MD_Motor_t* motor)
{
    uint8_t tx[8] = {MD_FRAME_READ_REGISTER, 0x00};
    MD_PutReg(tx, 2, MD_REG_FIRMWARE_VERSION, "\0\0\0\0", 4);
    uint8_t  rx[64];
    uint32_t rxLen, rtt;

    motor->alive = false;
    MD_DelayUs(MD_INTER_FRAME_US);
    MD_Xfer_E res = md_transact(motor->canId, tx, sizeof(tx), rx, &rxLen, &rtt);
    if (res == MD_XFER_TX_FAIL)
    {
        LOG("[PING] id %3u : TX FIFO full / FDCAN not started", motor->canId);
    }
    else if (res == MD_XFER_TIMEOUT)
    {
        LOG("[PING] id %3u : no response", motor->canId);
    }
    /* Like the SDK, don't check rx[0]: fw 2.5.x echoes 0x41, fw 2.4.x replies 0x00.
     * Reject only error frames and replies that don't echo the register address. */
    else if (rx[0] == MD_FRAME_RESPONSE_ERROR || rx[2] != tx[2] || rx[3] != tx[3])
    {
        LOG("[PING] id %3u : bad response %02X %02X %02X %02X %02X %02X %02X %02X",
            motor->canId, rx[0], rx[1], rx[2], rx[3], rx[4], rx[5], rx[6], rx[7]);
    }
    else
    {
        /* version_ut little-endian: {tag, revision, minor, major} */
        motor->tag      = (char)rx[4];
        motor->revision = rx[5];
        motor->minor    = rx[6];
        motor->major    = rx[7];
        motor->rttUs    = rtt;
        motor->alive    = (motor->major != 0U);
        LOG("[PING] id %3u : %s fw v%u.%u.%u%c  rtt %lu us", motor->canId,
            motor->alive ? "OK " : "BAD", motor->major, motor->minor, motor->revision,
            (motor->tag >= 0x20 && motor->tag < 0x7F) ? motor->tag : ' ', rtt);
    }

    if (motor->alive) motor->pingOk++;
    else              motor->pingFail++;
    return motor->alive;
}

uint8_t MD_PingAll(void)
{
    uint8_t found = 0U;
    for (uint32_t i = 0; i < MD_NUM_MOTORS; i++)
        if (MD_Ping(&mdMotors[i]))
            found++;
    MD_UpdateBusDiag();
    return found;
}

/* ============================================================================
 * Register access
 * ========================================================================== */
/* The ack is the drive's default response (0xA0); only 0xA1 means the write was rejected. */
bool MD_WriteRegs(MD_Motor_t* motor, const uint8_t* payload, uint32_t len, const char* what)
{
    uint8_t tx[64] = {MD_FRAME_WRITE_REGISTER_LEGACY, 0x00};
    uint8_t rx[64];
    uint32_t rxLen, rtt;
    if (len > sizeof(tx) - 2U)
        return false;
    memcpy(&tx[2], payload, len);

    MD_DelayUs(MD_INTER_FRAME_US);
    if (md_transact(motor->canId, tx, 2U + len, rx, &rxLen, &rtt) != MD_XFER_OK)
    {
        LOG("[CFG] id %3u : %s -> no response", motor->canId, what);
        return false;
    }
    if (rx[0] == MD_FRAME_RESPONSE_ERROR)
    {
        LOG("[CFG] id %3u : %s -> error code %d, reg 0x%02X%02X",
            motor->canId, what, (int8_t)rx[1], rx[3], rx[2]);
        return false;
    }
    LOG("[CFG] id %3u : %s -> ok (ack 0x%02X)", motor->canId, what, rx[0]);
    return true;
}

/* Read one register with 0x41. value gets the raw bytes after the echoed address. */
bool MD_ReadReg(MD_Motor_t* motor, uint16_t reg, void* value, uint32_t size)
{
    uint8_t tx[8] = {MD_FRAME_READ_REGISTER, 0x00};
    uint8_t rx[64];
    uint32_t rxLen, rtt;
    if (size > 4U)
        return false;
    MD_PutReg(tx, 2, reg, "\0\0\0\0", size);

    MD_DelayUs(MD_INTER_FRAME_US);
    /* Like the SDK, don't check rx[0]: fw 2.5.x echoes 0x41, fw 2.4.x replies 0x00. */
    if (md_transact(motor->canId, tx, 4U + size, rx, &rxLen, &rtt) != MD_XFER_OK)
        return false;
    if (rx[0] == MD_FRAME_RESPONSE_ERROR || rx[2] != tx[2] || rx[3] != tx[3])
        return false;
    memcpy(value, &rx[4], size);
    return true;
}

/* ============================================================================
 * Impedance mode with all gains / targets at zero, then enable
 * ========================================================================== */
bool MD_ConfigureImpedanceZero(MD_Motor_t* motor)
{
    const float    zero   = 0.0f;
    const uint8_t  mode   = MD_MODE_IMPEDANCE;
    const uint16_t enable = MD_STATE_ENABLE;
    uint8_t  p[62];
    uint32_t n = 0;

    n = MD_PutReg(p, n, MD_REG_IMP_KP,        &zero, 4);
    n = MD_PutReg(p, n, MD_REG_IMP_KD,        &zero, 4);
    n = MD_PutReg(p, n, MD_REG_TARGET_POS,    &zero, 4);
    n = MD_PutReg(p, n, MD_REG_TARGET_VEL,    &zero, 4);
    n = MD_PutReg(p, n, MD_REG_TARGET_TORQUE, &zero, 4);
    if (!MD_WriteRegs(motor, p, n, "kp=kd=0, targets=0")) return false;

    n = MD_PutReg(p, 0, MD_REG_MOTION_MODE_CMD, &mode, 1);
    if (!MD_WriteRegs(motor, p, n, "motionMode=IMPEDANCE")) return false;

    n = MD_PutReg(p, 0, MD_REG_STATE, &enable, 2);
    if (!MD_WriteRegs(motor, p, n, "enable")) return false;

    uint8_t  modeStat = 0;
    uint16_t watchdog = 0;
    if (MD_ReadReg(motor, MD_REG_MOTION_MODE_STAT, &modeStat, 1) &&
        MD_ReadReg(motor, MD_REG_CAN_WATCHDOG, &watchdog, 2))
    {
        LOG("[CFG] id %3u : motionModeStatus=%u canWatchdog=%u ms",
            motor->canId, modeStat, watchdog);
    }
    return true;
}

void MD_Disable(MD_Motor_t* motor)
{
    const uint16_t disable = MD_STATE_DISABLE;
    uint8_t p[8];
    uint32_t n = MD_PutReg(p, 0, MD_REG_STATE, &disable, 2);
    MD_WriteRegs(motor, p, n, "disable");
    motor->enabled = false;
}

bool MD_ParseDefaultResponse(MD_Motor_t* motor, const uint8_t* d, uint32_t len)
{
    if (len < 16U || d[0] != MD_FRAME_RESPONSE_LEGACY)
        return false;
    memcpy(&motor->quickStatus, &d[1], 2);
    motor->temperature = d[3];
    memcpy(&motor->pos, &d[4], 4);
    memcpy(&motor->vel, &d[8], 4);
    memcpy(&motor->tau, &d[12], 4);
    return true;
}
