/**
 * fdcan_m4.ino - Portenta H7 M4 core : FDCAN link to MAB MD drivers
 * ============================================================================
 * 500 Hz impedance-mode loop, stability test (all gains / targets zero).
 *
 * Planned system:  host PC <-UDP-> M7 <-shared memory-> M4 <-FDCAN-> 6 x MD
 * The M4 owns the bus. Commands live in cmd[], feedback in state[]; later the
 * M7 will fill cmd[] and read state[] through shared memory.
 *
 * Startup
 *   1. Ping: read firmwareVersion (0x802) from every motor (CANdle-SDK MD::init()).
 *   2. Per motor (0x40 writes, fw 2.4.x ignores 0x42): kp = kd = 0, targets = 0,
 *      motionMode = IMPEDANCE (4), state = 39 (enable).
 *
 * Cycle (every CYCLE_PERIOD_US, timed with micros(): millis() is ~1.7 % off on the M4)
 *   Pipelined: queue one 32-byte 0x40 command per motor, then collect the replies.
 *     command: [0x40][0x00] [0x150 pos] [0x151 vel] [0x152 tau] [0x050 kp] [0x051 kd]
 *     reply  : 24 B  [0xA0][quickStatus u16][temp u8][pos f32][vel f32][tau f32][8 B fw 2.5 extra]
 *   Benchmark result: 849 us avg / 896 us max for 6 motors, 0 lost replies.
 *   (20-byte pos/vel/tau commands lose replies on fw 2.4 -> always send all 5 fields.)
 *
 * Safety: all motors are disabled and the loop stops on
 *   - MAX_CONSEC_MISSES cycles in a row without a reply from one motor
 *     (stays far below the drives' 100 ms CAN watchdog)
 *   - a quickStatus error bit (0..6) on any motor
 *   - FDCAN bus-off
 *
 * Bus: FDCAN1, PH13 = TX, PB8 = RX. 1 Mbps nominal / 8 Mbps data (BRS).
 * Output: SerialRPC -> serial_bridge_m7 on the M7 core -> USB Serial.
 * LED   : green = running, red = fault (motors disabled), blue = FDCAN init failed.
 * ============================================================================
 */

#include <Arduino.h>
#include <RPC.h>
#include <SerialRPC.h>
#include <cstring>
#include <cmath>
#include "stm32h7xx_hal.h"

#define LOG SerialRPC

// ─── User configuration ───
static const uint16_t MOTOR_IDS[]        = {10, 20, 30, 40, 50, 60};
static const int      NUM_MOTORS         = sizeof(MOTOR_IDS) / sizeof(MOTOR_IDS[0]);
static const uint32_t NOMINAL_BITRATE    = 1000000;   // arbitration phase
static const uint32_t DATA_BITRATE       = 8000000;   // must match MD canBaudrate register
static const uint32_t CYCLE_PERIOD_US    = 2000;      // 500 Hz
static const uint32_t UPDATE_TIMEOUT_US  = 1500;      // wait for replies, leaves slack in the period
static const uint32_t REPLY_TIMEOUT_US   = 3000;      // blocking config transactions
static const uint32_t INTER_FRAME_US     = 300;       // MD drops frames arriving right after a reply
static const uint32_t REPORT_PERIOD_US   = 5000000;   // status report every 5 s
static const uint32_t MAX_CONSEC_MISSES  = 5;         // 10 ms at 500 Hz -> fault
static const uint16_t QS_ERROR_MASK      = 0x007F;    // quickStatus bits 0..6 are error flags

// Command limits (clamped before sending)
static const float KP_MAX  = 50.0f;   // Nm/rad
static const float KD_MAX  = 5.0f;    // Nm*s/rad
static const float TAU_MAX = 2.0f;    // Nm feed-forward

// Desired FDCAN kernel clock if we must generate it ourselves (80 MHz -> 10 tq @ 8 Mbps)
static const uint32_t FDCAN_TARGET_CLK   = 80000000;

// ─── MD protocol (CANdle-SDK commons/shared_data/md_types.hpp, mab_types.hpp) ───
enum : uint8_t {
    MD_FRAME_WRITE_REGISTER_LEGACY = 0x40,
    MD_FRAME_READ_REGISTER         = 0x41,
    MD_FRAME_WRITE_REGISTER        = 0x42,
    MD_FRAME_RESPONSE_LEGACY       = 0xA0,
    MD_FRAME_RESPONSE_ERROR        = 0xA1,
};
static const uint16_t MD_REG_CAN_WATCHDOG     = 0x003;  // u16 ms, RW
static const uint16_t MD_REG_IMP_KP           = 0x050;  // f32, RW
static const uint16_t MD_REG_IMP_KD           = 0x051;  // f32, RW
static const uint16_t MD_REG_MOTION_MODE_CMD  = 0x140;  // u8,  WO
static const uint16_t MD_REG_MOTION_MODE_STAT = 0x141;  // u8,  RO
static const uint16_t MD_REG_STATE            = 0x142;  // u16, RW (39 enable, 64 disable)
static const uint16_t MD_REG_TARGET_POS       = 0x150;  // f32, RW
static const uint16_t MD_REG_TARGET_VEL       = 0x151;  // f32, RW
static const uint16_t MD_REG_TARGET_TORQUE    = 0x152;  // f32, RW
static const uint16_t MD_REG_FIRMWARE_VERSION = 0x802;  // u32, RO
static const uint8_t  MD_MODE_IMPEDANCE       = 4;
static const uint16_t MD_STATE_ENABLE         = 39;
static const uint16_t MD_STATE_DISABLE        = 64;

// ─── FDCAN state ───
static FDCAN_HandleTypeDef   hfdcan;
static FDCAN_TxHeaderTypeDef txHeader;
static uint32_t              fdcanClk = 0;

struct MotorStatus {
    bool     alive;     // answered ping
    bool     enabled;   // configured + enabled
    uint8_t  major, minor, revision;
    char     tag;
};
static MotorStatus motors[NUM_MOTORS];

// Command sent every cycle (impedance: tau = kp*(pos-q) + kd*(vel-dq) + tau_ff)
struct MotorCommand {
    float pos, vel, tau, kp, kd;
};
// Latest feedback from the drive's 0x40 default response
struct MotorState {
    float    pos, vel, tau;
    uint16_t quickStatus;
    uint8_t  temperature;
    bool     valid;          // at least one reply received
    uint32_t consecMissed;   // cycles in a row without a reply
};
static MotorCommand cmd[NUM_MOTORS];
static MotorState   state[NUM_MOTORS];

// Per-motor reliability counters (window = since last report, total = since start)
struct MotorCounters {
    uint32_t missedWin, missedTotal;
    uint32_t invalidWin, invalidTotal;
    uint32_t maxConsecMissed;
    uint32_t latSumWin, latNWin, latMaxWin;
};
static MotorCounters counters[NUM_MOTORS];

// Loop timing statistics
struct LoopStats {
    uint32_t cycles, failedCycles;
    uint32_t updSum, updMin, updMax;   // command sent -> all replies in
    uint32_t lateMax;                  // cycle start later than scheduled
    uint32_t overruns;                 // whole cycle (incl. printing) longer than the period
    void reset() {
        cycles = failedCycles = updSum = updMax = lateMax = overruns = 0;
        updMin = 0xFFFFFFFF;
    }
};

// ============================================================================
// Helpers
// ============================================================================
static void setLed(bool r, bool g, bool b) {
    // Portenta LEDs are active LOW
    digitalWrite(LEDR, r ? LOW : HIGH);
    digitalWrite(LEDG, g ? LOW : HIGH);
    digitalWrite(LEDB, b ? LOW : HIGH);
}

static uint32_t dlcToBytes(uint32_t dlc) {
    switch (dlc) {
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

// Smallest DLC that holds len bytes (frame is zero padded, MD stops at reg 0x0000)
static uint32_t bytesToDlc(uint32_t len) {
    if (len <= 8)  return FDCAN_DLC_BYTES_8;
    if (len <= 12) return FDCAN_DLC_BYTES_12;
    if (len <= 16) return FDCAN_DLC_BYTES_16;
    if (len <= 20) return FDCAN_DLC_BYTES_20;
    if (len <= 24) return FDCAN_DLC_BYTES_24;
    if (len <= 32) return FDCAN_DLC_BYTES_32;
    if (len <= 48) return FDCAN_DLC_BYTES_48;
    return FDCAN_DLC_BYTES_64;
}

// Append [regLSB][regMSB][value bytes] to a register frame
static uint32_t putReg(uint8_t* buf, uint32_t idx, uint16_t reg, const void* val, uint32_t size) {
    buf[idx++] = reg & 0xFF;
    buf[idx++] = reg >> 8;
    memcpy(&buf[idx], val, size);
    return idx + size;
}

// Find prescaler/segments for an exact bitrate. Sync seg = 1 tq,
// sample point at samplePct. Smallest prescaler first (most tq per bit).
static bool calcBitTiming(uint32_t clk, uint32_t bitrate, uint32_t samplePct,
                          uint32_t maxPre, uint32_t maxSeg1, uint32_t maxSeg2,
                          uint32_t& pre, uint32_t& seg1, uint32_t& seg2) {
    const uint32_t MIN_TQ = 5;
    for (uint32_t p = 1; p <= maxPre; p++) {
        if (clk % (bitrate * p) != 0) continue;
        uint32_t tq = clk / (bitrate * p);
        if (tq < MIN_TQ) return false;  // larger prescalers only make it worse
        uint32_t s1 = (tq * samplePct) / 100 - 1;
        uint32_t s2 = tq - 1 - s1;
        if (s1 >= 1 && s1 <= maxSeg1 && s2 >= 1 && s2 <= maxSeg2) {
            pre = p; seg1 = s1; seg2 = s2;
            return true;
        }
    }
    return false;
}

static bool clockSupportsBitrates(uint32_t clk) {
    uint32_t p, s1, s2;
    return clk != 0 &&
           calcBitTiming(clk, NOMINAL_BITRATE, 80, 512, 256, 128, p, s1, s2) &&
           calcBitTiming(clk, DATA_BITRATE,    80,  32,  32,  16, p, s1, s2);
}

// Frequency of the oscillator feeding PLL1/2/3 (they share PLLSRC)
static uint32_t pllSourceFreq() {
    switch (__HAL_RCC_GET_PLL_OSCSOURCE()) {
        case RCC_PLLSOURCE_HSE: return HSE_VALUE;
        case RCC_PLLSOURCE_CSI: return CSI_VALUE;
        case RCC_PLLSOURCE_HSI: return HSI_VALUE >> ((RCC->CR & RCC_CR_HSIDIV) >> RCC_CR_HSIDIV_Pos);
        default:                return 0;
    }
}

// Configure PLL2 so that PLL2Q = FDCAN_TARGET_CLK (only if PLL2 is unused)
static bool startPll2ForFdcan() {
    uint32_t src = pllSourceFreq();
    const uint32_t VCO = FDCAN_TARGET_CLK * 10;   // 800 MHz, Q = 10
    for (uint32_t m = 1; m <= 63; m++) {
        if (src % m) continue;
        uint32_t ref = src / m;
        if (ref < 2000000 || ref > 16000000 || VCO % ref) continue;
        uint32_t n = VCO / ref;
        if (n < 4 || n > 512) continue;

        RCC_PeriphCLKInitTypeDef pclk = {0};
        pclk.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
        pclk.FdcanClockSelection  = RCC_FDCANCLKSOURCE_PLL2;
        pclk.PLL2.PLL2M = m;
        pclk.PLL2.PLL2N = n;
        pclk.PLL2.PLL2P = 10;
        pclk.PLL2.PLL2Q = 10;
        pclk.PLL2.PLL2R = 10;
        pclk.PLL2.PLL2RGE = ref < 4000000 ? RCC_PLL2VCIRANGE_1
                          : ref < 8000000 ? RCC_PLL2VCIRANGE_2
                                          : RCC_PLL2VCIRANGE_3;
        pclk.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
        pclk.PLL2.PLL2FRACN  = 0;
        return HAL_RCCEx_PeriphCLKConfig(&pclk) == HAL_OK;
    }
    return false;
}

// Pick an FDCAN kernel clock that can produce 1M / 8M exactly.
static bool setupFdcanClock() {
    uint32_t cur = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN);
    LOG.print("[CLK] FDCAN kernel clock now: "); LOG.print(cur); LOG.println(" Hz");
    if (clockSupportsBitrates(cur)) return true;

    // PLL1Q, if it is already running and enabled
    if (RCC->PLLCFGR & RCC_PLLCFGR_DIVQ1EN) {
        PLL1_ClocksTypeDef p1;
        HAL_RCCEx_GetPLL1ClockFreq(&p1);
        LOG.print("[CLK] PLL1Q = "); LOG.println(p1.PLL1_Q_Frequency);
        if (clockSupportsBitrates(p1.PLL1_Q_Frequency)) {
            __HAL_RCC_FDCAN_CONFIG(RCC_FDCANCLKSOURCE_PLL);
            return true;
        }
    }

    // PLL2: reuse if running with a suitable Q, otherwise start it ourselves
    if (RCC->CR & RCC_CR_PLL2RDY) {
        PLL2_ClocksTypeDef p2;
        HAL_RCCEx_GetPLL2ClockFreq(&p2);
        LOG.print("[CLK] PLL2 already in use, Q = "); LOG.println(p2.PLL2_Q_Frequency);
        if ((RCC->PLLCFGR & RCC_PLLCFGR_DIVQ2EN) && clockSupportsBitrates(p2.PLL2_Q_Frequency)) {
            __HAL_RCC_FDCAN_CONFIG(RCC_FDCANCLKSOURCE_PLL2);
            return true;
        }
        LOG.println("[CLK] ERROR: PLL2 busy and unsuitable, cannot reach the data bitrate");
        return false;
    }

    LOG.print("[CLK] Starting PLL2 for FDCAN, source osc = "); LOG.println(pllSourceFreq());
    return startPll2ForFdcan();
}

// ============================================================================
// FDCAN init
// ============================================================================
static bool fdcanInit() {
    if (!setupFdcanClock()) return false;
    fdcanClk = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN);
    LOG.print("[CLK] FDCAN kernel clock used: "); LOG.print(fdcanClk); LOG.println(" Hz");

    uint32_t nPre, nSeg1, nSeg2, dPre, dSeg1, dSeg2;
    if (!calcBitTiming(fdcanClk, NOMINAL_BITRATE, 80, 512, 256, 128, nPre, nSeg1, nSeg2) ||
        !calcBitTiming(fdcanClk, DATA_BITRATE,    80,  32,  32,  16, dPre, dSeg1, dSeg2)) {
        LOG.println("[CAN] ERROR: no exact bit timing for this clock");
        return false;
    }

    __HAL_RCC_FDCAN_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF9_FDCAN1;
    gpio.Pin = GPIO_PIN_8;  HAL_GPIO_Init(GPIOB, &gpio);   // RX
    gpio.Pin = GPIO_PIN_13; HAL_GPIO_Init(GPIOH, &gpio);   // TX

    hfdcan.Instance                  = FDCAN1;
    hfdcan.Init.FrameFormat          = FDCAN_FRAME_FD_BRS;
    hfdcan.Init.Mode                 = FDCAN_MODE_NORMAL;
    // Must be ENABLED for pipelining: a queued command that loses arbitration to a
    // motor's reply (lower ID wins) would otherwise be dropped. Frames that are never
    // ACKed are cancelled with abortPendingTx() on timeout instead.
    hfdcan.Init.AutoRetransmission   = ENABLE;
    hfdcan.Init.TransmitPause        = DISABLE;
    hfdcan.Init.ProtocolException    = DISABLE;

    hfdcan.Init.NominalPrescaler     = nPre;
    hfdcan.Init.NominalTimeSeg1      = nSeg1;
    hfdcan.Init.NominalTimeSeg2      = nSeg2;
    hfdcan.Init.NominalSyncJumpWidth = nSeg2;

    hfdcan.Init.DataPrescaler        = dPre;
    hfdcan.Init.DataTimeSeg1         = dSeg1;
    hfdcan.Init.DataTimeSeg2         = dSeg2;
    hfdcan.Init.DataSyncJumpWidth    = dSeg2;

    hfdcan.Init.MessageRAMOffset     = 0;
    hfdcan.Init.StdFiltersNbr        = 0;
    hfdcan.Init.ExtFiltersNbr        = 0;
    hfdcan.Init.RxFifo0ElmtsNbr      = 16;
    hfdcan.Init.RxFifo0ElmtSize      = FDCAN_DATA_BYTES_64;
    hfdcan.Init.RxFifo1ElmtsNbr      = 0;
    hfdcan.Init.RxBuffersNbr         = 0;
    hfdcan.Init.TxEventsNbr          = 0;
    hfdcan.Init.TxBuffersNbr         = 0;
    hfdcan.Init.TxFifoQueueElmtsNbr  = 16;
    hfdcan.Init.TxFifoQueueMode      = FDCAN_TX_FIFO_OPERATION;
    hfdcan.Init.TxElmtSize           = FDCAN_DATA_BYTES_64;

    if (HAL_FDCAN_Init(&hfdcan) != HAL_OK) { LOG.println("[CAN] HAL_FDCAN_Init failed"); return false; }

    // Transmitter delay compensation is mandatory at high data rates
    HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan, dPre * dSeg1, 0);
    HAL_FDCAN_EnableTxDelayCompensation(&hfdcan);

    // No filters: accept every standard/extended frame into FIFO0
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0,
                                 FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);

    if (HAL_FDCAN_Start(&hfdcan) != HAL_OK) { LOG.println("[CAN] HAL_FDCAN_Start failed"); return false; }

    txHeader.IdType              = FDCAN_STANDARD_ID;
    txHeader.TxFrameType         = FDCAN_DATA_FRAME;
    txHeader.DataLength          = FDCAN_DLC_BYTES_8;
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHeader.BitRateSwitch       = FDCAN_BRS_ON;
    txHeader.FDFormat            = FDCAN_FD_CAN;
    txHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    txHeader.MessageMarker       = 0;

    char buf[160];
    snprintf(buf, sizeof(buf),
             "[CAN] FDCAN1 up: nominal %lu bps (pre %lu, %lu+%lu+1 tq), data %lu bps (pre %lu, %lu+%lu+1 tq)",
             NOMINAL_BITRATE, nPre, nSeg1, nSeg2, DATA_BITRATE, dPre, dSeg1, dSeg2);
    LOG.println(buf);
    return true;
}

static void flushRx() {
    FDCAN_RxHeaderTypeDef h;
    uint8_t d[64];
    while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan, FDCAN_RX_FIFO0) > 0)
        HAL_FDCAN_GetRxMessage(&hfdcan, FDCAN_RX_FIFO0, &h, d);
}

// Cancel every TX FIFO request still pending (e.g. retrying because nobody ACKs)
static void abortPendingTx() {
    HAL_FDCAN_AbortTxRequest(&hfdcan, (1u << 16) - 1);   // all 16 TX elements
}

static bool sendFrame(uint16_t canId, const uint8_t* tx, uint32_t txLen) {
    txHeader.Identifier = canId;
    txHeader.DataLength = bytesToDlc(txLen);
    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan, &txHeader, (uint8_t*)tx) == HAL_OK;
}

static void printBusDiag() {
    FDCAN_ProtocolStatusTypeDef ps = {0};
    FDCAN_ErrorCountersTypeDef  ec = {0};
    HAL_FDCAN_GetProtocolStatus(&hfdcan, &ps);
    HAL_FDCAN_GetErrorCounters(&hfdcan, &ec);
    // LEC: 0 none, 1 stuff, 2 form, 3 ACK, 4 bit1, 5 bit0, 6 CRC, 7 no change
    char buf[128];
    snprintf(buf, sizeof(buf),
             "[DIAG] busOff=%lu errPassive=%lu warn=%lu TEC=%lu REC=%lu LEC=%lu DLEC=%lu",
             ps.BusOff, ps.ErrorPassive, ps.Warning, ec.TxErrorCnt, ec.RxErrorCnt,
             ps.LastErrorCode, ps.DataLastErrorCode);
    LOG.println(buf);
    if (ps.LastErrorCode == 3) LOG.println("[DIAG] LEC=ACK error: nobody acknowledged (wiring/transceiver/termination/power?)");
}

// ============================================================================
// Blocking MD transaction: send frame to canId, wait for its reply.
// rx must hold 64 bytes. Frames from other IDs are printed and skipped.
// ============================================================================
static bool mdTransact(uint16_t canId, const uint8_t* tx, uint32_t txLen,
                       uint8_t* rx, uint32_t& rxLen, uint32_t& rttUs) {
    flushRx();
    if (!sendFrame(canId, tx, txLen)) {
        LOG.println("[CAN] TX FIFO full / error");
        return false;
    }

    uint32_t start = micros();
    FDCAN_RxHeaderTypeDef h;
    while ((uint32_t)(micros() - start) < REPLY_TIMEOUT_US) {
        if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan, FDCAN_RX_FIFO0) == 0) continue;
        if (HAL_FDCAN_GetRxMessage(&hfdcan, FDCAN_RX_FIFO0, &h, rx) != HAL_OK) continue;

        if (h.Identifier == canId) {
            rttUs = micros() - start;
            rxLen = dlcToBytes(h.DataLength);
            return true;
        }
        char buf[96];
        snprintf(buf, sizeof(buf), "  (foreign frame id=%lu len=%lu: %02X %02X %02X %02X)",
                 h.Identifier, dlcToBytes(h.DataLength), rx[0], rx[1], rx[2], rx[3]);
        LOG.println(buf);
    }
    abortPendingTx();
    return false;
}

// Write registers and check the ack. Payload = putReg() entries.
// Uses 0x40 (WRITE_REGISTER_LEGACY): fw 2.4.x silently ignores 0x42, fw 2.5.x accepts both.
// The ack is the drive's default response (0xA0); only 0xA1 means the write was rejected.
static bool mdWrite(int i, const uint8_t* payload, uint32_t payloadLen, const char* what) {
    uint8_t tx[64] = {MD_FRAME_WRITE_REGISTER_LEGACY, 0x00};
    memcpy(&tx[2], payload, payloadLen);
    uint8_t rx[64];
    uint32_t rxLen, rtt;
    char buf[128];

    delayMicroseconds(INTER_FRAME_US);
    bool ok = mdTransact(MOTOR_IDS[i], tx, 2 + payloadLen, rx, rxLen, rtt);
    if (!ok) {
        snprintf(buf, sizeof(buf), "[CFG] id %3u : %s -> no response", MOTOR_IDS[i], what);
    } else if (rx[0] == MD_FRAME_RESPONSE_ERROR) {
        ok = false;
        snprintf(buf, sizeof(buf), "[CFG] id %3u : %s -> error code %d, reg 0x%02X%02X",
                 MOTOR_IDS[i], what, (int8_t)rx[1], rx[3], rx[2]);
    } else {
        snprintf(buf, sizeof(buf), "[CFG] id %3u : %s -> ok (ack 0x%02X)", MOTOR_IDS[i], what, rx[0]);
    }
    LOG.println(buf);
    return ok;
}

// Read one register with 0x41. value gets the raw bytes after the echoed address.
static bool mdRead(int i, uint16_t reg, uint8_t* value, uint32_t size) {
    uint8_t tx[8] = {MD_FRAME_READ_REGISTER, 0x00};
    putReg(tx, 2, reg, "\0\0\0\0", size);
    uint8_t rx[64];
    uint32_t rxLen, rtt;
    delayMicroseconds(INTER_FRAME_US);
    // Like the SDK, don't check rx[0]: fw 2.5.x echoes 0x41, fw 2.4.x replies 0x00.
    if (!mdTransact(MOTOR_IDS[i], tx, 4 + size, rx, rxLen, rtt)) return false;
    if (rx[0] == MD_FRAME_RESPONSE_ERROR || rx[2] != tx[2] || rx[3] != tx[3]) return false;
    memcpy(value, &rx[4], size);
    return true;
}

// ============================================================================
// Step 1: ping = read firmwareVersion, like MD::init() in the CANdle SDK
// ============================================================================
static bool pingMotor(int i) {
    MotorStatus& m = motors[i];
    uint8_t v[4];
    char buf[96];
    if (!mdRead(i, MD_REG_FIRMWARE_VERSION, v, 4)) {
        snprintf(buf, sizeof(buf), "[PING] id %3u : no / bad response", MOTOR_IDS[i]);
        LOG.println(buf);
        return m.alive = false;
    }
    // version_ut little-endian: {tag, revision, minor, major}
    m.tag = (char)v[0]; m.revision = v[1]; m.minor = v[2]; m.major = v[3];
    m.alive = (m.major != 0);
    snprintf(buf, sizeof(buf), "[PING] id %3u : %s fw v%u.%u.%u%c", MOTOR_IDS[i],
             m.alive ? "OK " : "BAD", m.major, m.minor, m.revision,
             (m.tag >= 0x20 && m.tag < 0x7F) ? m.tag : ' ');
    LOG.println(buf);
    return m.alive;
}

// ============================================================================
// Step 2: impedance mode with all gains / targets at zero, then enable
// ============================================================================
static bool configureImpedanceZero(int i) {
    const float zero = 0.0f;
    uint8_t p[62];
    uint32_t n = 0;
    n = putReg(p, n, MD_REG_IMP_KP,        &zero, 4);
    n = putReg(p, n, MD_REG_IMP_KD,        &zero, 4);
    n = putReg(p, n, MD_REG_TARGET_POS,    &zero, 4);
    n = putReg(p, n, MD_REG_TARGET_VEL,    &zero, 4);
    n = putReg(p, n, MD_REG_TARGET_TORQUE, &zero, 4);
    if (!mdWrite(i, p, n, "kp=kd=0, targets=0")) return false;

    n = putReg(p, 0, MD_REG_MOTION_MODE_CMD, &MD_MODE_IMPEDANCE, 1);
    if (!mdWrite(i, p, n, "motionMode=IMPEDANCE")) return false;

    n = putReg(p, 0, MD_REG_STATE, &MD_STATE_ENABLE, 2);
    if (!mdWrite(i, p, n, "enable")) return false;

    uint8_t mode = 0;
    uint16_t wd = 0;
    if (mdRead(i, MD_REG_MOTION_MODE_STAT, &mode, 1) && mdRead(i, MD_REG_CAN_WATCHDOG, (uint8_t*)&wd, 2)) {
        char buf[96];
        snprintf(buf, sizeof(buf), "[CFG] id %3u : motionModeStatus=%u canWatchdog=%u ms",
                 MOTOR_IDS[i], mode, wd);
        LOG.println(buf);
    }
    return true;
}

static void disableMotor(int i) {
    uint8_t p[8];
    uint32_t n = putReg(p, 0, MD_REG_STATE, &MD_STATE_DISABLE, 2);
    mdWrite(i, p, n, "disable");
}

// ============================================================================
// Step 3: 500 Hz pipelined impedance update
// ============================================================================
static float clampf(float v, float lo, float hi) {
    if (!std::isfinite(v)) return 0.0f;
    return v < lo ? lo : (v > hi ? hi : v);
}

static void queueCommand(int i, uint32_t& txFailures) {
    const MotorCommand& c = cmd[i];
    float pos = std::isfinite(c.pos) ? c.pos : state[i].pos;
    float vel = clampf(c.vel, -1e3f, 1e3f);
    float tau = clampf(c.tau, -TAU_MAX, TAU_MAX);
    float kp  = clampf(c.kp, 0.0f, KP_MAX);
    float kd  = clampf(c.kd, 0.0f, KD_MAX);

    uint8_t f[32];
    uint32_t n = 0;
    f[n++] = MD_FRAME_WRITE_REGISTER_LEGACY;
    f[n++] = 0x00;
    n = putReg(f, n, MD_REG_TARGET_POS,    &pos, 4);
    n = putReg(f, n, MD_REG_TARGET_VEL,    &vel, 4);
    n = putReg(f, n, MD_REG_TARGET_TORQUE, &tau, 4);
    n = putReg(f, n, MD_REG_IMP_KP,        &kp, 4);
    n = putReg(f, n, MD_REG_IMP_KD,        &kd, 4);
    if (!sendFrame(MOTOR_IDS[i], f, n)) txFailures++;
}

// Returns false if the reply is not a usable default response
static bool parseDefaultResponse(int i, const uint8_t* d, uint32_t len) {
    if (len < 16 || d[0] != MD_FRAME_RESPONSE_LEGACY) return false;
    MotorState& s = state[i];
    memcpy(&s.quickStatus, &d[1], 2);
    s.temperature = d[3];
    memcpy(&s.pos, &d[4], 4);
    memcpy(&s.vel, &d[8], 4);
    memcpy(&s.tau, &d[12], 4);
    s.valid = true;
    return true;
}

static uint32_t txFailWin = 0, rxLostWin = 0;

// Queue all commands, collect all replies. Returns true if every enabled motor answered.
static bool updateMotors(uint32_t& updUs) {
    bool got[NUM_MOTORS] = {false};
    int expected = 0, received = 0;
    flushRx();
    uint32_t start = micros();
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (!motors[i].enabled) continue;
        queueCommand(i, txFailWin);
        expected++;
    }

    FDCAN_RxHeaderTypeDef h;
    uint8_t d[64];
    while (received < expected && (uint32_t)(micros() - start) < UPDATE_TIMEOUT_US) {
        if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan, FDCAN_RX_FIFO0) == 0) continue;
        if (HAL_FDCAN_GetRxMessage(&hfdcan, FDCAN_RX_FIFO0, &h, d) != HAL_OK) continue;
        uint32_t lat = micros() - start;
        for (int i = 0; i < NUM_MOTORS; i++) {
            if (!motors[i].enabled || got[i] || h.Identifier != MOTOR_IDS[i]) continue;
            if (!parseDefaultResponse(i, d, dlcToBytes(h.DataLength))) {
                counters[i].invalidWin++;
                counters[i].invalidTotal++;
                break;
            }
            got[i] = true;
            received++;
            MotorCounters& c = counters[i];
            c.latSumWin += lat;
            c.latNWin++;
            if (lat > c.latMaxWin) c.latMaxWin = lat;
            break;
        }
    }
    updUs = micros() - start;
    if (received < expected) abortPendingTx();

    if (__HAL_FDCAN_GET_FLAG(&hfdcan, FDCAN_FLAG_RX_FIFO0_MESSAGE_LOST)) {
        __HAL_FDCAN_CLEAR_FLAG(&hfdcan, FDCAN_FLAG_RX_FIFO0_MESSAGE_LOST);
        rxLostWin++;
    }

    for (int i = 0; i < NUM_MOTORS; i++) {
        if (!motors[i].enabled) continue;
        MotorState& s = state[i];
        if (got[i]) { s.consecMissed = 0; continue; }
        counters[i].missedWin++;
        counters[i].missedTotal++;
        if (++s.consecMissed > counters[i].maxConsecMissed) counters[i].maxConsecMissed = s.consecMissed;
    }
    return received == expected;
}

// ============================================================================
// Control hook: runs once per cycle after fresh feedback arrived.
// Fill cmd[i]; it is sent at the start of the next cycle.
// Later the M7 (UDP from the host PC) will provide cmd[] instead.
// ============================================================================
static void controlUpdate() {
    // Stability test: everything stays at zero -> no torque.
    // Example (gentle spring around the startup position on motor 10):
    //   cmd[0].kp = 2.0f; cmd[0].kd = 0.05f;
}

// ============================================================================
// Safety
// ============================================================================
static bool faulted = false;

static void faultStop(const char* reason, int motorIdx) {
    faulted = true;
    for (int i = 0; i < NUM_MOTORS; i++) if (motors[i].enabled) disableMotor(i);
    setLed(true, false, false);
    char buf[128];
    if (motorIdx >= 0)
        snprintf(buf, sizeof(buf), "[FAULT] id %u : %s -> all motors disabled", MOTOR_IDS[motorIdx], reason);
    else
        snprintf(buf, sizeof(buf), "[FAULT] %s -> all motors disabled", reason);
    LOG.println(buf);
    printBusDiag();
}

static void checkSafety() {
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (!motors[i].enabled) continue;
        const MotorState& s = state[i];
        if (s.consecMissed >= MAX_CONSEC_MISSES) {
            char why[48];
            snprintf(why, sizeof(why), "%lu replies missed in a row", s.consecMissed);
            faultStop(why, i);
            return;
        }
        if (s.quickStatus & QS_ERROR_MASK) {
            char why[48];
            snprintf(why, sizeof(why), "quickStatus error 0x%04X", s.quickStatus);
            faultStop(why, i);
            return;
        }
    }
    FDCAN_ProtocolStatusTypeDef ps = {0};
    HAL_FDCAN_GetProtocolStatus(&hfdcan, &ps);
    if (ps.BusOff) faultStop("FDCAN bus-off", -1);
}

// ============================================================================
// Reporting: lines are queued and printed one per cycle so a report never
// blocks the loop for long.
// ============================================================================
static const int  REPORT_LINES = NUM_MOTORS + 3;
static char       reportBuf[REPORT_LINES][200];
static int        reportCount = 0, reportNext = 0;

static void buildReport(const LoopStats& ls, uint32_t windowUs, uint32_t runSec) {
    uint32_t ok = ls.cycles - ls.failedCycles;
    int k = 0;
    snprintf(reportBuf[k++], 200,
             "[%5lus] %.1f Hz  cycles %lu  failed %lu  overruns %lu  late max %lu us",
             runSec, ls.cycles * 1e6f / windowUs, ls.cycles, ls.failedCycles, ls.overruns, ls.lateMax);
    snprintf(reportBuf[k++], 200, "   update: avg %lu us  min %lu  max %lu",
             ok ? ls.updSum / ok : 0, ok ? ls.updMin : 0, ls.updMax);

    FDCAN_ErrorCountersTypeDef ec = {0};
    HAL_FDCAN_GetErrorCounters(&hfdcan, &ec);
    snprintf(reportBuf[k++], 200, "   bus: TEC %lu  REC %lu  txFail %lu  rxLost %lu",
             ec.TxErrorCnt, ec.RxErrorCnt, txFailWin, rxLostWin);

    for (int i = 0; i < NUM_MOTORS; i++) {
        if (!motors[i].enabled) continue;
        const MotorState& s = state[i];
        const MotorCounters& c = counters[i];
        snprintf(reportBuf[k++], 200,
                 "   id %3u : lat avg %4lu max %4lu  missed %lu (total %lu, max run %lu)  invalid %lu  qs 0x%04X  T %2u  pos %8.4f  vel %8.4f  tau %7.4f",
                 MOTOR_IDS[i], c.latNWin ? c.latSumWin / c.latNWin : 0, c.latMaxWin,
                 c.missedWin, c.missedTotal, c.maxConsecMissed, c.invalidTotal,
                 s.quickStatus, s.temperature, s.pos, s.vel, s.tau);
    }
    reportCount = k;
    reportNext = 0;
}

static void resetWindow(LoopStats& ls) {
    ls.reset();
    txFailWin = rxLostWin = 0;
    for (int i = 0; i < NUM_MOTORS; i++) {
        MotorCounters& c = counters[i];
        c.missedWin = c.invalidWin = 0;
        c.latSumWin = c.latNWin = c.latMaxWin = 0;
    }
}

// ============================================================================
static bool      canReady = false;
static int       enabledCount = 0;
static uint32_t  nextCycleUs, windowStartUs, runStartUs;
static LoopStats stats;

void setup() {
    pinMode(LEDR, OUTPUT);
    pinMode(LEDG, OUTPUT);
    pinMode(LEDB, OUTPUT);
    setLed(false, false, false);

    LOG.begin();
    delay(1000);  // give the M7 bridge time to start forwarding
    LOG.println();
    LOG.println("=== Portenta H7 M4 : MAB MD 500 Hz impedance loop (stability test, zero gains) ===");

    memset(motors, 0, sizeof(motors));
    memset(cmd, 0, sizeof(cmd));
    memset(state, 0, sizeof(state));
    memset(counters, 0, sizeof(counters));

    canReady = fdcanInit();
    if (!canReady) { setLed(false, false, true); return; }

    for (int i = 0; i < NUM_MOTORS; i++) pingMotor(i);
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (!motors[i].alive) continue;
        motors[i].enabled = configureImpedanceZero(i);
        if (motors[i].enabled) enabledCount++;
        else disableMotor(i);
    }

    char buf[64];
    snprintf(buf, sizeof(buf), "[CFG] %d/%d motors enabled (impedance, all zero)", enabledCount, NUM_MOTORS);
    LOG.println(buf);
    if (enabledCount < NUM_MOTORS) printBusDiag();
    LOG.println();

    stats.reset();
    setLed(false, enabledCount > 0, false);
    runStartUs = windowStartUs = nextCycleUs = micros();
}

void loop() {
    if (!canReady) {
        LOG.println("[CAN] init failed, halted");
        delay(2000);
        return;
    }
    if (faulted || enabledCount == 0) {
        while (reportNext < reportCount) LOG.println(reportBuf[reportNext++]);
        delay(100);
        return;
    }

    // Wait for the scheduled start; resync if we fell behind
    while ((int32_t)(micros() - nextCycleUs) < 0) {}
    uint32_t cycleStart = micros();
    uint32_t late = cycleStart - nextCycleUs;
    nextCycleUs += CYCLE_PERIOD_US;
    if ((int32_t)(cycleStart - nextCycleUs) >= 0) nextCycleUs = cycleStart + CYCLE_PERIOD_US;

    // 1. Exchange commands / feedback with all motors
    uint32_t updUs;
    bool ok = updateMotors(updUs);

    // 2. First valid feedback: hold the current position as target (matters once kp > 0)
    static bool targetsInit = false;
    if (!targetsInit) {
        targetsInit = true;
        for (int i = 0; i < NUM_MOTORS; i++) if (state[i].valid) cmd[i].pos = state[i].pos;
    }

    // 3. Safety, then control
    checkSafety();
    if (faulted) return;
    controlUpdate();

    // 4. Statistics
    stats.cycles++;
    if (!ok) stats.failedCycles++;
    else {
        stats.updSum += updUs;
        if (updUs < stats.updMin) stats.updMin = updUs;
        if (updUs > stats.updMax) stats.updMax = updUs;
    }
    if (late > stats.lateMax) stats.lateMax = late;

    // 5. Reporting: one queued line per cycle
    uint32_t now = micros();
    if (now - windowStartUs >= REPORT_PERIOD_US) {
        buildReport(stats, now - windowStartUs, (now - runStartUs) / 1000000);
        resetWindow(stats);
        windowStartUs = now;
    }
    if (reportNext < reportCount) {
        LOG.println(reportBuf[reportNext++]);
        if (reportNext == reportCount) LOG.println();
    }

    if (micros() - cycleStart > CYCLE_PERIOD_US) stats.overruns++;
}
