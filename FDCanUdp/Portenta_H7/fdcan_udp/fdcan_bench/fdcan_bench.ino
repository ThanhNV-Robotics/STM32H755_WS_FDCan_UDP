/**
 * fdcan_m4.ino - Portenta H7 M4 core : FDCAN link to MAB MD drivers
 * ============================================================================
 * Step 1: PING (startup)
 *   Reads firmwareVersion (0x802) from every motor, like CANdle-SDK MD::init().
 *     TX: [0x41][0x00][0x02][0x08][0 0 0 0]
 *     RX: [0x41 | 0x00 on fw 2.4][0x00][0x02][0x08][tag][rev][minor][major]
 *
 * Step 2: IMPEDANCE MODE, ZERO GAINS (motor must NOT move)
 *   Per motor, with WRITE_REGISTER_LEGACY (0x40; fw 2.4.x ignores 0x42):
 *     kp = kd = 0, targetPosition = targetVelocity = targetTorque = 0
 *     motionModeCommand (0x140) = 4 (IMPEDANCE), state (0x142) = 39 (enable)
 *
 * Step 3: FULL UPDATE TIMING with WRITE_REGISTER_LEGACY (0x40)
 *   One 32-byte frame per motor carries the whole command:
 *     [0x40][0x00] [0x150 pos f32] [0x151 vel f32] [0x152 tau f32] [0x050 kp f32] [0x051 kd f32]
 *   and the drive answers with its 16-byte "default response" (no read request needed):
 *     [0] header  [1..2] quickStatus u16  [3] temperature u8
 *     [4..7] position f32  [8..11] velocity f32  [12..15] torque f32
 *   (layout from the legacy CANdle lib; the first reply of each motor is dumped
 *    raw so it can be checked against the real hardware)
 *
 *   Benchmark: each BENCH[] step runs for WINDOW_MS, then all motors are disabled.
 *     SEQ  : send to motor i, wait for its reply, next motor
 *     PIPE : queue all frames into the TX FIFO at once, then collect replies
 *            (needs automatic retransmission: commands lose arbitration to replies)
 *     Command profiles: FULL (pos,vel,tau,kp,kd 32 B), PVT (pos,vel,tau 20 B), TAU (8 B)
 *     Period: 1000 us (1 kHz) or free-run (back-to-back, shows the max rate)
 *   Reported per window: full-update time avg/min/p99/max, cycles over period,
 *   per-motor latency, missed / invalid replies, longest miss streak,
 *   TX FIFO failures, RX FIFO overflows, CAN error counters.
 *
 * Bus: FDCAN1, PH13 = TX, PB8 = RX. 1 Mbps nominal / 8 Mbps data (BRS).
 * Output: SerialRPC -> serial_bridge_m7 on the M7 core -> USB Serial.
 * LED   : green = running OK, red = motor missing / errors, blue = FDCAN init failed,
 *         off = test finished, motors disabled.
 * ============================================================================
 */

#include <Arduino.h>
#include <RPC.h>
#include <SerialRPC.h>
#include <cstring>
#include "stm32h7xx_hal.h"

#define LOG SerialRPC

// ─── User configuration ───
static const uint16_t MOTOR_IDS[]        = {10, 20, 30, 40, 50, 60};
static const int      NUM_MOTORS         = sizeof(MOTOR_IDS) / sizeof(MOTOR_IDS[0]);
static const uint32_t NOMINAL_BITRATE    = 1000000;   // arbitration phase
static const uint32_t DATA_BITRATE       = 8000000;   // must match MD canBaudrate register
static const uint32_t REPLY_TIMEOUT_US   = 3000;      // blocking config transactions
static const uint32_t UPDATE_TIMEOUT_US  = 2000;      // one full 0x40 update cycle
static const uint32_t WINDOW_MS          = 3000;      // duration of each benchmark step
static const uint32_t INTER_FRAME_US     = 300;       // MD drops frames arriving right after a reply

// Impedance command sent every cycle. All zero -> drive outputs no torque.
static const float CMD_POS = 0.0f, CMD_VEL = 0.0f, CMD_TAU = 0.0f;
static const float CMD_KP  = 0.0f, CMD_KD  = 0.0f;

// Benchmark plan (each step runs WINDOW_MS)
enum CmdProfile : uint8_t {
    PROFILE_FULL,   // pos, vel, tau, kp, kd : 32 B
    PROFILE_PVT,    // pos, vel, tau         : 20 B (kp/kd keep their last written value)
    PROFILE_TAU,    // tau only              :  8 B
};
static const char* const PROFILE_NAMES[] = {"FULL 32B", "PVT  20B", "TAU   8B"};

struct BenchStep {
    bool       pipelined;
    CmdProfile profile;
    uint32_t   periodUs;   // 0 = free-run (next cycle starts as soon as the last one ends)
};
static const BenchStep BENCH[] = {
    {false, PROFILE_FULL, 1000},
    {true,  PROFILE_FULL, 1000},
    {true,  PROFILE_PVT,  1000},
    {true,  PROFILE_TAU,  1000},
    {true,  PROFILE_FULL, 0},
    {true,  PROFILE_PVT,  0},
    {true,  PROFILE_TAU,  0},
};
static const int NUM_STEPS = sizeof(BENCH) / sizeof(BENCH[0]);

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
    bool     dumped;    // raw 0x40 reply already printed
    uint8_t  major, minor, revision;
    char     tag;
    // last default response
    uint16_t quickStatus;
    uint8_t  temperature;
    float    pos, vel, tau;
    uint8_t  header;
    uint32_t replyLen;
    // per-window counters
    uint32_t missed;            // cycles without a valid reply
    uint32_t consecMissed, maxConsecMissed;
    uint32_t invalid;           // reply with wrong header / length
    uint32_t latSum, latN, latMax;
};
static MotorStatus motors[NUM_MOTORS];

struct CycleStats {
    static const uint32_t BIN_US = 10, NUM_BINS = 300;   // histogram up to 3 ms
    uint32_t periodUs;
    uint32_t n, failed, overPeriod;
    uint32_t sumUs, minUs, maxUs;
    uint32_t hist[NUM_BINS];
    void reset() {
        n = failed = overPeriod = sumUs = maxUs = 0;
        minUs = 0xFFFFFFFF;
        memset(hist, 0, sizeof(hist));
    }
    void add(uint32_t us, bool ok) {
        n++;
        if (periodUs && us > periodUs) overPeriod++;
        if (!ok) { failed++; return; }
        sumUs += us;
        if (us < minUs) minUs = us;
        if (us > maxUs) maxUs = us;
        uint32_t b = us / BIN_US;
        hist[b < NUM_BINS ? b : NUM_BINS - 1]++;
    }
    // Upper edge of the bin containing the pct-th percentile of successful cycles
    uint32_t percentile(uint32_t pct) const {
        uint32_t ok = n - failed, target = (ok * pct + 99) / 100, acc = 0;
        if (ok == 0) return 0;
        for (uint32_t b = 0; b < NUM_BINS; b++) {
            acc += hist[b];
            if (acc >= target) return (b + 1) * BIN_US;
        }
        return NUM_BINS * BIN_US;
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
    n = putReg(p, n, MD_REG_IMP_KP,        &CMD_KP, 4);
    n = putReg(p, n, MD_REG_IMP_KD,        &CMD_KD, 4);
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
// Step 3: update cycles with WRITE_REGISTER_LEGACY (0x40)
// ============================================================================
static uint8_t  updateFrame[64];
static uint32_t updateFrameLen = 0;

static void buildUpdateFrame(CmdProfile profile) {
    uint32_t n = 0;
    updateFrame[n++] = MD_FRAME_WRITE_REGISTER_LEGACY;
    updateFrame[n++] = 0x00;
    if (profile != PROFILE_TAU) {
        n = putReg(updateFrame, n, MD_REG_TARGET_POS, &CMD_POS, 4);
        n = putReg(updateFrame, n, MD_REG_TARGET_VEL, &CMD_VEL, 4);
    }
    n = putReg(updateFrame, n, MD_REG_TARGET_TORQUE, &CMD_TAU, 4);
    if (profile == PROFILE_FULL) {
        n = putReg(updateFrame, n, MD_REG_IMP_KP, &CMD_KP, 4);
        n = putReg(updateFrame, n, MD_REG_IMP_KD, &CMD_KD, 4);
    }
    updateFrameLen = n;
}

// Returns false if the reply is not a usable default response
static bool parseDefaultResponse(int i, const uint8_t* d, uint32_t len) {
    MotorStatus& m = motors[i];
    m.header   = d[0];
    m.replyLen = len;

    if (!m.dumped) {
        m.dumped = true;
        char buf[160];
        int k = snprintf(buf, sizeof(buf), "[RAW] id %3u len %lu :", MOTOR_IDS[i], len);
        for (uint32_t b = 0; b < len && b < 24 && k < (int)sizeof(buf) - 4; b++)
            k += snprintf(buf + k, sizeof(buf) - k, " %02X", d[b]);
        LOG.println(buf);
    }

    if (len < 16 || d[0] != MD_FRAME_RESPONSE_LEGACY) return false;
    memcpy(&m.quickStatus, &d[1], 2);
    m.temperature = d[3];
    memcpy(&m.pos, &d[4], 4);
    memcpy(&m.vel, &d[8], 4);
    memcpy(&m.tau, &d[12], 4);
    return true;
}

// Handle one received frame. Returns motor index if it is a valid reply, else -1.
static int acceptReply(const FDCAN_RxHeaderTypeDef& h, const uint8_t* d, uint32_t latencyUs) {
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (!motors[i].enabled || h.Identifier != MOTOR_IDS[i]) continue;
        if (!parseDefaultResponse(i, d, dlcToBytes(h.DataLength))) {
            motors[i].invalid++;
            return -1;
        }
        motors[i].latSum += latencyUs;
        motors[i].latN++;
        if (latencyUs > motors[i].latMax) motors[i].latMax = latencyUs;
        return i;
    }
    return -1;
}

static uint32_t txFailures = 0;   // TX FIFO full / HAL error
static uint32_t rxLost     = 0;   // RX FIFO overflow (message lost)

static void queueCommand(int i) {
    if (!sendFrame(MOTOR_IDS[i], updateFrame, updateFrameLen)) txFailures++;
}

static void checkRxOverflow() {
    if (__HAL_FDCAN_GET_FLAG(&hfdcan, FDCAN_FLAG_RX_FIFO0_MESSAGE_LOST)) {
        __HAL_FDCAN_CLEAR_FLAG(&hfdcan, FDCAN_FLAG_RX_FIFO0_MESSAGE_LOST);
        rxLost++;
    }
}

static void markMissed(int i, bool got) {
    MotorStatus& m = motors[i];
    if (got) { m.consecMissed = 0; return; }
    m.missed++;
    if (++m.consecMissed > m.maxConsecMissed) m.maxConsecMissed = m.consecMissed;
}

// SEQ: one motor at a time. Latency = that motor's round trip.
static bool updateSequential() {
    bool allOk = true;
    FDCAN_RxHeaderTypeDef h;
    uint8_t d[64];
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (!motors[i].enabled) continue;
        flushRx();
        uint32_t start = micros();
        queueCommand(i);
        bool got = false;
        while (!got && (uint32_t)(micros() - start) < UPDATE_TIMEOUT_US) {
            if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan, FDCAN_RX_FIFO0) == 0) continue;
            if (HAL_FDCAN_GetRxMessage(&hfdcan, FDCAN_RX_FIFO0, &h, d) != HAL_OK) continue;
            got = (acceptReply(h, d, micros() - start) == i);
        }
        if (!got) { allOk = false; abortPendingTx(); }
        markMissed(i, got);
    }
    checkRxOverflow();
    return allOk;
}

// PIPE: queue every command, then collect. Latency = cycle start -> that motor's reply.
static bool updatePipelined() {
    bool got[NUM_MOTORS] = {false};
    int expected = 0, received = 0;
    flushRx();
    uint32_t start = micros();
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (!motors[i].enabled) continue;
        queueCommand(i);
        expected++;
    }

    FDCAN_RxHeaderTypeDef h;
    uint8_t d[64];
    while (received < expected && (uint32_t)(micros() - start) < UPDATE_TIMEOUT_US) {
        if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan, FDCAN_RX_FIFO0) == 0) continue;
        if (HAL_FDCAN_GetRxMessage(&hfdcan, FDCAN_RX_FIFO0, &h, d) != HAL_OK) continue;
        int i = acceptReply(h, d, micros() - start);
        if (i >= 0 && !got[i]) { got[i] = true; received++; }
    }
    if (received < expected) abortPendingTx();
    for (int i = 0; i < NUM_MOTORS; i++)
        if (motors[i].enabled) markMissed(i, got[i]);
    checkRxOverflow();
    return received == expected;
}

static void printWindow(const BenchStep& step, const CycleStats& s, uint32_t windowMs) {
    char buf[220];
    uint32_t ok = s.n - s.failed;
    char period[16];
    if (step.periodUs) snprintf(period, sizeof(period), "%lu us", step.periodUs);
    else               snprintf(period, sizeof(period), "free-run");

    snprintf(buf, sizeof(buf), "[%s %s %s] %.0f Hz  cycles %lu  failed %lu  over-period %lu",
             step.pipelined ? "PIPE" : "SEQ ", PROFILE_NAMES[step.profile], period,
             s.n * 1000.0f / windowMs, s.n, s.failed, s.overPeriod);
    LOG.println(buf);
    snprintf(buf, sizeof(buf), "   full update: avg %lu us  min %lu  p99 %lu  max %lu",
             ok ? s.sumUs / ok : 0, ok ? s.minUs : 0, s.percentile(99), s.maxUs);
    LOG.println(buf);

    FDCAN_ErrorCountersTypeDef ec = {0};
    HAL_FDCAN_GetErrorCounters(&hfdcan, &ec);
    snprintf(buf, sizeof(buf), "   bus: TEC %lu  REC %lu  txFail %lu  rxLost %lu",
             ec.TxErrorCnt, ec.RxErrorCnt, txFailures, rxLost);
    LOG.println(buf);

    for (int i = 0; i < NUM_MOTORS; i++) {
        const MotorStatus& m = motors[i];
        if (!m.enabled) continue;
        snprintf(buf, sizeof(buf),
                 "   id %3u : lat avg %4lu max %4lu us  missed %lu (max run %lu)  invalid %lu  qs 0x%04X  T %3u  pos %8.4f  vel %8.4f  tau %7.4f",
                 MOTOR_IDS[i], m.latN ? m.latSum / m.latN : 0, m.latMax, m.missed, m.maxConsecMissed,
                 m.invalid, m.quickStatus, m.temperature, m.pos, m.vel, m.tau);
        LOG.println(buf);
    }
}

static void resetWindowCounters(CycleStats& s) {
    s.reset();
    txFailures = rxLost = 0;
    for (int i = 0; i < NUM_MOTORS; i++) {
        MotorStatus& m = motors[i];
        m.missed = m.maxConsecMissed = m.consecMissed = m.invalid = 0;
        m.latSum = m.latN = m.latMax = 0;
    }
}

// micros() and millis() disagreed by ~1.8 % on the M4: compare both with the
// core cycle counter over one second so we know which timebase to trust.
static void checkTimebases() {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    SystemCoreClockUpdate();

    uint32_t ms0 = millis();
    while (millis() == ms0) {}
    ms0 = millis();
    uint32_t us0 = micros(), c0 = DWT->CYCCNT;
    while (millis() - ms0 < 1000) {}
    uint32_t us = micros() - us0, cyc = DWT->CYCCNT - c0;

    char buf[160];
    snprintf(buf, sizeof(buf),
             "[TIME] over 1000 ms of millis(): micros() advanced %lu us, DWT %lu cycles = %.0f us @ SystemCoreClock %lu Hz",
             us, cyc, cyc / (SystemCoreClock / 1e6f), SystemCoreClock);
    LOG.println(buf);
}

// ============================================================================
static bool     canReady = false;
static bool     testDone = false;
static int      enabledCount = 0;
static int      stepIdx = 0;
static uint32_t windowStartMs, nextCycleUs;
static CycleStats stats;

static void startStep(int idx) {
    stepIdx = idx;
    buildUpdateFrame(BENCH[idx].profile);
    stats.periodUs = BENCH[idx].periodUs;
    resetWindowCounters(stats);
    windowStartMs = millis();
    nextCycleUs = micros();
}

void setup() {
    pinMode(LEDR, OUTPUT);
    pinMode(LEDG, OUTPUT);
    pinMode(LEDB, OUTPUT);
    setLed(false, false, false);

    LOG.begin();
    delay(1000);  // give the M7 bridge time to start forwarding
    LOG.println();
    LOG.println("=== Portenta H7 M4 : MAB MD impedance (zero gains) + 0x40 update benchmark ===");
    checkTimebases();

    memset(motors, 0, sizeof(motors));
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

    startStep(0);
}

void loop() {
    if (!canReady) {
        LOG.println("[CAN] init failed, halted");
        delay(2000);
        return;
    }
    if (testDone || enabledCount == 0) { delay(100); return; }

    const BenchStep& step = BENCH[stepIdx];

    // Fixed-rate cycle, or back-to-back when free-running / when a cycle overran
    if (step.periodUs) {
        while ((int32_t)(micros() - nextCycleUs) < 0) {}
        nextCycleUs += step.periodUs;
        if ((int32_t)(micros() - nextCycleUs) > 0) nextCycleUs = micros();
    }

    uint32_t t0 = micros();
    bool ok = step.pipelined ? updatePipelined() : updateSequential();
    stats.add(micros() - t0, ok);

    uint32_t now = millis();
    if (now - windowStartMs >= WINDOW_MS) {
        printWindow(step, stats, now - windowStartMs);
        bool anyMissed = stats.failed > 0;
        setLed(anyMissed || enabledCount < NUM_MOTORS, !anyMissed && enabledCount == NUM_MOTORS, false);
        if (anyMissed) printBusDiag();
        LOG.println();

        if (stepIdx + 1 < NUM_STEPS) {
            startStep(stepIdx + 1);
        } else {
            LOG.println("[END] benchmark finished, disabling motors");
            for (int i = 0; i < NUM_MOTORS; i++) if (motors[i].enabled) disableMotor(i);
            setLed(false, false, false);
            testDone = true;
        }
    }
}
