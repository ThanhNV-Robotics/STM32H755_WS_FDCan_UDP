/**
 * md_bench.c - MAB MD impedance (zero gains) + 0x40 full-update benchmark (CM4 core)
 *
 * Port of Portenta_H7/fdcan_udp/fdcan_bench/fdcan_bench.ino. Times are in us from
 * MD_Micros() (DWT cycle counter); window lengths in ms from HAL_GetTick().
 */
#include "md_bench.h"
#include "mab_md.h"
#include "fdcan.h"
#include "../../../Common/Inc/ipc_log.h"
#include <stdio.h>
#include <string.h>

/* ─── User configuration ─── */
#define UPDATE_TIMEOUT_US 2000U   /* one full 0x40 update cycle */
#define WINDOW_MS         3000U   /* duration of each benchmark step */
#define LED_BLINK_MS      250U

/* Impedance command sent every cycle. All zero -> drive outputs no torque. */
static const float CMD_POS = 0.0f, CMD_VEL = 0.0f, CMD_TAU = 0.0f;
static const float CMD_KP  = 0.0f, CMD_KD  = 0.0f;

typedef enum
{
    PROFILE_FULL,   /* pos, vel, tau, kp, kd : 32 B */
    PROFILE_PVT,    /* pos, vel, tau         : 20 B (kp/kd keep their last written value) */
    PROFILE_TAU,    /* tau only              :  8 B */
} CmdProfile_E;
static const char* const PROFILE_NAMES[] = {"FULL 32B", "PVT  20B", "TAU   8B"};

typedef struct
{
    bool         pipelined;
    CmdProfile_E profile;
    uint32_t     periodUs;   /* 0 = free-run (next cycle starts as soon as the last one ends) */
} BenchStep_t;

static const BenchStep_t BENCH[] = {
    {false, PROFILE_FULL, 1000},
    {true,  PROFILE_FULL, 1000},
    {true,  PROFILE_PVT,  1000},
    {true,  PROFILE_TAU,  1000},
    {true,  PROFILE_FULL, 0},
    {true,  PROFILE_PVT,  0},
    {true,  PROFILE_TAU,  0},
};
#define NUM_STEPS (sizeof(BENCH) / sizeof(BENCH[0]))

/* ─── Per-motor window counters ─── */
typedef struct
{
    bool     dumped;            /* raw 0x40 reply already printed */
    uint32_t missed;            /* cycles without a valid reply */
    uint32_t consecMissed, maxConsecMissed;
    uint32_t invalid;           /* reply with wrong header / length */
    uint32_t latSum, latN, latMax;
} MotorStats_t;
static MotorStats_t mstats[MD_NUM_MOTORS];

/* ─── Full-update cycle statistics ─── */
#define BIN_US   10U
#define NUM_BINS 300U           /* histogram up to 3 ms */
typedef struct
{
    uint32_t periodUs;
    uint32_t n, failed, overPeriod;
    uint32_t sumUs, minUs, maxUs;
    uint32_t hist[NUM_BINS];
} CycleStats_t;
static CycleStats_t stats;

static void stats_reset(CycleStats_t* s)
{
    s->n = s->failed = s->overPeriod = s->sumUs = s->maxUs = 0;
    s->minUs = 0xFFFFFFFFU;
    memset(s->hist, 0, sizeof(s->hist));
}

static void stats_add(CycleStats_t* s, uint32_t us, bool ok)
{
    s->n++;
    if (s->periodUs && us > s->periodUs) s->overPeriod++;
    if (!ok) { s->failed++; return; }
    s->sumUs += us;
    if (us < s->minUs) s->minUs = us;
    if (us > s->maxUs) s->maxUs = us;
    uint32_t b = us / BIN_US;
    s->hist[b < NUM_BINS ? b : NUM_BINS - 1U]++;
}

/* Upper edge of the bin containing the pct-th percentile of successful cycles */
static uint32_t stats_percentile(const CycleStats_t* s, uint32_t pct)
{
    uint32_t ok = s->n - s->failed, target = (ok * pct + 99U) / 100U, acc = 0;
    if (ok == 0) return 0;
    for (uint32_t b = 0; b < NUM_BINS; b++)
    {
        acc += s->hist[b];
        if (acc >= target) return (b + 1U) * BIN_US;
    }
    return NUM_BINS * BIN_US;
}

/* newlib-nano printf has no %f: print v with 4 decimals into buf (16 bytes) */
static const char* fmt4(char* buf, float v)
{
    int32_t  scaled = (int32_t)(v * 10000.0f + (v >= 0.0f ? 0.5f : -0.5f));
    uint32_t a = (scaled < 0) ? (uint32_t)(-scaled) : (uint32_t)scaled;
    snprintf(buf, 16, "%s%lu.%04lu", scaled < 0 ? "-" : "", a / 10000U, a % 10000U);
    return buf;
}

/* ─── LEDs (Nucleo LD1 green, LD3 red; pins configured by CM7) ─── */
#ifndef LED1_Green_Pin
#define LED1_Green_Pin       GPIO_PIN_0
#define LED1_Green_GPIO_Port GPIOB
#endif
#ifndef LED_Red_Pin
#define LED_Red_Pin          GPIO_PIN_14
#define LED_Red_GPIO_Port    GPIOB
#endif

typedef enum { LED_OFF, LED_GREEN, LED_RED } LedMode_E;
static LedMode_E ledMode = LED_OFF;
static uint32_t  ledLastMs;

static void led_set(LedMode_E mode)
{
    ledMode = mode;
    HAL_GPIO_WritePin(LED1_Green_GPIO_Port, LED1_Green_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, GPIO_PIN_RESET);
}

static void led_update(void)
{
    uint32_t now = HAL_GetTick();
    if (ledMode == LED_OFF || (now - ledLastMs) < LED_BLINK_MS) return;
    ledLastMs = now;
    if (ledMode == LED_GREEN) HAL_GPIO_TogglePin(LED1_Green_GPIO_Port, LED1_Green_Pin);
    else                      HAL_GPIO_TogglePin(LED_Red_GPIO_Port, LED_Red_Pin);
}

/* ============================================================================
 * Update cycles with WRITE_REGISTER_LEGACY (0x40)
 * ========================================================================== */
static uint8_t  updateFrame[64];
static uint32_t updateFrameLen = 0;
static uint32_t txFailures = 0;   /* TX FIFO full / HAL error */
static uint32_t rxLost     = 0;   /* RX FIFO overflow (message lost) */

static void buildUpdateFrame(CmdProfile_E profile)
{
    uint32_t n = 0;
    memset(updateFrame, 0, sizeof(updateFrame));
    updateFrame[n++] = MD_FRAME_WRITE_REGISTER_LEGACY;
    updateFrame[n++] = 0x00;
    if (profile != PROFILE_TAU)
    {
        n = MD_PutReg(updateFrame, n, MD_REG_TARGET_POS, &CMD_POS, 4);
        n = MD_PutReg(updateFrame, n, MD_REG_TARGET_VEL, &CMD_VEL, 4);
    }
    n = MD_PutReg(updateFrame, n, MD_REG_TARGET_TORQUE, &CMD_TAU, 4);
    if (profile == PROFILE_FULL)
    {
        n = MD_PutReg(updateFrame, n, MD_REG_IMP_KP, &CMD_KP, 4);
        n = MD_PutReg(updateFrame, n, MD_REG_IMP_KD, &CMD_KD, 4);
    }
    updateFrameLen = n;
}

/* Dump the first reply of each motor raw so it can be checked against the hardware */
static void dumpFirstReply(uint32_t i, const uint8_t* d, uint32_t len)
{
    if (mstats[i].dumped) return;
    mstats[i].dumped = true;
    char buf[160];
    int k = snprintf(buf, sizeof(buf), "[RAW] id %3u len %lu :", mdMotors[i].canId, len);
    for (uint32_t b = 0; b < len && b < 24U && k < (int)sizeof(buf) - 4; b++)
        k += snprintf(buf + k, sizeof(buf) - k, " %02X", d[b]);
    LOG("%s", buf);
}

/* Handle one received frame. Returns motor index if it is a valid reply, else -1. */
static int acceptReply(const FDCAN_RxHeaderTypeDef* h, const uint8_t* d, uint32_t latencyUs)
{
    for (uint32_t i = 0; i < MD_NUM_MOTORS; i++)
    {
        if (!mdMotors[i].enabled || h->Identifier != mdMotors[i].canId) continue;
        uint32_t len = MD_DlcToBytes(h->DataLength);
        dumpFirstReply(i, d, len);
        if (!MD_ParseDefaultResponse(&mdMotors[i], d, len))
        {
            mstats[i].invalid++;
            return -1;
        }
        mstats[i].latSum += latencyUs;
        mstats[i].latN++;
        if (latencyUs > mstats[i].latMax) mstats[i].latMax = latencyUs;
        return (int)i;
    }
    return -1;
}

static void queueCommand(uint32_t i)
{
    if (!MD_SendFrame(mdMotors[i].canId, updateFrame, updateFrameLen)) txFailures++;
}

static void checkRxOverflow(void)
{
    if (__HAL_FDCAN_GET_FLAG(&hfdcan1, FDCAN_FLAG_RX_FIFO0_MESSAGE_LOST))
    {
        __HAL_FDCAN_CLEAR_FLAG(&hfdcan1, FDCAN_FLAG_RX_FIFO0_MESSAGE_LOST);
        rxLost++;
    }
}

static void markMissed(uint32_t i, bool got)
{
    MotorStats_t* m = &mstats[i];
    if (got) { m->consecMissed = 0; return; }
    m->missed++;
    if (++m->consecMissed > m->maxConsecMissed) m->maxConsecMissed = m->consecMissed;
}

/* SEQ: one motor at a time. Latency = that motor's round trip. */
static bool updateSequential(void)
{
    bool allOk = true;
    FDCAN_RxHeaderTypeDef h;
    uint8_t d[64];
    for (uint32_t i = 0; i < MD_NUM_MOTORS; i++)
    {
        if (!mdMotors[i].enabled) continue;
        MD_FlushRx();
        uint32_t start = MD_Micros();
        queueCommand(i);
        bool got = false;
        while (!got && (uint32_t)(MD_Micros() - start) < UPDATE_TIMEOUT_US)
        {
            if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) == 0U) continue;
            if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &h, d) != HAL_OK) continue;
            got = (acceptReply(&h, d, MD_Micros() - start) == (int)i);
        }
        if (!got) { allOk = false; MD_AbortPendingTx(); }
        markMissed(i, got);
    }
    checkRxOverflow();
    return allOk;
}

/* PIPE: queue every command, then collect. Latency = cycle start -> that motor's reply.
 * Needs automatic retransmission: commands lose arbitration to replies. */
static bool updatePipelined(void)
{
    bool got[MD_NUM_MOTORS] = {false};
    uint32_t expected = 0, received = 0;
    MD_FlushRx();
    uint32_t start = MD_Micros();
    for (uint32_t i = 0; i < MD_NUM_MOTORS; i++)
    {
        if (!mdMotors[i].enabled) continue;
        queueCommand(i);
        expected++;
    }

    FDCAN_RxHeaderTypeDef h;
    uint8_t d[64];
    while (received < expected && (uint32_t)(MD_Micros() - start) < UPDATE_TIMEOUT_US)
    {
        if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) == 0U) continue;
        if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &h, d) != HAL_OK) continue;
        int i = acceptReply(&h, d, MD_Micros() - start);
        if (i >= 0 && !got[i]) { got[i] = true; received++; }
    }
    if (received < expected) MD_AbortPendingTx();
    for (uint32_t i = 0; i < MD_NUM_MOTORS; i++)
        if (mdMotors[i].enabled) markMissed(i, got[i]);
    checkRxOverflow();
    return received == expected;
}

static void printWindow(const BenchStep_t* step, const CycleStats_t* s, uint32_t windowMs)
{
    uint32_t ok = s->n - s->failed;
    char period[16];
    if (step->periodUs) snprintf(period, sizeof(period), "%lu us", step->periodUs);
    else                snprintf(period, sizeof(period), "free-run");

    LOG("[%s %s %s] %lu Hz  cycles %lu  failed %lu  over-period %lu",
        step->pipelined ? "PIPE" : "SEQ ", PROFILE_NAMES[step->profile], period,
        windowMs ? (s->n * 1000U + windowMs / 2U) / windowMs : 0U, s->n, s->failed, s->overPeriod);
    LOG("   full update: avg %lu us  min %lu  p99 %lu  max %lu",
        ok ? s->sumUs / ok : 0U, ok ? s->minUs : 0U, stats_percentile(s, 99), s->maxUs);

    MD_UpdateBusDiag();
    LOG("   bus: TEC %lu  REC %lu  txFail %lu  rxLost %lu",
        mdErrorCounters.TxErrorCnt, mdErrorCounters.RxErrorCnt, txFailures, rxLost);

    for (uint32_t i = 0; i < MD_NUM_MOTORS; i++)
    {
        const MD_Motor_t*   m  = &mdMotors[i];
        const MotorStats_t* ms = &mstats[i];
        if (!m->enabled) continue;
        char p[16], v[16], t[16];
        LOG("   id %3u : lat avg %4lu max %4lu us  missed %lu (max run %lu)  invalid %lu  qs 0x%04X  T %3u  pos %8s  vel %8s  tau %7s",
            m->canId, ms->latN ? ms->latSum / ms->latN : 0U, ms->latMax, ms->missed,
            ms->maxConsecMissed, ms->invalid, m->quickStatus, m->temperature,
            fmt4(p, m->pos), fmt4(v, m->vel), fmt4(t, m->tau));
    }
}

static void resetWindowCounters(CycleStats_t* s)
{
    stats_reset(s);
    txFailures = rxLost = 0;
    for (uint32_t i = 0; i < MD_NUM_MOTORS; i++)
    {
        MotorStats_t* m = &mstats[i];
        m->missed = m->maxConsecMissed = m->consecMissed = m->invalid = 0;
        m->latSum = m->latN = m->latMax = 0;
    }
}

/* Compare HAL_GetTick() (SysTick) with MD_Micros() (DWT) over one second, to check
 * that both timebases and SystemCoreClock agree (the Portenta's were 1.8 % apart). */
static void checkTimebases(void)
{
    uint32_t ms0 = HAL_GetTick();
    while (HAL_GetTick() == ms0) {}
    ms0 = HAL_GetTick();
    uint32_t us0 = MD_Micros();
    while (HAL_GetTick() - ms0 < 1000U) { MD_Micros(); }
    LOG("[TIME] over 1000 ms of HAL_GetTick(): MD_Micros() advanced %lu us @ SystemCoreClock %lu Hz",
        MD_Micros() - us0, SystemCoreClock);
}

/* ============================================================================
 * Setup / loop
 * ========================================================================== */
static bool     testDone = false;
static uint32_t enabledCount = 0;
static uint32_t stepIdx = 0;
static uint32_t windowStartMs, nextCycleUs;

static void startStep(uint32_t idx)
{
    stepIdx = idx;
    buildUpdateFrame(BENCH[idx].profile);
    stats.periodUs = BENCH[idx].periodUs;
    resetWindowCounters(&stats);
    windowStartMs = HAL_GetTick();
    nextCycleUs = MD_Micros();
}

void MD_Bench_Setup(void)
{
    led_set(LED_OFF);
    LOG("=== MAB MD impedance (zero gains) + 0x40 update benchmark ===");
    checkTimebases();
    MD_PrintConfig();

    for (uint32_t i = 0; i < MD_NUM_MOTORS; i++) MD_Ping(&mdMotors[i]);
    for (uint32_t i = 0; i < MD_NUM_MOTORS; i++)
    {
        if (!mdMotors[i].alive) continue;
        mdMotors[i].enabled = MD_ConfigureImpedanceZero(&mdMotors[i]);
        if (mdMotors[i].enabled) enabledCount++;
        else MD_Disable(&mdMotors[i]);
    }

    LOG("[CFG] %lu/%u motors enabled (impedance, all zero)", enabledCount, MD_NUM_MOTORS);
    if (enabledCount < MD_NUM_MOTORS)
    {
        MD_UpdateBusDiag();
        MD_PrintBusDiag();
    }
    LOG("");

    led_set(enabledCount == MD_NUM_MOTORS ? LED_GREEN : LED_RED);
    startStep(0);
}

void MD_Bench_Loop(void)
{
    led_update();
    if (testDone || enabledCount == 0) return;

    const BenchStep_t* step = &BENCH[stepIdx];

    /* Fixed-rate cycle, or back-to-back when free-running / when a cycle overran */
    if (step->periodUs)
    {
        while ((int32_t)(MD_Micros() - nextCycleUs) < 0) {}
        nextCycleUs += step->periodUs;
        if ((int32_t)(MD_Micros() - nextCycleUs) > 0) nextCycleUs = MD_Micros();
    }

    uint32_t t0 = MD_Micros();
    bool ok = step->pipelined ? updatePipelined() : updateSequential();
    stats_add(&stats, MD_Micros() - t0, ok);

    uint32_t now = HAL_GetTick();
    if (now - windowStartMs >= WINDOW_MS)
    {
        printWindow(step, &stats, now - windowStartMs);
        bool anyMissed = stats.failed > 0U;
        led_set((anyMissed || enabledCount < MD_NUM_MOTORS) ? LED_RED : LED_GREEN);
        if (anyMissed) MD_PrintBusDiag();
        LOG("");

        if (stepIdx + 1U < NUM_STEPS)
        {
            startStep(stepIdx + 1U);
        }
        else
        {
            LOG("[END] benchmark finished, disabling motors");
            for (uint32_t i = 0; i < MD_NUM_MOTORS; i++)
                if (mdMotors[i].enabled) MD_Disable(&mdMotors[i]);
            led_set(LED_OFF);
            testDone = true;
        }
    }
}
