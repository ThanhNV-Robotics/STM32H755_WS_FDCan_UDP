/**
 * md_bench.c - two-leg MAB MD impedance (zero gains) + 0x40 full-update benchmark (CM4 core)
 *
 * Port of Portenta_H7/fdcan_udp/fdcan_bench/fdcan_bench.ino, extended to two legs that
 * run in parallel (one FDCAN controller each). Times are in us from MD_Micros() (DWT
 * cycle counter); window lengths in ms from HAL_GetTick().
 */
#include "md_bench.h"
#include "mab_md.h"
#include "fdcan.h"
#include "../../../Common/Inc/ipc_log.h"
#include <stdio.h>
#include <string.h>

/* ─── User configuration ─── */
#define UPDATE_TIMEOUT_US 2000U   /* one full 0x40 update cycle (both legs) */
#define WINDOW_MS         3000U   /* duration of each benchmark step */
#define LED_BLINK_MS      250U
#define RETRY_IDLE_US     300U    /* no reply on a leg for this long -> resend to its motors still missing */

#define NUM_LEGS MD_NUM_LEGS
#define NUM_MOT  MD_MOTORS_PER_LEG

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
    bool         retry;      /* resend once to a motor whose reply is missing (PIPE only) */
} BenchStep_t;

static const BenchStep_t BENCH[] = {
    /* pipelined, profile, periodUs, retry */
    {false, PROFILE_FULL, 1000, false},
    {true,  PROFILE_FULL, 1000, false},
    {true,  PROFILE_PVT,  1000, false},
    {true,  PROFILE_TAU,  1000, false},
    {true,  PROFILE_FULL, 0,    false},
    {true,  PROFILE_PVT,  0,    false},
    {true,  PROFILE_TAU,  0,    false},
    /* with the in-cycle retry, as the control loop will run */
    {true,  PROFILE_FULL, 1000, true},
    {true,  PROFILE_PVT,  1000, true},
    {true,  PROFILE_TAU,  1000, true},
    {true,  PROFILE_PVT,  0,    true},
};
#define NUM_STEPS (sizeof(BENCH) / sizeof(BENCH[0]))

/* ─── Per-motor and per-leg window counters ─── */
typedef struct
{
    bool     dumped;            /* raw 0x40 reply already printed */
    uint32_t missed;            /* cycles without a valid reply */
    uint32_t consecMissed, maxConsecMissed;
    uint32_t invalid;           /* reply with wrong header / length */
    uint32_t latSum, latN, latMax;
    uint32_t retries;           /* commands sent a second time */
    uint32_t recovered;         /* ... and answered after the resend */
} MotorStats_t;
static MotorStats_t mstats[NUM_LEGS][NUM_MOT];

typedef struct
{
    uint32_t txFailures;        /* TX FIFO full / HAL error */
    uint32_t rxLost;            /* RX FIFO overflow (message lost) */
    uint32_t doneSum, doneMax;  /* cycle start -> this leg's last reply */
    uint32_t doneN;
} LegStats_t;
static LegStats_t lstats[NUM_LEGS];

/* ─── Full-update cycle statistics (both legs) ─── */
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
static void dumpFirstReply(uint32_t l, uint32_t i, const uint8_t* d, uint32_t len)
{
    if (mstats[l][i].dumped) return;
    mstats[l][i].dumped = true;
    char buf[160];
    int k = snprintf(buf, sizeof(buf), "[RAW] %s id %3u len %lu :",
                     mdLegs[l].name, mdLegs[l].motors[i].canId, len);
    for (uint32_t b = 0; b < len && b < 24U && k < (int)sizeof(buf) - 4; b++)
        k += snprintf(buf + k, sizeof(buf) - k, " %02X", d[b]);
    LOG("%s", buf);
}

/* Handle one frame received on leg l. Returns the motor index if it is a valid reply, else -1. */
static int acceptReply(uint32_t l, const FDCAN_RxHeaderTypeDef* h, const uint8_t* d, uint32_t latencyUs)
{
    MD_Leg_t* leg = &mdLegs[l];
    for (uint32_t i = 0; i < NUM_MOT; i++)
    {
        MD_Motor_t* m = &leg->motors[i];
        if (!m->enabled || h->Identifier != m->canId) continue;
        uint32_t len = MD_DlcToBytes(h->DataLength);
        dumpFirstReply(l, i, d, len);
        if (!MD_ParseDefaultResponse(m, d, len))
        {
            mstats[l][i].invalid++;
            return -1;
        }
        mstats[l][i].latSum += latencyUs;
        mstats[l][i].latN++;
        if (latencyUs > mstats[l][i].latMax) mstats[l][i].latMax = latencyUs;
        return (int)i;
    }
    return -1;
}

static void queueCommand(uint32_t l, uint32_t i)
{
    const MD_Motor_t* m = &mdLegs[l].motors[i];
    if (!MD_SendFrame(m->bus, m->canId, updateFrame, updateFrameLen)) lstats[l].txFailures++;
}

static void checkRxOverflow(uint32_t l)
{
    FDCAN_HandleTypeDef* bus = mdLegs[l].bus;
    if (__HAL_FDCAN_GET_FLAG(bus, FDCAN_FLAG_RX_FIFO0_MESSAGE_LOST))
    {
        __HAL_FDCAN_CLEAR_FLAG(bus, FDCAN_FLAG_RX_FIFO0_MESSAGE_LOST);
        lstats[l].rxLost++;
    }
}

static void markMissed(uint32_t l, uint32_t i, bool got)
{
    MotorStats_t* m = &mstats[l][i];
    if (got) { m->consecMissed = 0; return; }
    m->missed++;
    if (++m->consecMissed > m->maxConsecMissed) m->maxConsecMissed = m->consecMissed;
}

/* Per-leg state of one pipelined cycle */
typedef struct
{
    bool     got[NUM_MOT];
    bool     retried[NUM_MOT];
    uint32_t expected, received;
    uint32_t lastRxUs, doneUs;
} LegCycle_t;

/* Resend once to every enabled motor in positions [0, upTo) of leg l that has not answered */
static void retryMissing(uint32_t l, LegCycle_t* c, uint32_t upTo)
{
    for (uint32_t i = 0; i < upTo; i++)
    {
        if (!mdLegs[l].motors[i].enabled || c->got[i] || c->retried[i]) continue;
        c->retried[i] = true;
        mstats[l][i].retries++;
        queueCommand(l, i);
    }
}

/* SEQ: motor i of both legs at the same time, wait for both replies, then motor i+1.
 * Latency = that motor's round trip. */
static bool updateSequential(void)
{
    bool allOk = true;
    FDCAN_RxHeaderTypeDef h;
    uint8_t d[64];
    for (uint32_t i = 0; i < NUM_MOT; i++)
    {
        bool want[NUM_LEGS], got[NUM_LEGS];
        uint32_t pending = 0;
        for (uint32_t l = 0; l < NUM_LEGS; l++)
        {
            want[l] = mdLegs[l].motors[i].enabled;
            got[l]  = false;
            if (want[l]) { MD_FlushRx(mdLegs[l].bus); pending++; }
        }
        if (pending == 0) continue;

        uint32_t start = MD_Micros();
        for (uint32_t l = 0; l < NUM_LEGS; l++)
            if (want[l]) queueCommand(l, i);

        while (pending > 0 && (uint32_t)(MD_Micros() - start) < UPDATE_TIMEOUT_US)
        {
            for (uint32_t l = 0; l < NUM_LEGS; l++)
            {
                if (!want[l] || got[l]) continue;
                if (HAL_FDCAN_GetRxFifoFillLevel(mdLegs[l].bus, FDCAN_RX_FIFO0) == 0U) continue;
                if (HAL_FDCAN_GetRxMessage(mdLegs[l].bus, FDCAN_RX_FIFO0, &h, d) != HAL_OK) continue;
                if (acceptReply(l, &h, d, MD_Micros() - start) == (int)i) { got[l] = true; pending--; }
            }
        }
        for (uint32_t l = 0; l < NUM_LEGS; l++)
        {
            if (!want[l]) continue;
            if (!got[l]) { allOk = false; MD_AbortPendingTx(mdLegs[l].bus); }
            markMissed(l, i, got[l]);
        }
    }
    for (uint32_t l = 0; l < NUM_LEGS; l++) checkRxOverflow(l);
    return allOk;
}

/* PIPE: queue every command of both legs (ascending CAN ID per leg), then collect from both
 * controllers. They transmit in parallel, so a cycle lasts about as long as one leg.
 * Latency = cycle start -> that motor's reply. Needs automatic retransmission: commands
 * lose arbitration to replies.
 * With step->retry: replies arrive in queue order, so a reply from position p while an
 * earlier position is still missing means that reply was lost -> resend now. The last
 * positions are covered by a timeout (no reply on that leg for RETRY_IDLE_US). */
static bool updatePipelined(const BenchStep_t* step)
{
    LegCycle_t cyc[NUM_LEGS];
    memset(cyc, 0, sizeof(cyc));
    uint32_t expected = 0, received = 0;

    for (uint32_t l = 0; l < NUM_LEGS; l++)
    {
        MD_FlushRx(mdLegs[l].bus);
        for (uint32_t i = 0; i < NUM_MOT; i++)
            if (mdLegs[l].motors[i].enabled) cyc[l].expected++;
        expected += cyc[l].expected;
    }

    uint32_t start = MD_Micros();
    for (uint32_t l = 0; l < NUM_LEGS; l++)
    {
        cyc[l].lastRxUs = start;
        for (uint32_t i = 0; i < NUM_MOT; i++)
            if (mdLegs[l].motors[i].enabled) queueCommand(l, i);
    }

    FDCAN_RxHeaderTypeDef h;
    uint8_t d[64];
    while (received < expected && (uint32_t)(MD_Micros() - start) < UPDATE_TIMEOUT_US)
    {
        for (uint32_t l = 0; l < NUM_LEGS; l++)
        {
            LegCycle_t* c = &cyc[l];
            if (c->received == c->expected) continue;
            if (step->retry && (uint32_t)(MD_Micros() - c->lastRxUs) >= RETRY_IDLE_US)
            {
                retryMissing(l, c, NUM_MOT);
                c->lastRxUs = MD_Micros();
            }
            if (HAL_FDCAN_GetRxFifoFillLevel(mdLegs[l].bus, FDCAN_RX_FIFO0) == 0U) continue;
            if (HAL_FDCAN_GetRxMessage(mdLegs[l].bus, FDCAN_RX_FIFO0, &h, d) != HAL_OK) continue;
            uint32_t now = MD_Micros();
            int i = acceptReply(l, &h, d, now - start);
            if (i < 0 || c->got[i]) continue;
            c->got[i] = true;
            c->received++;
            received++;
            c->lastRxUs = now;
            if (c->received == c->expected) c->doneUs = now - start;
            if (c->retried[i]) mstats[l][i].recovered++;
            if (step->retry) retryMissing(l, c, (uint32_t)i);
        }
    }

    for (uint32_t l = 0; l < NUM_LEGS; l++)
    {
        LegCycle_t* c = &cyc[l];
        if (c->received < c->expected) MD_AbortPendingTx(mdLegs[l].bus);
        else if (c->expected > 0)
        {
            lstats[l].doneSum += c->doneUs;
            lstats[l].doneN++;
            if (c->doneUs > lstats[l].doneMax) lstats[l].doneMax = c->doneUs;
        }
        for (uint32_t i = 0; i < NUM_MOT; i++)
            if (mdLegs[l].motors[i].enabled) markMissed(l, i, c->got[i]);
        checkRxOverflow(l);
    }
    return received == expected;
}

static void printWindow(const BenchStep_t* step, const CycleStats_t* s, uint32_t windowMs)
{
    uint32_t ok = s->n - s->failed;
    char period[16];
    if (step->periodUs) snprintf(period, sizeof(period), "%lu us", step->periodUs);
    else                snprintf(period, sizeof(period), "free-run");

    LOG("[%s %s%s %s] %lu Hz  cycles %lu  failed %lu  over-period %lu",
        step->pipelined ? "PIPE" : "SEQ ", PROFILE_NAMES[step->profile],
        step->retry ? " retry" : "", period,
        windowMs ? (s->n * 1000U + windowMs / 2U) / windowMs : 0U, s->n, s->failed, s->overPeriod);
    LOG("   full update (both legs): avg %lu us  min %lu  p99 %lu  max %lu",
        ok ? s->sumUs / ok : 0U, ok ? s->minUs : 0U, stats_percentile(s, 99), s->maxUs);

    for (uint32_t l = 0; l < NUM_LEGS; l++)
    {
        MD_Leg_t*         leg = &mdLegs[l];
        const LegStats_t* ls  = &lstats[l];
        MD_UpdateBusDiag(leg);
        if (step->pipelined)
            LOG("   leg %s: done avg %lu max %lu us  TEC %lu  REC %lu  txFail %lu  rxLost %lu", leg->name,
                ls->doneN ? ls->doneSum / ls->doneN : 0U, ls->doneMax,
                leg->errorCounters.TxErrorCnt, leg->errorCounters.RxErrorCnt, ls->txFailures, ls->rxLost);
        else
            LOG("   leg %s: TEC %lu  REC %lu  txFail %lu  rxLost %lu", leg->name,
                leg->errorCounters.TxErrorCnt, leg->errorCounters.RxErrorCnt, ls->txFailures, ls->rxLost);

        for (uint32_t i = 0; i < NUM_MOT; i++)
        {
            const MD_Motor_t*   m  = &leg->motors[i];
            const MotorStats_t* ms = &mstats[l][i];
            if (!m->enabled) continue;
            char p[16], v[16], t[16];
            LOG("   %s id %3u : lat avg %4lu max %4lu us  missed %lu (max run %lu)  invalid %lu  qs 0x%04X  T %3u  pos %8s  vel %8s  tau %7s",
                leg->name, m->canId, ms->latN ? ms->latSum / ms->latN : 0U, ms->latMax, ms->missed,
                ms->maxConsecMissed, ms->invalid, m->quickStatus, m->temperature,
                fmt4(p, m->pos), fmt4(v, m->vel), fmt4(t, m->tau));
            if (step->retry && ms->retries)
                LOG("              retried %lu, answered after retry %lu", ms->retries, ms->recovered);
        }
    }
}

static void resetWindowCounters(CycleStats_t* s)
{
    stats_reset(s);
    memset(lstats, 0, sizeof(lstats));
    for (uint32_t l = 0; l < NUM_LEGS; l++)
        for (uint32_t i = 0; i < NUM_MOT; i++)
        {
            MotorStats_t* m = &mstats[l][i];
            bool dumped = m->dumped;
            memset(m, 0, sizeof(*m));
            m->dumped = dumped;
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

static void disableAll(void)
{
    for (uint32_t l = 0; l < NUM_LEGS; l++)
        for (uint32_t i = 0; i < NUM_MOT; i++)
            if (mdLegs[l].motors[i].enabled) MD_Disable(&mdLegs[l].motors[i]);
}

void MD_Bench_Setup(void)
{
    led_set(LED_OFF);
    LOG("=== MAB MD two-leg impedance (zero gains) + 0x40 update benchmark ===");
    checkTimebases();
    MD_PrintConfig();

    /* Ping everything first, then enable: the first enabled motor's CAN watchdog
     * (100 ms) starts running as soon as it is enabled. */
    for (uint32_t l = 0; l < NUM_LEGS; l++)
        LOG("[PING] leg %s: %u/%u motors answered", mdLegs[l].name, MD_PingLeg(&mdLegs[l]), NUM_MOT);

    for (uint32_t l = 0; l < NUM_LEGS; l++)
    {
        uint32_t legEnabled = 0;
        for (uint32_t i = 0; i < NUM_MOT; i++)
        {
            MD_Motor_t* m = &mdLegs[l].motors[i];
            if (!m->alive) continue;
            m->enabled = MD_ConfigureImpedanceZero(m);
            if (m->enabled) legEnabled++;
            else MD_Disable(m);
        }
        enabledCount += legEnabled;
        LOG("[CFG] leg %s: %lu/%u motors enabled (impedance, all zero)", mdLegs[l].name, legEnabled, NUM_MOT);
        if (legEnabled < NUM_MOT)
        {
            MD_UpdateBusDiag(&mdLegs[l]);
            MD_PrintBusDiag(&mdLegs[l]);
        }
    }
    LOG("");

    led_set(enabledCount == NUM_LEGS * NUM_MOT ? LED_GREEN : LED_RED);
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
    bool ok = step->pipelined ? updatePipelined(step) : updateSequential();
    stats_add(&stats, MD_Micros() - t0, ok);

    uint32_t now = HAL_GetTick();
    if (now - windowStartMs >= WINDOW_MS)
    {
        printWindow(step, &stats, now - windowStartMs);
        bool anyMissed = stats.failed > 0U;
        led_set((anyMissed || enabledCount < NUM_LEGS * NUM_MOT) ? LED_RED : LED_GREEN);
        if (anyMissed)
            for (uint32_t l = 0; l < NUM_LEGS; l++) MD_PrintBusDiag(&mdLegs[l]);
        LOG("");

        if (stepIdx + 1U < NUM_STEPS)
        {
            startStep(stepIdx + 1U);
        }
        else
        {
            LOG("[END] benchmark finished, disabling motors");
            disableAll();
            led_set(LED_OFF);
            testDone = true;
        }
    }
}
