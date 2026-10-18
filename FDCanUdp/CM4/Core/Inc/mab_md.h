/**
 * mab_md.h - MAB MD drivers over FDCAN (CM4 core), one leg per FDCAN controller
 *
 * Port of Portenta_H7/fdcan_udp/fdcan_bench/fdcan_bench.ino (protocol part).
 * A ping reads firmwareVersion (0x802), like MD::init() in the CANdle SDK:
 *   TX: [0x41][0x00][0x02][0x08][0 0 0 0]
 *   RX: [0x41 | 0x00 on fw 2.4][0x00][0x02][0x08][tag][rev][minor][major]
 * Writes use WRITE_REGISTER_LEGACY (0x40): fw 2.4.x silently ignores 0x42.
 * Every 0x40 frame is answered with the 24-byte default response (header 0xA0):
 *   [0] header  [1..2] quickStatus u16  [3] temperature u8
 *   [4..7] position f32  [8..11] velocity f32  [12..15] torque f32
 * An MD does NOT retransmit a reply that loses arbitration: queue commands in
 * ascending CAN ID order and resend once to a motor whose reply is missing.
 *
 * Buses: FDCAN1 / FDCAN2 configured by CubeMX (fdcan.c), 1 Mbps nominal / 8 Mbps data (BRS).
 */
#ifndef MAB_MD_H
#define MAB_MD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

#define MD_NUM_LEGS       2U
#define MD_MOTORS_PER_LEG 6U

/* MD protocol (CANdle-SDK commons/shared_data/md_types.hpp, mab_types.hpp) */
#define MD_FRAME_WRITE_REGISTER_LEGACY 0x40U
#define MD_FRAME_READ_REGISTER         0x41U
#define MD_FRAME_WRITE_REGISTER        0x42U
#define MD_FRAME_RESPONSE_LEGACY       0xA0U
#define MD_FRAME_RESPONSE_ERROR        0xA1U

#define MD_REG_CAN_WATCHDOG     0x003U  /* u16 ms, RW */
#define MD_REG_IMP_KP           0x050U  /* f32, RW */
#define MD_REG_IMP_KD           0x051U  /* f32, RW */
#define MD_REG_MOTION_MODE_CMD  0x140U  /* u8,  WO */
#define MD_REG_MOTION_MODE_STAT 0x141U  /* u8,  RO */
#define MD_REG_STATE            0x142U  /* u16, RW (39 enable, 64 disable) */
#define MD_REG_TARGET_POS       0x150U  /* f32, RW */
#define MD_REG_TARGET_VEL       0x151U  /* f32, RW */
#define MD_REG_TARGET_TORQUE    0x152U  /* f32, RW */
#define MD_REG_FIRMWARE_VERSION 0x802U  /* u32, RO */

#define MD_MODE_IMPEDANCE 4U
#define MD_STATE_ENABLE   39U
#define MD_STATE_DISABLE  64U

typedef struct
{
    FDCAN_HandleTypeDef* bus;       /* controller this motor is on */
    uint16_t canId;
    bool     alive;     /* answered the last ping with a valid firmware version */
    bool     enabled;   /* configured (impedance, zero gains) and enabled */
    uint8_t  major;
    uint8_t  minor;
    uint8_t  revision;
    char     tag;
    uint32_t rttUs;     /* round trip of the last successful ping */
    uint32_t pingOk;    /* successful pings since boot */
    uint32_t pingFail;  /* failed pings since boot */
    /* last default response (0xA0) */
    uint16_t quickStatus;
    uint8_t  temperature;
    float    pos, vel, tau;
} MD_Motor_t;

typedef struct
{
    const char*                 name;      /* "L" / "R", used in the log */
    FDCAN_HandleTypeDef*        bus;
    MD_Motor_t                  motors[MD_MOTORS_PER_LEG];   /* ascending CAN IDs */
    FDCAN_ProtocolStatusTypeDef protocolStatus;  /* LastErrorCode 3 = nobody ACKed */
    FDCAN_ErrorCountersTypeDef  errorCounters;
} MD_Leg_t;

/* Inspect in the debugger (Live Expressions) as well as in the log. */
extern MD_Leg_t mdLegs[MD_NUM_LEGS];

/* Assign a controller and the 6 CAN IDs (ascending) to a leg, clear its state. */
void MD_LegInit(MD_Leg_t* leg, const char* name, FDCAN_HandleTypeDef* bus,
                const uint16_t ids[MD_MOTORS_PER_LEG]);

/* Microsecond timebase from the DWT cycle counter. Call MD_TimeInit() once after
 * HAL_Init(), and MD_Micros() at least every ~17 s (CYCCNT wraps at 200 MHz). */
void     MD_TimeInit(void);
uint32_t MD_Micros(void);
void     MD_DelayUs(uint32_t us);

/* Low-level FDCAN helpers */
bool     MD_SendFrame(FDCAN_HandleTypeDef* bus, uint16_t canId, const uint8_t* tx, uint32_t len);  /* zero padded to the DLC */
void     MD_FlushRx(FDCAN_HandleTypeDef* bus);
void     MD_AbortPendingTx(FDCAN_HandleTypeDef* bus);
uint32_t MD_DlcToBytes(uint32_t dlc);
/* Append [regLSB][regMSB][value bytes] to a register frame, returns the new length */
uint32_t MD_PutReg(uint8_t* buf, uint32_t idx, uint16_t reg, const void* val, uint32_t size);

/* Ping = read firmwareVersion */
bool    MD_Ping(MD_Motor_t* motor);
uint8_t MD_PingLeg(MD_Leg_t* leg);   /* returns the number of motors that answered */

/* Blocking register access. payload = MD_PutReg() entries, max 62 bytes. */
bool MD_WriteRegs(MD_Motor_t* motor, const uint8_t* payload, uint32_t len, const char* what);
bool MD_ReadReg(MD_Motor_t* motor, uint16_t reg, void* value, uint32_t size);

/* Impedance mode with kp = kd = 0 and all targets 0 (motor must not move), then enable */
bool MD_ConfigureImpedanceZero(MD_Motor_t* motor);
void MD_Disable(MD_Motor_t* motor);

/* Fill quickStatus/temperature/pos/vel/tau from a 0xA0 reply. False if not a default response. */
bool MD_ParseDefaultResponse(MD_Motor_t* motor, const uint8_t* d, uint32_t len);

/* Serial log (CM4 -> CM7 -> ST-LINK virtual COM port) */
void MD_PrintConfig(void);              /* clocks, and per leg: bus, IDs, bit timing, started? */
void MD_UpdateBusDiag(MD_Leg_t* leg);   /* read the error counters of the leg's controller */
void MD_PrintBusDiag(const MD_Leg_t* leg);

#ifdef __cplusplus
}
#endif

#endif /* MAB_MD_H */
