/**
 * mab_md.h - MAB MD drivers over FDCAN1 (CM4 core)
 *
 * Port of Portenta_H7/fdcan_udp/fdcan_bench/fdcan_bench.ino (protocol part).
 * A ping reads firmwareVersion (0x802), like MD::init() in the CANdle SDK:
 *   TX: [0x41][0x00][0x02][0x08][0 0 0 0]
 *   RX: [0x41 | 0x00 on fw 2.4][0x00][0x02][0x08][tag][rev][minor][major]
 * Writes use WRITE_REGISTER_LEGACY (0x40): fw 2.4.x silently ignores 0x42.
 * Every 0x40 frame is answered with the 24-byte default response (header 0xA0):
 *   [0] header  [1..2] quickStatus u16  [3] temperature u8
 *   [4..7] position f32  [8..11] velocity f32  [12..15] torque f32
 *
 * Bus: FDCAN1 or FDCAN2 (MD_SetBus), configured by CubeMX (fdcan.c),
 * 1 Mbps nominal / 8 Mbps data (BRS).
 */
#ifndef MAB_MD_H
#define MAB_MD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

#define MD_NUM_MOTORS 6U

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

/* Inspect these in the debugger (Live Expressions) as well as in the log. */
extern MD_Motor_t                  mdMotors[MD_NUM_MOTORS];
extern FDCAN_ProtocolStatusTypeDef mdProtocolStatus;  /* LastErrorCode 3 = nobody ACKed */
extern FDCAN_ErrorCountersTypeDef  mdErrorCounters;

/* Select the FDCAN controller the motors are on (default &hfdcan1). Call before MD_Bench_Setup(). */
void                 MD_SetBus(FDCAN_HandleTypeDef* bus);
FDCAN_HandleTypeDef* MD_Bus(void);

/* Microsecond timebase from the DWT cycle counter. Call MD_TimeInit() once after
 * HAL_Init(), and MD_Micros() at least every ~17 s (CYCCNT wraps at 200 MHz). */
void     MD_TimeInit(void);
uint32_t MD_Micros(void);
void     MD_DelayUs(uint32_t us);

/* Low-level FDCAN helpers */
bool     MD_SendFrame(uint16_t canId, const uint8_t* tx, uint32_t len);  /* len bytes, zero padded to the DLC */
void     MD_FlushRx(void);
void     MD_AbortPendingTx(void);
uint32_t MD_DlcToBytes(uint32_t dlc);
/* Append [regLSB][regMSB][value bytes] to a register frame, returns the new length */
uint32_t MD_PutReg(uint8_t* buf, uint32_t idx, uint16_t reg, const void* val, uint32_t size);

/* Ping = read firmwareVersion */
bool    MD_Ping(MD_Motor_t* motor);
uint8_t MD_PingAll(void);   /* returns the number of motors that answered */

/* Blocking register access. payload = MD_PutReg() entries, max 62 bytes. */
bool MD_WriteRegs(MD_Motor_t* motor, const uint8_t* payload, uint32_t len, const char* what);
bool MD_ReadReg(MD_Motor_t* motor, uint16_t reg, void* value, uint32_t size);

/* Impedance mode with kp = kd = 0 and all targets 0 (motor must not move), then enable */
bool MD_ConfigureImpedanceZero(MD_Motor_t* motor);
void MD_Disable(MD_Motor_t* motor);

/* Fill quickStatus/temperature/pos/vel/tau from a 0xA0 reply. False if not a default response. */
bool MD_ParseDefaultResponse(MD_Motor_t* motor, const uint8_t* d, uint32_t len);

/* Serial log (CM4 -> CM7 -> ST-LINK virtual COM port) */
void MD_PrintConfig(void);   /* clocks, bit timing, started? */
void MD_PrintBusDiag(void);  /* error counters, refreshed by MD_PingAll() / MD_UpdateBusDiag() */
void MD_UpdateBusDiag(void);

#ifdef __cplusplus
}
#endif

#endif /* MAB_MD_H */
