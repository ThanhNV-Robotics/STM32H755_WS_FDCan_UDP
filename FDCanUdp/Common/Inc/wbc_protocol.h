/**
 * wbc_protocol.h - UDP packets between the PC whole-body controller (WBC) and the Nucleo
 *
 * Used by CM7 (UDP server), CM4 (control loop, via the SRAM4 mailboxes) and mirrored in
 * pc/wbc_udp_test.py. Little-endian, packed. See NOTES.md "UDP link to the PC (CM7)".
 *
 *   PC -> Nucleo  192.168.10.10:9999   WbcCommandPacket (256 bytes)
 *   Nucleo -> PC  sender's IP:port     WbcStatePacket   (224 bytes), one per command
 *
 * Motor order (index 0..11): left leg 10, 20, 30, 40, 50, 60, right leg 70 ... 120.
 */
#ifndef WBC_PROTOCOL_H
#define WBC_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define WBC_UDP_PORT      9999U
#define WBC_NUM_MOTORS    12U
#define WBC_VERSION       1U
#define WBC_CMD_MAGIC     0x31434257UL   /* "WBC1" */
#define WBC_STATE_MAGIC   0x31545357UL   /* "WST1" */

/* WbcCommandPacket.flags */
#define WBC_CMD_ENABLE       (1U << 0)   /* motors enabled; 0 = disable all */
#define WBC_CMD_GAINS_VALID  (1U << 1)   /* kp/kd are valid (sent as a FULL frame when changed) */

/* WbcStatePacket.status */
#define WBC_ST_ENABLED       (1U << 0)
#define WBC_ST_CMD_TIMEOUT   (1U << 1)   /* no fresh command: safe action active */
#define WBC_ST_MOTOR_FAULT   (1U << 2)
#define WBC_ST_NO_CM4_DATA   (1U << 3)   /* states are not from CM4 (UDP-only test firmware) */

/* WbcMotorState.flags */
#define WBC_MOT_ONLINE       (1U << 0)
#define WBC_MOT_ENABLED      (1U << 1)
#define WBC_MOT_MISSED       (1U << 2)   /* no reply in the last CAN cycle */

typedef struct __attribute__((packed))
{
    float pos;   /* rad */
    float vel;   /* rad/s */
    float tau;   /* Nm (feed-forward) */
    float kp;
    float kd;
} WbcMotorCmd;

typedef struct __attribute__((packed))
{
    uint32_t    magic;       /* WBC_CMD_MAGIC */
    uint16_t    version;     /* WBC_VERSION */
    uint16_t    flags;       /* WBC_CMD_* */
    uint32_t    seq;         /* incremented by the PC */
    uint32_t    pcTimeUs;
    WbcMotorCmd motors[WBC_NUM_MOTORS];
} WbcCommandPacket;

typedef struct __attribute__((packed))
{
    float    pos;
    float    vel;
    float    tau;
    uint16_t quickStatus;
    uint8_t  temperature;
    uint8_t  flags;      /* WBC_MOT_* */
} WbcMotorState;

typedef struct __attribute__((packed))
{
    uint32_t      magic;      /* WBC_STATE_MAGIC */
    uint16_t      version;    /* WBC_VERSION */
    uint16_t      status;     /* WBC_ST_* */
    uint32_t      seqEcho;    /* seq of the last command applied */
    uint32_t      stmTimeUs;
    uint32_t      cm4Cycle;   /* CM4 control cycle counter */
    uint16_t      cycleUs;    /* duration of the last full CAN update */
    uint16_t      cmdAgeUs;   /* age of the applied command */
    uint8_t       reserved[8];
    WbcMotorState motors[WBC_NUM_MOTORS];
} WbcStatePacket;

_Static_assert(sizeof(WbcMotorCmd) == 20, "WbcMotorCmd layout");
_Static_assert(sizeof(WbcCommandPacket) == 256, "WbcCommandPacket layout");
_Static_assert(sizeof(WbcMotorState) == 16, "WbcMotorState layout");
_Static_assert(sizeof(WbcStatePacket) == 224, "WbcStatePacket layout");

#ifdef __cplusplus
}
#endif

#endif /* WBC_PROTOCOL_H */
