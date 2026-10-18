/**
 * md_bench.h - two-leg MAB MD impedance (zero gains) + 0x40 full-update benchmark (CM4 core)
 *
 * Port of Portenta_H7/fdcan_udp/fdcan_bench/fdcan_bench.ino, extended to two legs
 * (mdLegs[], one FDCAN controller each, set up with MD_LegInit()):
 *   1. ping every motor of both legs (firmwareVersion)
 *   2. impedance mode, kp = kd = 0, all targets 0, enable (motor must NOT move)
 *   3. each BENCH step runs for 3 s on both legs at once, then all motors are disabled
 *      SEQ : motor i of both legs at the same time, wait for both replies, next i
 *      PIPE: queue all commands of both legs (ascending CAN ID), then collect replies
 *            from both controllers; the legs transmit in parallel
 *      Profiles: FULL (pos,vel,tau,kp,kd 32 B), PVT (pos,vel,tau 20 B), TAU (8 B)
 *      Period: 1000 us (1 kHz) or free-run; "retry" = resend once to a motor whose
 *      reply is missing (an MD does not retransmit a reply that loses arbitration)
 *
 * LEDs (Nucleo): green blinking = running OK, red blinking = motor missing / missed
 * replies, both off = test finished, motors disabled.
 */
#ifndef MD_BENCH_H
#define MD_BENCH_H

#ifdef __cplusplus
extern "C" {
#endif

void MD_Bench_Setup(void);   /* call once after MX_FDCANx_Init() and MD_LegInit() for both legs */
void MD_Bench_Loop(void);    /* call from the main loop, never blocks > 3 ms */

#ifdef __cplusplus
}
#endif

#endif /* MD_BENCH_H */
