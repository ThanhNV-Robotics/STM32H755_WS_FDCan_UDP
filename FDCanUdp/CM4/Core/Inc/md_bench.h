/**
 * md_bench.h - MAB MD impedance (zero gains) + 0x40 full-update benchmark (CM4 core)
 *
 * Port of Portenta_H7/fdcan_udp/fdcan_bench/fdcan_bench.ino (steps 1-3):
 *   1. ping every motor (firmwareVersion)
 *   2. impedance mode, kp = kd = 0, all targets 0, enable (motor must NOT move)
 *   3. each BENCH step runs for 3 s, then all motors are disabled
 *      SEQ : send to motor i, wait for its reply, next motor
 *      PIPE: queue all frames into the TX FIFO at once, then collect replies
 *      Profiles: FULL (pos,vel,tau,kp,kd 32 B), PVT (pos,vel,tau 20 B), TAU (8 B)
 *      Period: 1000 us (1 kHz) or free-run
 *
 * LEDs (Nucleo): green blinking = running OK, red blinking = motor missing / missed
 * replies, both off = test finished, motors disabled.
 */
#ifndef MD_BENCH_H
#define MD_BENCH_H

#ifdef __cplusplus
extern "C" {
#endif

void MD_Bench_Setup(void);   /* call once after MX_FDCAN1_Init() */
void MD_Bench_Loop(void);    /* call from the main loop, never blocks > 3 ms */

#ifdef __cplusplus
}
#endif

#endif /* MD_BENCH_H */
