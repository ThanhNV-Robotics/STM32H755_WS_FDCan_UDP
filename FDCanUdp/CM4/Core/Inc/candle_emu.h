/**
 * candle_emu.h - CANdle emulation on CM4 for nucleotool (MAINTENANCE mode only)
 *
 * Executes CANdle commands that CM7 received over UDP (Common/Inc/tool_shm.h) on the
 * leg's FDCAN bus, so the unmodified CANdle-SDK MD/PDS code works through the board.
 * Kept apart from the control loop: it runs only from the MAINTENANCE branch of the
 * CM4 main loop, and CM7 refuses tool requests in any other mode.
 */
#ifndef CANDLE_EMU_H
#define CANDLE_EMU_H

#ifdef __cplusplus
extern "C" {
#endif

/* Publish the CM4 mode to CM7 (TOOL_MODE_* in tool_shm.h). */
void CandleEmu_SetMode(unsigned mode);
/* Advance the tool requests of both buses (take new ones, check for replies and
 * timeouts). Never waits; call it as often as possible, only in MAINTENANCE mode. */
void CandleEmu_Poll(void);

#ifdef __cplusplus
}
#endif

#endif /* CANDLE_EMU_H */
