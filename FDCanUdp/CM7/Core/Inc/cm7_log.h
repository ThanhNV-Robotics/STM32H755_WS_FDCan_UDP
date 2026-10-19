/**
 * cm7_log.h - CM7 serial log on USART3 (ST-LINK virtual COM port, 115200 8N1)
 *
 * Two sources share the UART: CM7's own messages (CM7_LOG, local queue) and the CM4
 * log (Common/ipc_log, SRAM4). CM7_LogPump() moves at most one byte per call into
 * the UART and never waits, so the network loop is not stalled by log output.
 */
#ifndef CM7_LOG_H
#define CM7_LOG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void CM7_Printf(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
#define CM7_LOG(fmt, ...) CM7_Printf(fmt "\r\n", ##__VA_ARGS__)

/* Call every main-loop pass: sends the next byte if the UART is free. */
void CM7_LogPump(void);

/* Blocking send of everything queued (startup / Error_Handler only). */
void CM7_LogFlush(void);

/* Print fault registers and the stacked PC/LR (blocking, for the fault handlers).
 * msp = __get_MSP() inside the handler; the exception frame is located automatically. */
void CM7_FaultDump(const char* what, uint32_t msp);

#ifdef __cplusplus
}
#endif

#endif /* CM7_LOG_H */
