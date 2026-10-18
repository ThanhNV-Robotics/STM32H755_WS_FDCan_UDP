/**
 * ipc_log.h - CM4 -> CM7 log channel through shared SRAM4 (D3, 0x38000000)
 *
 * CM4 writes text with LOG(), CM7 forwards it to USART3, which is the ST-LINK
 * virtual COM port on the NUCLEO-H755ZI-Q (/dev/ttyACM0, 115200 8N1).
 * Same role as SerialRPC -> serial_bridge_m7 on the Portenta.
 *
 * One producer (CM4) and one consumer (CM7). SRAM4 is not used by either
 * linker script. The CM7 D-cache is off; if it is ever enabled, SRAM4 must be
 * made non-cacheable with the MPU.
 */
#ifndef IPC_LOG_H
#define IPC_LOG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* CM7: clear the buffer. Call before releasing CM4 (boot sync). */
void IPC_LogInit(void);

/* CM4: append text. Waits up to 20 ms for room, then drops the message. */
void IPC_LogWrite(const char* data, uint32_t len);
void IPC_Printf(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
#define LOG(fmt, ...) IPC_Printf(fmt "\r\n", ##__VA_ARGS__)

/* CM7: pass everything written so far to out(). Call from the main loop. */
typedef void (*IPC_LogOut_t)(const uint8_t* data, uint32_t len);
void IPC_LogDrain(IPC_LogOut_t out);

#ifdef __cplusplus
}
#endif

#endif /* IPC_LOG_H */
