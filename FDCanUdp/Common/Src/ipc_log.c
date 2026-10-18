/**
 * ipc_log.c - CM4 -> CM7 log channel through shared SRAM4 (compiled into both cores)
 */
#include "../Inc/ipc_log.h"
#include "stm32h7xx_hal.h"
#include <stdarg.h>
#include <stdio.h>

#define IPC_LOG_ADDR    0x38000000UL   /* SRAM4, D3 domain, visible to both cores */
#define IPC_LOG_SIZE    8192U          /* power of two */
#define IPC_LOG_MAGIC   0x214C4F47UL
#define IPC_LOG_WAIT_MS 20U

typedef struct
{
    volatile uint32_t magic;
    volatile uint32_t head;      /* free-running byte count written by CM4 */
    volatile uint32_t tail;      /* free-running byte count sent by CM7 */
    volatile uint32_t dropped;   /* messages CM4 had to drop (buffer full) */
    volatile uint8_t  buf[IPC_LOG_SIZE];
} IpcLog_t;

#define LOG_SHM ((IpcLog_t*)IPC_LOG_ADDR)

void IPC_LogInit(void)
{
    /* Whole-word writes: SRAM ECC must be initialised before any byte access */
    volatile uint32_t* p = (volatile uint32_t*)IPC_LOG_ADDR;
    for (uint32_t i = 0; i < sizeof(IpcLog_t) / 4U; i++)
        p[i] = 0;
    __DMB();
    LOG_SHM->magic = IPC_LOG_MAGIC;
}

void IPC_LogWrite(const char* data, uint32_t len)
{
    IpcLog_t* log = LOG_SHM;
    if (log->magic != IPC_LOG_MAGIC || len > IPC_LOG_SIZE)
        return;

    uint32_t start = HAL_GetTick();
    while (IPC_LOG_SIZE - (log->head - log->tail) < len)
    {
        if ((HAL_GetTick() - start) >= IPC_LOG_WAIT_MS)
        {
            log->dropped++;
            return;
        }
    }

    uint32_t head = log->head;
    for (uint32_t i = 0; i < len; i++)
        log->buf[(head + i) % IPC_LOG_SIZE] = (uint8_t)data[i];
    __DMB();   /* data visible before the new head */
    log->head = head + len;
}

void IPC_Printf(const char* fmt, ...)
{
    char line[256];
    va_list args;
    va_start(args, fmt);
    int n = vsnprintf(line, sizeof(line), fmt, args);
    va_end(args);
    if (n <= 0)
        return;
    if ((uint32_t)n >= sizeof(line))
        n = sizeof(line) - 1;
    IPC_LogWrite(line, (uint32_t)n);
}

void IPC_LogDrain(IPC_LogOut_t out)
{
    IpcLog_t* log = LOG_SHM;
    if (log->magic != IPC_LOG_MAGIC)
        return;

    uint32_t head = log->head;
    __DMB();   /* read head before the data it covers */
    uint32_t tail = log->tail;
    while (tail != head)
    {
        uint32_t idx   = tail % IPC_LOG_SIZE;
        uint32_t chunk = head - tail;
        if (chunk > IPC_LOG_SIZE - idx)
            chunk = IPC_LOG_SIZE - idx;
        out((const uint8_t*)&log->buf[idx], chunk);
        tail += chunk;
        __DMB();
        log->tail = tail;
    }
}
