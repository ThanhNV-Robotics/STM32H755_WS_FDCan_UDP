/**
 * cm7_log.c - CM7 serial log on USART3, polled and non-blocking
 *
 * USART3 has no interrupt enabled on CM7, so bytes are written to TDR whenever TXE
 * is set. At 115200 baud one byte takes 87 us, far longer than a main-loop pass.
 */
#include "cm7_log.h"
#include "main.h"
#include "../../../Common/Inc/ipc_log.h"
#include <stdarg.h>
#include <stdio.h>

#define CM7_QUEUE_SIZE 2048U   /* power of two */

static char     sQueue[CM7_QUEUE_SIZE];   /* CM7's own messages */
static uint32_t sHead, sTail;
static uint8_t  sChunk[64];               /* bytes taken from the CM4 log, being sent */
static uint32_t sChunkLen, sChunkPos;
static uint32_t sDropped;

void CM7_Printf(const char* fmt, ...)
{
    char line[256];
    va_list args;
    va_start(args, fmt);
    int n = vsnprintf(line, sizeof(line), fmt, args);
    va_end(args);
    if (n <= 0) return;
    if ((uint32_t)n >= sizeof(line)) n = sizeof(line) - 1;
    if (CM7_QUEUE_SIZE - (sHead - sTail) < (uint32_t)n)
    {
        sDropped++;
        return;
    }
    for (int i = 0; i < n; i++)
        sQueue[(sHead + (uint32_t)i) % CM7_QUEUE_SIZE] = line[i];
    sHead += (uint32_t)n;
}

/* Next byte to send: CM7's own messages first, then the CM4 log. Returns -1 if none. */
static int nextByte(void)
{
    if (sTail != sHead)
        return (uint8_t)sQueue[(sTail++) % CM7_QUEUE_SIZE];
    if (sChunkPos == sChunkLen)
    {
        sChunkLen = IPC_LogRead(sChunk, sizeof(sChunk));
        sChunkPos = 0;
        if (sChunkLen == 0) return -1;
    }
    return sChunk[sChunkPos++];
}

void CM7_LogPump(void)
{
    if ((USART3->ISR & USART_ISR_TXE_TXFNF) == 0U) return;
    int b = nextByte();
    if (b >= 0) USART3->TDR = (uint8_t)b;
}

static void putsBlocking(const char* s)
{
    while (*s)
    {
        while ((USART3->ISR & USART_ISR_TXE_TXFNF) == 0U) {}
        USART3->TDR = (uint8_t)*s++;
    }
    while ((USART3->ISR & USART_ISR_TC) == 0U) {}
}

void CM7_FaultDump(const char* what, uint32_t msp)
{
    char line[200];
    /* The fault handler's own prologue sits between msp and the exception frame
     * (r0 r1 r2 r3 r12 lr pc xpsr). Find the frame: xPSR has the Thumb bit set and
     * the stacked PC points into flash. */
    const uint32_t* frame = NULL;
    for (uint32_t off = 0; off <= 64U; off += 4U)
    {
        const uint32_t* f = (const uint32_t*)(msp + off);
        if ((f[7] & (1UL << 24)) && f[6] >= 0x08000000UL && f[6] < 0x08200000UL)
        {
            frame = f;
            break;
        }
    }
    if (frame)
        snprintf(line, sizeof(line),
                 "\r\n[CM7] %s  PC=0x%08lX  LR=0x%08lX  CFSR=0x%08lX  HFSR=0x%08lX  MMFAR=0x%08lX  BFAR=0x%08lX\r\n",
                 what, frame[6], frame[5], SCB->CFSR, SCB->HFSR, SCB->MMFAR, SCB->BFAR);
    else
        snprintf(line, sizeof(line),
                 "\r\n[CM7] %s  (frame not found)  CFSR=0x%08lX  HFSR=0x%08lX  MMFAR=0x%08lX  BFAR=0x%08lX\r\n",
                 what, SCB->CFSR, SCB->HFSR, SCB->MMFAR, SCB->BFAR);
    putsBlocking(line);
}

void CM7_LogFlush(void)
{
    int b;
    while ((b = nextByte()) >= 0)
    {
        while ((USART3->ISR & USART_ISR_TXE_TXFNF) == 0U) {}
        USART3->TDR = (uint8_t)b;
    }
    while ((USART3->ISR & USART_ISR_TC) == 0U) {}
}
