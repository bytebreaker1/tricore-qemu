/*
 * TC1797 DMA controller — block-move engine on its OWN worker thread.
 *
 * Register block at 0xF0003C00. Per-channel: SADR (source), DADR (destination),
 * ADICR (address/increment + interrupt control), CHCR (transfer count + width +
 * enable + software trigger). On an enabled software trigger the channel is
 * queued to a worker thread that performs the move SADR->DADR asynchronously
 * and, on completion, raises the channel's service request -- so the CPU is not
 * stalled at the trigger instruction for the whole transfer (the speedup that
 * threading buys for large block moves; real DMA runs concurrently with the CPU).
 *
 * This firmware does not drive the DMA, so the engine is idle here; the worker
 * thread sleeps until a transfer is queued.
 */
#ifndef HW_TRICORE_TC1797_DMA_H
#define HW_TRICORE_TC1797_DMA_H

#include "qemu/thread.h"

#define TC1797_DMA_BASE  0xF0003C00u
#define TC1797_DMA_NCH   8
#define TC1797_DMA_CHSZ  0x20u
#define TC1797_DMA_END   (TC1797_DMA_BASE + TC1797_DMA_NCH * TC1797_DMA_CHSZ)

/* Transfer-complete service request (host callback, invoked under the BQL). */
typedef void (*DmaIrqFn)(void *opaque, uint8_t srpn);

typedef struct Tc1797DmaCh {
    uint32_t sadr, dadr, adicr, chcr;
    uint8_t  done_srpn;              /* completion service-request priority (0=none) */
} Tc1797DmaCh;

typedef struct Tc1797Dma {
    Tc1797DmaCh ch[TC1797_DMA_NCH];
    uint64_t transfers;             /* completed block moves (telemetry) */

    /* worker thread */
    QemuThread thread;
    QemuMutex  mutex;
    QemuCond   cond;
    uint8_t    queue[TC1797_DMA_NCH * 4];
    unsigned   qhead, qtail, qcount;
    bool       running, started;
    DmaIrqFn   irq_fn;
    void      *irq_opaque;
} Tc1797Dma;

void tc1797_dma_init(Tc1797Dma *d, DmaIrqFn irq_fn, void *opaque);
void tc1797_dma_start(Tc1797Dma *d);   /* spawn the worker thread */
uint32_t tc1797_dma_read(Tc1797Dma *d, uint32_t addr);
void tc1797_dma_write(Tc1797Dma *d, uint32_t addr, uint32_t val);
void tc1797_dma_selftest(void);

#endif /* HW_TRICORE_TC1797_DMA_H */
