/*
 * TC1797 DMA controller — threaded block-move engine (tc1797_dma.h).
 */
#include "qemu/osdep.h"
#include "qemu/error-report.h"
#include "qemu/main-loop.h"        /* bql_lock/bql_unlock */
#include "exec/memattrs.h"
#include "system/address-spaces.h"
#include "system/memory.h"
#include "tc1797_dma.h"

/* per-channel register offsets */
#define DMA_SADR  0x00
#define DMA_DADR  0x04
#define DMA_ADICR 0x08
#define DMA_CHCR  0x0C

/* CHCR fields: TREL[15:0] count, CHDW[18:16] width, EN[20] enable, SCH[24] trigger */
#define CHCR_TREL(v)  ((v) & 0xFFFF)
#define CHCR_CHDW(v)  (((v) >> 16) & 0x7)
#define CHCR_EN       (1u << 20)
#define CHCR_SCH      (1u << 24)

static uint32_t dma_mem_read(uint32_t addr, unsigned sz)
{
    uint8_t b[4] = {0};
    address_space_rw(&address_space_memory, addr, MEMTXATTRS_UNSPECIFIED, b, sz, false);
    uint32_t v = 0;
    for (unsigned i = 0; i < sz; i++) {
        v |= (uint32_t)b[i] << (8 * i);
    }
    return v;
}
static void dma_mem_write(uint32_t addr, unsigned sz, uint32_t val)
{
    uint8_t b[4];
    for (unsigned i = 0; i < sz; i++) {
        b[i] = (val >> (8 * i)) & 0xff;
    }
    address_space_rw(&address_space_memory, addr, MEMTXATTRS_UNSPECIFIED, b, sz, true);
}

static unsigned dma_step(unsigned mode, unsigned width)
{
    return (mode == 1) ? width : (mode == 2) ? (unsigned)(-(int)width) : 0;
}

/* Perform channel n's block move (caller holds the BQL). */
static void dma_run_channel(Tc1797Dma *d, int n)
{
    Tc1797DmaCh *c = &d->ch[n];
    unsigned width = 1u << (CHCR_CHDW(c->chcr) <= 2 ? CHCR_CHDW(c->chcr) : 2);
    unsigned count = CHCR_TREL(c->chcr);
    unsigned sinc = dma_step(c->adicr & 0x7, width);
    unsigned dinc = dma_step((c->adicr >> 4) & 0x7, width);
    for (unsigned i = 0; i < count; i++) {
        dma_mem_write(c->dadr, width, dma_mem_read(c->sadr, width));
        c->sadr += sinc;
        c->dadr += dinc;
    }
    c->chcr &= ~(CHCR_SCH | 0xFFFFu);          /* clear trigger + count */
    d->transfers++;
}

static int dma_decode(uint32_t addr, unsigned *off)
{
    if (addr < TC1797_DMA_BASE || addr >= TC1797_DMA_END) {
        return -1;
    }
    unsigned rel = addr - TC1797_DMA_BASE;
    *off = rel % TC1797_DMA_CHSZ;
    return rel / TC1797_DMA_CHSZ;
}

/* ── worker thread: run queued transfers asynchronously to the CPU ── */
static void *dma_thread_fn(void *arg)
{
    Tc1797Dma *d = arg;
    qemu_mutex_lock(&d->mutex);
    while (d->running) {
        while (d->running && d->qcount == 0) {
            qemu_cond_wait(&d->cond, &d->mutex);
        }
        if (!d->running) {
            break;
        }
        int n = d->queue[d->qhead];
        d->qhead = (d->qhead + 1) % (TC1797_DMA_NCH * 4);
        d->qcount--;
        qemu_mutex_unlock(&d->mutex);

        bql_lock();
        dma_run_channel(d, n);
        uint8_t srpn = d->ch[n].done_srpn;
        if (srpn && d->irq_fn) {
            d->irq_fn(d->irq_opaque, srpn);     /* transfer-complete service request */
        }
        bql_unlock();

        qemu_mutex_lock(&d->mutex);
    }
    qemu_mutex_unlock(&d->mutex);
    return NULL;
}

void tc1797_dma_init(Tc1797Dma *d, DmaIrqFn irq_fn, void *opaque)
{
    memset(d, 0, sizeof(*d));
    d->irq_fn = irq_fn;
    d->irq_opaque = opaque;
    qemu_mutex_init(&d->mutex);
    qemu_cond_init(&d->cond);
}

void tc1797_dma_start(Tc1797Dma *d)
{
    if (d->started) {
        return;
    }
    d->running = true;
    d->started = true;
    qemu_thread_create(&d->thread, "tc1797-dma", dma_thread_fn, d,
                       QEMU_THREAD_JOINABLE);
}

uint32_t tc1797_dma_read(Tc1797Dma *d, uint32_t addr)
{
    unsigned off;
    int n = dma_decode(addr, &off);
    if (n < 0) {
        return 0;
    }
    switch (off) {
    case DMA_SADR:  return d->ch[n].sadr;
    case DMA_DADR:  return d->ch[n].dadr;
    case DMA_ADICR: return d->ch[n].adicr;
    case DMA_CHCR:  return d->ch[n].chcr;
    default:        return 0;
    }
}

void tc1797_dma_write(Tc1797Dma *d, uint32_t addr, uint32_t val)
{
    unsigned off;
    int n = dma_decode(addr, &off);
    if (n < 0) {
        return;
    }
    switch (off) {
    case DMA_SADR:  d->ch[n].sadr = val; break;
    case DMA_DADR:  d->ch[n].dadr = val; break;
    case DMA_ADICR: d->ch[n].adicr = val; break;
    case DMA_CHCR:
        d->ch[n].chcr = val;
        if ((val & CHCR_EN) && (val & CHCR_SCH)) {
            /* Queue the transfer for the worker thread (async): the CPU's
             * trigger store returns immediately and the move runs concurrently.
             * If the worker is not started (e.g. unit test), run inline. */
            if (d->started) {
                qemu_mutex_lock(&d->mutex);
                if (d->qcount < sizeof(d->queue)) {
                    d->queue[d->qtail] = n;
                    d->qtail = (d->qtail + 1) % (TC1797_DMA_NCH * 4);
                    d->qcount++;
                    qemu_cond_signal(&d->cond);
                }
                qemu_mutex_unlock(&d->mutex);
            } else {
                dma_run_channel(d, n);
            }
        }
        break;
    default: break;
    }
}

void tc1797_dma_selftest(void)
{
    Tc1797Dma d;
    tc1797_dma_init(&d, NULL, NULL);           /* not started -> inline transfer */
    unsigned pass = 0, fail = 0;
#define CHK(c) do { if (c) pass++; else { fail++; \
        error_report("DMA selftest FAIL @%d", __LINE__); } } while (0)
    uint32_t src = 0xAFE80100, dst = 0xAFE80200;

    for (int i = 0; i < 4; i++) {
        dma_mem_write(src + i * 4, 4, 0x11111111u * (i + 1));
        dma_mem_write(dst + i * 4, 4, 0);
    }
    uint32_t base = TC1797_DMA_BASE;           /* channel 0 */
    tc1797_dma_write(&d, base + DMA_SADR, src);
    tc1797_dma_write(&d, base + DMA_DADR, dst);
    tc1797_dma_write(&d, base + DMA_ADICR, 0x00000011);
    tc1797_dma_write(&d, base + DMA_CHCR, 4u | (2u << 16) | CHCR_EN | CHCR_SCH);

    bool ok = (d.transfers == 1);
    for (int i = 0; i < 4; i++) {
        ok &= (dma_mem_read(dst + i * 4, 4) == 0x11111111u * (i + 1));
    }
    CHK(ok);
    info_report("tc1797 DMA selftest: %u passed, %u failed", pass, fail);
#undef CHK
}
