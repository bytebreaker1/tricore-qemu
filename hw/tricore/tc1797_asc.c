/*
 * TC1797 ASC0/ASC1 UART — native chardev-backed model (tc1797_asc.h).
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "tc1797_asc.h"

/* register offsets (TC1796/TC1797 ASC) */
#define ASC_CLC   0x00
#define ASC_ID    0x08
#define ASC_CON   0x10      /* control: R (run/enable) = bit15 */
#define ASC_BG    0x14
#define ASC_FDV   0x18
#define ASC_TBUF  0x20      /* transmit buffer (write transmits) */
#define ASC_RBUF  0x24      /* receive buffer (read consumes)    */
#define ASC_RXFCON 0x40
#define ASC_TXFCON 0x44
#define ASC_FSTAT 0x48      /* RXFFL[3:0] rx-fill, TXFFL[11:8] tx-fill */
#define ASC_TSRC  0xF0      /* TX service request node  */
#define ASC_TBSRC 0xF4
#define ASC_RSRC  0xF8      /* RX service request node  */
#define ASC_ESRC  0xFC      /* error service request node */

#define SRC_SRPN(v)  ((v) & 0xFF)
#define SRC_SRE      (1u << 12)
#define SRC_SRR      (1u << 13)
#define SRC_SETR     (1u << 15)
#define SRC_CLRR     (1u << 14)

static inline uint32_t asc_idx(Tc1797Asc *a, uint32_t addr)
{
    return ((addr - a->base) & (TC1797_ASC_SIZE - 1)) >> 2;
}

/* Raise (or update) a service-request node: set SRR, and if enabled fire IRQ. */
static void asc_raise(Tc1797Asc *a, unsigned node_off)
{
    uint32_t *n = &a->regs[node_off >> 2];
    *n |= SRC_SRR;
    if ((*n & SRC_SRE) && SRC_SRPN(*n) && a->irq_fn) {
        a->irq_fn(a->irq_opaque, SRC_SRPN(*n));
    }
}

static int asc_can_receive(void *opaque)
{
    Tc1797Asc *a = opaque;
    return fifo8_num_free(&a->rx);
}

static void asc_receive(void *opaque, const uint8_t *buf, int size)
{
    Tc1797Asc *a = opaque;
    for (int i = 0; i < size && !fifo8_is_full(&a->rx); i++) {
        fifo8_push(&a->rx, buf[i]);
    }
    asc_raise(a, ASC_RSRC);                 /* RX service request */
}

void tc1797_asc_init(Tc1797Asc *a, uint32_t base, Chardev *chr,
                     AscIrqFn irq_fn, void *opaque)
{
    memset(a, 0, sizeof(*a));
    a->base = base;
    a->irq_fn = irq_fn;
    a->irq_opaque = opaque;
    fifo8_create(&a->rx, TC1797_ASC_RXFIFO);
    a->regs[ASC_ID >> 2] = 0x0048C000;      /* a plausible ASC module ID */
    if (chr) {
        qemu_chr_fe_init(&a->chr, chr, &error_abort);
        qemu_chr_fe_set_handlers(&a->chr, asc_can_receive, asc_receive,
                                 NULL, NULL, a, NULL, true);
    }
}

uint32_t tc1797_asc_read(Tc1797Asc *a, uint32_t addr)
{
    uint32_t off = (addr - a->base) & (TC1797_ASC_SIZE - 1);
    switch (off) {
    case ASC_RBUF: {
        uint8_t b = fifo8_is_empty(&a->rx) ? 0 : fifo8_pop(&a->rx);
        if (fifo8_is_empty(&a->rx)) {
            a->regs[ASC_RSRC >> 2] &= ~SRC_SRR;   /* no more pending */
        }
        return b;
    }
    case ASC_FSTAT: {
        unsigned rxffl = TC1797_ASC_RXFIFO - fifo8_num_free(&a->rx);
        return (rxffl & 0xF) /* | TXFFL=0 (TX always drained) */;
    }
    default:
        return a->regs[asc_idx(a, addr)];
    }
}

void tc1797_asc_write(Tc1797Asc *a, uint32_t addr, uint32_t val)
{
    uint32_t off = (addr - a->base) & (TC1797_ASC_SIZE - 1);
    switch (off) {
    case ASC_TBUF: {
        uint8_t b = val & 0xFF;
        if (qemu_chr_fe_backend_connected(&a->chr)) {
            qemu_chr_fe_write_all(&a->chr, &b, 1);
        }
        asc_raise(a, ASC_TSRC);             /* TX done (buffer empty) */
        break;
    }
    case ASC_TSRC:
    case ASC_TBSRC:
    case ASC_RSRC:
    case ASC_ESRC: {
        /* SRC node: SRPN/SRE config + SETR/CLRR write-1 to SRR. */
        uint32_t *n = &a->regs[off >> 2];
        uint32_t srr = (*n >> 13) & 1;
        if (val & SRC_SETR) {
            srr = 1;
        }
        if (val & SRC_CLRR) {
            srr = 0;
        }
        *n = (val & 0x00001FFFu) | (srr << 13);
        break;
    }
    default:
        a->regs[asc_idx(a, addr)] = val;
        break;
    }
}
