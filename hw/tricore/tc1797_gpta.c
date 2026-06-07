/*
 * TC1797 GPTA0/GPTA1/LTCA2 timer arrays — native model (see tc1797_gpta.h).
 */
#include "qemu/osdep.h"
#include "qemu/error-report.h"
#include "tc1797_gpta.h"

void tc1797_gpta_init(Tc1797Gpta *g, uint32_t base, GptaIrqFn irq_fn, void *opaque)
{
    memset(g, 0, sizeof(*g));
    g->base = base;
    g->irq_fn = irq_fn;
    g->irq_opaque = opaque;
}

static inline uint32_t gpta_idx(uint32_t addr)
{
    return (addr - TC1797_GPTA_LO) >> 2;
}

uint32_t tc1797_gpta_read(Tc1797Gpta *g, uint32_t addr, uint64_t now_ns)
{
    addr = TC1797_GPTA_LO + (addr - g->base);   /* relocate to IP-relative space */
    uint32_t idx = gpta_idx(addr);
    if (idx >= TC1797_GPTA_NREG) {
        return 0;
    }
    /*
     * Config / capture-latch registers the firmware wrote read back exactly.
     * An un-written register reads 0 by default (silicon reset; the firmware's
     * boot polls tolerate this). Free-running cell timers (a virtual-clock
     * count, 100 MHz) are opt-in (g->freerun): exposing an advancing value on
     * EVERY unwritten register perturbs a firmware status branch and stalls the
     * boot, so it is gated rather than default.
     */
    uint32_t rv;
    if (g->written[idx]) {
        rv = g->shadow[idx];
    } else if (g->freerun) {
        rv = (uint32_t)((now_ns - g->t0_ns) / 10u);   /* 10 ns == 100 MHz */
    } else {
        rv = 0;
    }
    if (getenv("TC1797_GPTALOG")) {
        static unsigned gc;
        if (gc++ < 6000) {
            fprintf(stderr, "GPTAR [0x%08x] = 0x%08x\n", (uint32_t)addr, rv);
            fflush(stderr);
        }
    }
    return rv;
}

void tc1797_gpta_write(Tc1797Gpta *g, uint32_t addr, uint32_t val)
{
    addr = TC1797_GPTA_LO + (addr - g->base);
    uint32_t idx = gpta_idx(addr);
    if (idx < TC1797_GPTA_NREG) {
        g->shadow[idx] = val;
        g->written[idx] = 1;
    }
}

void tc1797_gpta_capture(Tc1797Gpta *g, uint32_t cell_off,
                         uint32_t value, uint8_t srpn)
{
    uint32_t idx = cell_off >> 2;
    if (idx < TC1797_GPTA_NREG) {
        g->shadow[idx] = value;     /* latch the captured timer value */
        g->written[idx] = 1;
    }
    if (srpn && g->irq_fn) {
        g->irq_fn(g->irq_opaque, srpn);
    }
}

/* ───────────────────────── opt-in self-test ─────────────────────────────── */
static unsigned gpta_test_srpn;
static void gpta_test_irq(void *o, uint8_t srpn) { (void)o; gpta_test_srpn = srpn; }

void tc1797_gpta_selftest(GptaIrqFn irq_fn, void *opaque)
{
    Tc1797Gpta g;
    unsigned pass = 0, fail = 0;
#define CHK(c) do { if (c) pass++; else { fail++; \
        error_report("GPTA selftest FAIL @%d", __LINE__); } } while (0)

    tc1797_gpta_init(&g, TC1797_GPTA_LO, gpta_test_irq, NULL);
    g.freerun = true;

    /* free-running timer advances with virtual time */
    uint32_t t_a = tc1797_gpta_read(&g, 0xF0002020, 1000);
    uint32_t t_b = tc1797_gpta_read(&g, 0xF0002020, 100000);
    CHK(t_b > t_a);

    /* config write reads back exactly (no longer free-running) */
    tc1797_gpta_write(&g, 0xF0002020, 0xCAFE1234);
    CHK(tc1797_gpta_read(&g, 0xF0002020, 999999) == 0xCAFE1234);

    /* capture latches a value + raises the cell SRC */
    gpta_test_srpn = 0;
    tc1797_gpta_capture(&g, 0xF0001840 - TC1797_GPTA_LO, 0x00ABCDEF, 28);
    CHK(gpta_test_srpn == 28
        && tc1797_gpta_read(&g, 0xF0001840, 0) == 0x00ABCDEF);

    info_report("tc1797 GPTA selftest: %u passed, %u failed", pass, fail);
    (void)irq_fn; (void)opaque;
#undef CHK
}
